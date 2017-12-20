/*
 * Copyright (c) 2016-2017 Chris Reed
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "argon/argon.h"
#include "samplbaer.h"
#include "board.h"
#include "pin_irq_manager.h"
#include "wav_file.h"
#include "utility.h"
#include "led.h"
#include "channel_led.h"
#include "fader_led.h"
#include "microseconds.h"
#include "debug_log.h"
#include "reader_thread.h"
#include "channel_adc_processor.h"
#include "channel_gate.h"
#include "channel_cv.h"
#include "analog_in.h"
#include "sampler_synth.h"
#include "ui.h"
#include "persistent_data_store.h"
#include "fsl_sd_disk.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_adc16.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <ctype.h>
#include <utility>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

#define ENABLE_LOAD_REPORT (0)

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void init_thread(void * arg);

#if ENABLE_LOAD_REPORT
void load_thread(void * arg);
#endif

void flash_leds();

void init_dma();
void init_irq_priorities();
void init_adc_config();
void init_audio_out();
void init_fs();

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

namespace slab {

int16_t g_outBuf[kAudioBufferCount][kAudioBufferSize * kAudioChannelCount] __attribute__ ((section(".buffers"),aligned(4)));
int16_t g_sampleBufs[kVoiceCount][SampleBufferManager::kBufferCount * SampleBufferManager::kBufferSize] __attribute__ ((section(".buffers"),aligned(4)));

Ar::Thread * g_initThread = NULL;

#if ENABLE_LOAD_REPORT
Ar::ThreadWithStack<2048> g_loadReportThread("load", load_thread, 0, kUIThreadPriority-1, kArStartThread);
#endif

UI g_ui;
AudioOutput g_audioOut;
CardManager g_cardManager;
FileManager g_fileManager;
SamplerSynth g_sampler;
ReaderThread g_readerThread;
PinIrqManager g_pinManager;
ChannelLEDManager g_channelLedManager;
SamplerVoice g_voice[kVoiceCount];
ChannelGate g_gates[kVoiceCount];
ChannelCV g_cvs[kVoiceCount];
Pot g_pots[kVoiceCount];
ChannelAdcProcessor g_adcProcessor;
ChannelLED<0> g_ch1Led;
ChannelLED<1> g_ch2Led;
ChannelLED<2> g_ch3Led;
ChannelLED<3> g_ch4Led;
LEDBase * g_channelLeds[] = { &g_ch1Led, &g_ch2Led, &g_ch3Led, &g_ch4Led};
FaderLED<BUTTON1_LED_FTM_BASE, BUTTON1_LED_FTM_CHANNEL> g_button1Led;

struct CalibrationData
{
    uint32_t a;
    uint32_t b;
    uint32_t c;
    uint32_t d;
};

namespace persistent_data {
PersistentDataStore g_store;
PersistentData<kCalibrationDataKey, CalibrationData> g_calibrationData;
PersistentData<kLastSelectedBankKey, uint32_t> g_lastSelectedBank;
PersistentData<kLastVoiceMode, VoiceMode> g_lastVoiceMode;
}

adc16_config_t g_adcConfig;

} // namespace slab

DEFINE_DEBUG_LOG

const char kLedStartupPattern[][5] = {
        "r---",
        "-r--",
        "--r-",
        "---r",
        "---y",
        "--y-",
        "-y--",
        "y---",
        "----",
        "rrrr",
        "----",
        "yyyy",
        "----",
        "rrrr",
        "----",
    };

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

uint64_t ar_get_microseconds()
{
    return Microseconds::get();
}

void flash_leds()
{
    uint32_t i;
    for (i = 0; i < ARRAY_SIZE(kLedStartupPattern); ++i)
    {
        char const * pattern = kLedStartupPattern[i];
        uint32_t which;
        for (which = 0; which < kVoiceCount; ++which)
        {
            switch (pattern[which])
            {
                case 'r':
                    g_channelLeds[which]->set_color(LEDBase::kRed);
                    g_channelLeds[which]->on();
                    break;
                case 'y':
                    g_channelLeds[which]->set_color(LEDBase::kYellow);
                    g_channelLeds[which]->on();
                    break;
                default:
                    g_channelLeds[which]->off();
            }
        }

        g_channelLedManager.flush();
        Ar::Thread::sleep(100);
    }
}

void calibrate_pots()
{
    AnalogIn ch1(CH1_POT_ADC, CH1_POT_CHANNEL);
    AnalogIn ch2(CH2_POT_ADC, CH2_POT_CHANNEL);
    AnalogIn ch3(CH3_POT_ADC, CH3_POT_CHANNEL);
    AnalogIn ch4(CH4_POT_ADC, CH4_POT_CHANNEL);
    ch1.init(g_adcConfig);
    ch2.init(g_adcConfig);
    ch3.init(g_adcConfig);
    ch4.init(g_adcConfig);

    uint32_t reading[4];
    uint32_t minReading[4] = { ~0ul, ~0ul, ~0ul, ~0ul };
    uint32_t maxReading[4] = { 0 };
    uint32_t n;
    uint32_t i;

    for (n = 0; n < 100; ++n)
    {
        reading[0] = ch1.read();
        reading[1] = ch2.read();
        reading[2] = ch3.read();
        reading[3] = ch4.read();

        for (i = 0; i < 4; ++i)
        {
            if (reading[i] < minReading[i])
            {
                minReading[i] = reading[i];
            }
            if (reading[i] > maxReading[i])
            {
                maxReading[i] = reading[i];
            }
        }
    }

    for (i = 0; i < 4; ++i)
    {
        uint32_t noise = maxReading[i] - minReading[i];
        g_pots[i].set_noise(noise);
    }
}

extern "C" void DMA_Error_IRQ_Handler(void)
{
    uint32_t errorStatus = DMA0->ES;
    uint32_t errorChannel = (errorStatus & DMA_ES_ERRCHN_MASK) >> DMA_ES_ERRCHN_SHIFT;
    DEBUG_PRINTF(ERROR_MASK, "DMA error status=0x%08lx ch#%lu", errorStatus, errorChannel);

    // Clear all error flags.
    DMA0->CERR = DMA_CERR_CAEI_MASK;

#if DEBUG
    __BKPT(0);
#endif
}

void init_dma()
{
    // Init eDMA and DMAMUX.
    edma_config_t dmaConfig = {0};
    EDMA_GetDefaultConfig(&dmaConfig);
    dmaConfig.enableHaltOnError = false;
    dmaConfig.enableRoundRobinArbitration = false;
    dmaConfig.enableDebugMode = true;
    EDMA_Init(DMA0, &dmaConfig);
    DMAMUX_Init(DMAMUX0);

    // Set DMA channel priorities. Each DMA channel must have a unique priority. Channels 0 and 1,
    // used for the SAI, are set to the highest priorities and have preemption enabled. Other
    // channels have preemption disabled so only the channels used for SAI can preempt.
    edma_channel_Preemption_config_t priority;
    uint32_t channel;
    for (channel = 0; channel < 16; ++channel)
    {
        priority.channelPriority = 15 - channel;
        priority.enableChannelPreemption = (channel != kAudioPongDmaChannel);
        priority.enablePreemptAbility = (channel <= kAudioPongDmaChannel);
        EDMA_SetChannelPreemptionConfig(DMA0, channel, &priority);

        // Enable DMA error interrupts for channels we use.
        if (channel <= kAllocatedDmaChannelCount)
        {
            EDMA_EnableChannelInterrupts(DMA0, channel, kEDMA_ErrorInterruptEnable);
        }
    }
}

void init_irq_priorities()
{
    // Audio DMA.
    NVIC_SetPriority(DMA0_IRQn, 0);
    NVIC_SetPriority(DMA1_IRQn, 0);

    // SDHC
    NVIC_SetPriority(SDHC_IRQn, 1);

    // ADC command DMA
    NVIC_SetPriority(DMA2_IRQn, 2);
    NVIC_SetPriority(DMA4_IRQn, 2);

    // ADC read DMA
    NVIC_SetPriority(DMA3_IRQn, 2);
    NVIC_SetPriority(DMA5_IRQn, 2);

    // Pin IRQ
    NVIC_SetPriority(PORTA_IRQn, 3);
    NVIC_SetPriority(PORTB_IRQn, 3);
    NVIC_SetPriority(PORTC_IRQn, 3);
    NVIC_SetPriority(PORTD_IRQn, 3);
    NVIC_SetPriority(PORTE_IRQn, 3);

    // SPI used for channel LED update.
    NVIC_SetPriority(SPI0_IRQn, 4);

    // Timer used for button1 LED.
    NVIC_SetPriority(FTM3_IRQn, 5);

    // Error interrupts.
    NVIC_SetPriority(I2S0_Tx_IRQn, 6);
    NVIC_SetPriority(DMA_Error_IRQn, 6);
}

void init_adc_config()
{
    ADC16_GetDefaultConfig(&g_adcConfig);

    // ADCK = 12 MHz OSC
    // 16b = 25 cyc
    // long sample = +16 cyc
    // high speed = +2 cyc
    // = 43 * 32 avg = 1376 cyc = 115 µs
    // 4x channels = 460 µs
    // = 2173 Hz
    g_adcConfig.clockSource = kADC16_ClockSourceAlt2;
    g_adcConfig.enableAsynchronousClock = false;
    g_adcConfig.clockDivider = kADC16_ClockDivider1;
    g_adcConfig.resolution = kADC16_ResolutionSE16Bit;
    g_adcConfig.longSampleMode = kADC16_LongSampleCycle16;
    g_adcConfig.enableHighSpeed = true;
    g_adcConfig.enableLowPower = false;
    g_adcConfig.enableContinuousConversion = false;
}

void init_audio_out()
{
    // Reset DAC.
    GPIO_ClearPinsOutput(PIN_DAC_RESET_GPIO, PIN_DAC_RESET);

    // Configure the audio format.
    AudioOutput::Format format;
    format.bitsPerSample = kBitsPerSample;
    format.sampleRate_Hz = kSampleRate;
    format.oversampleRatio = kOversampleRatio;

    // Init audio output object.
    g_audioOut.init(format);
    g_audioOut.set_source(&g_sampler);

    // Add buffers to the audio output.
    AudioOutput::Buffer buf;
    buf.dataSize = kAudioBufferSize * kAudioChannelCount * sizeof(int16_t);
    uint32_t i;
    for (i = 0; i < kAudioBufferCount; ++i)
    {
        buf.data = (uint8_t *)&g_outBuf[i][0];
        g_audioOut.add_buffer(&buf);
    }

    // Release DAC from reset after sleeping a bit.
    Ar::Thread::sleep(10);
    GPIO_SetPinsOutput(PIN_DAC_RESET_GPIO, PIN_DAC_RESET);
}

void init_thread(void * arg)
{
    DEBUG_PRINTF(INIT_MASK, "\r\nSAMPLBäR Initializing...\r\n");

    g_channelLedManager.init();
    flash_leds();

    // Configure IRQ priorities.
    init_irq_priorities();

    // Init persistent data.
    persistent_data::g_store.init();
    persistent_data::g_calibrationData.init();
    persistent_data::g_lastSelectedBank.init();
    persistent_data::g_lastVoiceMode.init();

    // Must init reader thread before initing voices.
    g_readerThread.init();

    // Init channel-specific objects.
    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        g_gates[i].n = i;
        g_pots[i].n = i;
        g_voice[i].init(i, (int16_t *)&g_sampleBufs[i]);
    }

    init_adc_config();
    calibrate_pots();

    init_dma();
    init_audio_out();

    // Init timer for button1 led.
    ftm_config_t config;
    FTM_GetDefaultConfig(&config);
    config.prescale = kFTM_Prescale_Divide_16;
    FTM_Init(FTM3, &config);
    g_button1Led.init();
    FTM_StartTimer(FTM3, kFTM_SystemClock);

    // Init UI.
    g_ui.set_leds(g_channelLeds, &g_button1Led);
    g_ui.set_pots(g_pots);
    g_ui.init();

    // Init SD card and filesystem.
    g_cardManager.init();

    g_readerThread.start();

    // Start other threads.
    g_ui.start();
    g_audioOut.start();
    g_adcProcessor.init();

    DEBUG_PRINTF(INIT_MASK, "done.\r\n");

    delete g_initThread;
}

#if ENABLE_LOAD_REPORT
void load_thread(void * arg)
{
    while (true)
    {
        uint32_t ms = ar_get_millisecond_count();

        static char buffer[128];
        ar_thread_status_t report[10];

        uint32_t count = ar_thread_get_report(report, 10);

        uint32_t i;
        uint32_t l = 1;
        for (i = 0; i < count; ++i)
        {
            uint32_t percentInt = report[i].m_cpu / 10;
            uint32_t percentFrac = report[i].m_cpu - percentInt * 10;
            assert(l < sizeof(buffer));
            l += snprintf(buffer + (l - 1), sizeof(buffer) - l,
                "%2lu.%1lu%% %-8s%s",
                percentInt, percentFrac, report[i].m_name,
                (i == (count - 1) ? "\n" : ""));
        }
        printf(buffer);
        printf("total = %lu ‰\n", ar_get_system_load());

        Ar::Thread::sleepUntil(ms + 1000);
    }
}
#endif // ENABLE_LOAD_REPORT

int main(void)
{
#if DEBUG
    // Disable write buffer to make bus faults precise.
    SCnSCB->ACTLR = SCnSCB_ACTLR_DISDEFWBUF_Msk;
#endif

    // Enable configurable faults.
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

    init_board();
    Microseconds::init();

    g_initThread = new Ar::Thread("init", init_thread, 0, NULL, 3072, kInitThreadPriority, kArStartThread);
    ar_kernel_run();
    return 0;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
