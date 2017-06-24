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
#include "main.h"
#include "board.h"
#include "pin_irq_manager.h"
#include "file_system.h"
#include "wav_file.h"
#include "utility.h"
#include "led.h"
#include "microseconds.h"
#include "debug_log.h"
#include "reader_thread.h"
#include "adc_sequencer.h"
#include "channel_cv_gate.h"
#include "sampler_synth.h"
#include "ui.h"
#include "fsl_sd_disk.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <ctype.h>
#include <utility>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

#define ENABLE_LOAD_REPORT (1)

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void cv_thread(void * arg);
void init_thread(void * arg);

#if ENABLE_LOAD_REPORT
void load_thread(void * arg);
#endif

void flash_leds();

void init_dma();
void init_audio_out();
void init_fs();

void scan_for_files();


//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

namespace slab {

int16_t g_outBuf[kAudioBufferCount][kAudioBufferSize * kAudioChannelCount] __attribute__ ((section(".buffers"),aligned(4)));
int16_t g_sampleBufs[kVoiceCount][SampleBufferManager::kBufferCount * SampleBufferManager::kBufferSize] __attribute__ ((section(".buffers"),aligned(4)));

Ar::Thread * g_initThread = NULL;
Ar::ThreadWithStack<2048> g_cvThread("cv", cv_thread, 0, kCVThreadPriority, kArSuspendThread);

#if ENABLE_LOAD_REPORT
Ar::ThreadWithStack<2048> g_loadReportThread("load", load_thread, 0, kUIThreadPriority-1, kArStartThread);
#endif

UI g_ui;
AudioOutput g_audioOut;
fs::FileSystem g_fs;
SamplerSynth g_sampler;
ReaderThread g_readerThread;
SamplerVoice g_voice[kVoiceCount];
ChannelCVGate g_gates[kVoiceCount];
Pot g_pots[kVoiceCount];
AdcSequencer g_adc0Sequencer(ADC0, 2);
AdcSequencer g_adc1Sequencer(ADC1, 4);
LED<PIN_CH1_LED_GPIO_BASE, PIN_CH1_LED> g_ch1Led;
LED<PIN_CH2_LED_GPIO_BASE, PIN_CH2_LED> g_ch2Led;
LED<PIN_CH3_LED_GPIO_BASE, PIN_CH3_LED> g_ch3Led;
LED<PIN_CH4_LED_GPIO_BASE, PIN_CH4_LED> g_ch4Led;
LEDBase * g_channelLeds[] = { &g_ch1Led, &g_ch2Led, &g_ch3Led, &g_ch4Led};
LED<PIN_BUTTON1_LED_GPIO_BASE, PIN_BUTTON1_LED> g_button1Led;

} // namespace slab

DEFINE_DEBUG_LOG

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

uint64_t ar_get_microseconds()
{
    return Microseconds::get();
}

void flash_leds()
{
    int which;
    for (which = 0; which < 4; ++which)
    {
        g_channelLeds[which]->on();
        Ar::Thread::sleep(100);
        g_channelLeds[which]->off();
    }

    for (which = 2; which >= 0; --which)
    {
        g_channelLeds[which]->on();
        Ar::Thread::sleep(100);
        g_channelLeds[which]->off();
    }

    // sleep 100 ms
    Ar::Thread::sleep(100);

    // all on
    for (which = 0; which < 4; ++which)
    {
        g_channelLeds[which]->on();
    }

    // sleep 100 ms
    Ar::Thread::sleep(100);

    // all off
    for (which = 0; which < 4; ++which)
    {
        g_channelLeds[which]->off();
    }

}

void cv_thread(void * arg)
{
    g_gates[0].n = 0;
    g_pots[0].n = 0;
    g_gates[1].n = 1;
    g_gates[1].set_inverted(true);
    g_pots[1].n = 1;
    g_gates[2].n = 2;
    g_gates[2].set_inverted(true);
    g_pots[2].n = 2;
    g_gates[3].n = 3;
    g_gates[3].set_inverted(true);
    g_pots[3].n = 3;

    volatile uint32_t results0[4];
    Ar::Semaphore waitSem0(nullptr, 0);
    g_adc0Sequencer.set_channels(CH2_CV_CHANNEL_MASK | CH3_CV_CHANNEL_MASK | CH1_POT_CHANNEL_MASK | CH2_POT_CHANNEL_MASK);
    g_adc0Sequencer.set_result_buffer((uint32_t *)&results0[0]);
    g_adc0Sequencer.set_semaphore(&waitSem0);
    g_adc0Sequencer.init();

    volatile uint32_t results1[4];
    Ar::Semaphore waitSem1(nullptr, 0);
    g_adc1Sequencer.set_channels(CH1_CV_CHANNEL_MASK | CH4_CV_CHANNEL_MASK | CH3_POT_CHANNEL_MASK |  CH4_POT_CHANNEL_MASK);
    g_adc1Sequencer.set_result_buffer((uint32_t *)&results1[0]);
    g_adc1Sequencer.set_semaphore(&waitSem1);
    g_adc1Sequencer.init();

    g_adc0Sequencer.start();
    g_adc1Sequencer.start();
    while (true)
    {
        waitSem0.get();
        waitSem1.get();

        if (g_gates[0].process(results1[2]))
        {
            DEBUG_PRINTF(TRIG_MASK, "ch1 triggered\r\n");
            g_voice[0].trigger();
        }
        if (g_gates[1].process(results0[3]))
        {
            DEBUG_PRINTF(TRIG_MASK, "ch2 triggered\r\n");
            g_voice[1].trigger();
        }
        if (g_gates[2].process(results0[0]))
        {
            DEBUG_PRINTF(TRIG_MASK, "ch3 triggered\r\n");
            g_voice[2].trigger();
        }
        if (g_gates[3].process(results1[3]))
        {
            DEBUG_PRINTF(TRIG_MASK, "ch4 triggered\r\n");
            g_voice[3].trigger();
        }

        g_pots[0].process(results0[1]);
        g_pots[1].process(results0[2]);
        g_pots[2].process(results1[0]);
        g_pots[3].process(results1[1]);
    }
}

void scan_for_files()
{
    fs::DirectoryIterator dir = g_fs.open_dir("/");
    FILINFO info;

    while (dir.next(&info))
    {
        // Skip directories and hidden or system files.
        if (info.fattrib & (AM_DIR | AM_HID | AM_SYS))
        {
            continue;
        }

        // Look for '[0-9].wav' files.
        if (isdigit(info.fname[0]) && info.fname[1] == '.'
            && toupper(info.fname[2]) == 'W'
            && toupper(info.fname[3]) == 'A'
            && toupper(info.fname[4]) == 'V'
            && info.fsize > 0)
        {
            WaveFile wav(info.fname);

            bool inited = (wav.parse() == fs::kSuccess);

            if (inited && wav.get_channels() <= 2)
            {
                uint32_t channel = info.fname[0] - '1';
                if (channel >= 0 && channel < kVoiceCount)
                {
                    g_voice[channel].set_file(wav);

                    uint32_t frameCount = g_voice[channel].get_audio_stream().get_frames();

                    DEBUG_PRINTF(INIT_MASK, "%s: %lu Hz; %lu bits; %lu ch; %lu bytes/frame; %lu frames\r\n",
                        info.fname,
                        wav.get_sample_rate(),
                        wav.get_sample_size(),
                        wav.get_channels(),
                        wav.get_frame_size(),
                        frameCount);

                    g_voice[channel].prime();
                }
            }
            else
            {
                DEBUG_PRINTF(ERROR_MASK, "Failed to parse %s\r\n", info.fname);
            }
        }
    }
}

void init_dma()
{
    // Init eDMA and DMAMUX.
    edma_config_t dmaConfig = {0};
    EDMA_GetDefaultConfig(&dmaConfig);
    dmaConfig.enableRoundRobinArbitration = false;
    dmaConfig.enableDebugMode = true;
    EDMA_Init(DMA0, &dmaConfig);
    DMAMUX_Init(DMAMUX0);

    // Set DMA channel priorities. Each DMA channel must have a unique priority. Channels 0 and 1,
    // used for the SAI, are set to the highest priorities and have preemption enabled. Other
    // channels have preemption disabled so only the channels used for SAI can preempt.
    edma_channel_Preemption_config_t priority;
    int channel;
    for (channel = 0; channel < 16; ++channel)
    {
        priority.channelPriority = 15 - channel;
        priority.enableChannelPreemption = (channel != 1);
        priority.enablePreemptAbility = (channel < 2);
        EDMA_SetChannelPreemptionConfig(DMA0, channel, &priority);
    }
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

void init_fs()
{
    fs::error_t res = g_fs.init();

    if (res == fs::kSuccess)
    {
        scan_for_files();
    }
    else
    {
        DEBUG_PRINTF(ERROR_MASK, "fs init failed: %lu\r\n", res);
    }
}

void init_thread(void * arg)
{
    DEBUG_PRINTF(INIT_MASK, "\r\nSAMPLBäR Initializing...\r\n");

    flash_leds();

    g_voice[0].init(0, (int16_t *)&g_sampleBufs[0]);
    g_voice[1].init(1, (int16_t *)&g_sampleBufs[1]);
    g_voice[2].init(2, (int16_t *)&g_sampleBufs[2]);
    g_voice[3].init(3, (int16_t *)&g_sampleBufs[3]);

    g_ui.set_leds(g_channelLeds, &g_button1Led);
    g_ui.set_pots(g_pots);
    g_ui.init();

    init_dma();
    init_audio_out();
    g_readerThread.init();
    sd_init();
    init_fs();


    g_readerThread.start();

    // Wait until reader thread has filled all buffers.
    while (g_readerThread.get_pending_count())
    {
        Ar::Thread::sleep(20);
    }

    g_ui.start();
    g_audioOut.start();
    g_cvThread.resume();

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
