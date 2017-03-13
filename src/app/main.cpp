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
#include "audio_defs.h"
#include "audio_output.h"
#include "pin_irq_manager.h"
#include "file_system.h"
#include "wav_file.h"
#include "utility.h"
#include "ring_buffer.h"
#include "led.h"
#include "microseconds.h"
#include "debug_log.h"
#include "reader_thread.h"
#include "sampler_voice.h"
#include "adc_sequencer.h"
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

const uint32_t kAdcMax = 65536;
const uint32_t kTriggerThreshold = kAdcMax - (0.3 * (kAdcMax / 2));

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void cv_thread(void * arg);
void read_thread(void * arg);
void init_thread(void * arg);

void flash_leds();

void init_dma();
void init_audio_out();
void init_fs();

void scan_for_files();


//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

namespace slab {

int16_t g_outBuf[kAudioBufferCount][kAudioBufferSize * kAudioChannelCount];

AudioOutput g_audioOut;
FileSystem g_fs;

Ar::Thread * g_initThread = NULL;
Ar::ThreadWithStack<2048> g_cvThread("cv", cv_thread, 0, kCVThreadPriority, kArSuspendThread);


/*!
 * @brief
 */
class ChannelCVGate
{
public:
    enum Mode
    {
        kGate,
        kCV
    };

    ChannelCVGate();
    ~ChannelCVGate()=default;

    void init();

    void set_mode(Mode newMode);
    void set_inverted(bool isInverted) { _isInverted = isInverted; }

    uint32_t process(uint32_t value);

    uint32_t n;

protected:
    Mode _mode;
    bool _isInverted;
    uint32_t _last;
    bool _edge;
    uint32_t _highCount;
    RingBuffer<uint16_t, 128> _history;
};


/*!
 * @brief
 */
class Pot
{
public:
    Pot();
    ~Pot()=default;

    uint32_t process(uint32_t value);

    uint32_t n;

protected:
    uint32_t _last;
    RunningAverage<32> _avg;
    RingBuffer<uint16_t, 128> _history;
};

/*!
 * @brief Audio render source.
 */
class SamplerSynth : public AudioOutput::Source
{
public:
    SamplerSynth() {}
    virtual ~SamplerSynth()=default;

    virtual void render(uint32_t firstChannel, AudioOutput::Buffer & buffer) override;

protected:
};

ChannelCVGate g_gates[4];
Pot g_pots[4];
AdcSequencer g_adc0Sequencer(ADC0, 2);
AdcSequencer g_adc1Sequencer(ADC1, 4);
SamplerSynth g_sampler;
LED<PIN_CH1_LED_GPIO_BASE, PIN_CH1_LED> g_ch1Led;
LED<PIN_CH2_LED_GPIO_BASE, PIN_CH2_LED> g_ch2Led;
LED<PIN_CH3_LED_GPIO_BASE, PIN_CH3_LED> g_ch3Led;
LED<PIN_CH4_LED_GPIO_BASE, PIN_CH4_LED> g_ch4Led;
LEDBase * g_channelLeds[] = { &g_ch1Led, &g_ch2Led, &g_ch3Led, &g_ch4Led};
SamplerVoice g_voice[4];
ReaderThread g_readerThread;
LED<PIN_BUTTON1_LED_GPIO_BASE, PIN_BUTTON1_LED> g_button1Led;
UI g_ui;

} // namespace slab

DEFINE_DEBUG_LOG

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

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

ChannelCVGate::ChannelCVGate()
:   _mode(kGate),
    _isInverted(false),
    _last(0),
    _edge(false),
    _highCount(0)
{
}

void ChannelCVGate::init()
{
}

void ChannelCVGate::set_mode(Mode newMode)
{
    _mode = newMode;
}

uint32_t ChannelCVGate::process(uint32_t value)
{
    _history.put(value);

    uint32_t result = 0;
    value <<= 4;

    if (!_isInverted)
    {
        // Invert value to compensate for inverting opamp config;
        value = kAdcMax - value;
    }

    if (_mode == Mode::kGate)
    {
        uint32_t state = (value > kTriggerThreshold) ? 1 : 0;

        if (state == 0)
        {
            _edge = false;
            _highCount = 0;
        }

        if (state == 1)
        {
            if (_last == 0)
            {
                _edge = true;
                _highCount = 0;
            }

            ++_highCount;

            if (_highCount == 3)
            {
                result = 1;
            }
        }

        _last = state;
    }
    else
    {
    }

    return result;
}

Pot::Pot()
:   _last(0)
{
}

uint32_t Pot::process(uint32_t value)
{
    _history.put(value);
    value <<= 4;

    // Set gain for this channel.
    if (value <= kAdcMax)
    {
        value = _avg.update(value);
        float gain = float(value) / float(kAdcMax);
        if (value != _last)
        {
            _last = value;
            g_voice[n].set_gain(gain);
        }
    }

    return 0;
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

#define SQUARE_OUT (0)

//! Runs on the audio thread.
void SamplerSynth::render(uint32_t firstChannel, AudioOutput::Buffer & buffer)
{
    int16_t * data = (int16_t *)buffer.data;;
    int frameCount = buffer.dataSize / sizeof(int16_t) / kAudioChannelCount;

#if SQUARE_OUT
    // Output a naïve full-scale 440 Hz square wave for testing without the SD card.
    static int phase = 0;
    int w = kSampleRate / 440;
    int i;
    for (i = 0; i < frameCount; ++i)
    {
        int16_t intSample = (phase > w/2) ? 32767 : -32768;
        *out++ = intSample;
        *out++ = intSample;
        if (++phase > w)
        {
            phase = 0;
        }
    }
#else // SQUARE_OUT
    g_voice[firstChannel].render(data, frameCount);
    g_voice[firstChannel + 1].render(data + 1, frameCount);
#endif // SQUARE_OUT
}

void scan_for_files()
{
    DirectoryIterator dir = g_fs.open_dir("/");
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

            bool inited = wav.parse();

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
    int res = g_fs.init();

    if (res == 0)
    {
        scan_for_files();
    }
    else
    {
        DEBUG_PRINTF(ERROR_MASK, "fs init failed: %d\r\n", res);
    }
}

void init_thread(void * arg)
{
    DEBUG_PRINTF(INIT_MASK, "\r\nSAMPLBäR Initializing...\r\n");

    flash_leds();

    g_voice[0].init(0);
    g_voice[1].init(1);
    g_voice[2].init(2);
    g_voice[3].init(3);

    g_ui.set_leds(g_channelLeds, &g_button1Led);
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
