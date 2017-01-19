/*
 * Copyright (c) 2016 Chris Reed
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
#include "board.h"
#include "analog_in.h"
#include "audio_output.h"
#include "pin_irq_manager.h"
#include "file_system.h"
#include "wav_file.h"
#include "utility.h"
#include "led.h"
#include "microseconds.h"
#include "debug_log.h"
#include "reader_thread.h"
#include "sampler_voice.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "arm_math.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <ctype.h>
#include <utility>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

#define OVER_SAMPLE_RATE (256)
#define BUFFER_SIZE (256)
#define CHANNEL_NUM (2)
#define BUFFER_NUM (3)

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void cv_thread(void * arg);
void read_thread(void * arg);
void init_thread(void * arg);

void flash_leds();

void init_audio_out();
void init_audio_synth();
void init_fs();

void scan_for_files();

void button1_handler(PORT_Type * port, uint32_t pin, void * userData);
void button2_handler(PORT_Type * port, uint32_t pin, void * userData);

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

const float kSampleRate = 48000.0f; // 48kHz

int16_t g_outBuf[BUFFER_NUM][BUFFER_SIZE * CHANNEL_NUM];

AudioOutput g_audioOut;
FileSystem g_fs;

Ar::Thread * g_initThread = NULL;
Ar::ThreadWithStack<2048> g_cvThread("cv", cv_thread, 0, 80, kArSuspendThread);

/*!
 * @brief
 */
class ChannelCVGate : public AnalogIn
{
public:
    enum Mode
    {
        kGate,
        kCV
    };

    ChannelCVGate(uint32_t instance, uint32_t channel);
    ~ChannelCVGate()=default;

    void init();

    void set_mode(Mode newMode);
    void set_inverted(bool isInverted) { _isInverted = isInverted; }

    uint32_t read();

protected:
    Mode _mode;
    bool _isInverted;
    uint32_t _last;
    bool _edge;
    uint32_t _highCount;
};

/*!
 * @brief Audio render source.
 */
class SamplerSynth : public AudioOutput::Source
{
public:
    SamplerSynth() {}
    virtual ~SamplerSynth()=default;

    virtual void fill_buffer(uint32_t bufferIndex, AudioOutput::Buffer & buffer) override;

protected:
};

SamplerSynth g_sampler;
LED<PIN_CH1_LED_GPIO_BASE, PIN_CH1_LED> g_ch1Led;
LED<PIN_CH2_LED_GPIO_BASE, PIN_CH2_LED> g_ch2Led;
LED<PIN_CH3_LED_GPIO_BASE, PIN_CH3_LED> g_ch3Led;
LED<PIN_CH4_LED_GPIO_BASE, PIN_CH4_LED> g_ch4Led;
LEDBase * g_channelLeds[] = { &g_ch1Led, &g_ch2Led, &g_ch3Led, &g_ch4Led};
SamplerVoice g_voice[4];
ReaderThread g_readerThread;
LED<PIN_BUTTON1_LED_GPIO_BASE, PIN_BUTTON1_LED> g_button1Led;
bool g_button1LedState = false;

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
        Ar::Thread::sleep(250);
        g_channelLeds[which]->off();
    }

    for (which = 2; which >= 0; --which)
    {
        g_channelLeds[which]->on();
        Ar::Thread::sleep(250);
        g_channelLeds[which]->off();
    }

    // sleep 250 ms
    Ar::Thread::sleep(250);

    // all on
    for (which = 0; which < 4; ++which)
    {
        g_channelLeds[which]->on();
    }

    // sleep 250 ms
    Ar::Thread::sleep(250);

    // all off
    for (which = 0; which < 4; ++which)
    {
        g_channelLeds[which]->off();
    }

}

ChannelCVGate::ChannelCVGate(uint32_t instance, uint32_t channel)
:   AnalogIn(instance, channel),
    _mode(kGate),
    _isInverted(false),
    _last(0),
    _edge(false),
    _highCount(0)
{
}

void ChannelCVGate::init()
{
    AnalogIn::init();
}

void ChannelCVGate::set_mode(Mode newMode)
{
    _mode = newMode;
}

uint32_t ChannelCVGate::read()
{
    uint32_t result = 0;
    uint32_t value = AnalogIn::read();

    const uint32_t kAdcMax = 65536;

    if (!_isInverted)
    {
        // Invert value to compensate for inverting opamp config;
        value = kAdcMax - value;
    }

    if (_mode == Mode::kGate)
    {
        const uint32_t kThreshold = kAdcMax - (0.3 * (kAdcMax / 2));
        uint32_t state = (value > kThreshold) ? 1 : 0;

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

void cv_thread(void * arg)
{
    ChannelCVGate ch1Gate(CH1_CV_ADC, CH1_CV_CHANNEL);
    ChannelCVGate ch2Gate(CH2_CV_ADC, CH2_CV_CHANNEL);
    ChannelCVGate ch3Gate(CH3_CV_ADC, CH3_CV_CHANNEL);
    ChannelCVGate ch4Gate(CH4_CV_ADC, CH4_CV_CHANNEL);
    ch1Gate.init();
    ch2Gate.init();
    ch2Gate.set_inverted(true);
    ch3Gate.init();
    ch3Gate.set_inverted(true);
    ch4Gate.init();
    ch4Gate.set_inverted(true);

    AnalogIn ch1Pot(CH1_POT_ADC, CH1_POT_CHANNEL);
    AnalogIn ch2Pot(CH2_POT_ADC, CH2_POT_CHANNEL);
    AnalogIn ch3Pot(CH3_POT_ADC, CH3_POT_CHANNEL);
    AnalogIn ch4Pot(CH4_POT_ADC, CH4_POT_CHANNEL);
    ch1Pot.init();
    ch2Pot.init();
    ch3Pot.init();
    ch4Pot.init();

    while (1)
    {
        if (ch1Gate.read())
        {
//             printf("ch1 triggered\r\n");
            g_voice[0].trigger();
        }

        if (ch2Gate.read())
        {
//             printf("ch2 triggered\r\n");
            g_voice[1].trigger();
        }

        if (ch3Gate.read())
        {
//             printf("ch3 triggered\r\n");
            g_voice[2].trigger();
        }

        if (ch4Gate.read())
        {
//             printf("ch4 triggered\r\n");
            g_voice[3].trigger();
        }
    }
}

#define SQUARE_OUT (0)

//! Runs on the audio thread.
void SamplerSynth::fill_buffer(uint32_t bufferIndex, AudioOutput::Buffer & buffer)
{
    int16_t * data = (int16_t *)buffer.data;;
    int frameCount = buffer.dataSize / sizeof(int16_t) / CHANNEL_NUM;

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
    g_voice[0].fill(data, frameCount);
    g_voice[1].fill(data + 1, frameCount);
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
                int channel = info.fname[0] - '1';
                if (channel >= 0 && channel <= 3)
                {
                    g_voice[channel].set_file(wav);

                    uint32_t frameCount = g_voice[channel].get_audio_stream().get_frames();

                    DEBUG_PRINTF(INIT_MASK, "%s: %d Hz; %d bits; %d ch; %d bytes/frame; %d frames\r\n",
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

void button1_handler(PORT_Type * port, uint32_t pin, void * userData)
{
    DEBUG_PRINTF(BUTTON_MASK, "button1\r\n");
    if (g_button1LedState)
    {
        g_button1Led.off();
    }
    else
    {
        g_button1Led.on();
    }
    g_button1LedState = !g_button1LedState;
}

void button2_handler(PORT_Type * port, uint32_t pin, void * userData)
{
    DEBUG_PRINTF(BUTTON_MASK, "button2\r\n");
}

void init_audio_out()
{
    // Reset DAC.
    GPIO_ClearPinsOutput(PIN_DAC_RESET_GPIO, PIN_DAC_RESET);

    // Configure the audio format.
    sai_transfer_format_t format;
    format.bitWidth = kSAI_WordWidth16bits;
    format.channel = 0U;
    format.sampleRate_Hz = static_cast<sai_sample_rate_t>(kSampleRate);
    format.masterClockHz = OVER_SAMPLE_RATE * format.sampleRate_Hz;
    format.protocol = kSAI_BusLeftJustified;
    format.stereo = kSAI_Stereo;
    format.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;

    // Init audio output object.
    g_audioOut.init(&format);
    g_audioOut.set_source(&g_sampler);

    // Add buffers to the audio output.
    AudioOutput::Buffer buf;
    buf.dataSize = BUFFER_SIZE * CHANNEL_NUM * sizeof(int16_t);
    int i;
    for (i = 0; i < BUFFER_NUM; ++i)
    {
        buf.data = (uint8_t *)&g_outBuf[i][0];
        g_audioOut.add_buffer(&buf);
    }

    // Release DAC from reset after sleeping a bit.
    Ar::Thread::sleep(10);
    GPIO_SetPinsOutput(PIN_DAC_RESET_GPIO, PIN_DAC_RESET);
}

void init_audio_synth()
{
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

    g_voice[0].set_number(0);
    g_voice[0].set_led(&g_ch1Led);
    g_voice[1].set_number(1);
    g_voice[1].set_led(&g_ch2Led);
    g_voice[2].set_number(2);
    g_voice[2].set_led(&g_ch3Led);
    g_voice[3].set_number(3);
    g_voice[3].set_led(&g_ch4Led);

    init_audio_out();
    init_audio_synth();
    init_fs();

    PinIrqManager::get().connect(PIN_BUTTON1_PORT, PIN_BUTTON1_BIT, button1_handler, NULL);
//     PinIrqManager::get().connect(PIN_BUTTON2_PORT, PIN_BUTTON2_BIT, button1_handler, NULL);

    g_readerThread.start();

    // Wait until reader thread has filled all buffers.
    while (g_readerThread.get_pending_count())
    {
        Ar::Thread::sleep(20);
    }

    g_audioOut.start();
    g_cvThread.resume();

    DEBUG_PRINTF(INIT_MASK, "done.\r\n");

//     delete g_initThread;
}

int main(void)
{
    Microseconds::init();
    init_board();

    g_initThread = new Ar::Thread("init", init_thread, 0, NULL, 2500, 60, kArStartThread);
    ar_kernel_run();
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
