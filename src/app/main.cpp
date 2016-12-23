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
#include "audio_output_converter.h"
#include "audio_filter.h"
#include "audio_ramp.h"
#include "asr_envelope.h"
#include "rbj_filter.h"
#include "pin_irq_manager.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "Dialog7212.h"
#include "arm_math.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

#define OVER_SAMPLE_RATE (256) //(384U)
#define BUFFER_SIZE (256)
#define CHANNEL_NUM (2)
#define BUFFER_NUM (2)

template <typename T>
inline T abs(T a)
{
    return (a > 0) ? a : -a;
}

template <typename T>
inline T max3(T a, T b, T c)
{
    T tmp = (a > b) ? a : b;
    return (tmp > c) ? tmp : c;
}

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void pots_thread(void * arg);
void audio_init_thread(void * arg);
void init_thread(void * arg);

void init_i2c0();
void init_i2c1();
void init_audio_out();
void init_audio_synth();
void init_fs();

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

float g_audioBuf[BUFFER_SIZE];
float g_mixBuf[BUFFER_SIZE];
int16_t g_outBuf[BUFFER_NUM][BUFFER_SIZE * CHANNEL_NUM];

const float kSampleRate = 48000.0f; // 48kHz

AudioOutput g_audioOut;
AudioOutputConverter g_audioOutConverter;
// RBJFilter g_filter;
// DelayLine g_delay;
i2c_master_handle_t g_i2c1Handle;

Ar::Thread * g_audioInitThread = NULL;
Ar::Thread * g_initThread = NULL;

Ar::ThreadWithStack<2048> g_potsThread("pots", pots_thread, 0, 120, kArSuspendThread);

class ChannelCVGate : public AnalogIn
{
public:
    enum Mode
    {
        kGate,
        kCV
    };

    ChannelCVGate(uint32_t instance, uint32_t channel);

    void init();

    void set_mode(Mode newMode);

    uint32_t read();

protected:
    Mode _mode;
};

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

ChannelCVGate::ChannelCVGate(uint32_t instance, uint32_t channel)
:   AnalogIn(instance, channel),
    _mode(kGate)
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
    uint32_t value = AnalogIn::read();

    const uint32_t kAdcMax = 65536;

    // Invert value to compensate for inverting opamp config;
    value = kAdcMax - value;

    if (_mode == Mode::kGate)
    {
        const uint32_t kThreshold = kAdcMax - (0.3 * (kAdcMax / 2));
        return (value > kThreshold) ? 1 : 0;
    }
    else
    {
    }

    return value;
}

void pots_thread(void * arg)
{
    ChannelCVGate ch1(CH1_CV_ADC, CH1_CV_CHANNEL);
    ChannelCVGate ch2(CH2_CV_ADC, CH2_CV_CHANNEL);
    ch1.init();
//     ch1.set_mode(ChannelCVGate::Mode::kCV);
    ch2.init();

    uint32_t lastValue1 = ~0;
    uint32_t lastValue2 = ~0;
    while (1)
    {
        uint32_t value2 = ch2.read();
//         uint32_t value1 = ch1.read();

//         if (value1 != lastValue1)
//         {
//             printf("ch1 %s\r\n", (value1 ? "on" : "off"));
//
//             lastValue1 = value1;
//         }

        if (value2 != lastValue2)
        {
            printf("ch2 %s\r\n", (value2 ? "on" : "off"));

            lastValue2 = value2;
        }

//         if (value1 != lastValue1)
//         {
//             printf("ch1 %d\r\n", value1);
//
//             lastValue1 = value1;
//         }

//         Ar::Thread::sleep(20);
    }
}

// void init_i2c0()
// {
//     uint32_t i2cSourceClock = CLOCK_GetFreq(kCLOCK_BusClk);
//     i2c_master_config_t i2cConfig = {0};
//     I2C_MasterGetDefaultConfig(&i2cConfig);
//     I2C_MasterInit(I2C0, &i2cConfig, i2cSourceClock);
//     I2C_MasterTransferCreateHandle(I2C0, &g_i2cHandle, NULL, NULL);
// }

void init_i2c1()
{
    uint32_t i2cSourceClock = CLOCK_GetFreq(kCLOCK_BusClk);
    i2c_master_config_t i2cConfig = {0};
    I2C_MasterGetDefaultConfig(&i2cConfig);
    I2C_MasterInit(I2C1, &i2cConfig, i2cSourceClock);
    I2C_MasterTransferCreateHandle(I2C1, &g_i2c1Handle, NULL, NULL);
}

void audio_init_thread(void * arg)
{
    // Configure the audio format
    sai_transfer_format_t format;
    format.bitWidth = kSAI_WordWidth16bits;
    format.channel = 0U;
    format.sampleRate_Hz = kSAI_SampleRate48KHz;
    format.masterClockHz = OVER_SAMPLE_RATE * format.sampleRate_Hz;
    format.protocol = kSAI_BusI2S;
    format.stereo = kSAI_Stereo;
    format.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;

    g_audioOut.init(&format);

    // Configure audio codec
    DA7212_InitCodec(BOARD_CODEC_I2C_BASE);
    DA7212_ChangeFrequency(DA7212_SYS_FS_48K);
    DA7212_ChangeInput(DA7212_Input_MIC1_Dig);

    init_audio_synth();

//     delete g_audioInitThread;
}

void init_audio_out()
{
    g_audioInitThread = new Ar::Thread("auinit", audio_init_thread, 0, NULL, 1500, 200, kArStartThread);
}

void init_audio_synth()
{
    AudioOutput::Buffer buf;
    buf.dataSize = BUFFER_SIZE * CHANNEL_NUM * sizeof(int16_t);
    buf.data = (uint8_t *)&g_outBuf[0][0];
    g_audioOut.add_buffer(&buf);
    buf.data = (uint8_t *)&g_outBuf[1][0];
    g_audioOut.add_buffer(&buf);

    g_audioOut.set_source(&g_audioOutConverter);
    AudioBuffer audioBuf(&g_audioBuf[0], BUFFER_SIZE);
    g_audioOutConverter.set_buffer(audioBuf);
//     g_audioOutConverter.set_source(&g_mixer);

//     g_filter.set_sample_rate(kSampleRate);
//     g_filter.set_frequency(120.0f);
//     g_filter.set_q(0.4f);
//     g_filter.recompute_coefficients();
//     g_filter.set_input(&g_bassGen);
//
//     AudioBuffer mixBuf(&g_mixBuf[0], BUFFER_SIZE);
//     g_mixer.set_buffer(mixBuf);
//     g_mixer.set_input_count(2);
//     g_mixer.set_input(0, &g_delay, 0.5f);
//     g_mixer.set_input(1, &g_bassGen, 0.34f);
}

void init_fs()
{
}

void init_thread(void * arg)
{
    init_board();

    printf("Hello...\r\n");

//     init_i2c0();
    init_i2c1();
    init_audio_out();
//     init_fs();

//     PinIrqManager::get().connect(PIN_BTN1_PORT, PIN_BTN1_BIT, button_handler, NULL);
//     PinIrqManager::get().connect(PIN_BTN2_PORT, PIN_BTN2_BIT, button_handler, NULL);
//     PinIrqManager::get().connect(PIN_WAKEUP_PORT, PIN_WAKEUP_BIT, button_handler, NULL);
//
//     PinIrqManager::get().connect(PIN_ENCA_PORT, PIN_ENCA_BIT, rotary_handler, NULL);
//     PinIrqManager::get().connect(PIN_ENCB_PORT, PIN_ENCB_BIT, rotary_handler, NULL);

//     g_audioOut.start();
    g_potsThread.resume();

//     delete g_initThread;
}

int main(void)
{
    g_initThread = new Ar::Thread("init", init_thread, 0, NULL, 2500, 60, kArStartThread);

    ar_kernel_run();
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
