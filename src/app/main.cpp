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
#include "simple_queue.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "Dialog7212.h"
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
i2c_master_handle_t g_i2c1Handle;
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

    void init();

    void set_mode(Mode newMode);

    uint32_t read();

protected:
    Mode _mode;
};

/*!
 * @brief
 */
class SamplerSynth : public AudioOutput::Source
{
public:
    SamplerSynth() {}
    virtual ~SamplerSynth() {}

    virtual void fill_buffer(uint32_t bufferIndex, AudioOutput::Buffer & buffer) override;

protected:

};

/*!
 * @brief
 */
class SamplerVoice
{
public:
    static const uint32_t kBufferCount = 3;
    static const uint32_t kBufferSize = 2048;

    struct Buffer
    {
        SamplerVoice * voice;
        int16_t * data;
        uint32_t frameCount;
        uint32_t readHead;
    };

    SamplerVoice();
    ~SamplerVoice() {}

    bool is_valid() const { return _wav.is_valid(); }

    void set_file(WaveFile & file);

    void reset() { _data.seek(0); }

    void prime();

    Buffer * get_current_buffer();
    Buffer * dequeue_next_buffer();
    void enqueue_full_buffer(Buffer * buffer);

    WaveFile& get_wave_file() { return _wav; }
    Stream& get_audio_stream() { return _data; }

protected:
    WaveFile _wav;
    WaveFile::AudioDataStream _data;
    int16_t _bufferData[kBufferCount * kBufferSize];
    Buffer _buffer[kBufferCount];
    SimpleQueue<Buffer*, kBufferCount> _fullBuffers;
    Buffer * _currentBuffer;
};

class ReaderThread
{
public:
    ReaderThread() : _thread("reader", this, &ReaderThread::reader_thread, 120, kArSuspendThread), _queue("reader") {}
    ~ReaderThread() {}

    void start() { _thread.resume(); }

    void enqueue(SamplerVoice::Buffer * request) { _queue.send(request, kArNoTimeout); }

    uint32_t get_pending_count() const { return _queue.getCount(); }

protected:
    Ar::ThreadWithStack<2048> _thread;
    Ar::StaticQueue<SamplerVoice::Buffer*, 12> _queue;

    void reader_thread();
};

int16_t g_readBuf[SamplerVoice::kBufferSize * 2];
SamplerSynth g_sampler;
SamplerVoice g_voice[4];
ReaderThread g_readerThread;

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

void cv_thread(void * arg)
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
//             printf("ch2 %s\r\n", (value2 ? "on" : "off"));

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

#define SQUARE_OUT (0)

//! Runs on the audio thread.
void SamplerSynth::fill_buffer(uint32_t bufferIndex, AudioOutput::Buffer & buffer)
{
    int16_t * out = (int16_t *)buffer.data;
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
    SamplerVoice::Buffer * voiceBuffer0 = nullptr;
    if (g_voice[3].is_valid())
    {
        voiceBuffer0 = g_voice[3].get_current_buffer();
    }
    SamplerVoice::Buffer * voiceBuffer1 = nullptr;
    if (g_voice[1].is_valid())
    {
        voiceBuffer0 = g_voice[1].get_current_buffer();
    }

    int i;
    for (i = 0; i < frameCount; ++i)
    {
        int16_t intSample;
        if (voiceBuffer0 && voiceBuffer0->readHead < voiceBuffer0->frameCount)
        {
            intSample = voiceBuffer0->data[voiceBuffer0->readHead++];
        }
        else
        {
            intSample = 0;
        }
        *out++ = intSample;

        if (voiceBuffer1 && voiceBuffer1->readHead < voiceBuffer1->frameCount)
        {
            intSample = voiceBuffer1->data[voiceBuffer1->readHead++];
        }
        else
        {
            intSample = 0;
        }
        *out++ = intSample;
    }

    if (voiceBuffer0 && voiceBuffer0->readHead >= voiceBuffer0->frameCount)
    {
        g_readerThread.enqueue(voiceBuffer0);
        g_voice[3].dequeue_next_buffer();
    }
    if (voiceBuffer1 && voiceBuffer1->readHead >= voiceBuffer1->frameCount)
    {
        g_readerThread.enqueue(voiceBuffer1);
        g_voice[1].dequeue_next_buffer();
    }
#endif // SQUARE_OUT
}

SamplerVoice::SamplerVoice()
:   _wav(),
    _data(),
    _fullBuffers()
{
    int i;
    for (i = 0; i < kBufferCount; ++i)
    {
        _buffer[i].voice = this;
        _buffer[i].data = &_bufferData[i * kBufferSize];
        _buffer[i].frameCount = kBufferSize;
        _buffer[i].readHead = 0;
    }
}

void SamplerVoice::set_file(WaveFile& file)
{
    _wav = file;
    _data = _wav.get_audio_data();
}

void SamplerVoice::prime()
{
    int i;
    for (i = 0; i < kBufferCount; ++i)
    {
        g_readerThread.enqueue(&_buffer[i]);
    }
}

SamplerVoice::Buffer * SamplerVoice::get_current_buffer()
{
    if (!_currentBuffer)
    {
        _currentBuffer = dequeue_next_buffer();
    }
    return _currentBuffer;
}

SamplerVoice::Buffer * SamplerVoice::dequeue_next_buffer()
{
    Buffer * buffer;
    if (_fullBuffers.get(buffer))
    {
//         if (_currentBuffer)
//         {
//             g_readerThread.enqueue(_currentBuffer);
//         }
        _currentBuffer = buffer;
        return buffer;
    }
    else
    {
        return nullptr;
    }
}

void SamplerVoice::enqueue_full_buffer(Buffer * buffer)
{
    _fullBuffers.put(buffer);
}

void ReaderThread::reader_thread()
{
    while (true)
    {
        SamplerVoice::Buffer * request = _queue.receive();
        assert(request);
        assert(request->voice->is_valid());

        Stream& stream = request->voice->get_audio_stream();
        uint32_t frameSize = request->voice->get_wave_file().get_frame_size();
        uint32_t channelCount = request->voice->get_wave_file().get_channels();
        uint32_t bytesToRead = request->frameCount * frameSize;

        // Read mono files directly into the voice buffer, stereo into a temp buffer.
        void * targetBuffer = request->data;
        if (channelCount == 2)
        {
            targetBuffer = g_readBuf;
            assert(bytesToRead <= sizeof(g_readBuf));
        }

        uint32_t bytesRead = stream.read(bytesToRead, targetBuffer);
        uint32_t framesRead = bytesRead / frameSize;

        // For stereo data copy just the left channel into the voice buffer.
        if (channelCount == 2)
        {
            int i;
            int16_t * buf = g_readBuf;
            int16_t * data = request->data;
            for (i = 0; i < framesRead; ++i)
            {
                *data++ = *buf;
                buf += 2;
            }
        }

        if (framesRead < request->frameCount)
        {
            memset((uint8_t *)(request->data + framesRead), 0, (request->frameCount - framesRead) * sizeof(int16_t));
            request->voice->reset();
        }

        request->readHead = 0;
        request->voice->enqueue_full_buffer(request);
    }
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

                    uint32_t frameCount =
                        g_voice[channel].get_audio_stream().get_size() / wav.get_frame_size();

                    printf("%s: %d Hz; %d bits; %d ch; %d bytes/frame; %d frames\r\n",
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
                printf("Failed to parse %s\r\n", info.fname);
            }
        }
    }
}

void button1_handler(PORT_Type * port, uint32_t pin, void * userData)
{
    printf("button1\r\n");
}

void button2_handler(PORT_Type * port, uint32_t pin, void * userData)
{
    printf("button2\r\n");
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
    format.protocol = kSAI_BusI2S;
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
    printf("fs init = %d\r\n", res);

    if (res == 0)
    {
        scan_for_files();
    }
}

void init_thread(void * arg)
{
    init_board();

    printf("\r\nHello...\r\n");

    init_audio_out();
    init_audio_synth();
    init_fs();

    PinIrqManager::get().connect(PIN_BUTTON1_PORT, PIN_BUTTON1_BIT, button1_handler, NULL);
    PinIrqManager::get().connect(PIN_BUTTON2_PORT, PIN_BUTTON2_BIT, button1_handler, NULL);

    g_readerThread.start();

    // Wait until reader thread has filled all buffers.
    while (g_readerThread.get_pending_count())
    {
        Ar::Thread::sleep(20);
    }

    g_audioOut.start();
    g_cvThread.resume();

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
