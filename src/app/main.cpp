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
#include "led.h"
#include "microseconds.h"
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

#define ERROR_MASK (1<<0)
#define INIT_MASK (1<<1)
#define TRIG_MASK (1<<2)
#define RETRIG_MASK (1<<3)
#define QUEUE_MASK (1<<4)
#define RETIRE_MASK (1<<5)
#define CURBUF_MASK (1<<6)
#define FILL_MASK (1<<7)
#define TIME_MASK (1<<8)
#define BUTTON_MASK (1<<9)

#define ENABLE_DEBUG_PRINTF (1)
#define DEBUG_PRINTF_MASK (ERROR_MASK|INIT_MASK|BUTTON_MASK|TRIG_MASK|RETRIG_MASK|FILL_MASK|TIME_MASK|QUEUE_MASK)

template <uint32_t N>
class SimpleString
{
public:
    SimpleString() {}
    SimpleString(const char * s)
    {
        strncpy(_string, s, N);
    }
    SimpleString(const SimpleString<N>& other)=default;
    SimpleString& operator =(const SimpleString<N>& other)=default;
    ~SimpleString()=default;

    const char* get() const { return _string; }

protected:
    char _string[N];
};

#if ENABLE_DEBUG_PRINTF
#define DEBUG_LOG_SIZE (48)
char g_debugLogBuffer[DEBUG_LOG_SIZE];
RingBuffer<SimpleString<DEBUG_LOG_SIZE>, 50> g_debugLog;
#define DEBUG_PRINTF(f, m, ...) \
    do { if ((f) & DEBUG_PRINTF_MASK) { \
        snprintf(g_debugLogBuffer, sizeof(g_debugLogBuffer), "[%d] " m, Microseconds::get(), ##__VA_ARGS__); \
        SimpleString<DEBUG_LOG_SIZE> _ds(g_debugLogBuffer); \
        g_debugLog.put(_ds); \
    } } while (0)
#else
#define DEBUG_PRINTF(f, m, ...)
#endif

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
template <typename T, uint32_t N>
class ProtectedQueue : public SimpleQueue<T, N>
{
public:
    ProtectedQueue() : SimpleQueue<T, N>(), _lock(nullptr) {}
    ~ProtectedQueue()=default;

    void clear()
    {
        Ar::Mutex::Guard guard(_lock);
        SimpleQueue<T, N>::clear();
    }

    bool put(const T& value)
    {
        Ar::Mutex::Guard guard(_lock);
        return SimpleQueue<T, N>::put(value);
    }

    bool get(T& value)
    {
        Ar::Mutex::Guard guard(_lock);
        return SimpleQueue<T, N>::get(value);
    }

protected:
    Ar::Mutex _lock;
};

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

/*!
 * @brief Manages playback of a single sample file.
 */
class SamplerVoice
{
public:
    static const uint32_t kBufferCount = 4; //!< Number of buffers available to cycle through sample data. The first one will always be used to hold the first #kBufferSize frames of the sample.
    static const uint32_t kBufferSize = 1024; //!< Number of frames per buffer.
    static_assert(kBufferSize % BUFFER_SIZE == 0, "sai buffers must fit evenly in voice buffers");

    enum BufferState : uint32_t
    {
        kBufferUnused,
        kBufferStartFile,
        kBufferPlaying,
        kBufferReady,
        kBufferFree,
        kBufferReading,
    };

    struct Buffer
    {
        uint32_t number;
        BufferState state;
        int16_t * data;
        uint32_t startFrame;
        uint32_t frameCount;
        uint32_t readHead;
        bool reread;
    };

    SamplerVoice();
    ~SamplerVoice()=default;

    void set_number(uint32_t n) { _number = n; }
    uint32_t get_number() const { return _number; }

    void set_led(LEDBase * led) { _led = led; }
    void set_file(WaveFile & file);

    bool is_valid() const { return _wav.is_valid(); }

    void prime();

    void trigger();
    bool is_playing() { return _isPlaying; }

    void fill(int16_t * data, uint32_t frameCount);

    Buffer * get_current_buffer();
    Buffer * get_empty_buffer();
    void enqueue_full_buffer(Buffer * buffer);

    WaveFile& get_wave_file() { return _wav; }
    WaveFile::AudioDataStream& get_audio_stream() { return _data; }

protected:
    friend class ReaderThread;

    uint32_t _number;
    LEDBase * _led;
    WaveFile _wav;
    WaveFile::AudioDataStream _data;
    int16_t _bufferData[kBufferCount * kBufferSize];
    Buffer _buffer[kBufferCount];
    ProtectedQueue<Buffer*, kBufferCount> _fullBuffers;
    ProtectedQueue<Buffer*, kBufferCount> _emptyBuffers;
    Ar::Mutex _primeMutex;
    Buffer * _currentBuffer;
    uint32_t _activeBufferCount;
    uint32_t _totalSamples;
    uint32_t _samplesPlayed;
    uint32_t _samplesRead;
    uint32_t _samplesQueued;
    bool _isPlaying;
    bool _turnOnLedNextBuffer;

    void queue_buffer_for_read(Buffer * buffer);
    Buffer * dequeue_next_buffer();
    void retire_buffer(Buffer * buffer);

};

/*!
 * @brief Thread to fill channel audio buffers with sample file data.
 *
 * The reader thread maintains a queue of voices that need a buffer filled. A given voice
 * may be put in the queue multiple times.
 */
class ReaderThread
{
public:
    ReaderThread() : _thread("reader", this, &ReaderThread::reader_thread, 120, kArSuspendThread), _queue("reader") {}
    ~ReaderThread()=default;

    void start() { _thread.resume(); }

    void enqueue(SamplerVoice * request) { _queue.send(request, kArNoTimeout); }

    uint32_t get_pending_count() const { return _queue.getCount(); }

protected:
    Ar::ThreadWithStack<2048> _thread;
    Ar::StaticQueue<SamplerVoice*, 12> _queue;

    void reader_thread();
};

int16_t g_readBuf[SamplerVoice::kBufferSize * 2];
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

void SamplerVoice::fill(int16_t * data, uint32_t frameCount)
{
    Buffer * voiceBuffer = nullptr;
    if (is_valid() && is_playing())
    {
        voiceBuffer = get_current_buffer();
    }

    int i;
    int16_t * out = data;
    int16_t intSample;
    uint32_t readHead;
    uint32_t bufferFrameCount;

    if (voiceBuffer)
    {
        readHead = voiceBuffer->readHead;
        bufferFrameCount = voiceBuffer->frameCount;
        for (i = 0; i < frameCount; ++i)
        {
            if (readHead < bufferFrameCount)
            {
                intSample = voiceBuffer->data[readHead++];
            }
            else
            {
                intSample = 0;
            }
            *out = intSample;
            out += 2;
        }
        voiceBuffer->readHead = readHead;

        // Did we finish this buffer?
        if (readHead >= bufferFrameCount)
        {
            retire_buffer(voiceBuffer);
        }
    }
    else
    {
        for (i = 0; i < frameCount; ++i)
        {
            *out = 0;
            out += 2;
        }
    }
}

SamplerVoice::SamplerVoice()
:   _led(nullptr),
    _wav(),
    _data(),
    _fullBuffers(),
    _emptyBuffers(),
    _primeMutex("prime"),
    _currentBuffer(nullptr),
    _totalSamples(0),
    _samplesPlayed(0),
    _samplesRead(0),
    _samplesQueued(0),
    _isPlaying(false),
    _turnOnLedNextBuffer(false)
{
    int i;
    for (i = 0; i < kBufferCount; ++i)
    {
        _buffer[i].number = i;
        _buffer[i].state = kBufferUnused;
        _buffer[i].data = &_bufferData[i * kBufferSize];
        _buffer[i].startFrame = 0;
        _buffer[i].frameCount = kBufferSize;
        _buffer[i].readHead = 0;
        _buffer[i].reread = false;
    }
}

void SamplerVoice::set_file(WaveFile& file)
{
    _wav = file;
    _data = _wav.get_audio_data();
    _totalSamples = _data.get_frames();
    _activeBufferCount = min(round_up_div(_totalSamples, kBufferSize), kBufferCount);
    _samplesPlayed = 0;
    _isPlaying = false;

    // Read file start buffer.
    DEBUG_PRINTF(INIT_MASK, "V%d: set_file: queuing buffer %d for read\r\n", _number, 0);
    queue_buffer_for_read(&_buffer[0]);
}

void SamplerVoice::prime()
{
    Ar::Mutex::Guard guard(_primeMutex);
    DEBUG_PRINTF(QUEUE_MASK, "V%d: start prime\r\n", _number);

    // Reset state.
    _isPlaying = false;
    _samplesPlayed = 0;
    _samplesRead = kBufferSize;
    _samplesQueued = kBufferSize;

    // Clear buffers queues.
    _fullBuffers.clear();
    _emptyBuffers.clear();

    // Playing will start from the file start buffer.
    _currentBuffer = &_buffer[0];
    _buffer[0].readHead = 0;

    // Queue up the rest of the available buffers to be filled.
    int i;
    for (i = 1; i < _activeBufferCount; ++i)
    {
        if (_buffer[i].state == kBufferReading)
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%d: prime: marking b%d for reread\r\n", _number, i);
            _buffer[i].reread = true;
        }
        else
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%d: prime: queuing b%d for read\r\n", _number, i);
            _buffer[i].startFrame = i * kBufferSize;
            queue_buffer_for_read(&_buffer[i]);
        }
    }
    DEBUG_PRINTF(QUEUE_MASK, "V%d: end prime\r\n", _number);
}

void SamplerVoice::trigger()
{
    // Handle re-triggering while sample is already playing.
    if (_isPlaying)
    {
        DEBUG_PRINTF(RETRIG_MASK, "V%d: trigger while playing\r\n", _number);

        // Start playing over from file start.
        prime();

        _led->off();
        _turnOnLedNextBuffer = true;
    }
    else
    {
        _led->on();
    }
    _isPlaying = true;
}

SamplerVoice::Buffer * SamplerVoice::get_current_buffer()
{
    if (!_currentBuffer)
    {
        _currentBuffer = dequeue_next_buffer();
    }
    return _currentBuffer;
}

void SamplerVoice::queue_buffer_for_read(Buffer * buffer)
{
    _samplesQueued += buffer->frameCount;

    buffer->reread = false;
    buffer->state = kBufferFree;

    _emptyBuffers.put(buffer);
    g_readerThread.enqueue(this);
}

SamplerVoice::Buffer * SamplerVoice::get_empty_buffer()
{
    Ar::Mutex::Guard guard(_primeMutex);

    Buffer * buffer;
    if (_emptyBuffers.get(buffer))
    {
        buffer->state = kBufferReading;
        return buffer;
    }
    else
    {
        return nullptr;
    }
}

void SamplerVoice::retire_buffer(Buffer * buffer)
{
    _samplesPlayed += buffer->frameCount;

    if (_samplesPlayed >= _totalSamples)
    {
        DEBUG_PRINTF(RETIRE_MASK, "V%d: retiring b%d; played %d (done)\r\n", _number, buffer->number, _samplesPlayed);
        _led->off();

        prime();
    }
    else
    {
        DEBUG_PRINTF(RETIRE_MASK, "V%d: retiring b%d; played %d\r\n", _number, buffer->number, _samplesPlayed);
        if (_turnOnLedNextBuffer)
        {
            _turnOnLedNextBuffer = false;
            _led->on();
        }

        // Don't queue up file start buffer for reading.
        if (buffer != &_buffer[0] && _samplesQueued < _totalSamples)
        {
            DEBUG_PRINTF(RETIRE_MASK|QUEUE_MASK, "V%d: retire: queue b%d to read @ %d\r\n", _number, buffer->number, _samplesPlayed);
            buffer->startFrame = _samplesQueued;
            buffer->frameCount = min(kBufferSize, _totalSamples - _samplesQueued);
            queue_buffer_for_read(buffer);
        }

        dequeue_next_buffer();
    }
}

SamplerVoice::Buffer * SamplerVoice::dequeue_next_buffer()
{
    Buffer * buffer;
    if (_fullBuffers.get(buffer))
    {
        DEBUG_PRINTF(CURBUF_MASK, "V%d: current buffer = %d\r\n", _number, buffer->number);
        _currentBuffer = buffer;
    }
    else
    {
        DEBUG_PRINTF(ERROR_MASK, "V%d: *** NO READY BUFFERS ***\r\n", _number);
        Ar::_halt();
        _currentBuffer = nullptr;
    }
    buffer->state = kBufferPlaying;
    return _currentBuffer;
}

void SamplerVoice::enqueue_full_buffer(Buffer * buffer)
{
    assert(buffer);
    if (buffer != &_buffer[0])
    {
        if (buffer->reread)
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%d: queuing b%d for reread\r\n", _number, buffer->number);
            queue_buffer_for_read(buffer);
        }
        else
        {
            _samplesRead += buffer->frameCount;

            buffer->state = kBufferReady;
            DEBUG_PRINTF(QUEUE_MASK, "V%d: queuing b%d for play\r\n", _number, buffer->number);
            _fullBuffers.put(buffer);
        }
    }
}

void ReaderThread::reader_thread()
{
    while (true)
    {
        // Pull a voice that needs a buffer filled from the queue.
        SamplerVoice * voice = _queue.receive();
        assert(voice);
        assert(voice->is_valid());

        // Ask the voice for the next buffer to fill. This may return null, which is valid.
        SamplerVoice::Buffer * request = voice->get_empty_buffer();
        if (!request)
        {
            continue;
        }
        DEBUG_PRINTF(FILL_MASK, "R: filling b%d v%d @ %d\r\n", request->number, voice->get_number(), request->startFrame);

        // Figure out how much data to read.
        Stream& stream = voice->get_audio_stream();
        uint32_t frameSize = voice->get_wave_file().get_frame_size();
        uint32_t channelCount = voice->get_wave_file().get_channels();
        uint32_t bytesToRead = request->frameCount * frameSize;
        assert(channelCount <= 2);

        // Read mono files directly into the voice buffer, stereo into a temp buffer.
        void * targetBuffer = request->data;
        if (channelCount == 2)
        {
            targetBuffer = g_readBuf;
            assert(bytesToRead <= sizeof(g_readBuf));
        }

        // Read from sample file.
        uint32_t start = Microseconds::get();
        stream.seek(request->startFrame * frameSize);
        uint32_t bytesRead = stream.read(bytesToRead, targetBuffer);
        uint32_t stop = Microseconds::get();
        uint32_t framesRead = bytesRead / frameSize;

        uint32_t delta = stop - start;
        DEBUG_PRINTF(TIME_MASK, "R: read %d bytes in %d µs\r\n", bytesRead, delta);

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

        // If we hit EOF, fill the remainder of the buffer with zeroes.
        if (framesRead < request->frameCount)
        {
            memset((uint8_t *)(request->data + framesRead), 0, (request->frameCount - framesRead) * sizeof(int16_t));
        }

        request->readHead = 0;
        voice->enqueue_full_buffer(request);

        uint32_t stop1 = Microseconds::get();
        uint32_t delta1 = stop1 - start1;
        DEBUG_PRINTF(TIME_MASK, "R: %d µs to fill b%d v%d\r\n", delta1, request->number, voice->get_number());
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
