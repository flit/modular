/*
 * Copyright (c) 2017 Immo Software
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
#if !defined(_SAMPLER_VOICE_H_)
#define _SAMPLER_VOICE_H_

#include "argon/argon.h"
#include "wav_file.h"
#include "led.h"
#include "protected_queue.h"
#include "audio_defs.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

class SamplerVoice;

/*!
 * @brief A buffer of sample data read from the file.
 */
struct SampleBuffer
{
    enum State : uint8_t
    {
        kUnused,
        kStartFile,
        kPlaying,
        kReady,
        kFree,
        kReading,
    };

    uint8_t number;
    State state;
    bool reread;
    int16_t * data;
    uint32_t startFrame;
    uint32_t frameCount;
    uint32_t readHead;
};

/*!
 * @brief Manages the queue of sample data buffers and coordination with the reader thread.
 */
class SampleBufferManager
{
public:
    static const uint32_t kBufferCount = 4; //!< Number of buffers available to cycle through sample data. The first one will always be used to hold the first #kBufferSize frames of the sample.
    static const uint32_t kBufferSize = 1024; //!< Number of frames per buffer.
    static_assert(kBufferSize % kAudioBufferSize == 0, "sai buffers must fit evenly in voice buffers");

    SampleBufferManager();
    ~SampleBufferManager()=default;

    void init(SamplerVoice * voice);
    void set_file(uint32_t totalFrames);

    void prime();

    SampleBuffer * get_current_buffer();
    SampleBuffer * get_empty_buffer();

    void enqueue_full_buffer(SampleBuffer * buffer);
    void queue_buffer_for_read(SampleBuffer * buffer);
    SampleBuffer * dequeue_next_buffer();
    void retire_buffer(SampleBuffer * buffer);

    uint32_t get_total_samples() const { return _totalSamples; }
    uint32_t get_samples_played() const { return _samplesPlayed; }
    uint32_t get_samples_read() const { return _samplesRead; }
    uint32_t get_samples_queued() const { return _samplesQueued; }

protected:
    typedef ProtectedQueue<SampleBuffer*, kBufferCount> BufferQueue;

    SamplerVoice * _voice;
    uint32_t _number;
    int16_t _bufferData[kBufferCount * kBufferSize];
    SampleBuffer _buffer[kBufferCount];
    BufferQueue _fullBuffers;
    BufferQueue _emptyBuffers;
    Ar::Mutex _primeMutex;
    SampleBuffer * _currentBuffer;
    uint32_t _activeBufferCount;
    uint32_t _totalSamples;
    uint32_t _samplesPlayed;
    uint32_t _samplesRead;
    uint32_t _samplesQueued;
    bool _didReadFileStart;
};

/*!
 * @brief Manages playback of a single sample file.
 */
class SamplerVoice
{
public:
    SamplerVoice();
    ~SamplerVoice()=default;

    void init(uint32_t n);
    void set_led(LEDBase * led) { _led = led; }
    void set_file(WaveFile & file);

    uint32_t get_number() const { return _number; }
    bool is_valid() const { return _wav.is_valid(); }

    void prime();

    void trigger();
    bool is_playing() { return _isPlaying; }

    void set_gain(float gain) { _gain = gain; }

    void render(int16_t * data, uint32_t frameCount);

    WaveFile& get_wave_file() { return _wav; }
    WaveFile::AudioDataStream& get_audio_stream() { return _data; }

    SampleBufferManager& get_buffer_manager() { return _manager; }

protected:
    uint32_t _number;
    LEDBase * _led;
    WaveFile _wav;
    WaveFile::AudioDataStream _data;
    SampleBufferManager _manager;
    bool _isPlaying;
    bool _turnOnLedNextBuffer;
    float _gain;

};

} // namespace slab

#endif // _SAMPLER_VOICE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
