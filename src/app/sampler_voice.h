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

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

#define BUFFER_SIZE (256)

namespace slab {

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
    bool _readFileStart;
    bool _isPlaying;
    bool _turnOnLedNextBuffer;

    void queue_buffer_for_read(Buffer * buffer);
    Buffer * dequeue_next_buffer();
    void retire_buffer(Buffer * buffer);

};

} // namespace slab

#endif // _SAMPLER_VOICE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
