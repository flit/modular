/*
 * Copyright (c) 2017-2018 Immo Software
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
#if !defined(_SAMPLE_BUFFER_MANAGER_H_)
#define _SAMPLE_BUFFER_MANAGER_H_

#include "argon/argon.h"
#include "simple_queue.h"
#include "sample_buffer.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

class SamplerVoice;

/*!
 * @brief Manages the queue of sample data buffers and coordination with the reader thread.
 */
class SampleBufferManager
{
public:
    SampleBufferManager();
    ~SampleBufferManager()=default;

    void init(SamplerVoice * voice, int16_t * buffer);
    void set_file(uint32_t totalFrames);

    //! @brief Prepare for playback from start.
    void prime();

    //! @brief Modify the sample playback range.
    //!
    //! Out of range and out of order values are accepted. Start and end are limited so that start
    //! cannot be greater than end, and vice versa. The start and end are also limited to the valid
    //! total sample range of the data.
    //!
    //! @param start Sample number to start playing from, or -1 to use current value.
    //! @param start Sample number at which playback will cease. Pass -1 to use current value.
    void set_start_end_sample(int32_t start, int32_t end);

    bool is_ready() const { return _isReady; }
    SampleBuffer * get_current_buffer();
    SampleBuffer * get_empty_buffer();

    void enqueue_full_buffer(SampleBuffer * buffer);
    void retire_buffer(SampleBuffer * buffer);

    uint32_t get_total_samples() const { return _totalSamples; }
    uint32_t get_active_samples() const { return _endSample - _startSample; }
    uint32_t get_start_sample() const { return _startSample; }
    uint32_t get_end_sample() const { return _endSample; }
    uint32_t get_samples_played() const { return _samplesPlayed; }
    uint32_t get_samples_read() const { return _samplesRead; }
    uint32_t get_samples_queued() const { return _samplesQueued; }

    uint32_t get_buffered_samples() const;

protected:
    typedef SimpleQueue<SampleBuffer*, kVoiceBufferCount> BufferQueue;

    SamplerVoice * _voice;
    uint32_t _number;
    SampleBuffer _buffer[kVoiceBufferCount];
    BufferQueue _fullBuffers;
    BufferQueue _emptyBuffers;
    Ar::Mutex _primeMutex;
    SampleBuffer * _currentBuffer;
    uint32_t _activeBufferCount;
    uint32_t _totalSamples;
    uint32_t _startSample;
    uint32_t _endSample;
    uint32_t _samplesPlayed;
    uint32_t _samplesRead;
    uint32_t _samplesQueued;
    bool _didReadFileStart;
    bool _waitingForFileStart;
    bool _isReady;
    bool _snapToZeroStart;
    uint32_t _preppedCount;

    void _reset_buffers();

    void _queue_buffer_for_read(SampleBuffer * buffer);
    SampleBuffer * _dequeue_next_buffer();

    void _find_zero_crossing(SampleBuffer * buffer);

    void _trace_buffers();
};

} // namespace slab

#endif // _SAMPLE_BUFFER_MANAGER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
