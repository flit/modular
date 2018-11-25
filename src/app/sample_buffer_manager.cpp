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

#include "sampler_voice.h"
#include "reader_thread.h"
#include "ui.h"
#include "debug_log.h"
#include "itm_trace.h"
#include <algorithm>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

//! Number of samples to fade in when we can't find a zero crossing.
const uint32_t kFadeInSampleCount = 128;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

SampleBufferManager::SampleBufferManager()
:   _number(0),
    _fullBuffers(),
    _emptyBuffers(),
    _primeMutex(),
    _currentBuffer(nullptr),
    _activeBufferCount(0),
    _totalSamples(0),
    _startSample(0),
    _samplesPlayed(0),
    _samplesRead(0),
    _samplesQueued(0),
    _didReadFileStart(false),
    _waitingForFileStart(true),
    _isReady(false),
    _snapToZeroStart(false),
    _preppedCount(0)
{
}

void SampleBufferManager::init(SamplerVoice * voice, int16_t * buffer)
{
    _voice = voice;
    _number = voice->get_number();
    _primeMutex.init("prime");

    // Init invariant buffer fields.
    uint32_t i;
    for (i = 0; i < kVoiceBufferCount; ++i)
    {
        _buffer[i].number = i;
        _buffer[i].dataWithInterpolationFrames = &buffer[i * (kVoiceBufferSize + SampleBuffer::kInterpolationFrameCount)];
        _buffer[i].data = _buffer[i].dataWithInterpolationFrames + SampleBuffer::kInterpolationFrameCount;
    }

    set_file(0);
}

//! @brief Reset all buffers to unused.
void SampleBufferManager::_reset_buffers()
{
    uint32_t i;
    for (i = 0; i < kVoiceBufferCount; ++i)
    {
        _buffer[i].set_unused();
    }
}

void SampleBufferManager::set_file(uint32_t totalFrames)
{
    _startSample = 0;
    _totalSamples = totalFrames;
    _endSample = totalFrames;
    _activeBufferCount = min(round_up_div(_totalSamples, kVoiceBufferSize), kVoiceBufferCount);
    _currentBuffer = nullptr;
    _samplesPlayed = 0;
    _samplesRead = 0;
    _samplesQueued = 0;
    _didReadFileStart = false;
    _waitingForFileStart = true;
    _isReady = false;
    _snapToZeroStart = false;
    _preppedCount = 0;
    _reset_buffers();

    // Go ahead and prime.
    prime();
}

void SampleBufferManager::prime()
{
    Ar::Mutex::Guard guard(_primeMutex);
    DEBUG_PRINTF(QUEUE_MASK, "V%lu: start prime\r\n", _number);

    // Reset state.
    _samplesPlayed = _startSample;
    _samplesRead = _didReadFileStart ? min(_startSample + kVoiceBufferSize, _totalSamples) : _startSample;
    _samplesQueued = _samplesRead;

    // Clear buffers queues.
    ReaderThread::get().clear_voice_queue(_voice);
    _fullBuffers.clear();
    _emptyBuffers.clear();

    // Playing will start from the file start buffer.
    if (_activeBufferCount && _didReadFileStart)
    {
        _currentBuffer = &_buffer[0];
        _currentBuffer->state = SampleBuffer::State::kPlaying;
    }
    else
    {
        _currentBuffer = nullptr;
    }

    // Queue up the rest of the available buffers to be filled.
    uint32_t i = _didReadFileStart ? 1 : 0;
    for (; i < _activeBufferCount; ++i)
    {
        // If the buffer is currently being filled by the reader thread, then we can't touch
        // it. So just flag it for requeuing when it's finished. This will be handled in
        // enqueue_full_buffer().
        if (_buffer[i].state == SampleBuffer::State::kReading)
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%lu: prime: marking b%lu for reread\r\n", _number, i);
            _buffer[i].reread = true;
        }
        else
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%lu: prime: queuing b%lu for read\r\n", _number, i);
            _queue_buffer_for_read(&_buffer[i]);
        }
    }
    DEBUG_PRINTF(QUEUE_MASK, "V%lu: end prime\r\n", _number);

#if ENABLE_TRACE
    _trace_buffers();
#endif
}

SampleBuffer * SampleBufferManager::get_current_buffer()
{
    if (!_currentBuffer)
    {
        _currentBuffer = _dequeue_next_buffer();
    }
    return _currentBuffer;
}

void SampleBufferManager::_queue_buffer_for_read(SampleBuffer * buffer)
{
    buffer->startFrame = _samplesQueued;
    buffer->frameCount = min(kVoiceBufferSize, _endSample - _samplesQueued);
    buffer->reread = false;
    buffer->state = SampleBuffer::State::kFree;

    _emptyBuffers.put(buffer);
    _samplesQueued += buffer->frameCount;

    ReaderThread::get().enqueue(_voice);

#if ENABLE_TRACE
    _trace_buffers();
#endif
}

SampleBuffer * SampleBufferManager::get_empty_buffer()
{
    Ar::Mutex::Guard guard(_primeMutex);

    SampleBuffer * buffer;
    if (_emptyBuffers.get(buffer))
    {
        assert(buffer->state == SampleBuffer::State::kFree);
        buffer->state = SampleBuffer::State::kReading;
        return buffer;
    }
    else
    {
        return nullptr;
    }
}

void SampleBufferManager::retire_buffer(SampleBuffer * buffer)
{
    Ar::Mutex::Guard guard(_primeMutex);

    assert(buffer->state == SampleBuffer::State::kPlaying);
    _samplesPlayed += buffer->frameCount;
    buffer->state = (buffer->number == 0) ? SampleBuffer::State::kReady : SampleBuffer::State::kFree;

    if (_samplesPlayed >= _endSample)
    {
        DEBUG_PRINTF(RETIRE_MASK, "V%lu: retiring b%d; played %lu (done)\r\n", _number, buffer->number, _samplesPlayed);

        _voice->playing_did_finish();
    }
    else
    {
        DEBUG_PRINTF(RETIRE_MASK, "V%lu: retiring b%d; played %lu\r\n", _number, buffer->number, _samplesPlayed);

        // Don't queue up file start buffer for reading, and don't queue more than the active
        // number of samples.
        if (buffer->number != 0 && _samplesQueued < _endSample)
        {
            DEBUG_PRINTF(RETIRE_MASK|QUEUE_MASK, "V%lu: retire: queue b%d to read @ %lu\r\n", _number, buffer->number, _samplesPlayed);
            _queue_buffer_for_read(buffer);
        }

        _dequeue_next_buffer();
    }

#if ENABLE_TRACE
    _trace_buffers();
#endif
}

SampleBuffer * SampleBufferManager::_dequeue_next_buffer()
{
    Ar::Mutex::Guard guard(_primeMutex);

    SampleBuffer * buffer;
    if (_fullBuffers.get(buffer))
    {
        assert(buffer->state == SampleBuffer::State::kReady);
        DEBUG_PRINTF(CURBUF_MASK, "V%lu: current buffer = %d\r\n", _number, buffer->number);
        _currentBuffer = buffer;
        buffer->state = SampleBuffer::State::kPlaying;
    }
    else
    {
        DEBUG_PRINTF(ERROR_MASK, "V%lu: *** NO READY BUFFERS ***\r\n", _number);
//         Ar::_halt();
        _currentBuffer = nullptr;
        UI::get().indicate_voice_underflowed(_number);
    }

#if ENABLE_TRACE
    _trace_buffers();
#endif

    return _currentBuffer;
}

//! @todo clean up
void SampleBufferManager::enqueue_full_buffer(SampleBuffer * buffer)
{
    Ar::Mutex::Guard guard(_primeMutex);

    assert(buffer);
    bool isBuffer0 = (buffer->number == 0);
    bool isOutOfOrder = (buffer->startFrame != _samplesRead);

    if (buffer->reread || isOutOfOrder)
    {
        assert(!(isOutOfOrder && !isBuffer0 && !buffer->reread));
        DEBUG_PRINTF(QUEUE_MASK, "V%lu: queuing b%d for reread\r\n", _number, buffer->number);

        if (isBuffer0)
        {
            // If the file start buffer has to be reread, just do another prime. Need to
            // set the buffer state to ready so prime doesn't cause another reread.
            buffer->state = SampleBuffer::State::kReady;
            _didReadFileStart = false;
            prime();
        }
        else
        {
            // Queue up this buffer to be re-read instead of putting it in the full buffers queue.
            // This call will set the buffer state appropriately.
            _queue_buffer_for_read(buffer);
        }
    }
    else
    {
        _samplesRead += buffer->frameCount;
        buffer->state = SampleBuffer::State::kReady;
        buffer->zeroSnapOffset = 0;

        // Clear interpolation frames. The renderer will copy interpolation frames between buffers.
        std::fill_n(buffer->dataWithInterpolationFrames, SampleBuffer::kInterpolationFrameCount, 0);

        DEBUG_PRINTF(QUEUE_MASK, "V%lu: queuing b%d for play\r\n", _number, buffer->number);
        _fullBuffers.put(buffer);

#if ENABLE_TRACE
        _trace_buffers();
#endif

        if (isBuffer0)
        {
            _didReadFileStart = true;

            if (_snapToZeroStart)
            {
                _find_zero_crossing(buffer);
            }
        }

        // Wait until we get a full set of buffers before allowing play to start.
        if (_waitingForFileStart && isBuffer0)// && (++_preppedCount >= _activeBufferCount))
        {
            _waitingForFileStart = false;
            _isReady = true;
            _voice->manager_ready_did_change(true);
        }
    }
}

//! Finds the first zero crossing, meaning positive to negative or vice versa, or
//! an actual zero sample. If the sample is not zero, it sets the prior sample
//! to zero. The zero sample position is set in the buffer's zeroSnapOffset member.
//!
//! @note The buffer must have a frame count of at least 2, or this method will do nothing.
void SampleBufferManager::_find_zero_crossing(SampleBuffer * buffer)
{
    if (buffer->frameCount < 2)
    {
        return;
    }

    uint32_t i;
    int16_t previousSample = buffer->data[0];
    int16_t * sample = &buffer->data[1];
    for (i = 1; i < buffer->frameCount; ++i, ++sample)
    {
        int16_t thisSample = *sample;
        if ((thisSample <= 0 && previousSample >= 0)
            || (thisSample >= 0 && previousSample <= 0))
        {
            if (i > 0)
            {
                buffer->data[i - 1] = 0;
                buffer->zeroSnapOffset = i - 1;
            }
            else
            {
                *sample = 0;
                buffer->zeroSnapOffset = i;
            }
            return;
        }
        previousSample = thisSample;
    }

    // Failed to find a zero crossing, so apply a fade in.
    sample = &buffer->data[0];
    uint32_t fadeCount = min(buffer->frameCount, kFadeInSampleCount);
    for (i = 0; i < fadeCount; ++i, ++sample)
    {
        *sample = static_cast<int16_t>(static_cast<int32_t>(*sample) * i / fadeCount);
    }
}

void SampleBufferManager::set_start_end_sample(int32_t start, int32_t end)
{
    // Voice must not be playing.
    assert(!_voice->is_playing());

    // Handle sentinels to use current values.
    uint32_t ustart = (start == -1L) ? _startSample : start;
    uint32_t uend = (end == -1L) ? _endSample : end;
    DEBUG_PRINTF(MISC_MASK, "set_start_end: %lu - %lu\r\n", ustart, uend);

    uint32_t originalStart = _startSample;
    uint32_t originalEnd = _endSample;

    // Update parameters.
    _startSample = constrained(ustart, 0UL, _totalSamples);
    _endSample = constrained(uend, ustart, _totalSamples);

    // Update number of used buffers.
    _activeBufferCount = min(round_up_div(get_active_samples(), kVoiceBufferSize), kVoiceBufferCount);

    // Set any unused buffers to kUnused state.
    uint32_t i;
    for (i = _activeBufferCount; i < kVoiceBufferCount; ++i)
    {
        _buffer[i].set_unused();
    }

    // Reload start of the file if the start or end point changed.
    if (_startSample != originalStart || _endSample != originalEnd)
    {
        _isReady = false;
        _voice->manager_ready_did_change(false);
        _didReadFileStart = false;
        _waitingForFileStart = true;
        _snapToZeroStart = (_startSample != 0);
        _preppedCount = 0;
        prime();
    }
}

uint32_t SampleBufferManager::get_buffered_samples() const
{
    uint32_t count = 0;
    uint32_t i;
    for (i = 0; i < _fullBuffers.get_count(); ++i)
    {
        SampleBuffer * buf = _fullBuffers[i];
        count += buf->frameCount;
    }
    return count;
}

void SampleBufferManager::_trace_buffers()
{
    // Event structure:
    // [15:14] = 2-bit channel number
    // [13:7]  = free buffer count
    // [6:0]   = ready buffer count
    itm<kBufferCountChannel, uint16_t>::send((_number << 14)
                    | ((_emptyBuffers.get_count() && 0x7f) << 7)
                    | (_fullBuffers.get_count() & 0x7f));
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
