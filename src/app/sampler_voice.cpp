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
#include "utility.h"
#include <cmath>
#include <algorithm>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

//! Set to 1 to cause the voice to automatically and repeatedly trigger.
#define ENABLE_TEST_LOOP_MODE (0)

//! Number of samples to fade out.
const uint32_t kNoteOffSamples = 128;

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

float SamplerVoice::s_workBufferData[kAudioBufferSize];
AudioBuffer SamplerVoice::s_workBuffer(s_workBufferData, kAudioBufferSize);

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
    _snapToZeroEnd(false),
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
    for (i = 0; i < kBufferCount; ++i)
    {
        _buffer[i].number = i;
        _buffer[i].data = &buffer[i * kBufferSize];
    }

    set_file(0);
}

//! @brief Reset all buffers to unused.
void SampleBufferManager::_reset_buffers()
{
    uint32_t i;
    for (i = 0; i < kBufferCount; ++i)
    {
        _buffer[i].state = SampleBuffer::State::kUnused;
        _buffer[i].startFrame = 0;
        _buffer[i].frameCount = kBufferSize;
        _buffer[i].reread = false;
    }
}

void SampleBufferManager::set_file(uint32_t totalFrames)
{
    _startSample = 0;
    _totalSamples = totalFrames;
    _endSample = totalFrames;
    _activeBufferCount = min(round_up_div(_totalSamples, kBufferSize), kBufferCount);
    _currentBuffer = nullptr;
    _samplesPlayed = 0;
    _samplesRead = 0;
    _samplesQueued = 0;
    _didReadFileStart = false;
    _waitingForFileStart = true;
    _isReady = false;
    _snapToZeroStart = false;
    _snapToZeroEnd = false;
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
    _samplesRead = _didReadFileStart ? min(_startSample + kBufferSize, _totalSamples) : _startSample;
    _samplesQueued = _samplesRead;

    // Clear buffers queues.
    ReaderThread::get().clear_voice_queue(_voice);
    _fullBuffers.clear();
    _emptyBuffers.clear();

    // Playing will start from the file start buffer.
    if (_activeBufferCount)
    {
        if (_didReadFileStart)
        {
            _currentBuffer = &_buffer[0];
            _currentBuffer->state = SampleBuffer::State::kPlaying;
        }
        else
        {
            _currentBuffer = nullptr;
        }
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
    buffer->frameCount = min(kBufferSize, _endSample - _samplesQueued);
    buffer->reread = false;
    buffer->state = SampleBuffer::State::kFree;

    _emptyBuffers.put(buffer);
    _samplesQueued += buffer->frameCount;

    ReaderThread::get().enqueue(_voice);
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
    }
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

        DEBUG_PRINTF(QUEUE_MASK, "V%lu: queuing b%d for play\r\n", _number, buffer->number);
        _fullBuffers.put(buffer);

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
            _voice->manager_did_become_ready();
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
    assert(false);
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

    // Update parameters.
    _startSample = constrained(ustart, 0UL, _totalSamples);
    _endSample = constrained(uend, ustart, _totalSamples);

    _activeBufferCount = min(round_up_div(get_active_samples(), kBufferSize), kBufferCount);

    // Reload start of the file if the start sample changed.
    if (_startSample != originalStart)
    {
        _isReady = false;
        _didReadFileStart = false;
        _waitingForFileStart = true;
        _snapToZeroStart = (_startSample != 0);
        _preppedCount = 0;
        prime();
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

SamplerVoice::SamplerVoice()
:   _wav(),
    _data(),
    _manager(),
    _isValid(false),
    _isReady(false),
    _isPlaying(false),
    _doNoteOff(false),
    _doRetrigger(false),
    _noteOffSamplesRemaining(0),
    _readHead(0.0f),
    _pitchOctave(0.0f),
    _params()
{
}

void SamplerVoice::init(uint32_t n, int16_t * buffer)
{
    _number = n;
    _manager.init(this, buffer);
    clear_file();
}

void SamplerVoice::set_file(WaveFile& file)
{
    _isReady = false;
    _reset_voice();
    _params.reset();
    _pitchOctave = 0.0f;
    _wav = file;
    _data = _wav.get_audio_data();
    _isValid = true;

    // Prime the manager to start filling buffers.
    _manager.set_file(_data.get_frames());

#if ENABLE_TEST_LOOP_MODE
    _isPlaying = true;
#endif
}

void SamplerVoice::clear_file()
{
    _isValid = false;
    _isReady = false;
    _reset_voice();
    _params.reset();
    _pitchOctave = 0.0f;
    _wav = WaveFile();
    _data = WaveFile::AudioDataStream();
    _manager.set_file(0);
}

//! @brief Prepare voice to start playing from start sample.
void SamplerVoice::_reset_voice()
{
    _isPlaying = false;
    _doNoteOff = false;
    _doRetrigger = false;
    _noteOffSamplesRemaining = 0;
    _readHead = 0.0f;
    std::fill_n(&_interpolationBuffer[0], kInterpolationBufferLength, 0);
}

void SamplerVoice::prime()
{
    _reset_voice();
    _manager.prime();
}

void SamplerVoice::trigger()
{
    // Ignore the trigger if the manager isn't ready to play.
    if (!is_ready())
    {
        return;
    }

    // Handle re-triggering while sample is already playing.
    if (_isPlaying)
    {
        DEBUG_PRINTF(RETRIG_MASK, "V%lu: retrigger (@%lu)\r\n", _number, _manager.get_samples_played());

        // Start retrigger process.
        _doNoteOff = true;
        _doRetrigger = true;
        _noteOffSamplesRemaining = kNoteOffSamples;
    }
    else
    {
        UI::get().set_voice_playing(_number, true);
        _isPlaying = true;
    }
}

void SamplerVoice::note_off()
{
    // Ignore the note off event if the manager isn't ready to play.
    if (!is_ready())
    {
        return;
    }

    DEBUG_PRINTF(RETRIG_MASK, "V%lu: note off (@%lu)\r\n", _number, _manager.get_samples_played());

    if (_isPlaying)
    {
        // Start note off process.
        _doNoteOff = true;
        _noteOffSamplesRemaining = kNoteOffSamples;
    }
}

void SamplerVoice::playing_did_finish()
{
    UI::get().set_voice_playing(_number, false);
    prime();

#if ENABLE_TEST_LOOP_MODE
    trigger();
#endif
}

void SamplerVoice::render(int16_t * data, uint32_t frameCount)
{
    uint32_t i;

    // Get the current buffer if playing.
    SampleBuffer * voiceBuffer = nullptr;
    if (is_ready() && is_playing())
    {
        voiceBuffer = _manager.get_current_buffer();
    }

    // If not playing, just fill with silence.
    if (!voiceBuffer)
    {
        for (i = 0; i < frameCount; ++i)
        {
            *data = 0;
            data += 2;
        }
        return;
    }

    float rate = powf(2.0f, _params.baseOctaveOffset + _pitchOctave + (_params.baseCentsOffset / 1200.0f));
    int16_t * bufferData = voiceBuffer->data;
    float readHead = _readHead;
    uint32_t bufferFrameCount = voiceBuffer->frameCount;
    uint32_t remainingFrames = frameCount;
    float s;

    // Skip ahead to the buffer's zero snap offset. This will only apply to the file start.
    if (readHead == 0.0f)
    {
        readHead = voiceBuffer->zeroSnapOffset;
    }

    // Special case for unmodified pitch.
    if (rate == 1.0f && !_doNoteOff)
    {
        uint32_t integerReadHead = readHead;
        uint32_t framesRemainingInBuffer = bufferFrameCount - integerReadHead;
        uint32_t framesFromBuffer = min(framesRemainingInBuffer, frameCount);

        // Render sample data into the output buffer.
        for (i = 0; i < framesFromBuffer; ++i)
        {
            // Apply the gain.
            s = float(bufferData[integerReadHead++]);
            s *= _params.gain;
            *data = int16_t(s);
            data += 2;
        }

        readHead = integerReadHead;
        remainingFrames -= framesFromBuffer;
    }
    else if (_doNoteOff)
    {
        bool updateBuffer = false;

        // Render sample data into the output buffer at the specified playback rate.
        for (i = 0; i < frameCount; ++i)
        {
            // Check if we reached the end of this sample buffer.
            if (readHead >= bufferFrameCount)
            {
                std::copy_n(&bufferData[bufferFrameCount - kInterpolationBufferLength], kInterpolationBufferLength, _interpolationBuffer);

                _manager.retire_buffer(voiceBuffer);

                updateBuffer = true;
            }

            float gain = _params.gain;
            if (_doNoteOff)
            {
                if (_noteOffSamplesRemaining-- == 0)
                {
                    // Prime will clear all flags for us, so save retrigger locally to set
                    // isPlaying after priming.
                    bool savedRetrigger = _doRetrigger;
                    prime();
                    _isPlaying = savedRetrigger;

                    UI::get().indicate_voice_retriggered(_number);

                    updateBuffer = true;
                    readHead = bufferFrameCount;
                }
                else
                {
                    gain *= float(_noteOffSamplesRemaining) / float(kNoteOffSamples);
                }
            }

            if (updateBuffer)
            {
                updateBuffer = false;
                readHead -= bufferFrameCount;

                // If we're no longer playing, exit this loop.
                if (!_isPlaying)
                {
                    // Set some variables used outside the loop below.
                    readHead = 0.0f;
                    break;
                }

                // Get the next buffer and update locals.
                voiceBuffer = _manager.get_current_buffer();
                if (!voiceBuffer)
                {
                    break;
                }
                bufferData = voiceBuffer->data;
                bufferFrameCount = voiceBuffer->frameCount;
            }

            // Read interpolated sample from buffer.
            s = voiceBuffer->read<SampleBuffer::InterpolationMode::kHermite>(readHead, _interpolationBuffer);

            // Apply the gain and write into output buffer.
            s *= gain;
            *data = int16_t(s);
            data += 2;

            readHead += rate;
        }

        remainingFrames -= i;
    }
    else
    {
        assert(readHead < bufferFrameCount);
        while (remainingFrames)
        {
            assert(remainingFrames <= frameCount);
            uint32_t framesAtRate = std::ceil(remainingFrames * rate);
            uint32_t integerReadHead = readHead;
            uint32_t framesRemainingInBuffer = bufferFrameCount - integerReadHead;
            uint32_t outputFrames;

            // Handle case where there are fewer remaining frames in the buffer than
            // the playback rate, in which case we need to just move to the next buffer.
            if (framesRemainingInBuffer < rate)
            {
                readHead += rate;
            }
            else
            {
                if (framesAtRate > framesRemainingInBuffer)
                {
                    outputFrames = min(uint32_t(std::floor(framesRemainingInBuffer / rate)), remainingFrames);
                }
                else
                {
                    outputFrames = remainingFrames;
                }

                // Render sample data into the output buffer at the specified playback rate.
                readHead = voiceBuffer->read_into<SampleBuffer::InterpolationMode::kHermite>(
                            s_workBuffer, outputFrames, readHead, rate, _interpolationBuffer);
                s_workBuffer.multiply_scalar(_params.gain, outputFrames);
                s_workBuffer.copy_into(data, 2, outputFrames);

                remainingFrames -= outputFrames;
                data += outputFrames * 2;
            }

            if (readHead > (bufferFrameCount - 1))
            {
                readHead -= bufferFrameCount;
                std::copy_n(&bufferData[bufferFrameCount - kInterpolationBufferLength], kInterpolationBufferLength, _interpolationBuffer);

                _manager.retire_buffer(voiceBuffer);

                // If we're no longer playing, exit this loop.
                if (!_isPlaying)
                {
                    // Set some variables used outside the loop below.
                    readHead = 0.0f;
                    break;
                }

                // Get the next buffer and update locals.
                voiceBuffer = _manager.get_current_buffer();
                if (!voiceBuffer)
                {
                    break;
                }
                bufferData = voiceBuffer->data;
                bufferFrameCount = voiceBuffer->frameCount;
            }
            assert(readHead < bufferFrameCount);
        }
    }

    // Save local read head.
    _readHead = readHead;

    // Fill any leftover frames with silence.
    while (remainingFrames--)
    {
        *data = 0;
        data += 2;
    }

    // Did we finish this buffer?
    if (_readHead >= bufferFrameCount && voiceBuffer)
    {
        std::copy_n(&bufferData[bufferFrameCount - kInterpolationBufferLength], kInterpolationBufferLength, _interpolationBuffer);
        _manager.retire_buffer(voiceBuffer);
        _readHead -= bufferFrameCount;
    }
    assert(_readHead < bufferFrameCount);
}

void SamplerVoice::set_sample_start(float start)
{
    // Stop playing and turn off LED.
    _reset_voice();
    UI::get().set_voice_playing(_number, false);

    _params.startSample = start;

    // Tell sample manager to set and load new start point.
    uint32_t sample = float(_manager.get_total_samples()) * start;
    _manager.set_start_end_sample(sample, -1);
}

void SamplerVoice::set_sample_end(float end)
{
    // Stop playing and turn off LED.
    _reset_voice();
    UI::get().set_voice_playing(_number, false);

    _params.endSample = end;

    uint32_t sample = float(_manager.get_total_samples()) * end;
    _manager.set_start_end_sample(-1, sample);
}

void SamplerVoice::set_params(const VoiceParameters & params)
{
    // Stop playing and turn off LED.
    _reset_voice();
    UI::get().set_voice_playing(_number, false);

    // Update params.
    _params = params;

    // Update playback range in SBM.
    float totalSamples = _manager.get_total_samples();
    uint32_t start = totalSamples * _params.startSample;
    uint32_t end = totalSamples * _params.endSample;
    _manager.set_start_end_sample(start, end);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
