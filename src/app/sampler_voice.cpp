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

using namespace slab;

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
    _isReady(false)
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
        _buffer[i].readHead = 0;
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
        _currentBuffer = &_buffer[0];
        _buffer[0].readHead = 0;
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
            queue_buffer_for_read(&_buffer[i]);
        }
    }
    DEBUG_PRINTF(QUEUE_MASK, "V%lu: end prime\r\n", _number);
}

SampleBuffer * SampleBufferManager::get_current_buffer()
{
    if (!_currentBuffer)
    {
        _currentBuffer = dequeue_next_buffer();
    }
    return _currentBuffer;
}

void SampleBufferManager::queue_buffer_for_read(SampleBuffer * buffer)
{
    buffer->startFrame = _samplesQueued;
    buffer->frameCount = min(kBufferSize, _endSample - _samplesQueued);
    buffer->reread = false;
    buffer->state = SampleBuffer::State::kFree;

    _emptyBuffers.put(buffer);
    ReaderThread::get().enqueue(_voice);

    _samplesQueued += buffer->frameCount;
}

SampleBuffer * SampleBufferManager::get_empty_buffer()
{
    Ar::Mutex::Guard guard(_primeMutex);

    SampleBuffer * buffer;
    if (_emptyBuffers.get(buffer))
    {
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
    _samplesPlayed += buffer->frameCount;

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
        if (buffer != &_buffer[0] && _samplesQueued < _endSample)
        {
            DEBUG_PRINTF(RETIRE_MASK|QUEUE_MASK, "V%lu: retire: queue b%d to read @ %lu\r\n", _number, buffer->number, _samplesPlayed);
            queue_buffer_for_read(buffer);
        }

        dequeue_next_buffer();
    }
}

SampleBuffer * SampleBufferManager::dequeue_next_buffer()
{
    SampleBuffer * buffer;
    if (_fullBuffers.get(buffer))
    {
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

void SampleBufferManager::enqueue_full_buffer(SampleBuffer * buffer)
{
    assert(buffer);
    if (buffer == &_buffer[0])
    {
        _didReadFileStart = true;
        if (_waitingForFileStart)
        {
            _waitingForFileStart = false;
            _isReady = true;
        }
        _samplesRead += buffer->frameCount;
        buffer->state = SampleBuffer::State::kReady;
    }
    else
    {
        if (buffer->reread)
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%lu: queuing b%d for reread\r\n", _number, buffer->number);
            queue_buffer_for_read(buffer);
        }
        else
        {
            _samplesRead += buffer->frameCount;

            buffer->state = SampleBuffer::State::kReady;
            DEBUG_PRINTF(QUEUE_MASK, "V%lu: queuing b%d for play\r\n", _number, buffer->number);
            _fullBuffers.put(buffer);
        }
    }
}

void SampleBufferManager::set_start_sample(uint32_t start)
{
    // Voice must not be playing.
    assert(!_voice->is_playing());

    _isReady = false;
    _didReadFileStart = false;
    _waitingForFileStart = true;
    _startSample = start;
    if (_endSample < _startSample)
    {
        _endSample = _startSample;
    }

    _activeBufferCount = min(round_up_div(get_active_samples(), kBufferSize), kBufferCount);

    prime();
}

void SampleBufferManager::set_end_sample(uint32_t end)
{
    // Voice must not be playing.
    assert(!_voice->is_playing());
    assert(end >= _startSample);

    _endSample = end;

    _activeBufferCount = min(round_up_div(get_active_samples(), kBufferSize), kBufferCount);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

SamplerVoice::SamplerVoice()
:   _wav(),
    _data(),
    _manager(),
    _isPlaying(false),
    _turnOnLedNextBuffer(false),
    _lastBufferLastSample(0),
    _fraction(0.0f),
    _gain(1.0f),
    _pitch(1.0f)
{
}

void SamplerVoice::init(uint32_t n, int16_t * buffer)
{
    _number = n;
    _manager.init(this, buffer);
    set_bits(32);
    clear_file();
}

void SamplerVoice::set_file(WaveFile& file)
{
    _reset_voice();
    _wav = file;
    _data = _wav.get_audio_data();
    _manager.set_file(_data.get_frames()); // This wil prime the manager.
}

void SamplerVoice::clear_file()
{
    _reset_voice();
    _wav = WaveFile();
    _data = WaveFile::AudioDataStream();
    _manager.set_file(0);
}

void SamplerVoice::_reset_voice()
{
    _isPlaying = false;
    _fraction = 0.0f;
    _lastBufferLastSample = 0.0f;
}

void SamplerVoice::prime()
{
    _reset_voice();
    _manager.prime();
}

void SamplerVoice::trigger()
{
    // Ignore the trigger if the manager isn't ready to play.
    if (!is_valid() || !_manager.is_ready())
    {
        return;
    }

    // Handle re-triggering while sample is already playing.
    if (_isPlaying)
    {
        DEBUG_PRINTF(RETRIG_MASK, "V%lu: retrigger (@%lu)\r\n", _number, _manager.get_samples_played());

        // Start playing over from file start.
        prime();

        UI::get().set_voice_playing(_number, false);
        _turnOnLedNextBuffer = true;
    }
    else
    {
        UI::get().set_voice_playing(_number, true);
    }
    _isPlaying = true;
}

void SamplerVoice::playing_did_finish()
{
    UI::get().set_voice_playing(_number, false);
    prime();
}

void SamplerVoice::render(int16_t * data, uint32_t frameCount)
{
    uint32_t i;

    // Get the current buffer if playing.
    SampleBuffer * voiceBuffer = nullptr;
    if (is_valid() && is_playing())
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

    float rate = _pitch;
    int16_t * bufferData = voiceBuffer->data;
    uint32_t readHead = voiceBuffer->readHead;
    uint32_t bufferFrameCount = voiceBuffer->frameCount;
    float s0 = 0.0f;
    float s1 = 0.0f;

    // Special case for unmodified pitch.
    if (rate == 1.0f)
    {
        uint32_t framesRemainingInBuffer = bufferFrameCount - readHead;
        uint32_t framesFromBuffer = min(framesRemainingInBuffer, frameCount);

        // Render sample data into the output buffer.
        for (i = 0; i < framesFromBuffer; ++i)
        {
            s1 = float(bufferData[readHead++]);

            // Apply the gain.
            float s = s1;
//             s /= 32768.0f;
//             s = _step * floor((s * _inverseStep) + 0.5);
            s *= _gain;
//             s *= 32768.0f;
            *data = int16_t(s);
            data += 2;
        }
    }
    else
    {
        float readHeadFraction = _fraction;
        s1 = _lastBufferLastSample;

        // Render sample data into the output buffer at the specified playback rate.
        for (i = 0; i < frameCount; ++i)
        {
            readHeadFraction += rate;
            if (readHeadFraction >= 1.0f)
            {
                uint32_t index = uint32_t(readHeadFraction);
                readHeadFraction -= index;
                readHead += index;

                // Check if we reached the end of this sample buffer.
                if (readHead >= bufferFrameCount)
                {
                    _manager.retire_buffer(voiceBuffer);

                    // If we're no longer playing, exit this loop.
                    if (!_isPlaying)
                    {
                        // Set some variables used outside the loop below.
                        readHead = voiceBuffer->readHead;
                        readHeadFraction = 0.0f;
                        s1 = 0.0f;
                        break;
                    }

                    // Get the next buffer and update locals.
                    voiceBuffer = _manager.get_current_buffer();
                    if (!voiceBuffer)
                    {
                        break;
                    }
                    bufferData = voiceBuffer->data;
                    readHead = voiceBuffer->readHead + (bufferFrameCount - readHead);
                    bufferFrameCount = voiceBuffer->frameCount;
                }

                s0 = s1;
                s1 = float(bufferData[readHead]);
            }

            // Perform simple linear interpolation, then apply the gain.
            float s = (s0 + readHeadFraction * (s1 - s0));
//             s /= 32768.0f;
//             s = _step * floor((s * _inverseStep) + 0.5);
            s *= _gain;
//             s *= 32768.0f;
            *data = int16_t(s);
            data += 2;
        }

        _fraction = readHeadFraction;
    }

    if (voiceBuffer)
    {
        voiceBuffer->readHead = readHead;
    }
    _lastBufferLastSample = s1;

    // Fill any leftover frames with silence.
    for (; i < frameCount; ++i)
    {
        *data = 0;
        data += 2;
    }

    // Did we finish this buffer?
    if (readHead >= bufferFrameCount && voiceBuffer)
    {
        _manager.retire_buffer(voiceBuffer);
    }

    if (_turnOnLedNextBuffer)
    {
        _turnOnLedNextBuffer = false;
        UI::get().set_voice_playing(_number, true);
    }
}

void SamplerVoice::set_bits(uint32_t bits)
{
    _step = 2.0f * pow(0.5f, bits);
    _inverseStep = 1.0f / _step;
}

void SamplerVoice::set_sample_start(float start)
{
    // Stop playing and turn off LED.
    _reset_voice();
    UI::get().set_voice_playing(_number, false);

    // Tell sample manager to set and load new start point.
    uint32_t sample = uint32_t(float(_manager.get_total_samples()) * start);
    _manager.set_start_sample(sample);
}

void SamplerVoice::set_sample_end(float end)
{
    // Stop playing and turn off LED.
    _reset_voice();
    UI::get().set_voice_playing(_number, false);

    uint32_t startSample = _manager.get_start_sample();
    uint32_t s = _manager.get_total_samples() - startSample;
    uint32_t sample = startSample + uint32_t(float(s) * end);
    _manager.set_end_sample(sample);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
