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
#include "debug_log.h"
#include "utility.h"

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
    _samplesPlayed(0),
    _samplesRead(0),
    _samplesQueued(0),
    _didReadFileStart(false)
{
}

void SampleBufferManager::init(SamplerVoice * voice)
{
    _voice = voice;
    _number = voice->get_number();
    _primeMutex.init("prime");

    int i;
    for (i = 0; i < kBufferCount; ++i)
    {
        _buffer[i].number = i;
        _buffer[i].state = SampleBuffer::State::kUnused;
        _buffer[i].data = &_bufferData[i * kBufferSize];
        _buffer[i].startFrame = 0;
        _buffer[i].frameCount = kBufferSize;
        _buffer[i].readHead = 0;
        _buffer[i].reread = false;
    }
}

void SampleBufferManager::set_file(uint32_t totalFrames)
{
    _totalSamples = totalFrames;
    _activeBufferCount = min(round_up_div(_totalSamples, kBufferSize), kBufferCount);
    _samplesPlayed = 0;
    _samplesRead = 0;
    _samplesQueued = 0;
    _didReadFileStart = false;
}

SamplerVoice::SamplerVoice()
:   _led(nullptr),
    _wav(),
    _data(),
    _manager(),
    _isPlaying(false),
    _turnOnLedNextBuffer(false),
    _gain(1.0f)
{
}

void SamplerVoice::init(uint32_t n)
{
    _number = n;
    _manager.init(this);
}

void SamplerVoice::set_file(WaveFile& file)
{
    _wav = file;
    _data = _wav.get_audio_data();
    _isPlaying = false;
    _manager.set_file(_data.get_frames());
}

void SamplerVoice::prime()
{
    _isPlaying = false;
    _manager.prime();
}

void SampleBufferManager::prime()
{
    Ar::Mutex::Guard guard(_primeMutex);
    DEBUG_PRINTF(QUEUE_MASK, "V%d: start prime\r\n", _number);

    // Reset state.
    _samplesPlayed = 0;
    _samplesRead = _didReadFileStart ? kBufferSize : 0;
    _samplesQueued = _samplesRead;

    // Clear buffers queues.
    _fullBuffers.clear();
    _emptyBuffers.clear();

    // Playing will start from the file start buffer.
    _currentBuffer = &_buffer[0];
    _buffer[0].readHead = 0;

    // Queue up the rest of the available buffers to be filled.
    int i = _didReadFileStart ? 1 : 0;
    for (; i < _activeBufferCount; ++i)
    {
        if (_buffer[i].state == SampleBuffer::State::kReading)
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%d: prime: marking b%d for reread\r\n", _number, i);
            _buffer[i].reread = true;
        }
        else
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%d: prime: queuing b%d for read\r\n", _number, i);
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
        DEBUG_PRINTF(RETRIG_MASK, "V%d: retrigger (@%d)\r\n", _number, _manager.get_samples_played());

        // Start playing over from file start.
        _manager.prime();

        _led->off();
        _turnOnLedNextBuffer = true;
    }
    else
    {
        _led->on();
    }
    _isPlaying = true;
}

void SamplerVoice::render(int16_t * data, uint32_t frameCount)
{
    SampleBuffer * voiceBuffer = nullptr;
    if (is_valid() && is_playing())
    {
        voiceBuffer = _manager.get_current_buffer();
    }

    int i;
    int16_t * out = data;

    if (voiceBuffer)
    {
        int16_t intSample;
        uint32_t readHead = voiceBuffer->readHead;
        uint32_t bufferFrameCount = voiceBuffer->frameCount;
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
            *out = int16_t(intSample * _gain);
            out += 2;
        }
        voiceBuffer->readHead = readHead;

        // Did we finish this buffer?
        if (readHead >= bufferFrameCount)
        {
            _manager.retire_buffer(voiceBuffer);

            if (_turnOnLedNextBuffer)
            {
                _turnOnLedNextBuffer = false;
                _led->on();
            }
            else if (_manager.get_samples_played() >= _manager.get_total_samples())
            {
                _led->off();
            }
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
    buffer->frameCount = min(kBufferSize, _totalSamples - _samplesQueued);
    buffer->reread = false;
    buffer->state = SampleBuffer::State::kFree;

    _emptyBuffers.put(buffer);
    ReaderThread::get()->enqueue(_voice);

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

    if (_samplesPlayed >= _totalSamples)
    {
        DEBUG_PRINTF(RETIRE_MASK, "V%d: retiring b%d; played %d (done)\r\n", _number, buffer->number, _samplesPlayed);

        prime();
    }
    else
    {
        DEBUG_PRINTF(RETIRE_MASK, "V%d: retiring b%d; played %d\r\n", _number, buffer->number, _samplesPlayed);

        // Don't queue up file start buffer for reading.
        if (buffer != &_buffer[0] && _samplesQueued < _totalSamples)
        {
            DEBUG_PRINTF(RETIRE_MASK|QUEUE_MASK, "V%d: retire: queue b%d to read @ %d\r\n", _number, buffer->number, _samplesPlayed);
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
        DEBUG_PRINTF(CURBUF_MASK, "V%d: current buffer = %d\r\n", _number, buffer->number);
        _currentBuffer = buffer;
        buffer->state = SampleBuffer::State::kPlaying;
    }
    else
    {
        DEBUG_PRINTF(ERROR_MASK, "V%d: *** NO READY BUFFERS ***\r\n", _number);
        Ar::_halt();
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
        _samplesRead += buffer->frameCount;
        buffer->state = SampleBuffer::State::kReady;
    }
    else
    {
        if (buffer->reread)
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%d: queuing b%d for reread\r\n", _number, buffer->number);
            queue_buffer_for_read(buffer);
        }
        else
        {
            _samplesRead += buffer->frameCount;

            buffer->state = SampleBuffer::State::kReady;
            DEBUG_PRINTF(QUEUE_MASK, "V%d: queuing b%d for play\r\n", _number, buffer->number);
            _fullBuffers.put(buffer);
        }
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
