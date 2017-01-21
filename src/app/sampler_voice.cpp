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

void SamplerVoice::render(int16_t * data, uint32_t frameCount)
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
    _readFileStart(false),
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
}

void SamplerVoice::prime()
{
    Ar::Mutex::Guard guard(_primeMutex);
    DEBUG_PRINTF(QUEUE_MASK, "V%d: start prime\r\n", _number);

    // Reset state.
    _isPlaying = false;
    _samplesPlayed = 0;
    _samplesRead = _readFileStart ? kBufferSize : 0;
    _samplesQueued = _samplesRead;

    // Clear buffers queues.
    _fullBuffers.clear();
    _emptyBuffers.clear();

    // Playing will start from the file start buffer.
    _currentBuffer = &_buffer[0];
    _buffer[0].readHead = 0;

    // Queue up the rest of the available buffers to be filled.
    int i = _readFileStart ? 1 : 0;
    for (; i < _activeBufferCount; ++i)
    {
        if (_buffer[i].state == kBufferReading)
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
    buffer->startFrame = _samplesQueued;
    buffer->frameCount = min(kBufferSize, _totalSamples - _samplesQueued);
    buffer->reread = false;
    buffer->state = kBufferFree;

    _emptyBuffers.put(buffer);
    ReaderThread::get()->enqueue(this);

    _samplesQueued += buffer->frameCount;
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
    if (buffer == &_buffer[0])
    {
        _readFileStart = true;
        _samplesRead += buffer->frameCount;
        buffer->state = kBufferReady;
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

            buffer->state = kBufferReady;
            DEBUG_PRINTF(QUEUE_MASK, "V%d: queuing b%d for play\r\n", _number, buffer->number);
            _fullBuffers.put(buffer);
        }
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
