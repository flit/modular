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

#include "reader_thread.h"
#include "samplbaer.h"
#include "ui.h"
#include "debug_log.h"

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void ReaderStatistics::init()
{
    // Set up bins.
    uint32_t i;
    uint32_t n = 0;
    _perBin = kBinMax / kBinCount;
    for (i = 0; i < kBinCount; ++i)
    {
        _bins[i].minimum = n;
        _bins[i].maximum = n + _perBin - 1;
        n += _perBin;
    }
    // Last bin is overflow.
    _bins[i].minimum = n;
    _bins[i].maximum = 0xffffffff;
}

template <bool doMinMax>
void ReaderStatistics::Statistics::update(uint32_t value)
{
    count += 1;
    sum += value;
    mean = sum / count;

    if (doMinMax)
    {
        if (value < minimum)
        {
            minimum = value;
        }
        if (value > maximum)
        {
            maximum = value;
        }
    }
}

void ReaderStatistics::add(uint32_t voiceNumber, uint32_t bufferNumber, uint32_t elapsed, uint32_t bytes, uint32_t offset)
{
    Entry e;
    e.timestamp = Microseconds::get();
    e.voiceNumber = voiceNumber;
    e.bufferNumber = bufferNumber;
    e.elapsed = elapsed;
    e.bytes = bytes;
    e.offset = offset;
    _history.put(e);

    _elapsed.update<true>(elapsed);

    uint32_t n = elapsed / _perBin;
    if (n > kBinCount)
    {
        n = kBinCount;
    }
    assert(n >= 0 && n < kBinCount + 1);
    _bins[n].update<false>(elapsed);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

ReadRequestQueue::ReadRequestQueue()
:   _queueLock(),
    _first(nullptr),
    _last(nullptr),
    _free(nullptr),
    _count(0)
{
}

void ReadRequestQueue::init()
{
    _queueLock.init("reader");

    // Put all queue nodes on the free list.
    clear_all();
}

void ReadRequestQueue::enqueue(SamplerVoice * request)
{
    Ar::Mutex::Guard guard(_queueLock);

    QueueNode * node = _get_free_node();
    assert(node);
    if (!node)
    {
        return;
    }

    node->voice = request;

    // Find where to insert. Insert in the earliest voice number sequence that is missing
    // this voice.
    QueueNode * insertPosition = _find_insert_position(request);
    _insert_before(node, insertPosition);

    // Increment semaphore count to match queue size.
    ++_count;
}

//! @brief Find where to insert.
//!
//! Insert in the earliest voice number sequence that is missing this voice.
ReadRequestQueue::QueueNode * ReadRequestQueue::_find_insert_position(SamplerVoice * request)
{
    uint32_t requestVoiceNumber = request->get_number();
    uint32_t requestVoiceMask = 1 << requestVoiceNumber;
    uint32_t thisSequenceVoicesMask = 0;
    QueueNode * iter = _first;
    while (iter)
    {
        assert(iter->voice);
        uint32_t iterVoiceNumber = iter->voice->get_number();
        uint32_t iterVoiceMask = 1 << iterVoiceNumber;

        // Starting a new sequence because we've seen this voice before?
        if (thisSequenceVoicesMask & iterVoiceMask)
        {
            // Was the request voice number present in the previous sequence?
            if (!(thisSequenceVoicesMask & requestVoiceMask))
            {
                // No, so insert the request prior to this node, at the end of the previous sequence.
                break;
            }

            thisSequenceVoicesMask = 0;
        }

        thisSequenceVoicesMask |= iterVoiceMask;

        iter = iter->next;
    }

    return iter;
}

void ReadRequestQueue::clear_voice(SamplerVoice * voice)
{
    Ar::Mutex::Guard guard(_queueLock);

    QueueNode * iter = _first;
    while (iter)
    {
        if (iter->voice == voice)
        {
            // Remove this node from the queue.
            if (iter == _first)
            {
                _first = iter->next;
            }
            if (iter == _last)
            {
                _last = iter->previous;
            }

            if (iter->previous)
            {
                iter->previous->next = iter->next;
            }
            if (iter->next)
            {
                iter->next->previous = iter->previous;
            }

            // Update count.
            --_count;

            // Add node to free list.
            QueueNode * temp = iter;
            iter = iter->next;
            _add_free_node(temp);
        }
        else
        {
            iter = iter->next;
        }
    }
}

void ReadRequestQueue::clear_all()
{
    Ar::Mutex::Guard guard(_queueLock);

    // Reset pointers and count.
    _first = nullptr;
    _last = nullptr;
    _free = nullptr;
    _count = 0;

    // Put all queue nodes on the free list.
    uint32_t i;
    for (i=0; i < kQueueSize; ++i)
    {
        _add_free_node(&_nodes[i]);
    }
}

void ReadRequestQueue::_insert_before(QueueNode * node, QueueNode * beforeNode)
{
    if (beforeNode)
    {
        node->next = beforeNode;
        node->previous = beforeNode->previous;
        if (beforeNode->previous)
        {
            beforeNode->previous->next = node;
        }
        beforeNode->previous = node;

        if (beforeNode == _first)
        {
            _first = node;
        }
    }
    else
    {
        // Insert at end of list, or the list is empty.
        node->previous = _last;
        node->next = nullptr;

        if (_last)
        {
            _last->next = node;
        }
        else
        {
            _first = node;
        }
        _last = node;
    }
}

SamplerVoice * ReadRequestQueue::dequeue()
{
    Ar::Mutex::Guard guard(_queueLock);

    QueueNode * node = _first;
    if (!node)
    {
        return nullptr;
    }

    --_count;

    SamplerVoice * request = node->voice;

    _first = node->next;
    if (_first)
    {
        _first->previous = nullptr;

        if (!_first->next)
        {
            _last = _first;
        }
    }
    else
    {
        _last = nullptr;
    }

    _add_free_node(node);

    return request;
}

ReadRequestQueue::QueueNode * ReadRequestQueue::_get_free_node()
{
    QueueNode * node = _free;
    if (node)
    {
        _free = node->next;
    }
    return node;
}

void ReadRequestQueue::_add_free_node(QueueNode * node)
{
    node->voice = nullptr;
    node->previous = nullptr;
    node->next = _free;
    _free = node;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

ReaderThread::ReaderThread()
:   _thread(),
    _sem(),
    _queue()
{
}

void ReaderThread::init()
{
    _thread.init("reader", this, &ReaderThread::reader_thread, kReaderThreadPriority, kArSuspendThread);
    _sem.init("reader", 0);
    _queue.init();

#if DEBUG
    _statistics.init();
    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        _voiceStatistics[i].init();
    }
#endif
}

void ReaderThread::enqueue(SamplerVoice * request)
{
    _queue.enqueue(request);
    _sem.put();
}

void ReaderThread::reader_thread()
{
    while (_sem.get() == kArSuccess)
    {
        // Pull a voice that needs a buffer filled from the queue.
        SamplerVoice * voice = _queue.dequeue();

        // The voice can be null if clear_voice_queue() was called, and the sem count
        // is higher than the number of queue entries.
        if (!voice)
        {
            continue;
        }

        fill_buffer(voice);
    }
}

void ReaderThread::fill_buffer(SamplerVoice * voice)
{
    assert(voice->is_valid());
    uint32_t start1 = Microseconds::get();

    // Ask the voice for the next buffer to fill. This may return null, which is valid.
    SampleBufferManager& manager = voice->get_buffer_manager();
    SampleBuffer * request = manager.get_empty_buffer();
    if (!request)
    {
        return;
    }
    DEBUG_PRINTF(FILL_MASK, "R: filling b%i v%lu @ %lu\r\n", request->number, voice->get_number(), request->startFrame);

    // Figure out how much data to read.
    Stream& stream = voice->get_audio_stream();
    uint32_t frameSize = voice->get_wave_file().get_frame_size();
    uint32_t channelCount = voice->get_wave_file().get_channels();
    uint32_t bytesToRead = request->frameCount * frameSize;
    assert(channelCount <= 2);
    assert(bytesToRead > 0);

    // Read mono files directly into the voice buffer, stereo into a temp buffer.
    void * targetBuffer = request->data;
    if (channelCount == 2)
    {
        targetBuffer = _readBuf;
        assert(bytesToRead <= sizeof(_readBuf));
    }

    // Read from sample file.
#if DEBUG
    uint32_t start = Microseconds::get();
#endif

    fs::error_t status;
    status = stream.seek(request->startFrame * frameSize);
    if (status != fs::kSuccess)
    {
        DEBUG_PRINTF(ERROR_MASK, "R: seek error (b%i v%lu)\r\n", request->number, voice->get_number());
        UI::get().send_event(UIEvent(kCardRemoved));
        return;
    }
    uint32_t bytesRead;
    status = stream.read(bytesToRead, targetBuffer, &bytesRead);
    if (status != fs::kSuccess)
    {
        DEBUG_PRINTF(ERROR_MASK, "R: read error = %lu (b%i v%lu)\r\n", status, request->number, voice->get_number());
        UI::get().send_event(UIEvent(kCardRemoved));
        return;
    }
    uint32_t stop = Microseconds::get();
    uint32_t framesRead = bytesRead / frameSize;

#if DEBUG
    uint32_t delta = stop - start;
    DEBUG_PRINTF(TIME_MASK, "R: read %lu bytes in %lu µs\r\n", bytesRead, delta);
    _statistics.add(voice->get_number(), request->number, delta, bytesRead, manager.get_samples_read());
    _voiceStatistics[voice->get_number()].add(voice->get_number(), request->number, delta, bytesRead, manager.get_samples_read());
#endif

    // For stereo data copy just the left channel into the voice buffer.
    if (channelCount == 2)
    {
        fill_from_stereo(request->data, framesRead);
    }

    // Set request buffer frameCount to the number of frames read. This will always be
    // the size of the buffer except when we hit EOF.
    request->frameCount = framesRead;

    // Push the filled buffer back to the voice's SBM.
    manager.enqueue_full_buffer(request);

    uint32_t stop1 = Microseconds::get();
    uint32_t delta1 = stop1 - start1;
    DEBUG_PRINTF(TIME_MASK, "R: %lu µs to fill b%i v%lu\r\n", delta1, request->number, voice->get_number());
}

void ReaderThread::fill_from_stereo(int16_t * data, uint32_t framesRead)
{
    uint32_t i;
    int16_t * buf = _readBuf;
    for (i = 0; i < framesRead; ++i)
    {
        *data++ = *buf;
        buf += 2;
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
