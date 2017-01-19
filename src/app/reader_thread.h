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
#if !defined(_READER_THREAD_H_)
#define _READER_THREAD_H_

#include "argon/argon.h"
#include "sampler_voice.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Thread to fill channel audio buffers with sample file data.
 *
 * The reader thread maintains a queue of voices that need a buffer filled. A given voice
 * may be put in the queue multiple times.
 */
class ReaderThread
{
public:
    static ReaderThread * get() { return s_readerInstance; }

    ReaderThread();
    ~ReaderThread()=default;

    void start() { _thread.resume(); }

    void enqueue(SamplerVoice * request);

    uint32_t get_pending_count() const { return _count; }

protected:
    static const uint32_t kQueueSize = 16;

    static ReaderThread * s_readerInstance;

    int16_t _readBuf[SamplerVoice::kBufferSize * 2];
    Ar::ThreadWithStack<2048> _thread;
    Ar::Semaphore _sem;
    Ar::Mutex _queueLock;

    struct QueueNode
    {
        SamplerVoice * voice;
        QueueNode * next;
        QueueNode * previous;
    };

    QueueNode _nodes[kQueueSize];
    QueueNode * _first;
    QueueNode * _last;
    QueueNode * _free;
    uint32_t _count;

    void reader_thread();

    SamplerVoice * dequeue();
    QueueNode * get_free_node();
    void add_free_node(QueueNode * node);
};

} // namespace slab

#endif // _READER_THREAD_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
