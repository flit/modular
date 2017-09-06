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
#include "ring_buffer.h"
#include "singleton.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Computes statistics for sample data reads.
 */
class ReaderStatistics
{
public:
    ReaderStatistics() : _history() {}
    ~ReaderStatistics()=default;

    void init();
    void add(uint32_t voiceNumber, uint32_t bufferNumber, uint32_t elapsed, uint32_t bytes, uint32_t offset);

protected:
    struct Entry
    {
        uint32_t timestamp;
        uint8_t voiceNumber;
        uint8_t bufferNumber;
        uint16_t bytes;
        uint32_t elapsed;
        uint32_t offset;
    };

    struct Statistics
    {
        uint32_t minimum;
        uint32_t maximum;
        uint32_t mean;
        uint32_t count;
        uint64_t sum;

        Statistics() : minimum(0xffffffff), maximum(0), mean(0), count(0), sum(0) {}

        template <bool doMinMax>
        void update(uint32_t value);
    };

    static const uint32_t kHistoryCount = 64;
    static const uint32_t kBinCount = 10;
    static const uint32_t kBinMax = 5000; // 5 ms

    RingBuffer<Entry, kHistoryCount> _history;
    Statistics _elapsed;
    Statistics _bins[kBinCount + 1];
    uint32_t _perBin;
};

/*!
 * @brief Thread to fill channel audio buffers with sample file data.
 *
 * The reader thread maintains a queue of voices that need a buffer filled. A given voice
 * may be put in the queue multiple times.
 */
class ReaderThread : public Singleton<ReaderThread>
{
public:
    ReaderThread();
    ~ReaderThread()=default;

    void init();
    void start() { _thread.resume(); }

    void enqueue(SamplerVoice * request);
    void clear_voice_queue(SamplerVoice * voice);
    void clear_all();

    uint32_t get_pending_count() const { return _count; }

protected:
    static const uint32_t kQueueSize = 16;

    int16_t _readBuf[SampleBufferManager::kBufferSize * 2];
    Ar::ThreadWithStack<2048> _thread;
    Ar::Semaphore _sem;
    Ar::Mutex _queueLock;

#if DEBUG
    ReaderStatistics _statistics;
    ReaderStatistics _voiceStatistics[kVoiceCount];
#endif

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
    void fill_buffer(SamplerVoice * voice);
    void fill_from_stereo(int16_t * data, uint32_t framesRead);

    SamplerVoice * dequeue();
    void insert_before(QueueNode * node, QueueNode * beforeNode);
    QueueNode * get_free_node();
    void add_free_node(QueueNode * node);
};

} // namespace slab

#endif // _READER_THREAD_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
