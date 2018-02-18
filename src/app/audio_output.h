/*
 * Copyright (c) 2015-2018 Immo Software
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
#if !defined(_AUDIO_OUT_H_)
#define _AUDIO_OUT_H_

#include "argon/argon.h"
#include "simple_queue.h"
#include "stack_sizes.h"
#include "fsl_edma.h"
#include "fsl_sai.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Quad-channel audio output port.
 *
 * Uses two transmit pins on the SAI peripheral to output 4 audio channels. Two
 * linked DMA channels are used to fill the SAI FIFOs from buffers provided by
 * the caller. Audio content is rendered on a thread owned by this object.
 */
class AudioOutput
{
public:
    struct Format
    {
        uint32_t bitsPerSample;
        uint32_t sampleRate_Hz;
        uint32_t oversampleRatio;
    };

    struct Buffer
    {
        uint8_t * data;
        size_t dataSize;
    };

    class Source
    {
    public:
        virtual void render(uint32_t firstChannel, Buffer & buffer)=0;
    };

    AudioOutput();
    ~AudioOutput()=default;

    void init(const Format& format);
    void add_buffer(Buffer * newBuffer);
    void set_source(Source * source) { m_source = source; }

    void start();

protected:

    enum : uint32_t {
        kChannelCount = 4,
        kChannelsPerBuffer = 2,
        kDmaChannelCount = kChannelCount / kChannelsPerBuffer,
        kMaxBufferCount = 8,
        kDmaQueueSize = 4,
        kFirstDmaChannel = 0,
    };

    typedef SimpleQueue<Buffer*, kMaxBufferCount> BufferQueue;

    struct DmaQueue {
        AudioOutput * owner;    //!< Pointer back to the owning object used by the DMA callback.
        edma_handle_t handle;   //!< DMA channel handle.
        edma_tcd_t tcd[kDmaQueueSize] __attribute__((aligned(32))); //!< TCD pool for eDMA transfer.
        BufferQueue queuedBuffers;    //!< Buffers owned by DMA transfers.
    };

    uint8_t m_bytesPerSample;   //!< Bytes in a sample.
    uint8_t m_minorLoopCount;   //!< The number of DMA transfers to fill the SAI FIFO.
    DmaQueue m_dma[kDmaChannelCount];
    Ar::Semaphore m_transferDone;
    Ar::ThreadWithStack<kAudioThreadStack> m_audioThread;
    Buffer m_buffers[kMaxBufferCount];
    BufferQueue m_freeBufferQueue;
    uint32_t m_bufferCount;
    Source * m_source;
    edma_tcd_t m_sendTcd __attribute__((aligned(32)));

    void audio_thread();

    void send(Buffer& buffer, uint32_t txChannel);

    status_t enqueue_tcd(edma_handle_t *handle, const edma_tcd_t *tcd);

    void dma_callback(DmaQueue * dmaQueue, bool done, uint32_t tcds);

    static void dma_callback_stub(edma_handle_t *handle, void *userData, bool done, uint32_t tcds);

};

} // namespace slab

#endif // _AUDIO_OUT_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
