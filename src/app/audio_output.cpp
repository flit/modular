/*
 * Copyright (c) 2015-2017 Immo Software
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

#include "audio_output.h"
#include "fsl_dmamux.h"
#include <stdio.h>

using namespace slab;

#define EDMA_TRANSFER_ENABLED_MASK 0x80U

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

AudioOutput::AudioOutput()
:   m_bytesPerSample(0),
    m_minorLoopCount(0),
    m_transferDone(),
    m_audioThread(),
    m_freeBufferQueue(),
    m_bufferCount(0),
    m_source(nullptr)
{
}

void AudioOutput::init(const Format& format)
{
    m_bufferCount = 0;
    m_transferDone.init("txdone", 0);
    m_source = nullptr;

    // Route SAI TX request to the first DMA channel.
    DMAMUX_SetSource(DMAMUX0, kFirstDmaChannel, kDmaRequestMux0I2S0Tx & 0xff);
    DMAMUX_EnableChannel(DMAMUX0, kFirstDmaChannel);
    DMAMUX_EnableChannel(DMAMUX0, kFirstDmaChannel + 1);

    // Create DMA queues.
    uint32_t i;
    for (i = 0; i < kDmaChannelCount; ++i)
    {
        DmaQueue * dma = &m_dma[i];
        dma->owner = this;
        dma->queuedBuffers = BufferQueue();

        EDMA_CreateHandle(&dma->handle, DMA0, kFirstDmaChannel + i);
        EDMA_SetCallback(&dma->handle, dma_callback_stub, dma);

        memset(dma->tcd, 0, sizeof(dma->tcd));
        EDMA_InstallTCDMemory(&dma->handle, &dma->tcd[0], kDmaQueueSize);
    }

    // Init SAI module.
    sai_config_t saiConfig;
    SAI_TxGetDefaultConfig(&saiConfig);
    saiConfig.protocol = kSAI_BusLeftJustified;
    saiConfig.masterSlave = kSAI_Master;
    SAI_TxInit(I2S0, &saiConfig);

    // Configure the SAI audio format.
    sai_transfer_format_t saiFormat;
    saiFormat.bitWidth = static_cast<sai_word_width_t>(format.bitsPerSample);
    saiFormat.channel = 0U;
    saiFormat.sampleRate_Hz = static_cast<sai_sample_rate_t>(format.sampleRate_Hz);
    saiFormat.masterClockHz = format.oversampleRatio * format.sampleRate_Hz;
    saiFormat.protocol = kSAI_BusLeftJustified;
    saiFormat.stereo = kSAI_Stereo;
    saiFormat.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2;
    uint32_t mclkSourceClockHz = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    SAI_TxSetFormat(I2S0, &saiFormat, mclkSourceClockHz, saiFormat.masterClockHz);
    SAI_TxEnableInterrupts(I2S0, kSAI_FIFOErrorInterruptEnable);
    EnableIRQ(I2S0_Tx_IRQn);

    // Enable the second tx channel.
    I2S0->TCR3 |= I2S_TCR3_TCE(2);

    // Get the transfer size from format.
    m_bytesPerSample = saiFormat.bitWidth / 8;

    // Set the number of transfers per minor loop to half the amount needed to fill the FIFO
    // from the watermark.
    //
    // The SAI DMA request is asserted as long as either channel FIFO is below the watermark. This
    // causes the first DMA channel to retrigger its minor loop, which then retriggers the second
    // DMA channel's minor loop. The data from the extra minor loops was being lost because the SAI
    // FIFOs were already full.
    //
    // Setting the minor loops to transfer half as much data solves the issue by taking into account
    // that two minor loops will execute for each DMA request.
    m_minorLoopCount = (FSL_FEATURE_SAI_FIFO_COUNT - saiFormat.watermark) / 2;

    // Create audio thread.
    m_audioThread.init("audio", this, &AudioOutput::audio_thread, 180, kArSuspendThread);
}

void AudioOutput::add_buffer(Buffer * newBuffer)
{
    assert((m_bufferCount + 1) <= kMaxBufferCount);
    m_buffers[m_bufferCount] = *newBuffer;
    m_freeBufferQueue.put(&m_buffers[m_bufferCount]);
    ++m_bufferCount;
}

void AudioOutput::start()
{
    m_audioThread.resume();
}

void AudioOutput::audio_thread()
{
    assert(m_source);

    Buffer *buf;

    // Prime all buffers.
    uint32_t i;
    while (!m_freeBufferQueue.is_empty())
    {
        for (i = 0; i < kDmaChannelCount; ++i)
        {
            m_freeBufferQueue.get(buf);
            assert(buf);

            m_source->render(i * kChannelsPerBuffer, *buf);

            send(*buf, i);
        }
    }

    // Start DMA transfer.
    EDMA_StartTransfer(&m_dma[0].handle);

    // Enable SAI DMA and TX clock.
    SAI_TxEnableDMA(I2S0, kSAI_FIFORequestDMAEnable, true);
    SAI_TxEnable(I2S0, true);

    // Wait for buffers to complete, then refill and enqueue them.
    while (true)
    {
        for (i = 0; i < kDmaChannelCount; ++i)
        {
            // Block until the semaphore is put by the DMA completion callback.
            m_transferDone.get();

            m_freeBufferQueue.get(buf);
            assert(buf);

            m_source->render(i * kChannelsPerBuffer, *buf);

            send(*buf, i);
        }
    }
}

void AudioOutput::dma_callback(DmaQueue * dmaQueue, bool done, uint32_t tcds)
{
    // Put the audio buffer back in the free queue.
    Buffer * buf;
    dmaQueue->queuedBuffers.get(buf);
    m_freeBufferQueue.put(buf);

    // Release the thread.
    m_transferDone.put();
}

void AudioOutput::dma_callback_stub(edma_handle_t *handle, void *userData, bool done, uint32_t tcds)
{
    DmaQueue * dmaQueue = reinterpret_cast<DmaQueue*>(userData);
    assert(dmaQueue);
    assert(dmaQueue->owner);
    assert(handle == &dmaQueue->handle);
    dmaQueue->owner->dma_callback(dmaQueue, done, tcds);
}

void AudioOutput::send(Buffer& buffer, uint32_t txChannel)
{
    assert(buffer.data && buffer.dataSize);

    DmaQueue * dmaQueue = &m_dma[txChannel];

    // Place the buffer in the queue of buffers owned by this DMA.
    dmaQueue->queuedBuffers.put(&buffer);

    // Prepare edma transfer config.
    edma_transfer_config_t config = {0};
    uint32_t destAddr = SAI_TxGetDataRegisterAddress(I2S0, txChannel);
    EDMA_PrepareTransfer(&config,
        buffer.data,                // source addr
        m_bytesPerSample,            // source width
        (void *)destAddr,           // dest addr
        m_bytesPerSample,            // dest width
        m_minorLoopCount * m_bytesPerSample,  // bytes per request
        buffer.dataSize,            // total bytes
        kEDMA_MemoryToPeripheral);  // mode

    // Fill in TCD.
    EDMA_TcdReset(&m_sendTcd);
    EDMA_TcdSetTransferConfig(&m_sendTcd, &config, NULL);

    // Link channel 1 to channel 0.
    if (txChannel == 0)
    {
        EDMA_TcdSetChannelLink(&m_sendTcd, kEDMA_MinorLink, kFirstDmaChannel + 1);
        EDMA_TcdSetChannelLink(&m_sendTcd, kEDMA_MajorLink, kFirstDmaChannel + 1);
    }

    // Add TCD to DMA queue.
    enqueue_tcd(&dmaQueue->handle, &m_sendTcd);
}

//! This is a modified EDMA_SubmitTransfer() that takes a predefined TCD instead of a
//! edma_transfer_config_t struct.
status_t AudioOutput::enqueue_tcd(edma_handle_t *handle, const edma_tcd_t *tcd)
{
    assert(handle != NULL);

    volatile edma_tcd_t *tcdRegs = (volatile edma_tcd_t *)&handle->base->TCD[handle->channel];

    uint32_t primask;
    uint32_t csr;
    int8_t currentTcdIndex;
    int8_t previousTcdIndex;
    int8_t nextTcdIndex;
    edma_tcd_t * currentTcd;
    edma_tcd_t * nextTcd;
    edma_tcd_t * previousTcd;

    /* Check if tcd pool is full. */
    primask = DisableGlobalIRQ();
    uint32_t tempUsed = handle->tcdUsed;
    uint32_t tempSize = handle->tcdSize;
    if (tempUsed >= tempSize)
    {
        EnableGlobalIRQ(primask);

        return kStatus_EDMA_QueueFull;
    }
    currentTcdIndex = handle->tail;
    handle->tcdUsed++;

    /* Calculate index of next TCD */
    nextTcdIndex = currentTcdIndex + 1U;
    if (nextTcdIndex == handle->tcdSize)
    {
        nextTcdIndex = 0U;
    }

    /* Advance queue tail index */
    handle->tail = nextTcdIndex;
    EnableGlobalIRQ(primask);

    /* Calculate index of previous TCD */
    previousTcdIndex = currentTcdIndex ? currentTcdIndex - 1U : handle->tcdSize - 1U;

    currentTcd = &handle->tcdPool[currentTcdIndex];
    nextTcd = &handle->tcdPool[nextTcdIndex];
    previousTcd = &handle->tcdPool[previousTcdIndex];

    /* Configure current TCD block. */
    memcpy(currentTcd, tcd, sizeof(edma_tcd_t));

    /* Enable major interrupt */
    currentTcd->CSR |= DMA_CSR_INTMAJOR_MASK;

    /* Link current TCD with next TCD for identification of current TCD */
    currentTcd->DLAST_SGA = (uint32_t)nextTcd;

    /* Chain from previous descriptor unless tcd pool size is 1(this descriptor is its own predecessor). */
    if (currentTcd != previousTcd)
    {
        /* Enable scatter/gather feature in the previous TCD block. */
        csr = (previousTcd->CSR | DMA_CSR_ESG_MASK) & ~DMA_CSR_DREQ_MASK;
        previousTcd->CSR = csr;

        /*
            Check if the TCD block in the registers is the previous one (points to current TCD block). It
            is used to check if the previous TCD linked has been loaded in TCD register. If so, it need to
            link the TCD register in case link the current TCD with the dead chain when TCD loading occurs
            before link the previous TCD block.
        */
        if (tcdRegs->DLAST_SGA == (uint32_t)currentTcd)
        {
            /* Enable scatter/gather also in the TCD registers. */
            csr = (tcdRegs->CSR | DMA_CSR_ESG_MASK) & ~DMA_CSR_DREQ_MASK;

            /* Must write the CSR register one-time, because the transfer maybe finished anytime. */
            tcdRegs->CSR = csr;

            /*
                It is very important to check the ESG bit!
                Because this hardware design: if DONE bit is set, the ESG bit can not be set. So it can
                be used to check if the dynamic TCD link operation is successful. If ESG bit is not set
                and the DLAST_SGA is not the next TCD address(it means the dynamic TCD link succeed and
                the current TCD block has been loaded into TCD registers), it means transfer finished
                and TCD link operation fail, so must install TCD content into TCD registers and enable
                transfer again. And if ESG is set, it means transfer has notfinished, so TCD dynamic
                link succeed.
            */
            if (tcdRegs->CSR & DMA_CSR_ESG_MASK)
            {
                return kStatus_Success;
            }

            /*
                Check whether the current TCD block is already loaded in the TCD registers. It is another
                condition when ESG bit is not set: it means the dynamic TCD link succeed and the current
                TCD block has been loaded into TCD registers.
            */
            if (tcdRegs->DLAST_SGA == (uint32_t)nextTcd)
            {
                return kStatus_Success;
            }

            /*
                If go to this, means the previous transfer finished, and the DONE bit is set.
                So shall configure TCD registers.
            */
        }
        else if (tcdRegs->DLAST_SGA != 0)
        {
            /* The current TCD block has been linked successfully. */
            return kStatus_Success;
        }
        else
        {
            /*
                DLAST_SGA is 0 and it means the first submit transfer, so shall configure
                TCD registers.
            */
        }
    }

    /* Push tcd into hardware TCD register */
    EDMA_InstallTCD(handle->base, handle->channel, currentTcd);

    /* Enable channel request again. */
    if (handle->flags & EDMA_TRANSFER_ENABLED_MASK)
    {
        handle->base->SERQ = DMA_SERQ_SERQ(handle->channel);
    }

    return kStatus_Success;
}

extern "C" void I2S0_Tx_IRQHandler()
{
    // Check for error flag and clear it.
    if (I2S0->TCSR & I2S_TCSR_FEF_MASK)
    {
        // Reset FIFOs.
        I2S0->TCSR |= I2S_TCSR_FR_MASK;
        // Clear error flag.
        I2S0->TCSR |= I2S_TCSR_FEF_MASK;
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
