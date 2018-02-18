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

#include "adc_sequencer.h"
#include "debug_log.h"
#include "microseconds.h"
#include "fsl_adc16.h"
#include "fsl_dmamux.h"

using namespace slab;

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

static AdcSequencer * s_instance[2] = { 0 };

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

AdcSequencer::AdcSequencer(ADC_Type * base, uint32_t firstDmaChannel)
:   _base(base),
    _firstDmaChannel(firstDmaChannel),
    _channels(0),
    _results(nullptr)
{
    memset(_tcds, 0, sizeof(_tcds));
    memset(_requests, 0, sizeof(_requests));

#if DEBUG
    _ts = 0;
    _lts = 0;
    _elapsed = 0;
#endif // DEBUG
}

void AdcSequencer::init(const adc16_config_t & config)
{
    uint32_t instance = _base == ADC0 ? 0 : 1;
    s_instance[instance] = this;

    // Init and configure the ADC instance.
    ADC16_Init(_base, &config);
    ADC16_SetHardwareAverage(_base, kADC16_HardwareAverageCount16);
    ADC16_DoAutoCalibration(_base);
    ADC16_EnableDMA(_base, true);

    // Set up DMA muxing.
    DMAMUX_EnableChannel(DMAMUX0, _firstDmaChannel);
    DMAMUX_SetSource(DMAMUX0, _firstDmaChannel + 1, (_base == ADC0 ? kDmaRequestMux0ADC0 : kDmaRequestMux0ADC1) & 0xff);
    DMAMUX_EnableChannel(DMAMUX0, _firstDmaChannel + 1);

    // Fill in requests array from selected ADC channels.
    uint32_t channelCount = 0;
    uint32_t channel;
    for (channel = 0; channel < 32; ++channel)
    {
        // Check if this channel is enabled.
        if (_channels & (1 << channel))
        {
            // Set ADC SC1A register value.
            _requests[channelCount] = ADC_SC1_ADCH(channel);
            ++channelCount;
        }
    }

    // Configure first DMA channel to copy one _requests array entry to ADC SC1A register
    // for every minor loop.
    edma_transfer_config_t transfer;
    transfer.srcAddr = (uint32_t)&_requests[0];
    transfer.destAddr = (uint32_t)&_base->SC1[0];
    transfer.srcTransferSize = kEDMA_TransferSize4Bytes;
    transfer.destTransferSize = kEDMA_TransferSize4Bytes;
    transfer.srcOffset = 4;
    transfer.destOffset = 0;
    transfer.minorLoopBytes = 4;
    transfer.majorLoopCounts = channelCount;

    EDMA_TcdSetTransferConfig(&_tcds[0], &transfer, NULL);
    _tcds[0].SLAST = -(channelCount * 4);

    // Configure second DMA channel to copy ADC results register to _results array entry once
    // per minor loop. Minor channel linking is used to retrigger the first DMA channel, thus
    // causing another ADC conversion to be initiated. An interrupt will fire when the major
    // loop is complete.
    transfer.srcAddr = (uint32_t)&_base->R[0];
    transfer.destAddr = (uint32_t)_results;
    transfer.srcTransferSize = kEDMA_TransferSize4Bytes;
    transfer.destTransferSize = kEDMA_TransferSize4Bytes;
    transfer.srcOffset = 0;
    transfer.destOffset = 4;
    transfer.minorLoopBytes = 4;
    transfer.majorLoopCounts = channelCount;

    EDMA_TcdSetTransferConfig(&_tcds[1], &transfer, NULL);
    EDMA_TcdSetChannelLink(&_tcds[1], kEDMA_MinorLink, _firstDmaChannel);
    EDMA_TcdSetChannelLink(&_tcds[1], kEDMA_MajorLink, _firstDmaChannel);
    EDMA_TcdEnableInterrupts(&_tcds[1], kEDMA_MajorInterruptEnable);
    _tcds[1].DLAST_SGA = -(channelCount * 4);

    // Load TCDs into DMA channel registers.
    EDMA_InstallTCD(DMA0, _firstDmaChannel, &_tcds[0]);
    EDMA_InstallTCD(DMA0, _firstDmaChannel + 1, &_tcds[1]);

    // Enable the channels.
    EDMA_EnableChannelRequest(DMA0, _firstDmaChannel);
    EDMA_EnableChannelRequest(DMA0, _firstDmaChannel + 1);

    // Enable the DMA channel IRQ for the second channel.
    EnableIRQ(IRQn_Type(DMA0_IRQn + _firstDmaChannel + 1));
}

void AdcSequencer::start()
{
    EDMA_TriggerChannelStart(DMA0, _firstDmaChannel);
}

void AdcSequencer::handle_completion()
{
#if DEBUG
    _lts = _ts;
    _ts = Microseconds::get();
    _elapsed = _ts - _lts;
#endif // DEBUG

    assert(_sem);
    _sem->put();

    EDMA_ClearChannelStatusFlags(DMA0, _firstDmaChannel + 1, kEDMA_DoneFlag | kEDMA_ErrorFlag | kEDMA_InterruptFlag);
}

extern "C" void DMA3_IRQHandler()
{
    s_instance[0]->handle_completion();
}

extern "C" void DMA5_IRQHandler()
{
    s_instance[1]->handle_completion();
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
