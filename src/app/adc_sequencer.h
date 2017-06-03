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
#if !defined(_ADC_SEQUENCER_H_)
#define _ADC_SEQUENCER_H_

#include "argon/argon.h"
#include "fsl_edma.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Manages DMA based ADC conversions.
 *
 * Uses two DMA channels to sequence conversions of an arbitrary selection of ADC channels.
 * After the object is created, call set_channels() and pass a mask of which ADC channels
 * should be converted. Then call set_result_buffer() to provide a uint32_t array to which
 * the ADC conversion results will be written. The array must be at least as large as the
 * number of channels selected. Next, set_semaphore() is used to specify a semaphore that
 * will be put when the sequence of conversions is completed and all result data is available.
 * Finally, call init() to complete initialization. At this point, you may call start() to
 * run a sequence of conversions.
 */
class AdcSequencer
{
public:

    AdcSequencer(ADC_Type * base, uint32_t firstDmaChannel);
    AdcSequencer(const AdcSequencer& other)=delete;
    ~AdcSequencer()=default;

    void set_channels(uint32_t channels) { _channels = channels; }
    void set_result_buffer(uint32_t * buffer) { _results = buffer; }
    void set_semaphore(Ar::Semaphore * sem) { _sem = sem; }

    void init();

    void start();

    void handle_completion();

protected:
    ADC_Type * _base;
    uint32_t _firstDmaChannel;
    uint32_t _channels;
    uint32_t * _results;
    Ar::Semaphore * _sem;
    edma_tcd_t _tcds[2] __attribute__((aligned(32)));
    uint32_t _requests[32];

#if DEBUG
    uint32_t _ts;
    uint32_t _lts;
    uint32_t _elapsed;
#endif // DEBUG
};

} // namespace slab

#endif // _ADC_SEQUENCER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
