/*
 * Copyright (c) 2018 Immo Software
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
#if !defined(_ITM_TRACE_H_)
#define _ITM_TRACE_H_

#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

//! Set to 1 to enable trace output via ITM.
#define ENABLE_TRACE (1)

namespace slab {

//! ITM channels.
enum : uint32_t
{
    kBufferedTimeChannel = 1,
    kBufferCountChannel = 2,
    kReaderQueueChannel = 3,
};

// Primary template.
template <uint32_t C, typename T>
struct itm
{
    static inline void send(T value) {}
};

#if ENABLE_TRACE
// Specialized for 32-bit write.
template <uint32_t C>
struct itm<C, uint32_t>
{
    static inline void send(uint32_t value)
    {
        // Skip sending if the port is disabled or full.
        if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && ITM->PORT[C].u32)
        {
            ITM->PORT[C].u32 = value;
        }
    }
};

// Specialized for 16-bit write.
template <uint32_t C>
struct itm<C, uint16_t>
{
    static inline void send(uint16_t value)
    {
        // Skip sending if the port is disabled or full.
        if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && ITM->PORT[C].u32)
        {
            ITM->PORT[C].u16 = value;
        }
    }
};

// Specialized for 8-bit write.
template <uint32_t C>
struct itm<C, uint8_t>
{
    static inline void send(uint8_t value)
    {
        // Skip sending if the port is disabled or full.
        if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && ITM->PORT[C].u32)
        {
            ITM->PORT[C].u8 = value;
        }
    }
};
#endif // ENABLE_TRACE

} // namespace slab

#endif // _ITM_TRACE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
