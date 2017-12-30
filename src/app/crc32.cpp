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
#include "crc32.h"
#include "fsl_device_registers.h"
#include "fsl_clock.h"
#include <assert.h>

using namespace slab;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

Crc32::Crc32()
{
    CLOCK_EnableClock(kCLOCK_Crc0);
    const uint32_t ctrl = CRC_CTRL_TCRC_MASK    // 32-bit CRC
                        | CRC_CTRL_TOT(2)       // transpose both bits and bytes on write
                        | CRC_CTRL_TOTR(2)      // transpose both bits and bytes on read
                        | CRC_CTRL_FXOR_MASK;   // complement reads
    CRC0->CTRL = ctrl;
    CRC0->GPOLY = 0x04c11db7U;  // Standard Ethernet CRC-32 polynomial.
    CRC0->CTRL = ctrl | CRC_CTRL_WAS_MASK;
    CRC0->DATA = 0xffffffffU;   // Seed for Ethernet CRC-32.
    CRC0->CTRL = ctrl;
}

Crc32 & Crc32::compute(const void *inputData, uint32_t lengthInBytes)
{
    if (lengthInBytes)
    {
        assert(inputData);

        // Process unaligned bytes at start.
        const uint8_t * src = reinterpret_cast<const uint8_t *>(inputData);
        uint32_t byteCount = reinterpret_cast<uint32_t>(inputData) & 0x3;
        lengthInBytes -= byteCount;
        while (byteCount--)
        {
            CRC0->ACCESS8BIT.DATALL = *src++;
        }

        // Process aligned words.
        const uint32_t * src32 = reinterpret_cast<const uint32_t *>(src);
        uint32_t wordCount = lengthInBytes / sizeof(uint32_t);
        lengthInBytes -= wordCount * sizeof(uint32_t);
        while (wordCount--)
        {
            CRC0->DATA = *src32++;
        }

        // Process trailing unaligned bytes.
        src = reinterpret_cast<const uint8_t *>(src32);
        while (lengthInBytes--)
        {
            CRC0->ACCESS8BIT.DATALL = *src++;
        }
    }

    return *this;
}

uint32_t Crc32::get()
{
    return CRC0->DATA;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
