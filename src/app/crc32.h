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
#if !defined(_CRC32_H_)
#define _CRC32_H_

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

namespace slab {

/*!
 * @brief Standard Ethernet CRC-32.
 *
 * This clas is intended to be used by chaining one or more calls to compute()
 * followed by a call to get().
 *
 * Example
 * @code
 * uint32_t crc = Crc32().compute(d0, l0).compute(d1, l1).get();
 * @endcode
 */
class Crc32
{
public:
    /*!
     * @brief Constructor.
     */
    Crc32();

    Crc32(const Crc32 & other)=delete;
    Crc32& operator=(const Crc32 & other)=delete;

    /*!
     * @brief Destructor
     */
    ~Crc32()=default;

    /*!
     * @brief Compute the CRC-32 over the provided data.
     *
     * Multiple calls to this method can be made to update the CRC with non-contiguous
     * blocks of data.
     *
     * @param[in] src Pointer to data used for crc16.
     * @param[in] dataLength Data length.
     */
    Crc32 & compute(const void *src, uint32_t lengthInBytes);

    /*!
     * @brief Retrieve the CRC-32 current result after processing the input data.
     */
    uint32_t get();

};

} // namespace slab

#endif // _CRC32_H_
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
