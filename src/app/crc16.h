/*
 * Copyright 2017 NXP
 * Copyright (c) 2017 Immo Software.
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
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
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

#ifndef _CRC16_H_
#define _CRC16_H_

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////

namespace slab {

/*!
 * @brief ITU-CCITT CRC-16.
 *
 * This clas is intended to be used by chaining one or more calls to compute()
 * followed by a call to get().
 *
 * Example
 * @code
 * uint16_t crc = Crc16().compute(d0, l0).compute(d1, l1).get();
 * @endcode
 *
 * This implementation is slow but small in size.
 */
class Crc16
{
public:
    /*!
     * @brief Constructor.
     */
    Crc16() : _crc(0xffff) {}

    /*!
     * @brief Destructor
     */
    ~Crc16()=default;

    /*!
     * @brief Compute the CRC-16 over the provided data.
     *
     * Multiple calls to this method can be made to update the CRC with non-contiguous
     * blocks of data.
     *
     * @param[in] inputData Pointer to data used for crc16.
     * @param[in] dataLength Data length.
     */
    Crc16 & compute(const void *inputData, uint32_t lengthInBytes);

    /*!
     * @brief Retrieve the CRC-16 current result after processing the input data.
     */
    uint16_t get() const { return _crc; }

protected:
    uint16_t _crc;
};

} // namespace slab

#endif // _CRC16_H_
