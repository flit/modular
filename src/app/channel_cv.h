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
#if !defined(_CHANNEL_CV_H_)
#define _CHANNEL_CV_H_

#include <stdint.h>
#include "ring_buffer.h"
#include "calibration.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Processes ADC data for CV inputs.
 */
class ChannelCV
{
public:
    ChannelCV();
    ~ChannelCV()=default;

    void init(uint32_t number, const calibration::Points & points);

    //! @brief Convert ADC reading to V/oct.
    float process(uint32_t value);

    //! @brief Clear filter history.
    void reset();

    //! @brief Get the CV input's channel number.
    uint32_t get_number() const { return _number; }

protected:
    uint32_t _number;   //!< Channel number.
    float _offset;  //!< Calibration offset.
    float _scale;   //!< Calibration scale factor.
    float _out; //!< y^-1 value for filtering.
#if DEBUG
    RingBuffer<uint16_t, 128> _history;
#endif
};

} // namespace slab

#endif // _CHANNEL_CV_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
