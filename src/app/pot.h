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
#if !defined(_POT_H_)
#define _POT_H_

#include "calibration.h"
#include "ring_buffer.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Processes ADC data for pots.
 */
class Pot
{
public:
    Pot();
    ~Pot()=default;

    void init(uint32_t number, const calibration::Points & points);

    //! @brief Set hysteresis percent that prevents readings from being sent to UI.
    void set_hysteresis(uint32_t percent);

    //! @brief Process ADC reading and pass to UI.
    void process(uint32_t value);

    //! @brief Get the pot's channel number.
    uint32_t get_number() const { return _number; }

protected:
    uint32_t _number;   //!< Channel number.
    float _offset;  //!< Calibration offset.
    float _scale;   //!< Calibration scale factor.
    float _out; //!< y^-1 value for filtering.
    uint32_t _last; //!< Previous filtered value for hysteresis.
    uint32_t _hysteresis;   //!< Hysteresis amount.
#if DEBUG
    RingBuffer<uint16_t, 128> _history;
#endif
};

} // namespace slab

#endif // _POT_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
