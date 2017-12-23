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

#include "pot.h"
#include "ui.h"
#include "utility.h"

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const uint32_t kAdcMax = 65535;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

Pot::Pot()
:   _number(0),
    _last(0),
    _hysteresis(0)
{
}

void Pot::init(uint32_t number)
{
    _number = number;
}

void Pot::set_hysteresis(uint32_t percent)
{
    _hysteresis = kAdcMax * percent / 100;
}

uint32_t Pot::process(uint32_t value)
{
#if DEBUG
    _history.put(value);
#endif

    // Set gain for this channel.
    if (value <= kAdcMax)
    {
        value = _avg.update(value);

        uint32_t hysLow = (_last > _hysteresis / 2) ? (_last - _hysteresis / 2) : 0;
        uint32_t hysHigh = min(_last + _hysteresis / 2, kAdcMax);

        if (value < hysLow || value > hysHigh)
        {
            _last = value;
            _hysteresis = (4) << 4;

            UI::get().pot_did_change(*this, value);
        }
    }

    return 0;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
