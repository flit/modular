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

const float kAdcMax = 65535.0f;
const float kMinHysteresis = 128.0f;
const float kMinHysteresisPercent = kMinHysteresis * 100.0f / kAdcMax;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

Pot::Pot()
:   _number(0),
    _scale(0.0f),
    _out(0.0f),
    _last(0.0f),
    _hysteresis(0.0f),
    _hysteresisLow(0.0f),
    _hysteresisHigh(0.0f)
{
}

void Pot::init(uint32_t number, const calibration::Points & points)
{
    _number = number;
    _offset = float(points.low);
    _scale = kAdcMax / float(points.high - points.low);
}

void Pot::set_hysteresis(float percent)
{
    _hysteresis = kAdcMax * percent / 100.0f;
    _hysteresisLow = max(_last - _hysteresis / 2.0f, 0.0f);
    _hysteresisHigh = min(_last + _hysteresis / 2.0f, kAdcMax);
}

void Pot::process(uint32_t value)
{
#if DEBUG
    _history.put(value);
#endif

    // Apply calibration.
    float corrected = float(value) - _offset;
    corrected *= _scale;
    constrain(corrected, 0.0f, kAdcMax);

    _out += 0.05f * (corrected - _out);
    float floatValue = _out;

    if (floatValue < _hysteresisLow || floatValue > _hysteresisHigh)
    {
        _last = floatValue;
        set_hysteresis(kMinHysteresisPercent);

        UI::get().pot_did_change(*this, floatValue);
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
