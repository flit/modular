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

#include "channel_cv.h"
#include "utility.h"
#include <math.h>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const float kAdcMax = 65535.0f;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

ChannelCV::ChannelCV()
:   _number(0),
    _offset(0.0f),
    _scale(0.0f),
    _out(0.0f)
{
}

void ChannelCV::init(uint32_t number, const calibration::Points & points)
{
    _number = number;
    _offset = float(points.low);
    _scale = (kAdcMax / 5.0f * 2.0f) / float(points.high - points.low);
    _scale = _scale * 5.0f / kAdcMax; // Incorporate v/oct scaling into scaling factor.
}

float ChannelCV::process(uint32_t value)
{
    // Invert value to compensate for inverting opamp config.
    value = kAdcMax - value;

#if DEBUG
    _history.put(value);
#endif

    // Apply calibration.
    float corrected = float(value) - _offset;
    corrected *= _scale; // Also scales from 0..1 to 0..5.
    constrain(corrected, 0.0f, 5.0f);

    // If the pitch is changing more than half a semitone, just jump to
    // the new value. Otherwise filter it.
    if (fabsf(_out - corrected) > (1.0f / 24.0))
    {
        _out = corrected;
    }
    else
    {
        _out += 0.2f * (corrected - _out);
    }

    // Transpose down two octaves.
    return _out - 2.0f;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
