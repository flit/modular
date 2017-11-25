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

#include "channel_cv_gate.h"

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const uint32_t kAdcMax = 65536;
const uint32_t kTriggerThreshold = kAdcMax - (0.3 * (kAdcMax / 2));

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

ChannelCVGate::ChannelCVGate()
:   _mode(kGate),
    _last(0),
    _edge(false),
    _highCount(0)
{
}

void ChannelCVGate::init()
{
}

void ChannelCVGate::set_mode(Mode newMode)
{
    _mode = newMode;

    // Reset state variables.
    _last = 0;
    _edge = false;
    _highCount = 0;
}

uint32_t ChannelCVGate::process(uint32_t value)
{
    _history.put(value);

    uint32_t result = 0;

    if (_mode == Mode::kGate)
    {
        uint32_t state = (value > kTriggerThreshold) ? 1 : 0;

        if (state == 0)
        {
            _edge = false;
            _highCount = 0;
        }

        if (state == 1)
        {
            if (_last == 0)
            {
                _edge = true;
                _highCount = 0;
            }

            ++_highCount;

            if (_highCount == 3)
            {
                result = 1;
            }
        }

        _last = state;
    }
    else
    {
        result = value;
    }

    return result;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
