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

#include "channel_gate.h"
#include <assert.h>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const uint32_t kAdcMax = 65535;
const float kThresholdPercent = 0.3f;
const uint32_t kThresholdAmount = kThresholdPercent * kAdcMax;
const uint32_t kRisingThreshold = kAdcMax - kThresholdAmount;
const uint32_t kFallingThreshold = kThresholdAmount;

const uint32_t kMinTransitionCounts = 3;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

ChannelGate::ChannelGate()
:   _last(-1),
    _state(kInitial),
    _transitionCount(0)
{
}

void ChannelGate::init()
{
}

ChannelGate::Event ChannelGate::process(uint32_t value)
{
    // Invert value to compensate for inverting opamp config;
    value = kAdcMax - value;

#if DEBUG
    _history.put(value);
#endif

    ChannelGate::Event result = kNone;

    int32_t filteredValue = (value > kRisingThreshold) ? 1 :
        ((value < kFallingThreshold) ? 0 : -1);

    bool risingEdge = (_last != 1) && (filteredValue == 1);
    bool fallingEdge = (_last != 0) && (filteredValue == 0);

    switch (_state)
    {
        case kInitial:
            _state = (filteredValue == 0) ? kLowStable :
                        ((filteredValue == 1) ? kHighStable :
                            kInitial);
            break;
        case kLowStable:
            if (risingEdge)
            {
                _state = kRising;
                _transitionCount = 0;
            }
            break;
        case kHighStable:
            if (fallingEdge)
            {
                _state = kFalling;
                _transitionCount = 0;
            }
            break;
        case kRising:
            if (filteredValue != 1)
            {
                _state = kLowStable;
            }
            else if (++_transitionCount >= kMinTransitionCounts)
            {
                _state = kHighStable;
                result = kNoteOn;
            }
            break;
        case kFalling:
            if (filteredValue != 0)
            {
                 _state = kHighStable;
            }
            else if (++_transitionCount >= kMinTransitionCounts)
            {
                _state = kLowStable;
                result = kNoteOff;
            }
            break;
        default:
            assert(0);
    }

    _last = value;

    return result;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
