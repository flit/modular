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
#if !defined(_UI_EVENTS_H_)
#define _UI_EVENTS_H_

#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

enum UIEventType : uint16_t
{
    kButtonDown,
    kButtonUp,
    kButtonHeld,
    kPotAdjusted,
    kPotStopped,
    kCardInserted,
    kCardRemoved,
    kCardLowActivity,
    kSpecialFileDetected,
};

enum UIEventSource : uint16_t
{
    kNoEventSource,
    kButton1,
    kButton2,
    kPot1,
    kPot2,
    kPot3,
    kPot4
};

enum SpecialFile : int32_t
{
    kFirmwareUpdateFile,
    kRecalibrateCmdFile,
    kReportVersionCmdFile,
};

struct UIEvent
{
    UIEventType event;
    UIEventSource source;
    union {
        float floatValue;
        int32_t intValue;
    };

    UIEvent() {}
    UIEvent(UIEventType theEvent, UIEventSource theSource=kNoEventSource)
    :   event(theEvent),
        source(theSource)
    {
    }

    UIEvent& set_float_value(float theValue)
    {
        floatValue = theValue;
        return *this;
    }

    UIEvent& set_int_value(int32_t theValue)
    {
        intValue = theValue;
        return *this;
    }
};

} // namespace slab

#endif // _UI_EVENTS_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
