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
#if !defined(_CHANNEL_LED_H_)
#define _CHANNEL_LED_H_

#include <stdint.h>
#include "led.h"
#include "singleton.h"
#include "fsl_dspi.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Manager for bi-color channel LEDs.
 */
class ChannelLEDManager : public Singleton<ChannelLEDManager>
{
public:
    enum ChannelLedState : uint32_t
    {
        kOff = 0x00,    //!< 0b00 = off
        kRed = 0x01,    //!< 0b01 = red
        kYellow = 0x02, //!< 0b10 = yellow
    };

    ChannelLEDManager();
    ~ChannelLEDManager()=default;

    void init();

    void set_channel_state(uint32_t channel, ChannelLedState state)
    {
        uint32_t channelBitOffset = channel * 2;
        _editBuffer = (_editBuffer & ~(0x3 << channelBitOffset))
                        | (static_cast<uint8_t>(state) << channelBitOffset);
    }

    void flush();
    bool is_transferring() const { return _isTransferring; }

    void clear_is_transferring() { _isTransferring = false; }

protected:
    uint8_t _editBuffer;    //!< Buffer updated prior to flush.
    bool _isTransferring;   //!< Whether a transfer is in progress.

};

/*!
 * @brief Channel LED template.
 */
template <uint32_t channel>
class ChannelLED : public LEDBase
{
public:
    ChannelLED() : _state(false), _color(kRed) {}

    virtual void on() override
    {
        _state = true;
        _update();
    }

    virtual void off() override
    {
        _state = false;
        _update();
    }

    virtual void set_color(LEDColor color) override
    {
        _color = color;
        _update();
    }

    virtual bool is_on() override
    {
        return _state;
    }

    virtual void set_polarity(bool polarity) override
    {
        // No-op.
    }

protected:
    bool _state;
    LEDColor _color;

    void _update()
    {
        ChannelLEDManager::get().set_channel_state(channel,
            _state
            ? ((_color == LEDBase::kRed) ? ChannelLEDManager::kRed : ChannelLEDManager::kYellow)
            : ChannelLEDManager::kOff);
    }
};

} // namespace slab

#endif // _CHANNEL_LED_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
