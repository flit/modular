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
#if !defined(_LED_H_)
#define _LED_H_

#include <stdint.h>
#include "fsl_gpio.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Abstract LED base class.
 */
class LEDBase
{
public:

    virtual void on()=0;
    virtual void off()=0;
    void set(bool state) { state ? on() : off(); }
    virtual bool is_on()=0;
    virtual void set_polarity(bool polarity)=0;
};

/*!
 * @brief LED template.
 */
template <uint32_t gpio, uint32_t pin>
class LED : public LEDBase
{
public:
    LED() : _state(false), _polarity(false) {}

    virtual void on() override
    {
        GPIO_WritePinOutput((GPIO_Type *)gpio, pin, true ^ _polarity);
        _state = true;
    }

    virtual void off() override
    {
        GPIO_WritePinOutput((GPIO_Type *)gpio, pin, false ^ _polarity);
        _state = false;
    }

    virtual bool is_on() override
    {
        return _state;
    }

    virtual void set_polarity(bool polarity) override
    {
        _polarity = polarity;
    }

protected:
    bool _state;
    bool _polarity;
};

} // namespace slab

#endif // _LED_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
