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

#include "button.h"
#include "ui.h"
#include "pin_irq_manager.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

Button::Button(PORT_Type * port, GPIO_Type * gpio, uint32_t pin, UIEventSource source, bool isInverted)
:   _source(source),
    _port(port),
    _gpio(gpio),
    _pin(pin),
    _isInverted(isInverted),
    _state(false),
    _timer(),
    _timeoutCount(0)
{
}

void Button::init()
{
    _timer.init("button", this, &Button::handle_timer, kArPeriodicTimer, 20);
    UI::get().get_runloop()->addTimer(&_timer);

    PinIrqManager::get().connect(_port, _pin, irq_handler_stub, this);
}

bool Button::read()
{
    uint32_t value = GPIO_ReadPinInput(_gpio, _pin);
    return (value == 0) ^ _isInverted;
}

void Button::handle_irq()
{
    // Restart timer.
    if (_timer.isActive())
    {
        _timer.stop();
    }

    // Configure timer for 20 ms debounce timeout and start it.
    _timeoutCount = 0;
    _timer.setDelay(20);
    _timer.start();
}

void Button::handle_timer(Ar::Timer * timer)
{
    bool isPressed = read();

    // Handle first timeout, for debounce.
    if (_timeoutCount == 0)
    {
        // We're debouncing and the button state did not change, so ignore the transition.
        if (isPressed == _state)
        {
            _timer.stop();
            return;
        }

        _state = isPressed;

        // Don't need the timer continuing unless button is pressed.
        if (!isPressed)
        {
            _timer.stop();
        }
        // Set timer period to 1 second after initial debounce.
        else
        {
            _timer.setDelay(1000);
        }
    }

    // Send event to UI.
    UIEvent event;
    if (isPressed)
    {
        if (_timeoutCount >= 1)
        {
            event.event = kButtonHeld;
        }
        else
        {
            event.event = kButtonDown;
        }
    }
    else
    {
        event.event = kButtonUp;
    }
    event.source = _source;
    event.value = _timeoutCount;
    UI::get().send_event(event);

    ++_timeoutCount;
}

void Button::irq_handler_stub(PORT_Type * port, uint32_t pin, void * userData)
{
    Button * _this = reinterpret_cast<Button *>(userData);
    assert(_this);
    _this->handle_irq();
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
