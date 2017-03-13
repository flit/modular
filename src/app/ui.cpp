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

#include "ui.h"
#include "debug_log.h"
#include "pin_irq_manager.h"
#include "main.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include <assert.h>

using namespace slab;

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

UI * UI::s_ui = NULL;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

Button::Button(PORT_Type * port, GPIO_Type * gpio, uint32_t pin, UIEventSource source)
:   _source(source),
    _port(port),
    _gpio(gpio),
    _pin(pin),
    _state(false),
    _timer()
{
}

void Button::init()
{
    _timer.init("debounce", timer_cb_stub, this, kArOneShotTimer, 20);
    UI::get().get_runloop()->addTimer(&_timer);

    PinIrqManager::get().connect(_port, _pin, irq_handler_stub, this);
}

bool Button::read()
{
    uint32_t value = GPIO_ReadPinInput(_gpio, _pin);
    return (value != 0);
}

void Button::handle_irq()
{
    PORT_SetPinInterruptConfig(_port, _pin, kPORT_InterruptOrDMADisabled);

    if (!_timer.isActive())
    {
        _timer.start();
    }
}

void Button::handle_timer()
{
    bool value = read();

    UIEvent event;
    event.event = value ? kButtonUp : kButtonDown;
    event.source = _source;
    event.value = 0;
    UI::get().send_event(event);

    PORT_SetPinInterruptConfig(_port, _pin, kPORT_InterruptEitherEdge);
}

void Button::irq_handler_stub(PORT_Type * port, uint32_t pin, void * userData)
{
    Button * _this = reinterpret_cast<Button *>(userData);
    assert(_this);
    _this->handle_irq();
}

void Button::timer_cb_stub(Ar::Timer * timer, void * param)
{
    Button * _this = reinterpret_cast<Button *>(param);
    assert(_this);
    _this->handle_timer();
}

UI::UI()
:   _button1(PIN_BUTTON1_PORT, PIN_BUTTON1_GPIO, PIN_BUTTON1_BIT, kButton1),
    _button2(PIN_BUTTON2_PORT, PIN_BUTTON2_GPIO, PIN_BUTTON2_BIT, kButton2),
    _mode(kPlayMode)
{
}

void UI::init()
{
    s_ui = this;

    _thread.init("ui", this, &UI::ui_thread, kUIThreadPriority, kArSuspendThread);
    _runloop.init("ui", &_thread);
    _eventQueue.init("events");
    _runloop.addQueue(&_eventQueue, NULL, NULL);

    _button1.init();
}

void UI::set_leds(LEDBase ** channelLeds, LEDBase * button1Led)
{
    _channelLeds = channelLeds;
    _button1Led = button1Led;
}

void UI::start()
{
    _thread.resume();
}

void UI::ui_thread()
{
    while (true)
    {
        ar_queue_t * object;
        if (_runloop.run(kArInfiniteTimeout, (void **)&object) == kArRunLoopQueueReceived)
        {
            // _eventQueue is the only queue added to the runloop.
            assert(object == static_cast<ar_queue_t *>(&_eventQueue));

            UIEvent event = _eventQueue.receive();
            switch (event.event)
            {
                case kButtonDown:
//                     value = _button1.read();
//                     value ? _button1Led->off() : _button1Led->on();
                    break;

                case kButtonUp:
                    _button1Led->is_on() ? _button1Led->off() : _button1Led->on();
                    break;
            }
        }
    }
}

void UI::set_voice_playing(uint32_t voice, bool state)
{
    assert(voice < kVoiceCount);

    _voiceStates[voice] = state;
    if (_mode == kPlayMode)
    {
        _channelLeds[voice]->set(state);
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
