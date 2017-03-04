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
#if !defined(_UI_H_)
#define _UI_H_

#include "argon/argon.h"
#include "led.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

enum UIMode : uint32_t
{
    kPlayMode,
    kEditMode,
};

enum UIEventType : uint32_t
{
    kButtonDown,
    kButtonUp,
};

enum UIEventSource : uint32_t
{
    kButton1,
    kButton2,
};

struct UIEvent
{
    UIEventType event;
    UIEventSource source;
    uint32_t value;
};

/*!
 * @brief
 */
class Button
{
public:
    Button(PORT_Type * port, GPIO_Type * gpio, uint32_t pin, UIEventSource source);
    ~Button()=default;

    void init();

    bool read();

protected:
    UIEventSource _source;
    PORT_Type * _port;
    GPIO_Type * _gpio;
    uint32_t _pin;
    bool _state;
    Ar::Timer _timer;

    void handle_irq();
    void handle_timer();

    static void irq_handler_stub(PORT_Type * port, uint32_t pin, void * userData);
    static void timer_cb_stub(Ar::Timer * timer, void * param);
};

/*!
 * @brief
 */
class UI
{
public:
    static UI & get() { return *s_ui; }

    UI();
    ~UI()=default;

    void init();
    void set_leds(LEDBase ** channelLeds, LEDBase * button1Led);

    void start();

    void send_event(const UIEvent& event) { _eventQueue.send(event); }

    Ar::RunLoop * get_runloop() { return &_runloop; }

protected:
    static const uint32_t kMaxEvents = 20;

    static UI * s_ui;

    Ar::ThreadWithStack<2048> _thread;
    Ar::RunLoop _runloop;
    Ar::StaticQueue<UIEvent, kMaxEvents> _eventQueue;
    Ar::Timer _ledTimer;
    LEDBase ** _channelLeds;
    LEDBase * _button1Led;
    Button _button1;
    Button _button2;
    UIMode _mode;

    void ui_thread();
};

} // namespace slab

#endif // _UI_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
