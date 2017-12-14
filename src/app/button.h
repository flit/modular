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
#if !defined(_BUTTON_H_)
#define _BUTTON_H_

#include "argon/argon.h"
#include "ui_events.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief
 */
class Button
{
public:
    Button(PORT_Type * port, GPIO_Type * gpio, uint32_t pin, UIEventSource source, bool isInverted);
    ~Button()=default;

    void init();

    //! @brief Returns true if the button is pressed.
    bool read();

protected:
    UIEventSource _source;
    PORT_Type * _port;
    GPIO_Type * _gpio;
    uint32_t _pin;
    bool _isInverted;
    bool _state;
    Ar::TimerWithMemberCallback<Button> _timer;
    uint32_t _timeoutCount;

    void handle_irq();
    void handle_timer(Ar::Timer * timer);

    static void irq_handler_stub(PORT_Type * port, uint32_t pin, void * userData);
};

} // namespace slab

#endif // _BUTTON_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
