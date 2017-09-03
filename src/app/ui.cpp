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
#include "utility.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include <assert.h>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const uint32_t kAdcMax = 65536;
const uint32_t kPotEditHysteresisPercent = 5;

//! Interval for checking SD card presence.
const uint32_t kCardDetectInterval_ms = 250;

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
    _timer.init("debounce", this, &Button::handle_timer, kArOneShotTimer, 20);
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

void Button::handle_timer(Ar::Timer * timer)
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

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Pot::Pot()
:   _last(0),
    _hysteresis(0)
{
}

void Pot::set_hysteresis(uint32_t percent)
{
    _hysteresis = kAdcMax * percent / 100;
}

uint32_t Pot::process(uint32_t value)
{
    _history.put(value);
    value <<= 4; // 12-bit to 16-bit

    // Set gain for this channel.
    if (value <= kAdcMax)
    {
        value = _avg.update(value);

        uint32_t hysLow = (_last > _hysteresis / 2) ? (_last - _hysteresis / 2) : 0;
        uint32_t hysHigh = min(_last + _hysteresis / 2, kAdcMax);

        if (value < hysLow || value > hysHigh)
        {
            _last = value;
            _hysteresis = 0;

            UI::get().pot_did_change(*this, value);
        }
    }

    return 0;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

UI::UI()
:   _button1(PIN_BUTTON1_PORT, PIN_BUTTON1_GPIO, PIN_BUTTON1_BIT, kButton1),
    _button2(PIN_BUTTON2_PORT, PIN_BUTTON2_GPIO, PIN_BUTTON2_BIT, kButton2),
    _mode(kPlayMode),
    _voiceStates{0},
    _editChannel(0)
{
}

void UI::init()
{
    _channelLedColor = kLEDOff;

    _thread.init("ui", this, &UI::ui_thread, kUIThreadPriority, kArSuspendThread);
    _runloop.init("ui", &_thread);

    _eventQueue.init("events");
    _runloop.addQueue(&_eventQueue, NULL, NULL);

    _blinkTimer.init("blink", this, &UI::handle_blink_timer, kArPeriodicTimer, 20);
    _potReleaseTimer.init("pot-release", this, &UI::handle_pot_release_timer, kArOneShotTimer, 20);

    _button1.init();
    _button2.init();

    set_channel_led_color(kLEDRed);
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

template <UIMode mode>
void UI::set_mode()
{
    uint32_t n;

    _mode = mode;

    // Switch to edit mode.
    if (mode == kEditMode)
    {
        _button1Led->on();

        // Turn off all LEDs.
        set_channel_led_color(kLEDYellow);

        // Select last channel being edited.
        _channelLeds[_editChannel]->on();
    }
    // Switch to play mode.
    else
    {
        _button1Led->off();

        // Restore channel LEDs to play state.
        set_channel_led_color(kLEDRed);
    }

    // Set hysteresis on all pots.
    for (n = 0; n < kVoiceCount; ++n)
    {
        _channelPots[n].set_hysteresis(kPotEditHysteresisPercent);
    }
}

void UI::ui_thread()
{
    while (true)
    {
        ar_runloop_result_t object;
        if (_runloop.run(kArInfiniteTimeout, &object) == kArRunLoopQueueReceived)
        {
            // _eventQueue is the only queue added to the runloop.
            assert(object.m_queue == static_cast<ar_queue_t *>(&_eventQueue));

            UIEvent event = _eventQueue.receive();
//             uint32_t n;
            switch (event.event)
            {
                case kButtonDown:
                    break;

                case kButtonUp:
                    switch (event.source)
                    {
                        case kButton1:
                            // Switch to edit mode.
                            if (_mode == kPlayMode)
                            {
                                set_mode<kEditMode>();
                            }
                            // Switch to play mode.
                            else
                            {
                                set_mode<kPlayMode>();
                            }
                            break;

                        case kButton2:
                            // Bank switch in play mode.
                            if (_mode == kPlayMode)
                            {

                            }
                            // In edit mode, select next channel to edit.
                            else
                            {
                                // Update edit LED.
                                _channelLeds[_editChannel]->off();
                                _editChannel = (_editChannel + 1) % kVoiceCount;
                                _channelLeds[_editChannel]->on();

                                // Set hysteresis on all pots.
                                uint32_t n;
                                for (n = 0; n < kVoiceCount; ++n)
                                {
                                    _channelPots[n].set_hysteresis(kPotEditHysteresisPercent);
                                }
                            }
                            break;

                        default:
                            break;
                    }
                    break;

                case kPotAdjusted:
//                  n = event.source - kPot1;
                    break;

                default:
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

void UI::handle_blink_timer(Ar::Timer * timer)
{
}

void UI::handle_pot_release_timer(Ar::Timer * timer)
{
}

void UI::pot_did_change(Pot& pot, uint32_t value)
{
    uint32_t potNumber = pot.n;
    float fvalue = float(value) / float(kAdcMax);

    // In play mode, the pots control the gain of their corresponding channel.
    if (get_mode() == kPlayMode)
    {
        g_voice[potNumber].set_gain(fvalue);
    }
    else
    {
        // In edit mode, each pot controls a different voice parameter of the selected channel.
        switch (potNumber)
        {
            case 0:
                // Shift value from 0..1 to 0.1..2.5
                fvalue = (fvalue * 2.4f) + 0.1f;
                g_voice[_editChannel].set_pitch(fvalue);
                break;
            case 1:
                // 0..1
                g_voice[_editChannel].set_sample_start(fvalue);
                break;
            case 2:
                // 0..1
                g_voice[_editChannel].set_sample_end(fvalue);
                break;
            case 3:
                break;
        }
    }
}

void UI::set_all_channel_leds(bool state)
{
    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        _channelLeds[i]->set(state);
    }
}

//! Perform safe channel LED color/state transitions. We have to be careful
//! to not enable both the N and P FETs simultaneously. And the transitions are
//! done such that switching colors also turns off all LEDs, so they don't all
//! suddenly switch on/off or flicker.
void UI::set_channel_led_color(LEDColor newColor)
{
    if (newColor == _channelLedColor)
    {
        return;
    }

    // off = P-FET off (1), N-FET off (0)
    // red = P-FET off (1), N-FET on (1), leds on = 1
    // yellow = P-FET on (0), N-FET off (0), leds on = 0
    switch (newColor)
    {
        // Turn off LEDs.
        case kLEDOff:
            switch (_channelLedColor)
            {
                // yellow -> off
                case kLEDYellow:
                    // Set LEDs high.
                    set_all_channel_leds(true);
                    break;
                // red -> off
                case kLEDRed:
                    // Set LEDs low.
                    set_all_channel_leds(false);
                    break;
                default:
                    assert(0 && "bad led transition");
            }
            // Turn off P-FET and N-FET.
            GPIO_SetPinsOutput(PIN_CHLEDP_GPIO, PIN_CHLEDP);
            GPIO_ClearPinsOutput(PIN_CHLEDN_GPIO, PIN_CHLEDN);
            break;

        // Switch to yellow.
        case kLEDYellow:
            switch (_channelLedColor)
            {
                // off -> yellow
                case kLEDOff:
                    break;
                // red -> yellow
                case kLEDRed:
                    // Set LEDs to red off state.
                    set_all_channel_leds(true);
                    // Turn off N-FET.
                    GPIO_ClearPinsOutput(PIN_CHLEDN_GPIO, PIN_CHLEDN);
                    break;
                default:
                    assert(0 && "bad led transition");
            }
            // Set LEDs to yellow off state.
            set_all_channel_leds(true);
            // Turn on P-FET.
            GPIO_ClearPinsOutput(PIN_CHLEDP_GPIO, PIN_CHLEDP);
            break;

        // Switch to red.
        case kLEDRed:
            switch (_channelLedColor)
            {
                // off -> red
                case kLEDOff:
                    break;
                // yellow -> red
                case kLEDYellow:
                    // Set LEDs to yellow off state.
                    set_all_channel_leds(true);
                    // Turn off P-FET.
                    GPIO_SetPinsOutput(PIN_CHLEDP_GPIO, PIN_CHLEDP);
                    break;
                default:
                    assert(0 && "bad led transition");
            }
            // Set LEDs to red off state.
            set_all_channel_leds(true);
            // Turn on N-FET.
            GPIO_ClearPinsOutput(PIN_CHLEDN_GPIO, PIN_CHLEDN);
            break;
        default:
            assert(0 && "bad led transition");
    }

    _channelLedColor = newColor;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
