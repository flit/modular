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
#include "fsl_sd_disk.h"
#include <assert.h>
#include <math.h>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const uint32_t kAdcMax = 65536;
const float kAdcLsbFloat = 1.0f / 4096.0f;
const uint32_t kPotEditHysteresisPercent = 5;

//! Interval for checking SD card presence.
const uint32_t kCardDetectInterval_ms = 250;

//! Delay for applying a change to the sample start parameter.
const uint32_t kSampleStartPotReleaseDelay_ms = 100;

const int32_t kButton1LedDutyCycleDelta = 1;

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
    bool value = read();

    // Handle first timeout, for debounce.
    if (_timeoutCount == 0)
    {
        // We're debouncing and the button state did not change, so ignore the transition.
        if (value == _state)
        {
            _timer.stop();
            return;
        }

        _state = value;

        // Don't need the timer continuing unless button is pressed.
        if (!value)
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
    event.event = (_timeoutCount >= 1) ? kButtonHeld : (value ? kButtonDown : kButtonUp);
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
//     value <<= 4; // 12-bit to 16-bit

    // Set gain for this channel.
    if (value <= kAdcMax)
    {
        value = _avg.update(value);

        uint32_t hysLow = (_last > _hysteresis / 2) ? (_last - _hysteresis / 2) : 0;
        uint32_t hysHigh = min(_last + _hysteresis / 2, kAdcMax);

        if (value < hysLow || value > hysHigh)
        {
            _last = value;
            _hysteresis = (4) << 4;

            UI::get().pot_did_change(*this, value);
        }
    }

    return 0;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

UI::UI()
:   _button1(PIN_BUTTON1_PORT, PIN_BUTTON1_GPIO, PIN_BUTTON1_BIT, kButton1, true),
    _button2(PIN_BUTTON2_PORT, PIN_BUTTON2_GPIO, PIN_BUTTON2_BIT, kButton2, false),
    _mode(kNoCardMode),
    _voiceStates{0},
    _editChannel(0),
    _isCardPresent(false),
    _debounceCardDetect(false),
    _lastSampleStart(-1.0f),
    _selectedBank(0),
    _button1LedDutyCycle(0),
    _button1LedDutyCycleDelta(kButton1LedDutyCycleDelta),
    _firstSwitchToPlayMode(true)
{
}

void UI::init()
{
    _channelLedColor = kLEDOff;

    // Set LED FETs to a known state.
    GPIO_WritePinOutput(PIN_CHLEDN_GPIO, PIN_CHLEDN_BIT, 0);
    GPIO_WritePinOutput(PIN_CHLEDP_GPIO, PIN_CHLEDP_BIT, 1);

    // Invert polarity of channel LEDs.
    set_all_channel_leds_polarity(true);

    // Create UI thread and its runloop.
    _thread.init("ui", this, &UI::ui_thread, kUIThreadPriority, kArSuspendThread);
    _runloop.init("ui", &_thread);

    // Create event queue and add it to the runloop.
    _eventQueue.init("events");
    _runloop.addQueue(&_eventQueue, NULL, NULL);

    // Create LED blink timer.
    _blinkTimer.init("blink", this, &UI::handle_blink_timer, kArPeriodicTimer, 50);
    _runloop.addTimer(&_blinkTimer);
    _blinkTimer.start();

    // Create pot edit release timer.
    _potReleaseTimer.init("pot-release", this, &UI::handle_pot_release_timer, kArOneShotTimer, kSampleStartPotReleaseDelay_ms);
    _runloop.addTimer(&_potReleaseTimer);

    // Set up card detection timer.
    _cardDetectTimer.init("card-detect", this, &UI::handle_card_detect_timer, kArPeriodicTimer, kCardDetectInterval_ms);
    _runloop.addTimer(&_cardDetectTimer);
    _cardDetectTimer.start();

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

    if (_mode == mode)
    {
        return;
    }

    _mode = mode;

    switch (mode)
    {
        // Switch to edit mode.
        case kEditMode:
            _button1Led->on();

            // Turn off all LEDs.
            set_channel_led_color(kLEDYellow);

            set_all_channel_leds(false);

            // Select last channel being edited.
            _channelLeds[_editChannel]->on();

            _lastSampleStart = -1.0f;
            break;

        // Switch to play mode.
        case kPlayMode:
            _button1Led->off();

            // Restore channel LEDs to play state.
            set_channel_led_color(kLEDRed);

            // Restore LEDs to current voice state.
            for (n = 0; n < kVoiceCount; ++n)
            {
                _channelLeds[n]->set(_voiceStates[n]);
            }

            if (_firstSwitchToPlayMode)
            {
                _firstSwitchToPlayMode = false;

                // Set no hysteresis on all pots.
                for (n = 0; n < kVoiceCount; ++n)
                {
                    _channelPots[n].set_hysteresis(0);
                }
                return;
            }
            break;

        // Switch to the no-card mode.
        case kNoCardMode:
            _button1Led->off();
            set_all_channel_leds(false);
            break;
    }

    // Set hysteresis on all pots.
    for (n = 0; n < kVoiceCount; ++n)
    {
        _channelPots[n].set_hysteresis(kPotEditHysteresisPercent);
    }
}

void UI::ui_thread()
{
    uint32_t n;
    while (true)
    {
        ar_runloop_result_t object;
        if (_runloop.run(kArInfiniteTimeout, &object) == kArRunLoopQueueReceived)
        {
            // _eventQueue is the only queue added to the runloop.
            assert(object.m_queue == static_cast<ar_queue_t *>(&_eventQueue));

            UIEvent event = _eventQueue.receive();
            switch (event.event)
            {
                case kButtonDown:
                    break;

                case kButtonUp:
                    switch (event.source)
                    {
                        case kButton1:
                            // Only allow mode switches if the card is inserted.
                            switch (_mode)
                            {
                                // Switch to edit mode.
                                case kPlayMode:
                                    set_mode<kEditMode>();
                                    break;

                                // Switch to play mode.
                                case kEditMode:
                                    set_mode<kPlayMode>();
                                    break;

                                // Do nothing.
                                case kNoCardMode:
                                    break;
                            }
                            break;

                        case kButton2:
                            switch (_mode)
                            {
                                // Bank switch in play mode.
                                case kPlayMode:
                                    if (++_selectedBank >= kVoiceCount)
                                    {
                                        _selectedBank = 0;
                                    }
                                    load_sample_bank(_selectedBank);
                                    break;

                                // In edit mode, select next channel to edit.
                                case kEditMode:
                                    // Update edit LED.
                                    _channelLeds[_editChannel]->off();
                                    _editChannel = (_editChannel + 1) % kVoiceCount;
                                    _channelLeds[_editChannel]->on();

                                    // Set hysteresis on all pots.
                                    for (n = 0; n < kVoiceCount; ++n)
                                    {
                                        _channelPots[n].set_hysteresis(kPotEditHysteresisPercent);
                                    }
                                    break;

                                // Ignore button2 if we don't have a card present.
                                case kNoCardMode:
                                    break;
                            }
                            break;

                        default:
                            break;
                    }
                    break;

                case kButtonHeld:
                    switch (event.source)
                    {
                        case kButton1:
                            if (event.value == 5)
                            {
                                _blinkTimer.start();
                            }
                            break;

                        case kButton2:
                            break;

                        default:
                            break;
                    }
                    break;

                case kPotAdjusted:
//                  n = event.source - kPot1;
                    break;

                case kCardInserted:
                    if (!_isCardPresent)
                    {
                        if (FileManager::get().mount())
                        {
                            _blinkTimer.stop();
                            FileManager::get().scan_for_files();
                            _isCardPresent = true;
                            set_mode<kPlayMode>();

                            _selectedBank = 0;
                            load_sample_bank(_selectedBank);
                        }
                        _cardDetectTimer.start();
                    }
                    break;

                case kCardRemoved:
                    if (_isCardPresent)
                    {
                        // Clear files in all voices. Each voice will clear pending buffers for
                        // itself from the reader thread's queue.
                        for (n = 0; n < kVoiceCount; ++n)
                        {
                            g_voice[n].clear_file();
                            set_voice_playing(n, false);
                        }
                        FileManager::get().unmount();
                        SD_HostReset(&g_sd.host);
                        _isCardPresent = false;
                        _cardDetectTimer.start();

                        set_mode<kNoCardMode>();
                        _button1LedDutyCycle = 0;
                        _button1LedDutyCycleDelta = kButton1LedDutyCycleDelta;
                        _blinkTimer.start();
                    }
                    break;

                default:
                    break;
            }
        }
    }
}

void UI::load_sample_bank(uint32_t bankNumber)
{
    SampleBank & bank = FileManager::get().get_bank(bankNumber);

    uint32_t channel;
    for (channel = 0; channel < kVoiceCount; ++channel)
    {
        set_voice_playing(channel, false);
        if (bank.has_sample(channel))
        {
            bank.load_sample_to_voice(channel, g_voice[channel]);
        }
        else
        {
            g_voice[channel].clear_file();
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
    // Update button1 LED duty cycle.
    _button1LedDutyCycle += _button1LedDutyCycleDelta;
    if (_button1LedDutyCycle <= 0)
    {
        _button1LedDutyCycle = 1;
        _button1LedDutyCycleDelta = kButton1LedDutyCycleDelta;
    }
    else if (_button1LedDutyCycle >= 100)
    {
        _button1LedDutyCycle = 99;
        _button1LedDutyCycleDelta = -kButton1LedDutyCycleDelta;
    }
    _button1Led->set_duty_cycle(_button1LedDutyCycle);
}

void UI::handle_pot_release_timer(Ar::Timer * timer)
{
    _channelPots[kSampleStartPot].set_hysteresis(1);
    g_voice[_editChannel].set_sample_start(_lastSampleStart);
    _lastSampleStart = -1.0f;
}

void UI::handle_card_detect_timer(Ar::Timer * timer)
{
    // Detect change in card presence. If a change is detected, set the debounce flag
    // and exit. We'll process the debounce next time this timer fires.
    bool isPresent = CardManager::get().check_presence();
    if (!_debounceCardDetect)
    {
        // Set debounce flag if a change is presence is detected.
        _debounceCardDetect = (isPresent != _isCardPresent);
    }
    else
    {
        _debounceCardDetect = false;

        // Card inserted.
        if (isPresent && !_isCardPresent)
        {
            _cardDetectTimer.stop();
            send_event(UIEvent(kCardInserted));
        }
        // Card removed.
        else if (!isPresent && _isCardPresent)
        {
            _cardDetectTimer.stop();
            send_event(UIEvent(kCardRemoved));
        }
    }
}

void UI::pot_did_change(Pot& pot, uint32_t value)
{
    uint32_t potNumber = pot.n;
    float fvalue = float(value) / float(kAdcMax);

    switch (_mode)
    {
        // In play mode, the pots control the gain of their corresponding channel.
        case kPlayMode:
            // Below a certain threshold, force gain to 0. This compensates for the physical nature
            // of the gain pot, which may not result in an ADC value of zero when fully turnd down.
            if (value < 15)
            {
                fvalue = 0.0f;
            }
            else
            {
                // Apply curve.
                fvalue = powf(fvalue, 2.0f);
            }

            g_voice[potNumber].set_gain(fvalue);
            break;

        // In edit mode, each pot controls a different voice parameter of the selected channel.
        case kEditMode:
            switch (potNumber)
            {
                case kPitchPot:
                    // Shift value from 0..1 to 0.1..2.5
                    fvalue = (fvalue * 2.4f) + 0.1f;
                    g_voice[_editChannel].set_pitch(fvalue);
                    break;
                case kSampleStartPot:
                {
                    // 0..1
                    float delta = fabsf(fvalue - _lastSampleStart);
                    _lastSampleStart = fvalue;
                    if (delta > (kAdcLsbFloat * 16))
                    {
                        _potReleaseTimer.start();
                    }
                    break;
                }
                case kSampleEndPot:
                    // 0..1
                    g_voice[_editChannel].set_sample_end(fvalue);
                    break;
                case kEffectPot:
                {
                    // 0..1 -> 1..32
                    uint32_t bits = (fvalue * 32) + 1;
                    g_voice[_editChannel].set_bits(bits);
                    break;
                }
            }
            break;

        // Ignore pots when there is no card.
        case kNoCardMode:
            break;
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

void UI::set_all_channel_leds_polarity(bool polarity)
{
    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        _channelLeds[i]->set_polarity(polarity);
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
                    set_all_channel_leds(false);
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
                    set_all_channel_leds(false);
                    // Turn off N-FET.
                    GPIO_ClearPinsOutput(PIN_CHLEDN_GPIO, PIN_CHLEDN);
                    break;
                default:
                    assert(0 && "bad led transition");
            }
            // Set LEDs to yellow off state.
            set_all_channel_leds_polarity(false);
            set_all_channel_leds(false);
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
                    set_all_channel_leds(false);
                    // Turn off P-FET.
                    GPIO_SetPinsOutput(PIN_CHLEDP_GPIO, PIN_CHLEDP);
                    break;
                default:
                    assert(0 && "bad led transition");
            }
            // Set LEDs to red off state.
            set_all_channel_leds_polarity(true);
            set_all_channel_leds(false);
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
