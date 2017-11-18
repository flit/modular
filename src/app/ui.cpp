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
#include "channel_led.h"
#include "debug_log.h"
#include "pin_irq_manager.h"
#include "main.h"
#include "board.h"
#include "utility.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_sd_disk.h"
#include <assert.h>
#include <cmath>

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

//! Time in seconds button1 must be held to switch voice modes.
const uint32_t kVoiceModeLongPressTime = 5;

//! Map of bank channel to voice channel for each voice mode.
//!
//! The first index is the voice mode, second index is bank channel number. The value
//! is the voice channel number that the bank channel is installed in. If the value
//! is negative, then the voice channel equal to the absolute value is unused.
static const int8_t kVoiceModeChannelMap[kVoiceModeCount][kVoiceCount] = {
        // k4VoiceMode
        { 0, 1, 2, 3, },
        // k2VoiceMode
        { 0, 2, -1, -3, },
        // k1VoiceMode
        { 0, -1, -2, -3, },
    };

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
    _timeoutCount(0),
    _ignoreRelease(false)
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
    _ignoreRelease = false;
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

    // Send event to UI, unless this is a button release and the UI asked to ignore it.
    if (isPressed || !_ignoreRelease)
    {
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
    }

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
    _voiceMode(k4VoiceMode),
    _uiMode(kNoCardMode),
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

void UI::set_ui_mode(UIMode mode)
{
    uint32_t n;

    if (_uiMode == mode)
    {
        return;
    }

    _uiMode = mode;

    switch (mode)
    {
        // Switch to edit mode.
        case kEditMode:
            _button1Led->on();

            // Turn off all LEDs.
            set_all_channel_leds(false);

            // Select last channel being edited.
            _channelLeds[_editChannel]->on();
            ChannelLEDManager::get().flush();

            _lastSampleStart = -1.0f;
            break;

        // Switch to play mode.
        case kPlayMode:
            _button1Led->off();

            // Restore LEDs to current voice state.
            for (n = 0; n < kVoiceCount; ++n)
            {
                _channelLeds[n]->set(_voiceStates[n]);
            }
            ChannelLEDManager::get().flush();

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

void UI::set_voice_mode(VoiceMode mode)
{
    if (_voiceMode == mode)
    {
        return;
    }

    _voiceMode = mode;

    load_sample_bank(_selectedBank);

    switch (mode)
    {
        case k4VoiceMode:
            g_gates[1].set_mode(ChannelCVGate::kGate);
            g_gates[3].set_mode(ChannelCVGate::kGate);
            break;

        case k2VoiceMode:
            g_gates[1].set_mode(ChannelCVGate::kCV);
            g_gates[3].set_mode(ChannelCVGate::kCV);
            break;

        default:
            assert(0 && "Unsupported voice mode");
    }

    // Always switch to play mode on voice mode change.
    set_ui_mode(kPlayMode);
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
                            switch (_uiMode)
                            {
                                // Switch to edit mode.
                                case kPlayMode:
                                    set_ui_mode(kEditMode);
                                    break;

                                // Switch to play mode.
                                case kEditMode:
                                    set_ui_mode(kPlayMode);
                                    break;

                                // Do nothing.
                                case kNoCardMode:
                                    break;
                            }
                            break;

                        case kButton2:
                            switch (_uiMode)
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

                                    // Select next valid channel.
                                    do {
                                        _editChannel = (_editChannel + 1) % kVoiceCount;
                                    } while (!g_voice[_editChannel].is_valid());

                                    _channelLeds[_editChannel]->on();

                                    ChannelLEDManager::get().flush();

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
                            if (event.value == kVoiceModeLongPressTime)
                            {
                                // We don't want the button up event.
                                _button1.ignore_release();

                                // Select new voice mode.
                                if (_voiceMode == k4VoiceMode)
                                {
                                    set_voice_mode(k2VoiceMode);
                                }
                                else
                                {
                                    set_voice_mode(k4VoiceMode);
                                }
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
                            set_ui_mode(kPlayMode);

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

                        set_ui_mode(kNoCardMode);
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
        uint32_t mappedChannel = kVoiceModeChannelMap[_voiceMode][channel];
        if (mappedChannel >= 0 && bank.has_sample(channel))
        {
            bank.load_sample_to_voice(channel, g_voice[mappedChannel]);
        }
        else
        {
            if (mappedChannel < 0)
            {
                mappedChannel = -mappedChannel;
            }
            g_voice[mappedChannel].clear_file();
        }
    }
}

void UI::set_voice_playing(uint32_t voice, bool state)
{
    assert(voice < kVoiceCount);

    _voiceStates[voice] = state;
    if (_uiMode == kPlayMode)
    {
        _channelLeds[voice]->set(state);
        ChannelLEDManager::get().flush();
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

    switch (_uiMode)
    {
        // In play mode, the pots control the gain of their corresponding channel.
        case kPlayMode:
            // In 2 voice mode, second pot for each channel adjusts pitch CV amount.
            if ((_voiceMode == k2VoiceMode && (potNumber == 1 || potNumber == 3)))
            {
                uint32_t voiceNumber = (potNumber == 1) ? 0 : (potNumber == 3 ? 2 : 0);
                g_voice[voiceNumber].set_pitch_cv_amount(fvalue);
            }
            else
            {
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
            }
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
    ChannelLEDManager::get().flush();
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
