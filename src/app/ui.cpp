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
#include "main.h"
#include "board.h"
#include "utility.h"
#include "fsl_sd_disk.h"
#include <assert.h>
#include <cmath>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const uint32_t kAdcMax = 65535;
const float kAdcLsbFloat = 1.0f / 4096.0f;
const uint32_t kPotEditHysteresisPercent = 5;

//! Interval for checking SD card presence.
const uint32_t kCardDetectInterval_ms = 250;

//! Delay for applying a change to the sample start parameter.
const uint32_t kSampleStartPotReleaseDelay_ms = 100;

//! Duty cycle percent change per 100 ms.
const int32_t kButton1LedDutyCycleDelta = 2;

//! Time in seconds button1 must be held to switch voice modes.
const uint32_t kVoiceModeLongPressTime = 2;

//! Number of 100ms ticks that the new bank is shown on channel LEDs.
const uint32_t kBankSwitchLedDelayCount = 4;

//! Number of 100ms ticks per flash on voice mode switch.
const uint32_t kVoiceModeSwitchLedDelayCount = 1;

//! Map of bank channel to voice channel for each voice mode.
//!
//! The first index is the voice mode, second index is bank channel number. The value
//! is the voice channel number that the bank channel is installed in. If the value
//! is negative, then the voice channel equal to the absolute value is unused.
static const int8_t kVoiceModeChannelMap[kVoiceModeCount][kVoiceCount] = {
        // k4VoiceMode
        { 0, 1, 2, 3, },
        // k3VoiceMode
        { 0, 2, 3, -1, },
        // k2VoiceMode
        { 0, 2, -1, -3, },
        // k1VoiceMode
        { 0, -1, -2, -3, },
    };

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

UI::UI()
:   _button1(PIN_BUTTON1_PORT, PIN_BUTTON1_GPIO, PIN_BUTTON1_BIT, kButton1, true),
    _button2(PIN_BUTTON2_PORT, PIN_BUTTON2_GPIO, PIN_BUTTON2_BIT, kButton2, false),
    _voiceMode(k4VoiceMode),
    _uiMode(kNoCardMode),
    _ledMode(LedMode::kCardDetection),
    _voiceStates{0},
    _isCardPresent(false),
    _debounceCardDetect(false),
    _firstSwitchToPlayMode(true),
    _isChannelLedFlushPending(false),
    _ignoreButton1Release(false),
    _potReleaseEditSampleStart(false),
    _potReleaseEditSampleEnd(false),
    _lastSampleStart(-1.0f),
    _lastSampleEnd(-1.0f),
    _editChannel(0),
    _selectedBank(0),
    _button1LedDutyCycle(0),
    _button1LedDutyCycleDelta(kButton1LedDutyCycleDelta),
    _button1LedFlashes(0),
    _ledTimeoutCount(0)
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
    _blinkTimer.init("blink", this, &UI::handle_blink_timer, kArPeriodicTimer, 100);
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
            _ledMode = LedMode::kVoiceEdit;

            _button1Led->on();

            // Show LED for last channel being edited.
            set_all_channel_leds(false, true, LEDBase::kRed);
            _channelLeds[_editChannel]->on();
            update_channel_leds();

            _lastSampleStart = -1.0f;
            _lastSampleEnd = -1.0f;
            _potReleaseEditSampleStart = false;
            _potReleaseEditSampleEnd = false;
            break;

        // Switch to play mode.
        case kPlayMode:
            // Restore LEDs to current voice state.
            set_voice_activity_led_mode();

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
            _ledMode = LedMode::kCardDetection;
            _button1Led->off();
            set_all_channel_leds(false);
            update_channel_leds();
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
    if (_voiceMode == mode && !_firstSwitchToPlayMode)
    {
        return;
    }

    _voiceMode = mode;
    persistent_data::g_lastVoiceMode.write(_voiceMode);

    load_sample_bank(_selectedBank);

    switch (mode)
    {
        case k4VoiceMode:
            g_voice[1].set_pitch_octave(0.0f);
            g_voice[3].set_pitch_octave(0.0f);
            _button1LedFlashes = 8;
            break;

        case k2VoiceMode:
            _button1LedFlashes = 4;
            break;

        default:
            assert(0 && "Unsupported voice mode");
    }

    // Always switch to play mode on voice mode change.
    set_ui_mode(kPlayMode);

    _ledMode = LedMode::kVoiceModeSwitch;
    _ledTimeoutCount = kVoiceModeSwitchLedDelayCount;
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
            switch (event.event)
            {
                case kButtonDown:
                case kButtonUp:
                case kButtonHeld:
                    handle_button_event(event);
                    break;

                case kCardInserted:
                case kCardRemoved:
                    handle_card_event(event);
                    break;

                default:
                    break;
            }
        }
    }
}

void UI::handle_button_event(const UIEvent & event)
{
    uint32_t n;
    switch (event.event)
    {
        case kButtonDown:
            break;

        case kButtonUp:
            switch (event.source)
            {
                case kButton1:
                    if (_ignoreButton1Release)
                    {
                        _ignoreButton1Release = false;
                        return;
                    }

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
                            // Select the next valid bank.
                            select_next_bank();

                            _ledMode = LedMode::kBankSwitch;
                            _ledTimeoutCount = kBankSwitchLedDelayCount;
                            set_all_channel_leds(false);

                            _channelLeds[_selectedBank]->set_color(LEDBase::kYellow);
                            _channelLeds[_selectedBank]->on();
                            update_channel_leds();

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

                            _channelLeds[_editChannel]->set_color(LEDBase::kRed);
                            _channelLeds[_editChannel]->on();

                            update_channel_leds();

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
                        // We don't want to process the button up event.
                        _ignoreButton1Release = true;

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

        default:
            break;
    }
}

void UI::handle_card_event(const UIEvent & event)
{
    uint32_t n;
    switch (event.event)
    {
        case kCardInserted:
            if (!_isCardPresent)
            {
                if (FileManager::get().mount())
                {
                    FileManager::get().scan_for_files();

                    // Verify that the card has at least one bank before continuing.
                    if (!FileManager::get().has_any_banks())
                    {
                        return;
                    }

                    _isCardPresent = true;

                    if (_firstSwitchToPlayMode)
                    {
                        // Restore selected bank.
                        if (persistent_data::g_lastSelectedBank.is_present())
                        {
                            //! @todo Validate the restored selected bank.
                            _selectedBank = persistent_data::g_lastSelectedBank;
                        }
                        else
                        {
                            // Select the first valid bank.
                            _selectedBank = kMaxBankCount;
                            select_next_bank();
                        }

                        // Restore voice mode.
                        if (persistent_data::g_lastVoiceMode.is_present())
                        {
                            set_voice_mode(persistent_data::g_lastVoiceMode);
                        }
                        else
                        {
                            set_voice_mode(k4VoiceMode);
                        }
                    }
                    else
                    {
                        set_ui_mode(kPlayMode);

                        // Select the first valid bank.
                        _selectedBank = kMaxBankCount;
                        select_next_bank();
                        load_sample_bank(_selectedBank);
                    }
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
            }
            break;

        default:
            break;
    }
}

void UI::select_next_bank()
{
    assert(FileManager::get().has_any_banks());
    do {
        _selectedBank = (_selectedBank + 1) % kMaxBankCount;
    } while (!FileManager::get().has_bank(_selectedBank));
}

void UI::load_sample_bank(uint32_t bankNumber)
{
    persistent_data::g_lastSelectedBank.write(bankNumber);

    SampleBank & bank = FileManager::get().get_bank(bankNumber);

    uint32_t channel;
    for (channel = 0; channel < kVoiceCount; ++channel)
    {
        set_voice_playing(channel, false);
        int32_t mappedChannel = kVoiceModeChannelMap[_voiceMode][channel];
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

    if (_ledMode == LedMode::kVoiceActivity)
    {
        _channelLeds[voice]->set(state);
        update_channel_leds();
    }
}

void UI::handle_blink_timer(Ar::Timer * timer)
{
    switch (_ledMode)
    {
        case LedMode::kCardDetection:
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
            break;

        case LedMode::kBankSwitch:
            if (_ledTimeoutCount-- == 0)
            {
                // Restore LEDs to current voice state.
                set_voice_activity_led_mode();
            }
            break;

        case LedMode::kVoiceModeSwitch:
            if (_ledTimeoutCount-- == 0)
            {
                // Toggle button1 LED.
                _button1Led->set(!_button1Led->is_on());
                _ledTimeoutCount = kVoiceModeSwitchLedDelayCount;

                if (--_button1LedFlashes == 0)
                {
                    // Restore LEDs to current voice state.
                    set_voice_activity_led_mode();
                }
            }
            break;

        default:
            break;
    }
}

void UI::handle_pot_release_timer(Ar::Timer * timer)
{
    if (_potReleaseEditSampleStart)
    {
        _channelPots[kSampleStartPot].set_hysteresis(1);
        g_voice[_editChannel].set_sample_start(_lastSampleStart);
        _lastSampleStart = -1.0f;
        _potReleaseEditSampleStart = false;
    }
    if (_potReleaseEditSampleEnd)
    {
        _channelPots[kSampleEndPot].set_hysteresis(1);
        g_voice[_editChannel].set_sample_end(_lastSampleEnd);
        _lastSampleEnd = -1.0f;
        _potReleaseEditSampleEnd = false;
    }
}

void UI::handle_card_detect_timer(Ar::Timer * timer)
{
    // Detect change in card presence. If a change is detected, set the debounce flag
    // and exit. We'll process the debounce next time this timer fires.
    bool isPresent = CardManager::get().check_presence();
    if (!_debounceCardDetect)
    {
        // Set debounce flag if a change in presence is detected.
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
    float delta;

    switch (_uiMode)
    {
        // In play mode, the pots control the gain of their corresponding channel.
        case kPlayMode:
            // In 2 voice mode, second pot for each channel is ignored.
            if ((_voiceMode == k2VoiceMode && (potNumber == 1 || potNumber == 3)))
            {
                // ignore pot
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
                case kCoarsePitchPot:
                    // Shift value from 0..1 to -3..+3 octaves
                    fvalue = (fvalue * 6.0f) - 3.0f;
                    g_voice[_editChannel].set_base_octave_offset(fvalue);
                    break;
                case kFinePitchPot:
                    // Shift value from 0..1 to -100..+100 cents
                    fvalue = (fvalue * 200.0f) - 100.0f;
                    g_voice[_editChannel].set_base_cents_offset(fvalue);
                    break;
                case kSampleStartPot:
                    // 0..1
                    delta = fabsf(fvalue - _lastSampleStart);
                    _lastSampleStart = fvalue;
                    if (delta > (kAdcLsbFloat * 16))
                    {
                        _potReleaseEditSampleStart = true;
                        _potReleaseTimer.start();
                    }
                    break;
                case kSampleEndPot:
                    // 0..1
                    delta = fabsf(fvalue - _lastSampleEnd);
                    _lastSampleEnd = fvalue;
                    if (delta > (kAdcLsbFloat * 16))
                    {
                        _potReleaseEditSampleEnd = true;
                        _potReleaseTimer.start();
                    }
                    break;
            }
            break;

        // Ignore pots when there is no card.
        case kNoCardMode:
            break;
    }
}

void UI::set_voice_activity_led_mode()
{
    _ledMode = LedMode::kVoiceActivity;

    _button1Led->off();

    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        _channelLeds[i]->set_color(LEDBase::kRed);
        _channelLeds[i]->set(_voiceStates[i]);
    }

    update_channel_leds();
}

void UI::set_all_channel_leds(bool state, bool setColor, LEDBase::LEDColor color)
{
    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        _channelLeds[i]->set(state);
        if (setColor)
        {
            _channelLeds[i]->set_color(color);
        }
    }
}

void UI::update_channel_leds()
{
    if (!_isChannelLedFlushPending)
    {
        _isChannelLedFlushPending = true;
        _runloop.perform(flush_channel_leds, this);
    }
}

void UI::flush_channel_leds(void * param)
{
    UI * _ui = reinterpret_cast<UI *>(param);
    _ui->_isChannelLedFlushPending = false;
    ChannelLEDManager::get().flush();
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
