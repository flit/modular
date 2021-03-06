/*
 * Copyright (c) 2017-2018 Immo Software
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
#include "samplbaer.h"
#include "board.h"
#include "utility.h"
#include "calibrator.h"
#include "channel_adc_processor.h"
#include "reader_thread.h"
#include "fsl_sd_disk.h"
#include <assert.h>
#include <cmath>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const float kAdcMax = 65535.0f;
const float kAdcLsbFloat = 1.0f / 4096.0f;

//! ADC counts within either end of the ADC range that will cause a snap to 0.0/1.0.
const float kPotExtremaSnapValue = 100.0f;

//! ADC count below which will cause a snap to 0.0.
const float kPotMinSnapValue = kPotExtremaSnapValue;

//! ADC count above which will cause a snap to 1.0.
const float kPotMaxSnapValue = kAdcMax - kPotExtremaSnapValue;

//! Hysteresis to use when switching between edit and play mode and edit pages.
const float kPotEditHysteresisPercent = 3.0f;

//! Power used to apply a curve to envelope attack and release pots.
const float kEnvCurvePower = 1.8f;

//! Pot value to switch from one-shot to looping envelope mode.
const float kEnvLoopSwitchPoint = 0.13f;

//! Maximum envelope speed multiplier.
const float kEnvMaxLoopSpeed = 100.0f;

//! Interval for checking SD card presence.
const uint32_t kCardDetectInterval_ms = 500;

//! Delay for applying a change to the sample start parameter.
const uint32_t kPotReleaseDelay_ms = 100;

//! Duty cycle percent change per 100 ms.
const int32_t kButton1LedDutyCycleDelta = 2;

//! Time in seconds button1 must be held to switch voice modes.
const uint32_t kVoiceModeLongPressTime = 2;

//! Number of 100ms ticks that the new bank is shown on channel LEDs.
const uint32_t kBankSwitchLedDelayCount = 4;

//! Number of 100ms ticks per flash on voice mode switch.
const uint32_t kVoiceModeSwitchLedDelayCount = 1;

//! LED blink timer period in milliseconds.
const uint32_t kLedUpdateTimerPeriod_ms = 100;

//! Length of time to turn off a channel LED to indicate a voice retrigger.
const uint32_t kVoiceRetriggerLedBlinkTime_ms = 100;

//! Time window within which we go ahead and handle a retrigger LED.
//!
//! This window is used to reduce the frequency of firing the retrigger timer.
const uint32_t kVoiceRetriggerLedWindowTime_ms = 10;

//! Map of bank channel to voice channel for each voice mode.
//!
//! - *First index*: voice mode
//! - *Second index*: bank channel number
//! - *Value*: voice channel number
//!
//! The first index is the voice mode, second index is bank channel number. The value
//! is the voice channel number that the bank channel is installed in. If the value
//! is negative, then the bank channel equal to the second index and the voice channel
//! equal to the absolute value are both unused.
static const int8_t kBankToVoiceChannelMap[kVoiceModeCount][kVoiceCount] = {
        // k4VoiceMode
        {
            [0] = 0,    // Bank ch 0 -> voice 0
            [1] = 1,    // Bank ch 1 -> voice 1
            [2] = 2,    // Bank ch 2 -> voice 2
            [3] = 3,    // Bank ch 3 -> voice 3
        },
        // k3VoiceMode
        {
            [0] = 0,    // Bank ch 0 -> voice 0
            [1] = 2,    // Bank ch 1 -> voice 2
            [2] = 3,    // Bank ch 2 -> voice 3
            [3] = -1,   // Bank ch 3, voice 1 both unused
        },
        // k2VoiceMode
        {
            [0] = 0,    // Bank ch 0 -> voice 0
            [1] = 2,    // Bank ch 1 -> voice 2
            [2] = -1,   // Bank ch 2, voice 1 both unused
            [3] = -3,   // Bank ch 3, voice 3 both unused
        },
        // k1VoiceMode
        {
            [0] = 0,    // Bank ch 0 -> voice 0
            [1] = -1,   // Bank ch 1, voice 1 both unused
            [2] = -2,   // Bank ch 2, voice 2 both unused
            [3] = -3,   // Bank ch 3, voice 3 both unused
        },
    };

//! Map of voice channel to bank channel for each voice mode.
//!
//! - *First index*: voice mode
//! - *Second index*: voice channel number
//! - *Value*: bank channel number
//!
//! If the value is negative, then the voice channel equal to the second index and
//! the bank channel equal to the absolute value are both unused.
static const int8_t kVoiceToBankChannelMap[kVoiceModeCount][kVoiceCount] = {
        // k4VoiceMode
        {
            [0] = 0,    // Bank ch 0 <- voice 0
            [1] = 1,    // Bank ch 1 <- voice 1
            [2] = 2,    // Bank ch 2 <- voice 2
            [3] = 3,    // Bank ch 3 <- voice 3
        },
        // k3VoiceMode
        {
            [0] = 0,    // Bank ch 0 <- voice 0
            [1] = -3,   // Bank ch 3, voice 1 both unused
            [2] = 1,    // Bank ch 2 <- voice 3
            [3] = 2,    // Bank ch 1 <- voice 2
        },
        // k2VoiceMode
        {
            [0] = 0,    // Bank ch 0 <- voice 0
            [1] = -2,   // Bank ch 2, voice 1 both unused
            [2] = 1,    // Bank ch 1 <- voice 2
            [3] = -3,   // Bank ch 3, voice 3 both unused
        },
        // k1VoiceMode
        {
            [0] = 0,    // Bank ch 0 <- voice 0
            [1] = -1,   // Bank ch 1, voice 1 both unused
            [2] = -2,   // Bank ch 2, voice 2 both unused
            [3] = -3,   // Bank ch 3, voice 3 both unused
        },
    };

//! Number of edit pages.
const uint32_t kEditPageCount = 3;

//! Map from pot number within an edit page to voice parameter.
static const VoiceParameters::ParameterName kPotToParameterMap[kEditPageCount][kVoiceCount] = {
        [0] = {
                [0] = VoiceParameters::kBaseOctave,
                [1] = VoiceParameters::kSampleStart,
                [2] = VoiceParameters::kSampleEnd,
                [3] = VoiceParameters::kPlaybackMode,
            },
        [1] = {
                [0] = VoiceParameters::kPitchEnvDepth,
                [1] = VoiceParameters::kPitchEnvAttack,
                [2] = VoiceParameters::kPitchEnvRelease,
                [3] = VoiceParameters::kPitchEnvMode, // also adjusts loop speed
            },
        [2] = {
                [0] = VoiceParameters::kVolumeEnvDepth,
                [1] = VoiceParameters::kVolumeEnvAttack,
                [2] = VoiceParameters::kVolumeEnvRelease,
                [3] = VoiceParameters::kVolumeEnvMode, // also adjusts loop speed
            },
    };

//! Number of 100 ms ticks over which the edit page blink patterns are played.
const uint32_t kEditPageBlinkPatternLength = 15;

//! Button1 blink pattern for edit pages.
static const uint8_t kEditPageBlinkStates[kEditPageCount][kEditPageBlinkPatternLength] = {
        [0] = { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, },
        [1] = { 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, },
        [2] = { 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, },
    };

//! Number of 100 ms ticks to alternate between selected channel and param indicator.
const uint32_t kEditPageChannelLedBlinkTime = 2;

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
    _firstSwitchToPlayMode(true),
    _isChannelLedFlushPending(false),
    _ignoreButton1Release(false),
    _button2LongPressTriggered(false),
    _potReleaseEditSampleStart(false),
    _potReleaseEditSampleEnd(false),
    _potReleaseSaveGain(false),
    _isBankSavePending(false),
    _lastSampleStart(-1.0f),
    _lastSampleEnd(-1.0f),
    _underflowTimestamp{0},
    _editPage(0),
    _editChannel(0),
    _selectedBank(0),
    _button1LedDutyCycle(0),
    _button1LedDutyCycleDelta(kButton1LedDutyCycleDelta),
    _button1LedFlashes(0),
    _ledTimeoutCount(0),
    _potReleaseSaveGainChannels{false},
    _editPageBlinkCounter(0),
    _editPageChannelLedBlinkCounter(0),
    _editPageChannelLedState(false),
    _options{true}
{
}

void UI::init()
{
    // Set listener on all voices.
    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        g_voice[i].set_listener(this);
    }

    // Create UI thread and its runloop.
    _thread.init("ui", this, &UI::ui_thread, kUIThreadPriority, kArSuspendThread);
    _runloop.init("ui");

    // Create event queue and add it to the runloop.
    _eventQueue.init("events");
    _runloop.addQueue(&_eventQueue, NULL, NULL);

    // Create LED blink timer.
    _blinkTimer.init("blink", this, &UI::handle_blink_timer, kArPeriodicTimer, kLedUpdateTimerPeriod_ms);
    _runloop.addTimer(&_blinkTimer);
    _blinkTimer.start();

    // Create channel LED retrigger timer.
    _retriggerTimer.init("retrig", this, &UI::handle_retrigger_timer, kArOneShotTimer, kVoiceRetriggerLedBlinkTime_ms);
    _runloop.addTimer(&_retriggerTimer);

    // Create pot edit release timer.
    _potReleaseTimer.init("pot-release", this, &UI::handle_pot_release_timer, kArOneShotTimer, kPotReleaseDelay_ms);
    _runloop.addTimer(&_potReleaseTimer);

    // Set up card detection timer.

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

    // Set hysteresis on all pots before we change modes.
    set_all_pot_hysteresis(kPotEditHysteresisPercent);

    UIMode prevMode = _uiMode;
    _uiMode = mode;

    switch (mode)
    {
        // Switch to edit mode.
        case kEditMode:
            _editPage = 0;
            _ledMode = LedMode::kVoiceEdit;

            _button1Led->on();

            // Show LED for last channel being edited.
            set_all_channel_leds(false, true, LEDBase::kRed);
            _channelLeds[_editChannel]->on();
            update_channel_leds();
            update_edit_page_leds();

            _lastSampleStart = -1.0f;
            _lastSampleEnd = -1.0f;
            _potReleaseEditSampleStart = false;
            _potReleaseEditSampleEnd = false;
            break;

        // Switch to play mode.
        case kPlayMode:
            // If we're exiting edit mode, save edit changes to sample in bank.
            if (prevMode == kEditMode)
            {
                save_voice_params(_editChannel);
            }

            // Restore LEDs to current voice state.
            set_voice_activity_led_mode();

            if (_firstSwitchToPlayMode)
            {
                _firstSwitchToPlayMode = false;

                // Set no hysteresis on all pots.
                set_all_pot_hysteresis(0);
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

        case kCalibrationMode:
            _ledMode = LedMode::kCalibration;
            set_all_channel_leds(true, true, LEDBase::kRed);
            update_channel_leds();
            set_all_pot_hysteresis(0);
            break;
    }
}

void UI::set_voice_mode(VoiceMode mode)
{
    if (_voiceMode == mode && !_firstSwitchToPlayMode)
    {
        return;
    }

    // Hold off ADC processing until we finish switching the voice mode.
    ChannelAdcProcessor::get().suspend(true);

    _voiceMode = mode;
    persistent_data::g_lastVoiceMode.write(_voiceMode);

    load_sample_bank(_selectedBank);

    switch (mode)
    {
        case k4VoiceMode:
            // Make sure voices are not affected by CV pitch.
            g_voice[0].set_pitch_octave(0.0f);
            g_voice[1].set_pitch_octave(0.0f);
            g_voice[2].set_pitch_octave(0.0f);
            g_voice[3].set_pitch_octave(0.0f);

            // Set all voices to trigger mode.
            g_voice[0].set_trigger_mode(SamplerVoice::TriggerMode::kTrigger);
            g_voice[1].set_trigger_mode(SamplerVoice::TriggerMode::kTrigger);
            g_voice[2].set_trigger_mode(SamplerVoice::TriggerMode::kTrigger);
            g_voice[3].set_trigger_mode(SamplerVoice::TriggerMode::kTrigger);

            // Reset inputs.
            g_gates[0].reset();
            g_gates[1].reset();
            g_gates[2].reset();
            g_gates[3].reset();

            // 4 flashes.
            _button1LedFlashes = 8;
            break;

        case k3VoiceMode:
            // Make sure gated voices are not affected by CV pitch.
            g_voice[2].set_pitch_octave(0.0f);
            g_voice[3].set_pitch_octave(0.0f);

            // Set volume env mode.
            g_voice[0].set_trigger_mode(SamplerVoice::TriggerMode::kGate);
            g_voice[2].set_trigger_mode(SamplerVoice::TriggerMode::kTrigger);
            g_voice[3].set_trigger_mode(SamplerVoice::TriggerMode::kTrigger);

            // Reset inputs.
            g_gates[0].reset();
            g_cvs[1].reset();
            g_gates[2].reset();
            g_gates[3].reset();

            // 3 flashes.
            _button1LedFlashes = 6;
            break;

        case k2VoiceMode:
            // Set volume env mode.
            g_voice[0].set_trigger_mode(SamplerVoice::TriggerMode::kGate);
            g_voice[2].set_trigger_mode(SamplerVoice::TriggerMode::kGate);

            // Reset inputs.
            g_gates[0].reset();
            g_cvs[1].reset();
            g_gates[2].reset();
            g_cvs[3].reset();

            // 2 flashes.
            _button1LedFlashes = 4;
            break;

        default:
            assert(0 && "Unsupported voice mode");
    }

    // Always switch to play mode on voice mode change.
    set_ui_mode(kPlayMode);

    _ledMode = LedMode::kVoiceModeSwitch;
    _ledTimeoutCount = kVoiceModeSwitchLedDelayCount;

    // Restore ADC processing.
    ChannelAdcProcessor::get().suspend(false);
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
                case kCardLowActivity:
                    handle_card_event(event);
                    break;

                case kSpecialFileDetected:
                    handle_special_file(event);
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
            switch (event.source)
            {
                case kButton2:
                    _button2LongPressTriggered = false;
                    break;

                default:
                    break;
            }
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

                        // Move to next edit page or switch to play mode.
                        case kEditMode:
                            select_next_edit_page();
                            break;

                        // Do nothing.
                        case kNoCardMode:
                        case kCalibrationMode:
                            break;
                    }
                    break;

                case kButton2:
                    switch (_uiMode)
                    {
                        // Bank switch in play mode.
                        case kPlayMode:
                            check_pending_bank_save();

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
                            if (_button2LongPressTriggered)
                            {
                                handle_reset_edit_page_params_button();
                            }
                            else
                            {
                                handle_next_edit_channel_button();
                            }
                            break;

                        // Ignore button2 if we don't have a card present.
                        case kNoCardMode:
                            break;

                        case kCalibrationMode:
                            handle_calibration_mode_button();
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
                    if (event.intValue == kVoiceModeLongPressTime)
                    {
                        if (_uiMode == kPlayMode)
                        {
                            // We don't want to process the button up event.
                            _ignoreButton1Release = true;

                            // Select new voice mode.
                            switch (_voiceMode)
                            {
                                case k4VoiceMode:
                                    set_voice_mode(k3VoiceMode);
                                    break;
                                case k3VoiceMode:
                                    set_voice_mode(k2VoiceMode);
                                    break;
                                case k2VoiceMode:
                                    set_voice_mode(k4VoiceMode);
                                    break;
                                default:
                                    break;
                            }
                        }
                        else if (_uiMode == kCalibrationMode)
                        {
                            handle_calibration_mode_button1_long_press();
                        }
                    }
                    break;

                case kButton2:
                    if (event.intValue == kVoiceModeLongPressTime)
                    {
                        if (_uiMode == kEditMode)
                        {
                            _button2LongPressTriggered = true;
                        }
                    }
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
                    voice_did_change_playing_state(n, false);
                }
                FileManager::get().unmount();
                _isCardPresent = false;

                set_ui_mode(kNoCardMode);
                _button1LedDutyCycle = 0;
                _button1LedDutyCycleDelta = kButton1LedDutyCycleDelta;
            }
            break;

        case kCardLowActivity:
            if (_isCardPresent)
            {
                check_pending_bank_save();
            }
            break;

        default:
            break;
    }
}

void UI::handle_special_file(const UIEvent & event)
{
    switch (event.intValue)
    {
        case kFirmwareUpdateFile:
            // Immediately reboot into the bootloader if a firmware update file is detected.
            NVIC_SystemReset();
            break;

        case kRecalibrateCmdFile:
            // Erase the persistent data store, including calibration settings.
            PersistentDataStore::get().reset();

            // Then reboot to start calibration process.
            NVIC_SystemReset();
            break;

        case kReportVersionCmdFile:
            write_version_info_file();
            break;

        default:
            break;
    }
}

void UI::handle_calibration_mode_button()
{
    // Tell the calibrator that a button was pressed.
    Calibrator & calibrator = Calibrator::get();
    calibrator.button_was_pressed();

    // When calibration is finished, reboot the system.
    if (calibrator.is_done())
    {
        // Turn off all LEDs and wait until they are updated before rebooting.
        set_all_channel_leds(false);
        update_channel_leds();
        _runloop.run(kArNoTimeout, nullptr); // Run runloop to flush leds.

#if DEBUG
        __BKPT(0);
#endif

        NVIC_SystemReset();
    }
    else if (calibrator.is_calibrating_pots())
    {
        set_all_channel_leds(true, true,
            calibrator.is_calibrating_low_point()
                ? LEDBase::kRed
                : LEDBase::kYellow);
        update_channel_leds();
    }
    else
    {
        // Light CV channel being calibrated in the correct color.
        // Low point is red, high point is yellow.
        set_all_channel_leds(false);
        _channelLeds[calibrator.get_current_channel()]->set_color(
            calibrator.is_calibrating_low_point()
                ? LEDBase::kRed
                : LEDBase::kYellow);
        _channelLeds[calibrator.get_current_channel()]->on();
        update_channel_leds();
    }
}

//! A button1 long press in calibration mode tells us to just skip the calibration
//! process and store fake "perfect" calibration points. This is useful when testing.
void UI::handle_calibration_mode_button1_long_press()
{
    Calibrator & calibrator = Calibrator::get();
    calibrator.skip();
    NVIC_SystemReset();
}

//! When editing voice params, a button2 long press will reset the current edit page's parameters.
void UI::handle_reset_edit_page_params_button()
{
    assert(_uiMode == kEditMode);

    // Iterate over the knobs on this edit page and reset the associated parameters.
    uint32_t paramIndex;
    for (paramIndex = 0; paramIndex < kVoiceCount; ++paramIndex)
    {
        g_voice[_editChannel].reset_parameter(kPotToParameterMap[_editPage][paramIndex]);
    }
}

void UI::handle_next_edit_channel_button()
{
    // Update edit LED.
    _channelLeds[_editChannel]->off();

    // Save params for the previous edit channel.
    save_voice_params(_editChannel);

    // Select next valid channel.
    select_next_edit_channel();

    _editPageChannelLedBlinkCounter = 0;
    update_edit_page_leds();

    _channelLeds[_editChannel]->set_color(LEDBase::kRed);
    _channelLeds[_editChannel]->on();

    update_channel_leds();

    // Set hysteresis on all pots.
    set_all_pot_hysteresis(kPotEditHysteresisPercent);
}

//! Select the next bank for playback that has samples. At least one valid bank
//! must exist. (If there are no valid banks, we should not be in play mode.)
void UI::select_next_bank()
{
    assert(FileManager::get().has_any_banks());
    do {
        _selectedBank = (_selectedBank + 1) % kMaxBankCount;
    } while (!FileManager::get().has_bank(_selectedBank));
}

//! Select the next valid voice for editing.
void UI::select_next_edit_channel()
{
    do {
        _editChannel = (_editChannel + 1) % kVoiceCount;
    } while (!g_voice[_editChannel].is_valid());
}

void UI::select_next_edit_page()
{
    ++_editPage;

    // If we're on the last edit page, switch back to play mode.
    if (_editPage >= kEditPageCount)
    {
        set_ui_mode(kPlayMode);
        return;
    }

    update_edit_page_leds();

    set_all_pot_hysteresis(kPotEditHysteresisPercent);

    _potReleaseTimer.stop();
    _potReleaseEditSampleStart = false;
    _potReleaseEditSampleEnd = false;
    _potReleaseSaveGain = false;

    _editPageBlinkCounter = 0;
}

void UI::check_pending_bank_save()
{
    if (_isBankSavePending)
    {
        _isBankSavePending = false;

        // Save params to card.
        SampleBank & bank = FileManager::get().get_bank(_selectedBank);
        bank.save_params();
    }
}

void UI::save_voice_params(uint32_t channel)
{
    // Map voice selected for editing to the bank channel.
    uint32_t mappedBankChannel = kVoiceToBankChannelMap[_voiceMode][channel];
    assert(mappedBankChannel >= 0);

    // Copy params from the voice to the bank sample.
    SampleBank & bank = FileManager::get().get_bank(_selectedBank);
    const VoiceParameters & params = g_voice[channel].get_params();
    bank.get_sample(mappedBankChannel).set_params(params);

    // Save params to card when we get a chance.
    _isBankSavePending = true;
    ReaderThread::get().request_lull_event();
}

void UI::load_sample_bank(uint32_t bankNumber)
{
    persistent_data::g_lastSelectedBank.write(bankNumber);

    SampleBank & bank = FileManager::get().get_bank(bankNumber);

    uint32_t channel;
    for (channel = 0; channel < kVoiceCount; ++channel)
    {
        voice_did_change_playing_state(channel, false);
        int32_t mappedChannel = kBankToVoiceChannelMap[_voiceMode][channel];
        if (mappedChannel >= 0 && bank.has_sample(channel))
        {
            bank.get_sample(channel).load_to_voice(g_voice[mappedChannel]);
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

void UI::voice_did_change_playing_state(uint32_t voiceNumber, bool isPlaying)
{
    assert(voiceNumber < kVoiceCount);

    bool prevState = _voiceStates[voiceNumber];
    _voiceStates[voiceNumber] = isPlaying;

    if (_ledMode == LedMode::kVoiceActivity && prevState != isPlaying)
    {
        _channelLeds[voiceNumber]->set(isPlaying);
        update_channel_leds();
    }
}

void UI::voice_did_underflow(uint32_t voice)
{
    assert(voice < kVoiceCount);

    if (_ledMode == LedMode::kVoiceActivity)
    {
        // Record the current time.
        _underflowTimestamp[voice] = ar_get_millisecond_count();

        // Only start the one-shot timer if it hasn't already been started.
        if (!_retriggerTimer.isActive())
        {
            _retriggerTimer.setDelay(kVoiceRetriggerLedBlinkTime_ms);
            _retriggerTimer.start();
        }

        // Set channel LED to yellow.
        _channelLeds[voice]->set_color(LEDBase::kYellow);
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

        case LedMode::kVoiceEdit:
            if (++_editPageBlinkCounter == kEditPageBlinkPatternLength)
            {
                _editPageBlinkCounter = 0;
            }

            _button1Led->set(!kEditPageBlinkStates[_editPage][_editPageBlinkCounter]);

            // If the same voice is selected as an edit param that has its corresponding LED
            // turned on, we need to blink back and forth between the channel indicator and edit LED.
            if (pot_has_edit_page_led(_editChannel))
            {
                if (++_editPageChannelLedBlinkCounter > kEditPageChannelLedBlinkTime)
                {
                    _editPageChannelLedBlinkCounter = 0;
                    _editPageChannelLedState = !_editPageChannelLedState;

                    bool ledStatus = get_edit_page_led_status(_editChannel);
                    if (ledStatus)
                    {
                        if (_editPageChannelLedState)
                        {
                            _channelLeds[_editChannel]->set_color(LEDBase::kYellow);
                            _channelLeds[_editChannel]->on();
                        }
                        else
                        {
                            _channelLeds[_editChannel]->set_color(LEDBase::kRed);
                            _channelLeds[_editChannel]->on();
                        }

                        update_channel_leds();
                    }
                }
            }
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

        case LedMode::kCalibration:
            // Toggle button1 LED.
            _button1Led->set(!_button1Led->is_on());
            break;

        default:
            break;
    }
}

void UI::handle_retrigger_timer(Ar::Timer * timer)
{
    uint32_t i;
    uint32_t nextDelay = ~0UL;
    uint32_t delta;
    uint32_t now = ar_get_millisecond_count();
    for (i = 0; i < kVoiceCount; ++i)
    {
        if (_underflowTimestamp[i])
        {
            delta = now - _underflowTimestamp[i];

            if (delta >= kVoiceRetriggerLedBlinkTime_ms - kVoiceRetriggerLedWindowTime_ms)
            {
                if (_ledMode == LedMode::kVoiceActivity)
                {
                    _channelLeds[i]->set_color(LEDBase::kRed);
                    _channelLeds[i]->set(_voiceStates[i]);
                }
                _underflowTimestamp[i] = 0;
            }
            else if (delta < nextDelay)
            {
                nextDelay = delta;
            }
        }
    }

    // If there is another pending retrigger LED update, reschedule the timer.
    if (nextDelay != ~0UL)
    {
        _retriggerTimer.setDelay(nextDelay);
        _retriggerTimer.start();
    }

    if (_ledMode == LedMode::kVoiceActivity)
    {
        update_channel_leds();
    }
}

void UI::handle_pot_release_timer(Ar::Timer * timer)
{
    if (_potReleaseEditSampleStart)
    {
        _channelPots[2 /*kSampleStartPot*/].set_hysteresis(1);
        g_voice[_editChannel].set_sample_start(_lastSampleStart);
        _lastSampleStart = -1.0f;
        _potReleaseEditSampleStart = false;
    }
    if (_potReleaseEditSampleEnd)
    {
        _channelPots[3 /*kSampleEndPot*/].set_hysteresis(1);
        g_voice[_editChannel].set_sample_end(_lastSampleEnd);
        _lastSampleEnd = -1.0f;
        _potReleaseEditSampleEnd = false;
    }
    if (_potReleaseSaveGain)
    {
        uint32_t i;
        for (i = 0; i < kVoiceCount; ++i)
        {
            if (_potReleaseSaveGainChannels[i])
            {
                save_voice_params(i);
                _potReleaseSaveGainChannels[i] = false;
            }
        }
        _potReleaseSaveGain = false;
    }
}

void UI::pot_did_change(Pot& pot, float value)
{
    uint32_t potNumber = pot.get_number();
    float fvalue = value / kAdcMax;

    switch (_uiMode)
    {
        // In play mode, the pots control the gain of their corresponding channel.
        case kPlayMode:
            handle_gain_pot(potNumber, value);
            break;

        // In edit mode, each pot controls a different voice parameter of the selected channel.
        case kEditMode:
            handle_edit_pot(potNumber, value);
            break;

        // Ignore pots in other UI modes.
        default:
            break;
    }
}

void UI::handle_gain_pot(uint32_t potNumber, float value)
{
    // In 2 voice mode, second pot for each channel is ignored.
    // In 3 voice mode, second pot for the first channel is ignored.
    if ((_voiceMode == k2VoiceMode && (potNumber == 1 || potNumber == 3))
        || (_voiceMode == k3VoiceMode && (potNumber == 1)))
    {
        // ignore pot
        return;
    }

    // Detect lower and upper ends of range and snap to 0/1.
    float fvalue;
    if (value < kPotMinSnapValue)
    {
        fvalue = 0.0f;
    }
    else if (value > kPotMaxSnapValue)
    {
        fvalue = 1.0f;
    }
    else
    {
        // Apply curve.
        fvalue = powf(static_cast<float>(value) / kAdcMax, 2.0f);
    }

    // Update gain in the voice.
    g_voice[potNumber].set_gain(fvalue);

    if (_options.saveGainInBank)
    {
        // Set flag and start timer to save the voice after gain is edited.
        _potReleaseSaveGainChannels[potNumber] = true;
        _potReleaseSaveGain = true;
        _potReleaseTimer.start();
    }
}

void UI::handle_edit_pot(uint32_t potNumber, float value)
{
    float fvalue = value / kAdcMax;
    bool bvalue = fvalue >= 0.5f; // Convert float pot value to boolean switch.
    float delta;
    VoiceParameters::ParameterName editParam = kPotToParameterMap[_editPage][potNumber];
    switch (editParam)
    {
        case VoiceParameters::kBaseOctave:
            // Shift value from 0..1 to -3..+3 octaves
            fvalue = (fvalue * 6.0f) - 3.0f;
            g_voice[_editChannel].set_base_octave_offset(fvalue);
            break;

        case VoiceParameters::kBaseCents:
            // Shift value from 0..1 to -100..+100 cents
            fvalue = (fvalue * 200.0f) - 100.0f;
            g_voice[_editChannel].set_base_cents_offset(fvalue);
            break;

        case VoiceParameters::kSampleStart:
            // 0..1
            delta = fabsf(fvalue - _lastSampleStart);
            _lastSampleStart = fvalue;
            if (delta > (kAdcLsbFloat * 16))
            {
                _potReleaseEditSampleStart = true;
                _potReleaseTimer.start();
            }
            break;

        case VoiceParameters::kSampleEnd:
            // 0..1
            delta = fabsf(fvalue - _lastSampleEnd);
            _lastSampleEnd = fvalue;
            if (delta > (kAdcLsbFloat * 16))
            {
                _potReleaseEditSampleEnd = true;
                _potReleaseTimer.start();
            }
            break;

        case VoiceParameters::kPlaybackMode:
            g_voice[_editChannel].set_playback_mode(static_cast<VoiceParameters::PlaybackMode>(bvalue));
            break;

        case VoiceParameters::kVolumeEnvMode:
            if (fvalue < kEnvLoopSwitchPoint)
            {
                g_voice[_editChannel].set_volume_env_mode(VoiceParameters::kOneShotEnv);
            }
            else
            {
                fvalue -= kEnvLoopSwitchPoint; // n..1 -> 0..(1-n)
                fvalue *= 1.0f / (1.0f - kEnvLoopSwitchPoint); // 0..(1-n) -> 0..1
                fvalue = fvalue * (kEnvMaxLoopSpeed - 1.0f) + 1.0f; // -> 1..max

                g_voice[_editChannel].set_volume_env_mode(VoiceParameters::kLoopEnv);
                g_voice[_editChannel].set_volume_env_loop_speed(fvalue);
            }
            break;

        case VoiceParameters::kVolumeEnvAttack:
            // Apply curve and shift from 0..1 to 0..(sample length)
            fvalue = powf(fvalue, kEnvCurvePower);
            fvalue *= g_voice[_editChannel].get_sample_length_in_seconds();
            g_voice[_editChannel].set_volume_env_attack(fvalue);
            break;

        case VoiceParameters::kVolumeEnvRelease:
            // Apply curve and shift from 0..1 to 0..(sample length)
            fvalue = powf(fvalue, kEnvCurvePower);
            fvalue *= g_voice[_editChannel].get_sample_length_in_seconds();
            g_voice[_editChannel].set_volume_env_release(fvalue);
            break;

        case VoiceParameters::kVolumeEnvDepth:
            // Shift from 0..1 to -1..+1.
            fvalue = (fvalue * 2.0f) - 1.0f;
            g_voice[_editChannel].set_volume_env_depth(fvalue);
            break;

        case VoiceParameters::kPitchEnvMode:
            if (fvalue < kEnvLoopSwitchPoint)
            {
                g_voice[_editChannel].set_pitch_env_mode(VoiceParameters::kOneShotEnv);
            }
            else
            {
                fvalue -= kEnvLoopSwitchPoint;
                fvalue *= 1.0f / (1.0f - kEnvLoopSwitchPoint);
                fvalue = fvalue * (kEnvMaxLoopSpeed - 1.0f) + 1.0f; // -> 1..max

                g_voice[_editChannel].set_pitch_env_mode(VoiceParameters::kLoopEnv);
                g_voice[_editChannel].set_pitch_env_loop_speed(fvalue);
            }
            break;

        case VoiceParameters::kPitchEnvAttack:
            // Apply curve and shift from 0..1 to 0..(sample length)
            fvalue = powf(fvalue, kEnvCurvePower);
            fvalue *= g_voice[_editChannel].get_sample_length_in_seconds();
            g_voice[_editChannel].set_pitch_env_attack(fvalue);
            break;

        case VoiceParameters::kPitchEnvRelease:
            // Apply curve and shift from 0..1 to 0..(sample length)
            fvalue = powf(fvalue, kEnvCurvePower);
            fvalue *= g_voice[_editChannel].get_sample_length_in_seconds();
            g_voice[_editChannel].set_pitch_env_release(fvalue);
            break;

        case VoiceParameters::kPitchEnvDepth:
            // Shift from 0..1 to -2..+2 octaves
            fvalue = (fvalue * 4.0f) - 2.0f;
            g_voice[_editChannel].set_pitch_env_depth(fvalue);
            break;

        // Pot is unused in this edit page.
        case VoiceParameters::kUnused:
        default:
            break;
    }

    update_one_edit_page_led(potNumber);
}

void UI::set_all_pot_hysteresis(uint32_t percent)
{
    uint32_t n;
    for (n = 0; n < kVoiceCount; ++n)
    {
        _channelPots[n].set_hysteresis(percent);
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
    if (!ChannelLEDManager::get().flush())
    {
        if (!_isChannelLedFlushPending)
        {
            _isChannelLedFlushPending = true;
            _runloop.perform(flush_channel_leds, this);
        }
    }
}

void UI::flush_channel_leds(void * param)
{
    UI * _ui = reinterpret_cast<UI *>(param);
    if (!ChannelLEDManager::get().flush())
    {
        _ui->_runloop.perform(flush_channel_leds, _ui);
        return;
    }
    _ui->_isChannelLedFlushPending = false;
}

//! @brief Helper to determine whether a float value is close to zero.
static bool near_zero(float value)
{
    return (value > -0.01) && (value < 0.01);
}

//! @brief Returns whether the specified pot uses the channel LED to indicate value.
bool UI::pot_has_edit_page_led(uint32_t potNumber)
{
    VoiceParameters::ParameterName param = kPotToParameterMap[_editPage][potNumber];
    switch (param)
    {
        case VoiceParameters::kBaseOctave:
        case VoiceParameters::kBaseCents:
        case VoiceParameters::kPlaybackMode:
        case VoiceParameters::kPitchEnvDepth:
        case VoiceParameters::kPitchEnvMode:
        case VoiceParameters::kVolumeEnvDepth:
        case VoiceParameters::kVolumeEnvMode:
            return true;

        default:
            // no action
            return false;
    }
}

//! @brief Returns the LED enabled state for the param associated with the specified pot.
bool UI::get_edit_page_led_status(uint32_t potNumber)
{
    VoiceParameters::ParameterName param = kPotToParameterMap[_editPage][potNumber];
    const VoiceParameters & voiceParams = g_voice[_editChannel].get_params();
    float value;
    VoiceParameters::PlaybackMode playbackMode;
    VoiceParameters::EnvMode envMode;
    bool ledEnabled = false;

    switch (param)
    {
        case VoiceParameters::kBaseOctave:
            value = voiceParams.baseOctaveOffset;
            return near_zero(value);

        case VoiceParameters::kBaseCents:
            value = voiceParams.baseCentsOffset;
            return near_zero(value);

        case VoiceParameters::kPlaybackMode:
            playbackMode = voiceParams.playbackMode;
            return (playbackMode == VoiceParameters::kReversePlayback);

        case VoiceParameters::kPitchEnvDepth:
            value = voiceParams.pitchEnvDepth;
            return near_zero(value);

        case VoiceParameters::kPitchEnvMode:
            envMode = voiceParams.pitchEnvMode;
            return (envMode == VoiceParameters::kLoopEnv);

        case VoiceParameters::kVolumeEnvDepth:
            value = voiceParams.volumeEnvDepth;
            return near_zero(value);

        case VoiceParameters::kVolumeEnvMode:
            envMode = voiceParams.volumeEnvMode;
            return (envMode == VoiceParameters::kLoopEnv);

        default:
            // no action
            return false;
    }
}

//! @brief Update the specified LED based on the edit param value.
void UI::update_one_edit_page_led(uint32_t potNumber)
{
    VoiceParameters::ParameterName param = kPotToParameterMap[_editPage][potNumber];
    if (!pot_has_edit_page_led(potNumber))
    {
        return;
    }

    // Update edit LED if it's for a param not showing the currently selected edit
    // channel, or the led should be showing the param value.
    if (_editChannel != potNumber || _editPageChannelLedState)
    {
        bool ledEnabled = get_edit_page_led_status(potNumber);
        if (ledEnabled)
        {
            _channelLeds[potNumber]->set_color(LEDBase::kYellow);
        }
        else if (_editChannel == potNumber)
        {
            // If the param is shared with the currently selected edit channel, then
            // we need to set the LED back to red if the param value is false.
            _channelLeds[potNumber]->set_color(LEDBase::kRed);
            ledEnabled = true;
        }
        _channelLeds[potNumber]->set(ledEnabled);
        update_channel_leds();
    }
}

//! @brief Update state for all channel LEDs in the current edit page.
void UI::update_edit_page_leds()
{
    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        update_one_edit_page_led(i);
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
