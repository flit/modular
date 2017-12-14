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
#include "audio_defs.h"
#include "moving_average.h"
#include "callback.h"
#include "singleton.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

//! @brief Major voice architecture modes.
enum VoiceMode : uint32_t
{
    k4VoiceMode,    //!< 4-voice: default mode, just trigger input per voice (plus edit params).
    k3VoiceMode,    //!< 3-voice: one voice with gate plus pitch CV, two voices with only trigger.
    k2VoiceMode,    //!< 2-voice: each voice has gate plus pitch CV.
    k1VoiceMode,    //!< 1-voice: the single voice has gate plus pitch, volume, and (other) CV.
    kVoiceModeCount,
};

enum UIMode : uint32_t
{
    kPlayMode,
    kEditMode,
    kNoCardMode,
};

enum UIEventType : uint16_t
{
    kButtonDown,
    kButtonUp,
    kButtonHeld,
    kPotAdjusted,
    kPotStopped,
    kCardInserted,
    kCardRemoved,
};

enum UIEventSource : uint16_t
{
    kNoEventSource,
    kButton1,
    kButton2,
    kPot1,
    kPot2,
    kPot3,
    kPot4
};

struct UIEvent
{
    UIEventType event;
    UIEventSource source;
    float value;

    UIEvent() {}
    UIEvent(UIEventType theEvent, UIEventSource theSource=kNoEventSource, float theValue=0)
    :   event(theEvent),
        source(theSource),
        value(theValue)
    {
    }
};

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

/*!
 * @brief
 */
class Pot
{
public:
    Pot();
    ~Pot()=default;

    void set_noise(uint32_t noise) { _noise = noise; }
    void set_hysteresis(uint32_t percent);

    uint32_t process(uint32_t value);

    uint32_t n;

protected:
    uint32_t _number;
    uint32_t _last;
    MovingAverage<32> _avg;
    RingBuffer<uint16_t, 128> _history;
    uint32_t _hysteresis;
    uint32_t _noise;
};

//!
enum EditPotNames : uint32_t
{
    kPitchPot = 0,
    kSampleStartPot = 1,
    kSampleEndPot = 2,
    kEffectPot = 3,
};

/*!
 * @brief
 */
class UI : public Singleton<UI>
{
public:
    UI();
    ~UI()=default;

    void init();
    void set_leds(LEDBase ** channelLeds, LEDBase * button1Led);
    void set_pots(Pot * channelPots) { _channelPots = channelPots; }

    void start();

    void send_event(const UIEvent& event) { _eventQueue.send(event); }

    void set_voice_playing(uint32_t voice, bool state);

    void pot_did_change(Pot& pot, uint32_t value);

    UIMode get_ui_mode() const { return _uiMode; }
    VoiceMode get_voice_mode() const { return _voiceMode; }

    void set_ui_mode(UIMode mode);
    void set_voice_mode(VoiceMode mode);

    void load_sample_bank(uint32_t bankNumber);

    Ar::RunLoop * get_runloop() { return &_runloop; }

protected:
    static const uint32_t kMaxEvents = 20;

    //! @brief LED display modes.
    enum class LedMode : uint32_t
    {
        kCardDetection,
        kVoiceActivity,
        kVoiceEdit,
        kBankSwitch,
        kVoiceModeSwitch,
    };

    Ar::ThreadWithStack<4096> _thread;
    Ar::RunLoop _runloop;
    Ar::StaticQueue<UIEvent, kMaxEvents> _eventQueue;
    Ar::TimerWithMemberCallback<UI> _blinkTimer;
    Ar::TimerWithMemberCallback<UI> _potReleaseTimer;
    Ar::TimerWithMemberCallback<UI> _cardDetectTimer;
    LEDBase ** _channelLeds;
    LEDBase * _button1Led;
    Pot * _channelPots;
    Button _button1;
    Button _button2;
    VoiceMode _voiceMode;
    UIMode _uiMode;
    LedMode _ledMode;
    bool _voiceStates[kVoiceCount];
    bool _isCardPresent;
    bool _debounceCardDetect;
    bool _firstSwitchToPlayMode;
    bool _isChannelLedFlushPending;
    bool _ignoreButton1Release;
    float _lastSampleStart;
    uint32_t _editChannel;
    uint32_t _selectedBank;
    int32_t _button1LedDutyCycle;
    int32_t _button1LedDutyCycleDelta;
    uint32_t _button1LedFlashes;
    uint32_t _ledTimeoutCount;

    void ui_thread();

    void handle_button_event(const UIEvent & event);
    void handle_card_event(const UIEvent & event);

    void select_next_bank();

    void handle_blink_timer(Ar::Timer * timer);
    void handle_pot_release_timer(Ar::Timer * timer);
    void handle_card_detect_timer(Ar::Timer * timer);

    void set_voice_activity_led_mode();
    void set_all_channel_leds(bool on, bool setColor=false, LEDBase::LEDColor color=LEDBase::kRed);
    void update_channel_leds();
    static void flush_channel_leds(void * param);
};

} // namespace slab

#endif // _UI_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
