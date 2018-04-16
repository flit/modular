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
#if !defined(_FADER_LED_H_)
#define _FADER_LED_H_

#include "utility.h"
#include "fsl_ftm.h"
#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

extern const uint8_t kCie1931[];

/*!
 * @brief LED template.
 */
template <uintptr_t ftmBase, ftm_chnl_t channel>
class FaderLED : public LEDBase
{
    static const uint32_t kPwmFreq_Hz = 30000; // 30 kHz

public:
    FaderLED() : _polarity(false), _dutyCycle(0) {}

    void init()
    {
        ftm_chnl_pwm_signal_param_t params;
        params.chnlNumber = channel;
        params.level = _polarity ? kFTM_LowTrue : kFTM_HighTrue;
        params.dutyCyclePercent = _dutyCycle;
        params.firstEdgeDelayPercent = 0;
        FTM_SetupPwm((FTM_Type *)ftmBase, &params, 1, kFTM_EdgeAlignedPwm, kPwmFreq_Hz, CLOCK_GetBusClkFreq());
    }

    virtual void on() override
    {
        set_duty_cycle(100);
    }

    virtual void off() override
    {
        set_duty_cycle(0);
    }

    virtual void set_duty_cycle(uint32_t percent) override
    {
        _dutyCycle = kCie1931[constrained(percent, 0ul, 100ul)];
        FTM_UpdatePwmDutycycle((FTM_Type *)ftmBase, channel, kFTM_EdgeAlignedPwm, _dutyCycle);
        FTM_SetSoftwareTrigger((FTM_Type *)ftmBase, true);
    }

    virtual bool is_on() override
    {
        return _dutyCycle > 0;
    }

    virtual void set_polarity(bool polarity) override
    {
        _polarity = polarity;
        init();
    }

protected:
    bool _polarity;
    uint8_t _dutyCycle;
};

} // namespace slab

#endif // _FADER_LED_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
