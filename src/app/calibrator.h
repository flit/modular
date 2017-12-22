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
#if !defined(_CALIBRATOR_H_)
#define _CALIBRATOR_H_

#include "singleton.h"
#include "samplbaer.h"
#include "moving_average.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Manages the calibration process.
 */
class Calibrator : public Singleton<Calibrator>
{
public:
    Calibrator();
    ~Calibrator()=default;

    void init();

    void button_was_pressed();
    void update_readings(const uint32_t * pots, const uint32_t * cvs);

    bool is_calibrating_pots() const;
    bool is_calibrating_low_point() const;
    bool is_done() const { return _stage == Stage::kDone; }
    uint32_t get_current_channel() const { return _currentChannel; }

protected:

    enum class Stage : uint32_t
    {
        kPotsLow,
        kPotsHigh,
        kCVsLow,
        kCVsHigh,
        kDone
    };

    Stage _stage;
    MovingAverage<32> _currentReadings[kVoiceCount];
    uint32_t _currentChannel;
    calibration::Data _data;
};

} // namespace slab

#endif // _CALIBRATOR_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
