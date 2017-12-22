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

#include "calibrator.h"
#include "debug_log.h"

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const uint32_t kAdcMax = 65535;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

Calibrator::Calibrator()
:   _stage(Stage::kPotsLow),
    _currentChannel(0)
{
}

void Calibrator::init()
{
    memset(&_data, 0, sizeof(_data));
    _data.version = calibration::Data::kVersion;
}

bool Calibrator::is_calibrating_pots() const
{
    return (_stage == Stage::kPotsLow) || (_stage == Stage::kPotsHigh);
}

bool Calibrator::is_calibrating_low_point() const
{
    return (_stage == Stage::kPotsLow) || (_stage == Stage::kCVsLow);
}

void Calibrator::button_was_pressed()
{
    uint32_t i;
    Stage nextStage = static_cast<Stage>(static_cast<uint32_t>(_stage) + 1);

    // Save reading into calibration data.
    switch (_stage)
    {
        case Stage::kPotsLow:
            for (i = 0; i < kVoiceCount; ++i)
            {
                _data.pots[i].low = _currentReadings[i].get();
            }
            break;

        case Stage::kPotsHigh:
            for (i = 0; i < kVoiceCount; ++i)
            {
                _data.pots[i].high = _currentReadings[i].get();
            }
            break;

        case Stage::kCVsLow:
            _data.cvs[_currentChannel].low = _currentReadings[0].get();
            break;

        case Stage::kCVsHigh:
            _data.cvs[_currentChannel].high = _currentReadings[0].get();

            // Move to next channel and/or stage.
            ++_currentChannel;
            if (_currentChannel >= kVoiceCount)
            {
                _currentChannel = 0;
            }
            else if (_stage == Stage::kCVsHigh)
            {
                nextStage = Stage::kCVsLow;
            }
            break;

        case Stage::kDone:
            return;
    }

    _stage = nextStage;

    // Clear reading state.
    for (i = 0; i < kVoiceCount; ++i)
    {
        _currentReadings[i].clear();
    }

    // Save data to persistent store if we're finished now.
    if (is_done())
    {
        persistent_data::g_calibrationData.write(_data);
    }
}

void Calibrator::update_readings(const uint32_t * pots, const uint32_t * cvs)
{
    switch (_stage)
    {
        case Stage::kPotsLow:
        case Stage::kPotsHigh:
        {
            uint32_t i;
            for (i = 0; i < kVoiceCount; ++i)
            {
                _currentReadings[i].update(pots[i]);
            }
            break;
        }

        case Stage::kCVsLow:
        case Stage::kCVsHigh:
            _currentReadings[0].update(kAdcMax - cvs[_currentChannel]);
            break;

        case Stage::kDone:
            break;
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
