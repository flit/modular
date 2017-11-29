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

#include "channel_adc_processor.h"
#include "main.h"
#include "ui.h"
#include "board.h"
#include "debug_log.h"

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const uint32_t kAdc0ChannelMask =   CH2_CV_CHANNEL_MASK |
                                    CH3_CV_CHANNEL_MASK |
                                    CH1_POT_CHANNEL_MASK |
                                    CH2_POT_CHANNEL_MASK;

const uint32_t kAdc1ChannelMask =   CH1_CV_CHANNEL_MASK |
                                    CH4_CV_CHANNEL_MASK |
                                    CH3_POT_CHANNEL_MASK |
                                    CH4_POT_CHANNEL_MASK;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

ChannelAdcProcessor::ChannelAdcProcessor()
:   _thread("cv", this, &ChannelAdcProcessor::cv_thread, kCVThreadPriority, kArSuspendThread)
{
}

void ChannelAdcProcessor::init()
{
    // Init ADC0 sequencer.
    memset(&_adc0Data, 0, sizeof(_adc0Data));
    _adc0Data.sequencer = AdcSequencer(ADC0, kAdc0CommandDmaChannel);
    _adc0Data.sequencer.set_channels(kAdc0ChannelMask);
    _adc0Data.sequencer.set_result_buffer((uint32_t *)&_adc0Data.results[0]);
    _adc0Data.sequencer.set_semaphore(&_adc0Data.waitSem);
    _adc0Data.sequencer.init(g_adcConfig);
    _adc0Data.waitSem.init(nullptr, 0);

    // Init ADC1 sequencer.
    memset(&_adc1Data, 0, sizeof(_adc1Data));
    _adc1Data.sequencer = AdcSequencer(ADC1, kAdc1CommandDmaChannel);
    _adc1Data.sequencer.set_channels(kAdc1ChannelMask);
    _adc1Data.sequencer.set_result_buffer((uint32_t *)&_adc1Data.results[0]);
    _adc1Data.sequencer.set_semaphore(&_adc1Data.waitSem);
    _adc1Data.sequencer.init(g_adcConfig);
    _adc1Data.waitSem.init(nullptr, 0);

    _thread.resume();
}

void ChannelAdcProcessor::cv_thread()
{
    AdcData & data0 = _adc0Data;
    AdcData & data1 = _adc1Data;
    UI & ui = UI::get();

    data0.sequencer.start();
    data1.sequencer.start();
    while (true)
    {
        // Wait until all new ADC samples are available.
        data0.waitSem.get();
        data1.waitSem.get();

        VoiceMode mode = ui.get_voice_mode();
        ChannelGate::Event event;
        float fvalue;

        // Process gate and CV inputs, trigger voices and adjust pitch. This is where the
        // voice mode determines whether an input channel is treated as a gate or CV.
        // If a voice is invalid, it will ignore triggers.

        // Channel 1
        event = g_gates[0].process(data1.results[2]);
        if (event == ChannelGate::kNoteOn)
        {
            DEBUG_PRINTF(TRIG_MASK, "ch1 triggered\r\n");
            g_voice[0].trigger();
        }
        else if (mode == k2VoiceMode && event == ChannelGate::kNoteOff)
        {
            DEBUG_PRINTF(TRIG_MASK, "ch1 note off\r\n");
            g_voice[0].note_off();
        }

        // Channel 2
        if (mode == k4VoiceMode)
        {
            event = g_gates[1].process(data0.results[3]);
            if (event == ChannelGate::kNoteOn)
            {
                DEBUG_PRINTF(TRIG_MASK, "ch2 triggered\r\n");
                g_voice[1].trigger();
            }
        }
        else if (mode == k2VoiceMode)
        {
            fvalue = g_cvs[1].process(data0.results[3]);
            g_voice[0].set_pitch_octave(fvalue);
        }

        // Channel 3
        event = g_gates[2].process(data0.results[0]);
        if (event == ChannelGate::kNoteOn)
        {
            DEBUG_PRINTF(TRIG_MASK, "ch3 triggered\r\n");
            g_voice[2].trigger();
        }
        else if (mode == k2VoiceMode && event == ChannelGate::kNoteOff)
        {
            DEBUG_PRINTF(TRIG_MASK, "ch3 note off\r\n");
            g_voice[2].note_off();
        }

        // Channel 4
        if (mode == k4VoiceMode)
        {
            event = g_gates[3].process(data1.results[3]);
            if (event == ChannelGate::kNoteOn)
            {
                DEBUG_PRINTF(TRIG_MASK, "ch4 triggered\r\n");
                g_voice[3].trigger();
            }
        }
        else if (mode == k2VoiceMode)
        {
            fvalue = g_cvs[3].process(data1.results[3]);
            g_voice[2].set_pitch_octave(fvalue);
        }

        // Process pots.
        g_pots[0].process(data0.results[1]);
        g_pots[1].process(data0.results[2]);
        g_pots[2].process(data1.results[0]);
        g_pots[3].process(data1.results[1]);
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
