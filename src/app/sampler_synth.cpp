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

#include "sampler_synth.h"
#include "samplbaer.h"
#include "microseconds.h"
#include "audio_defs.h"

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

#define SQUARE_OUT (0)

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

namespace slab {
uint32_t g_timestamp1;
uint32_t g_timestamp2;
}

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

//! Runs on the audio thread.
void SamplerSynth::render(uint32_t firstChannel, AudioOutput::Buffer & buffer)
{
    g_timestamp2 = Microseconds::get() - g_timestamp1;
    int16_t * data = (int16_t *)buffer.data;;
    int frameCount = buffer.dataSize / sizeof(int16_t) / kAudioChannelCount;

#if SQUARE_OUT
    // Output a naïve full-scale 440 Hz square wave for testing without the SD card.
    static int phase = 0;
    int w = kSampleRate / 440;
    int w2 = w / 2;
    int i;
    for (i = 0; i < frameCount; ++i)
    {
        int16_t intSample = (phase > w2) ? 32767 : -32768;
        *data++ = intSample;
        *data++ = intSample;
        if (++phase > w)
        {
            phase = 0;
        }
    }
#else // SQUARE_OUT
    DECLARE_ELAPSED_TIME(synth);
    START_ELAPSED_TIME(synth);
    g_voice[firstChannel].render(data, frameCount);
    g_voice[firstChannel + 1].render(data + 1, frameCount);
    END_ELAPSED_TIME(synth);
#endif // SQUARE_OUT
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
