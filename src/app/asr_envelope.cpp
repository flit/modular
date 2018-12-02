/*
 * Copyright (c) 2015,2018 Immo Software
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

#include "asr_envelope.h"
#include "arm_math.h"

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

ASREnvelope::ASREnvelope()
:   AudioFilter(),
    m_attack(),
    m_release(),
    m_peak(1.0f),
    m_mode(kOneShotAR),
    m_enableSustain(false),
    m_releaseOffset(0),
    m_elapsedSamples(0)
{
}

void ASREnvelope::set_sample_rate(float rate)
{
    AudioFilter::set_sample_rate(rate);
    m_attack.set_sample_rate(rate);
    m_release.set_sample_rate(rate);
}

void ASREnvelope::set_mode(EnvelopeMode mode)
{
    m_mode = mode;
    m_enableSustain = (mode == kOneShotASR);
}

void ASREnvelope::set_peak(float peak)
{
    m_peak = peak;
    m_attack.set_begin_value(0.0f);
    m_attack.set_end_value(peak);
    m_release.set_begin_value(peak);
    m_release.set_end_value(0.0f);
}

void ASREnvelope::set_length_in_seconds(EnvelopeStage stage, float seconds)
{
    switch (stage)
    {
        case kAttack:
            m_attack.set_length_in_seconds(seconds);
            break;

        case kRelease:
            m_release.set_length_in_seconds(seconds);
            break;

        default:
            break;
    }
}

void ASREnvelope::set_length_in_samples(EnvelopeStage stage, uint32_t samples)
{
    switch (stage)
    {
        case kAttack:
            m_attack.set_length_in_samples(samples);
            break;

        case kRelease:
            m_release.set_length_in_samples(samples);
            break;

        default:
            break;
    }
}

float ASREnvelope::get_length_in_seconds(EnvelopeStage stage)
{
    switch (stage)
    {
        case kAttack:
            return m_attack.get_length_in_seconds();

        case kRelease:
            return m_release.get_length_in_seconds();

        default:
            break;
    }
    return 0.0f;
}

uint32_t ASREnvelope::get_length_in_samples(EnvelopeStage stage)
{
    switch (stage)
    {
        case kAttack:
            return m_attack.get_length_in_samples();

        case kRelease:
            return m_release.get_length_in_samples();

        default:
            break;
    }
    return 0;
}

void ASREnvelope::set_curve_type(EnvelopeStage stage, AudioRamp::CurveType theType)
{
    switch (stage)
    {
        case kAttack:
            m_attack.set_curve_type(theType);
            break;

        case kRelease:
            m_release.set_curve_type(theType);
            break;

        default:
            break;
    }
}

void ASREnvelope::recompute()
{
    // Recompute the slope of both ramps.
    m_attack.recompute();
    m_release.recompute();
}

void ASREnvelope::set_release_offset(uint32_t offset)
{
    m_releaseOffset = m_elapsedSamples + offset;
}

void ASREnvelope::reset()
{
    m_attack.reset();
    m_release.reset();
    m_elapsedSamples = 0;
    m_releaseOffset = 0;
}

float ASREnvelope::next()
{
    float sample;
    process(&sample, 1);
    return sample;
}

bool ASREnvelope::is_finished()
{
    return (m_mode != kLoopingAR) && (m_attack.is_finished()
            && (!m_enableSustain || (m_releaseOffset != 0 && m_elapsedSamples >= m_releaseOffset))
            && m_release.is_finished());
}

void ASREnvelope::process(float * samples, uint32_t count)
{
    uint32_t totalRemaining = count;
    while (totalRemaining)
    {
        // Special case to prevent infinite loop if the stages are both 0 length and
        // we're in the looping envelope mode.
        if (m_mode == kLoopingAR
            && m_attack.get_length_in_samples() == 0
            && m_release.get_length_in_samples() == 0)
        {
            arm_fill_f32(m_peak, samples, totalRemaining);
            return;
        }

        // Attack.
        uint32_t attackCount = m_attack.get_remaining_samples();
        if (attackCount > count)
        {
            attackCount = count;
        }
        if (attackCount)
        {
            m_attack.process(samples, attackCount);
        }

        // Sustain.
        if (attackCount < count)
        {
            int32_t sustainCount = 0;
            if (m_enableSustain)
            {
                sustainCount = count - attackCount;
                if (m_releaseOffset > 0)
                {
                    if (attackCount + sustainCount + m_elapsedSamples > m_releaseOffset)
                    {
                        sustainCount = m_releaseOffset - m_elapsedSamples - attackCount;

                        if (sustainCount < 0)
                        {
                            sustainCount = 0;
                        }
                    }
                }
                if (sustainCount > 0)
                {
                    arm_fill_f32(m_peak, samples + attackCount, sustainCount);
                }
            }

            // Release.
            uint32_t attackSustainCount = attackCount + sustainCount;
            if (attackSustainCount < count)
            {
                uint32_t releaseCount = count - attackSustainCount;
                if (m_mode == kLoopingAR)
                {
                    uint32_t releaseRemaining = m_release.get_remaining_samples();
                    if (releaseRemaining > releaseCount)
                    {
                        m_release.process(samples + attackSustainCount, releaseCount);
                    }
                    else
                    {
                        // Fill last part of release stage, then retrigger and loop.
                        m_release.process(samples + attackSustainCount, releaseRemaining);
                        reset();

                        uint32_t thisLoopCount = attackSustainCount + releaseRemaining;
                        totalRemaining -= thisLoopCount;
                        samples += thisLoopCount;
                        m_elapsedSamples += thisLoopCount;
                        count = releaseCount - releaseRemaining;
                        continue;
                    }
                }
                else
                {
                    // For non-looping modes, we can just let the release stage fill to the end.
                    if (releaseCount)
                    {
                        m_release.process(samples + attackSustainCount, releaseCount);
                    }
                }
            }
        }

        totalRemaining -= count;
        m_elapsedSamples += count;
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
