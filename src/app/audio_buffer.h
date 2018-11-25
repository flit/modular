/*
 * Copyright (c) 2015-2017 Immo Software
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
#if !defined(_AUDIO_BUFFER_H_)
#define _AUDIO_BUFFER_H_

#include "utility.h"
#include "arm_math.h"
#include <limits>
#include <cassert>
#include <stddef.h>
#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Represents a single channel audio buffer.
 *
 * Objects of this class only wrap the actual audio data buffers, they do not own them. Multiple
 * instances may reference the same underlying data buffer. If a data buffer is dynamically allocated,
 * it must be freed using a mechanism outside of this class, and any instances of this class that
 * reference the freed buffer must be properly updated or disposed.
 */
class AudioBuffer
{
public:
    AudioBuffer() : m_samples(NULL), m_count(0) {}
    AudioBuffer(float * samples, uint32_t count) : m_samples(samples), m_count(count) {}
    AudioBuffer(const AudioBuffer & other) : m_samples(other.m_samples), m_count(other.m_count) {}
    AudioBuffer & operator = (const AudioBuffer & other)
    {
        m_samples = other.m_samples;
        m_count = other.m_count;
        return *this;
    }

    ~AudioBuffer()=default;

    void set(float * samples, uint32_t count)
    {
        m_samples = samples;
        m_count = count;
    }

    float * get_buffer() { return m_samples; }
    const float * get_buffer() const { return m_samples; }
    uint32_t get_count() const { return m_count; }

    void clear() { set_scalar(0.0f); }

    void set_scalar(float value)
    {
        arm_fill_f32(value, m_samples, m_count);
    }

    void negate()
    {
        arm_negate_f32(m_samples, m_samples, m_count);
    }

    void add_scalar(float value)
    {
        arm_offset_f32(m_samples, value, m_samples, m_count);
    }

    void add_vector(const float *vector)
    {
        arm_add_f32(m_samples, const_cast<float *>(vector), m_samples, m_count);
    }

    void subtract_scalar(float value)
    {
        arm_offset_f32(m_samples, -value, m_samples, m_count);
    }

    void subtract_vector(const float *vector)
    {
        arm_sub_f32(m_samples, const_cast<float *>(vector), m_samples, m_count);
    }

    void multiply_scalar(float value)
    {
        arm_scale_f32(m_samples, value, m_samples, m_count);
    }

    void multiply_vector(const float * vector)
    {
        arm_mult_f32(m_samples, const_cast<float *>(vector), m_samples, m_count);
    }

    template <typename T>
    void copy_into(T * buffer, uint32_t step, uint32_t count, uint32_t start=0, float scale=1.0f)
    {
        constexpr float tmin = static_cast<float>(std::numeric_limits<T>::min());
        constexpr float tmax = static_cast<float>(std::numeric_limits<T>::max());
        uint32_t i;
        float * sample = &m_samples[start];
        if (scale == 1.0f)
        {
            for (i = 0; i < count; ++i)
            {
                float f = *sample++;
                constrain(f, tmin, tmax);
                *buffer = static_cast<T>(f);
                buffer += step;
            }
        }
        else
        {
            for (i = 0; i < count; ++i)
            {
                float f = *sample++ * scale;
                constrain(f, tmin, tmax);
                *buffer = static_cast<T>(f);
                buffer += step;
            }
        }
    }

    operator float * () { return m_samples; }
    operator const float * () const { return m_samples; }
    operator bool () { return m_samples != NULL; }

protected:
    float * m_samples;
    uint32_t m_count;
};

} // namespace slab

#endif // _AUDIO_BUFFER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
