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
#if !defined(_SAMPLE_BUFFER_H_)
#define _SAMPLE_BUFFER_H_

#include <stdint.h>
#include <assert.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief A buffer of sample data read from the file.
 */
struct SampleBuffer
{
    //! @brief Extra samples required for interpolation.
    static const uint32_t kInterpolationFrameCount = 3;

    //! @brief States of the buffer.
    enum class State : uint8_t
    {
        kUnused,
        kStartFile,
        kPlaying,
        kReady,
        kFree,
        kReading,
    };

    //! @brief Options for interpolation.
    enum class InterpolationMode
    {
        kLinear,
        kCubic,
        kHermite,
    };

    uint8_t number; //!< Sample buffer number, mostly for debugging.
    State state;    //!< Current state of the buffer.
    bool reread;    //!< Whether to auto re-queue the buffer for reading after current read is finished.
    int16_t * data; //!< Pointer to the buffer's data, post interpolation frames. The size is #kVoiceBufferSize.
    int16_t * dataWithInterpolationFrames;  //!< Pointer to interpolation frames and data that follows.
    uint32_t startFrame;    //!< Frame number within the source file for this buffer's first frame.
    uint32_t zeroSnapOffset;    //!< Offset frame snapped to zero. This is frame from which we should start reading. Only applies to the file start buffer (will be zero for all others).
    uint32_t frameCount;    //!< Number of valid frames within this buffer.

    //! @brief Set the buffer to unused state and clear variant members.
    void set_unused()
    {
        state = SampleBuffer::State::kUnused;
        startFrame = 0;
        zeroSnapOffset = 0;
        frameCount = 0;
        reread = false;
    }

    //! @brief Fill a buffer with interpolated samples.
    template <InterpolationMode mode>
    float read_into(float * buffer, uint32_t count, float fractionalFrame, float rate, const float * preBufferFrames)
    {
        assert(fractionalFrame + float(count) * rate < frameCount + 1);
        uint32_t n;
        for (n = 0; n < count; ++n)
        {
            *buffer++ = read<mode>(fractionalFrame, preBufferFrames);
            fractionalFrame += rate;
        }
        return fractionalFrame;
    }

    //! @brief Return one interpolated sample at a fractional position within this buffer.
    template <InterpolationMode mode>
    float read(float fractionalFrame, const float * preBufferFrames)
    {
        uint32_t intOffset = static_cast<uint32_t>(fractionalFrame);
        float fractOffset = fractionalFrame - static_cast<float>(intOffset);

        float x0;
        float x1;

        if (mode == InterpolationMode::kLinear)
        {
            // Linear interpolator.
            if (intOffset == 0)
            {
                x0 = preBufferFrames[2];
            }
            else
            {
                x0 = data[intOffset - 1];
            }
            x1 = data[intOffset];
            return (x0 + fractOffset * (x1 - x0));
        }
        else
        {
            // Read the 4 points used for interpolation.
            float xm1 = dataWithInterpolationFrames[intOffset];
            x0 = dataWithInterpolationFrames[intOffset + 1];
            x1 = dataWithInterpolationFrames[intOffset + 2];
            float x2 = dataWithInterpolationFrames[intOffset + 3];

            if (mode == InterpolationMode::kCubic)
            {
                // 4 point cubic interpolator.
                float a0 = x2 - x1 - xm1 + x0;
                float a1 = xm1 - x0 - a0;
                float a2 = x1 - xm1;
                return ((a0 * (fractOffset * fractOffset * fractOffset)) + (a1 * (fractOffset * fractOffset)) + (a2 * fractOffset) + x0);
            }
            else
            {
                // 4 point, 3rd order Hermite interpolator by Laurent de Soras.
                float c = (x1 - xm1) * 0.5f;
                float v = x0 - x1;
                float w = c + v;
                float a = w + v + (x2 - x0) * 0.5f;
                float wa = w + a;
                return ((((a * fractOffset) - wa) * fractOffset + c) * fractOffset + x0);
            }
        }
    }
};

} // namespace slab

#endif // _SAMPLE_BUFFER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
