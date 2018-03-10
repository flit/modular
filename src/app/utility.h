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
#if !defined(_UTILITY_H_)
#define _UTILITY_H_

#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

template <typename T>
inline T abs(T a)
{
    return (a > 0) ? a : -a;
}

template <typename T>
inline T min(T a, T b)
{
    return (a < b) ? a : b;
}

template <typename T>
inline T max(T a, T b)
{
    return (a > b) ? a : b;
}

template <typename T>
inline T max3(T a, T b, T c)
{
    T tmp = (a > b) ? a : b;
    return (tmp > c) ? tmp : c;
}

template <typename T>
inline T round_up_div(T a, T b)
{
    return (a + b - 1) / b;
}

template <uint32_t A, typename T>
inline constexpr T align_up(T v)
{
    return (v + (T(A) - 1)) & (~(T(A) - 1));
}

template <uint32_t A, typename T>
inline constexpr T align_down(T v)
{
    return v & (~(T(A) - 1));
}

template <typename T>
inline void constrain(T & v, T _min, T _max)
{
    if (v < _min)
    {
        v = _min;
    }
    else if (v > _max)
    {
        v = _max;
    }
}

template <typename T>
inline T constrained(T v, T _min, T _max)
{
    return (v < _min) ? _min
            : ((v > _max) ? _max
                : v);
}

#if DEBUG
#define DECLARE_ELAPSED_TIME(name) uint64_t start_ ## name ## _time = 0ULL; static uint64_t elapsed_ ## name ## _time __attribute__((used)) = 0ULL
#define START_ELAPSED_TIME(name) start_ ## name ## _time = Microseconds::get()
#define END_ELAPSED_TIME(name) elapsed_ ## name ## _time = Microseconds::get() - start_ ## name ## _time
#define GET_ELAPSED_TIME(name) (elapsed_ ## name ## _time)
#else // DEBUG
#define DECLARE_ELAPSED_TIME(name)
#define START_ELAPSED_TIME(name)
#define END_ELAPSED_TIME(name)
#define GET_ELAPSED_TIME(name) (0)
#endif // DEBUG

} // namespace slab

#endif // _UTILITY_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
