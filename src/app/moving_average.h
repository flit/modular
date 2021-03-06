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
#if !defined(_MOVING_AVERAGE_H_)
#define _MOVING_AVERAGE_H_

#include "ring_buffer.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief
 */
template <uint32_t N>
class MovingAverage : public RingBuffer<uint32_t, N>
{
public:
    MovingAverage() : RingBuffer<uint32_t, N>(), _sum(0), _average(0) {}
    ~MovingAverage()=default;

    uint32_t get() const { return _average; }

    uint32_t update(uint32_t value)
    {
        if (this->is_full())
        {
            uint32_t temp = 0;
            this->RingBuffer<uint32_t, N>::get(temp);
            _sum -= temp;
        }
        _sum += value;
        this->put(value);
        _average = _sum / this->get_count();
        return _average;
    }

    void clear()
    {
        _sum = 0;
        _average = 0;
        RingBuffer<uint32_t, N>::clear();
    }

protected:
    uint32_t _sum;
    uint32_t _average;
};

} // namespace slab

#endif // _MOVING_AVERAGE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
