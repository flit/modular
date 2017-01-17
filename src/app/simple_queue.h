/*
 * Copyright (c) 2016 Immo Software
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
#if !defined(_SIMPLE_QUEUE_H_)
#define _SIMPLE_QUEUE_H_

#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

template <typename T, uint32_t N>
class SimpleQueue
{
public:
    SimpleQueue()
    :   _head(0),
        _tail(0),
        _count(0)
    {
    }
    ~SimpleQueue()=default;

    bool is_empty() const { return _count == 0; }
    bool is_full() const { return _count == N; }
    uint32_t get_count() const { return _count; }

    void clear()
    {
        _head = 0;
        _tail = 0;
        _count = 0;
    }

    T& peek()
    {
        return _queue[_head];
    }

    bool put(const T& value)
    {
        if (is_full())
        {
            return false;
        }
        _queue[_tail] = value;
        _tail = (_tail + 1) % N;
        ++_count;
        return true;
    }

    bool get(T& value)
    {
        if (is_empty())
        {
            return false;
        }
        value = _queue[_head];
        _head = (_head + 1) % N;
        --_count;
        return true;
    }

protected:
    T _queue[N];
    uint32_t _head;
    uint32_t _tail;
    uint32_t _count;
};

} // namespace slab

#endif // _SIMPLE_QUEUE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
