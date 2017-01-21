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
#if !defined(_RING_BUFFER_H_)
#define _RING_BUFFER_H_

#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

template <typename T, uint32_t N>
class RingBuffer
{
public:
    RingBuffer()
    :   _read(0),
        _write(0),
        _count(0)
    {
    }
    RingBuffer(const RingBuffer<T,N>& other)=default;
    RingBuffer<T,N>& operator=(const RingBuffer<T,N>& other)=default;
    ~RingBuffer() {}

    bool is_empty() const { return _count == 0; }
    bool is_full() const { return _count == N; }
    uint32_t get_count() const { return _count; }

    void clear()
    {
        _read = 0;
        _write = 0;
        _count = 0;
    }

    T peek(int32_t index)
    {
        if (index >= _count || index <= -_count)
        {
            return 0;
        }
        if (index < 0)
        {
            index = _count + index;
        }
        index = (_read + index) % N;
        return _buffer[index];
    }

    void put(const T& value)
    {
        _buffer[_write] = value;
        _write = (_write + 1) % N;
        if (_count == N)
        {
            _read = _write;
        }
        else if (_count < N)
        {
            ++_count;
        }
    }

    bool get(T& value)
    {
        if (is_empty())
        {
            return false;
        }
        value = _buffer[_read];
        _read = (_read + 1) % N;
        --_count;
        return true;
    }

protected:
    T _buffer[N];
    uint32_t _read;
    uint32_t _write;
    uint32_t _count;
};

} // namespace slab

#endif // _RING_BUFFER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
