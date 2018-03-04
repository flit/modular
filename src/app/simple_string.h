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
#if !defined(_SIMPLE_STRING_H_)
#define _SIMPLE_STRING_H_

#include <stdint.h>
#include <string.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

template <uint32_t N>
class SimpleString
{
public:
    SimpleString()
    {
        _string[0] = 0;
    }
    SimpleString(const char * s)
    {
        strncpy(_string, s, N);
    }
    SimpleString(const SimpleString<N>& other)=default;
    SimpleString<N>& operator =(const SimpleString<N>& other)=default;
    ~SimpleString()=default;

    char* get() { return _string; }
    const char* get() const { return _string; }

    void set(const char * s) { strncpy(_string, s, N); }

    template <uint32_t J>
    void set(const SimpleString<J> s) { strncpy(_string, s.get(), N); }

    void append(const char * s) { strncat(_string, s, N); }

    template <uint32_t J>
    void append(const SimpleString<J> & s) { strncat(_string, s.get(), N); }

    operator char* () { return _string; }
    operator const char* () const { return _string; }

protected:
    char _string[N];
};

} // namespace slab

#endif // _SIMPLE_STRING_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
