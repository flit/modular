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
#if !defined(_PROTECTED_QUEUE_H_)
#define _PROTECTED_QUEUE_H_

#include "simple_queue.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Mutex protected queue.
 */
template <typename T, uint32_t N>
class ProtectedQueue : public SimpleQueue<T, N>
{
public:
    ProtectedQueue() : SimpleQueue<T, N>(), _lock(nullptr) {}
    ~ProtectedQueue()=default;

    void clear()
    {
        Ar::Mutex::Guard guard(_lock);
        SimpleQueue<T, N>::clear();
    }

    bool put(const T& value)
    {
        Ar::Mutex::Guard guard(_lock);
        return SimpleQueue<T, N>::put(value);
    }

    bool get(T& value)
    {
        Ar::Mutex::Guard guard(_lock);
        return SimpleQueue<T, N>::get(value);
    }

protected:
    Ar::Mutex _lock;
};

} // namespace slab

#endif // _PROTECTED_QUEUE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
