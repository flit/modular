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
#if !defined(_SINGLETON_H_)
#define _SINGLETON_H_

#include <assert.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Base class for singleton objects.
 *
 * This singleton class only keeps track of the single instance, it does not
 * allocate it for you. You must still create a global object of the superclass
 * type. This allows for easily passing arbitrary constructor parameters and
 * controlling where instance storage is allocated.
 */
template <class S>
class Singleton
{
public:
    //! @brief Get the singleton instance.
    static S & get()
    {
        assert(s_instance);
        return *s_instance;
    }

protected:
    //! @brief Constructor, saves the instance.
    Singleton()
    {
        s_instance = static_cast<S*>(this);
    }

    // Disable copy ctor and assignment operator.
    Singleton(const Singleton<S> & other)=delete;
    Singleton<S> & operator = (const Singleton<S> & other)=delete;

private:
    static S * s_instance; //!< Instance pointer.
};

template <class S>
S * Singleton<S>::s_instance = nullptr;

} // namespace slab

#endif // _SINGLETON_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
