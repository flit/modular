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
#if !defined(_SERIALIZER_H_)
#define _SERIALIZER_H_

#include <stdint.h>
#include "stream.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

class Archive;

/*!
 * @brief Mix-in class.
 */
class Serializable
{
public:
    Serializable();
    virtual ~Serializable()=default;

    virtual bool serialize(Archive & serializer) { return true; }
    virtual bool deserialize(Archive & serializer) { return true; }

};

/*!
 * @brief
 */
class Archive
{
public:

    virtual bool read(const char * name, void * value, uint32_t length)=0;
    virtual bool write(const char * name, const void * value, uint32_t length)=0;

    template <typename T>
    bool read(const char * name, T * value)
    {
        return read(name, value, sizeof(T));
    }

    template <typename T>
    bool write(const char * name, const T & value)
    {
        return write(name, &value, sizeof(T));
    }

};

/*!
 * @brief
 */
class BinaryArchive : public Archive
{
public:
    BinaryArchive(Stream & stream) : _stream(stream) {}
    virtual ~BinaryArchive()=default;

    bool open(uint32_t * dataVersion);
    bool init(uint32_t dataVersion);

    virtual bool read(const char * name, void * value, uint32_t length) override;
    virtual bool write(const char * name, const void * value, uint32_t length) override;

    template <typename T>
    bool read(const char * name, T * value)
    {
        return read(name, value, sizeof(T));
    }

    template <typename T>
    bool write(const char * name, const T & value)
    {
        return write(name, &value, sizeof(T));
    }

protected:
    Stream & _stream;

    static const uint32_t kSignature = 'barc';
    static const uint32_t kFormatVersion = 1;

    struct Header
    {
        uint32_t signature;
        uint32_t formatVersion;
        uint32_t dataVersion;
    };
};

} // namespace slab

#endif // _SERIALIZER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
