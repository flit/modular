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

#include "serializer.h"
#include "string.h"

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

bool BinaryArchive::open(uint32_t * dataVersion)
{
    Header header;
    if (!read(nullptr, &header, sizeof(header)))
    {
        return false;
    }

    if (header.signature != kSignature || header.formatVersion != kFormatVersion)
    {
        return false;
    }

    if (dataVersion)
    {
        *dataVersion = header.dataVersion;
    }

    return true;
}

bool BinaryArchive::init(uint32_t dataVersion)
{
    Header header;
    header.signature = kSignature;
    header.formatVersion = kFormatVersion;
    header.dataVersion = dataVersion;

    return write(nullptr, &header, sizeof(header));
}

bool BinaryArchive::read(const char * name, void * value, uint32_t length)
{
    uint32_t bytesRead;
    uint32_t lengthInArchive = 0;
    if (_stream.read(sizeof(lengthInArchive), &lengthInArchive, &bytesRead)
        || (bytesRead != sizeof(lengthInArchive)))
    {
        return false;
    }

    if (lengthInArchive != length)
    {
        return false;
    }

    if (_stream.read(length, value, &bytesRead)
        || (bytesRead != length))
    {
        return false;
    }

    return true;
}

bool BinaryArchive::write(const char * name, const void * value, uint32_t length)
{
    if (_stream.write(sizeof(length), &length, nullptr))
    {
        return false;
    }
    if (_stream.write(length, value, nullptr))
    {
        return false;
    }

    return true;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
