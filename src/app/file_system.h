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
#if !defined(_FILE_SYSTEM_H_)
#define _FILE_SYSTEM_H_

#include "stream.h"
#include "ff.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief
 */
class File : public Stream
{
public:

    File();
    File(const char * path);
    virtual ~File();

    File& operator = (const File& other);

    void set(const char * path);

    bool open(bool writable=false, bool create=false);
    void close();

    virtual uint32_t read(uint32_t count, void * data) override;
    virtual uint32_t write(uint32_t count, const void * data) override;
    virtual bool seek(uint32_t offset) override;

    uint32_t get_size() { return f_size(&_fp); }
    uint32_t get_offset() { return f_tell(&_fp); }

protected:
    char _path[_MAX_LFN + 1];
    FIL _fp;
    bool _isOpen;
};

/*!
 * @brief
 */
class DirectoryIterator
{
public:

    DirectoryIterator(const char * path);
    virtual ~DirectoryIterator();

    bool next(FILINFO * info);

protected:
    DIR _dir;
};


/*!
 * @brief
 */
class FileSystem
{
public:

    FileSystem();
    ~FileSystem();

    int init(const char * path="");

    DirectoryIterator open_dir(const char * path);

protected:
    FATFS _fs;
};

} // namespace slab

#endif // _FILE_SYSTEM_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
