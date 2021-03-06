/*
 * Copyright (c) 2016-2018 Immo Software
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
#include "simple_string.h"
#include "ff.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {
namespace fs {

//! @brief File system errors.
enum fs_error : uint32_t
{
    kSuccess = 0,
    kGenericError = 1,
    kDiskError = 2,
    kNameError = 3,
    kAccessError = 4,
    kInvalidError = 5,
    kTimeoutError = 6,
};

//! @brief Filesystem error type.
using error_t = uint32_t;

//! @brief String type for paths.
using Path = SimpleString<_MAX_LFN + 1>;

/*!
 * @brief File object.
 */
class File : public Stream
{
public:
    File();
    File(const char * path);
    File(const File & other);
    File(File && other);
    virtual ~File();

    File& operator = (const File& other);
    File& operator = (File&& other);

    void set(const char * path);

    error_t open(bool writable=false, bool create=false);
    void close();

    virtual error_t read(uint32_t count, void * data, uint32_t * actualCount) override;
    virtual error_t write(uint32_t count, const void * data, uint32_t * actualCount) override;
    virtual error_t seek(uint32_t offset) override;

    virtual uint32_t get_size() const override { return f_size(&_fp); }
    virtual uint32_t get_offset() const override { return f_tell(&_fp); }

    virtual bool remove();

protected:
    char _path[_MAX_LFN + 1];
    FIL _fp;
    bool _isOpen;
};

/*!
 * @brief Iterator over directory contents.
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
 * @brief File system.
 */
class FileSystem
{
public:
    FileSystem(const char * dev="");
    ~FileSystem();

    error_t mount();
    void unmount();

    DirectoryIterator open_dir(const char * path);

protected:
    FATFS _fs;
    const char * _dev;
};

} // namespace fs
} // namespace slab

#endif // _FILE_SYSTEM_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
