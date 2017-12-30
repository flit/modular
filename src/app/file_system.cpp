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

#include "file_system.h"
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

using namespace slab::fs;

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

static error_t convert_ff_err(FRESULT result);

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

error_t convert_ff_err(FRESULT result)
{
    switch (result)
    {
        case FR_OK:
            return kSuccess;
        case FR_DISK_ERR:
        case FR_NOT_READY:
        case FR_NOT_ENABLED:
        case FR_NO_FILESYSTEM:
        case FR_WRITE_PROTECTED:
            return kDiskError;
        case FR_NO_FILE:
        case FR_NO_PATH:
            return kNameError;
        case FR_DENIED:
        case FR_EXIST:
            return kAccessError;
        case FR_INVALID_NAME:
        case FR_INVALID_OBJECT:
        case FR_INVALID_DRIVE:
        case FR_INVALID_PARAMETER:
            return kInvalidError;
        case FR_TIMEOUT:
        case FR_LOCKED:
            return kTimeoutError;
        case FR_INT_ERR:
        case FR_MKFS_ABORTED:
        case FR_NOT_ENOUGH_CORE:
        case FR_TOO_MANY_OPEN_FILES:
        default:
            return kGenericError;
    }
}

FileSystem::FileSystem(const char * dev)
:   _dev(dev)
{
}

FileSystem::~FileSystem()
{
}

error_t FileSystem::mount()
{
    // Pass 1 for options to force immediate mounting.
    return convert_ff_err(f_mount(&_fs, _dev, 1));
}

void FileSystem::unmount()
{
    f_mount(NULL, _dev, 1);
}

DirectoryIterator FileSystem::open_dir(const char * path)
{
    return DirectoryIterator(path);
}

DirectoryIterator::DirectoryIterator(const char * path)
{
    f_opendir(&_dir, path);
}

DirectoryIterator::~DirectoryIterator()
{
    f_closedir(&_dir);
}

bool DirectoryIterator::next(FILINFO * info)
{
    static char longFileNameBuffer[_MAX_LFN + 1];
    info->lfname = longFileNameBuffer;
    info->lfsize = sizeof(longFileNameBuffer);

    FRESULT result = f_readdir(&_dir, info);
    return result == FR_OK && info->fname[0] != 0;
}

File::File()
:   _isOpen(false)
{
    memset(_path, 0, sizeof(_path));
    memset(&_fp, 0, sizeof(_fp));
}

File::File(const char * path)
:   _isOpen(false)
{
    memset(&_fp, 0, sizeof(_fp));
    set(path);
}

File::File(const File & other)
:   _isOpen(other._isOpen)
{
    strncpy(_path, other._path, sizeof(_path));
    _fp = other._fp;
}

File::File(File && other)
:   _isOpen(other._isOpen)
{
    strncpy(_path, other._path, sizeof(_path));
    _fp = other._fp;
    other._isOpen = false;
}

File::~File()
{
    close();
}

File& File::operator = (const File& other)
{
    strncpy(_path, other._path, sizeof(_path));
    _fp = other._fp;
    _isOpen = other._isOpen;
    return *this;
}

File& File::operator = (File&& other)
{
    strncpy(_path, other._path, sizeof(_path));
    _fp = other._fp;
    _isOpen = other._isOpen;
    other._isOpen = false;
    return *this;
}

void File::set(const char * path)
{
    strncpy(_path, path, sizeof(_path));
}

File::error_t File::open(bool writable, bool create)
{
    if (_isOpen)
    {
        return true;
    }

    int mode = FA_READ | FA_OPEN_EXISTING;
    if (writable)
    {
        mode |= FA_WRITE;
    }
    if (create)
    {
        mode |= FA_CREATE_NEW;
    }

    FRESULT result = f_open(&_fp, _path, mode);
    _isOpen = (result == FR_OK);
    return convert_ff_err(result);
}

void File::close()
{
    if (_isOpen)
    {
        f_close(&_fp);
        _isOpen = false;
    }
}

bool File::remove()
{
    close();
    FRESULT result = f_unlink(_path);
    return result == FR_OK;
}

File::error_t File::read(uint32_t count, void * data, uint32_t * actualCount)
{
    uint32_t bytesRead = 0;
    FRESULT result = f_read(&_fp, data, count, (UINT *)&bytesRead);
    if (actualCount)
    {
        *actualCount = bytesRead;
    }
    return convert_ff_err(result);
}

File::error_t File::write(uint32_t count, const void * data, uint32_t * actualCount)
{
    uint32_t bytesWritten = 0;
    FRESULT result = f_write(&_fp, data, count, (UINT *)&bytesWritten);
    if (actualCount)
    {
        *actualCount = bytesWritten;
    }
    return convert_ff_err(result);
}

File::error_t File::seek(uint32_t offset)
{
    return convert_ff_err(f_lseek(&_fp, offset));
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
