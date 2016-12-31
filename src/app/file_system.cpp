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

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

FileSystem::FileSystem()
{
}

FileSystem::~FileSystem()
{
}

int FileSystem::init(const char * path)
{
    // Pass 1 for options to force immediate mounting.
    return f_mount(&_fs, path, 1);
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

File::~File()
{
    close();
}

File& File::operator = (const File& other)
{
    strncpy(_path, other._path, sizeof(_path));
//     memcpy(&_fp, &other._fp, sizeof(_fp));
    _fp = other._fp;
    _isOpen = other._isOpen;
    return *this;
}

void File::set(const char * path)
{
    strncpy(_path, path, sizeof(_path));
}

bool File::open(bool writable, bool create)
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
    return _isOpen;
}

void File::close()
{
    if (_isOpen)
    {
        f_close(&_fp);
        _isOpen = false;
    }
}

uint32_t File::read(uint32_t count, void * data)
{
    uint32_t bytesRead = 0;
    f_read(&_fp, data, count, (UINT *)&bytesRead);
    return bytesRead;
}

uint32_t File::write(uint32_t count, const void * data)
{
    uint32_t bytesWritten = 0;
    f_write(&_fp, data, count, (UINT *)&bytesWritten);
    return bytesWritten;
}

bool File::seek(uint32_t offset)
{
    return f_lseek(&_fp, offset) == FR_OK;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
