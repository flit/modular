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

#include "file_manager.h"
#include "debug_log.h"
#include "main.h"
#include <ctype.h>
#include <stdio.h>

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

FileManager::FileManager()
:   _fs()
{
}

bool FileManager::mount()
{
    fs::error_t res = _fs.mount();

    return (res == fs::kSuccess);
}

void FileManager::unmount()
{
    _fs.unmount();
    _reset_banks();
}

void FileManager::_reset_banks()
{
    // Reset all the banks.
    uint32_t bank;
    for (bank = 0; bank < kMaxBankCount; ++bank)
    {
        _banks[bank].clear_sample_paths();
    }
}

bool FileManager::has_any_banks() const
{
    uint32_t bank;
    for (bank = 0; bank < kMaxBankCount; ++bank)
    {
        if (_banks[bank].is_valid())
        {
            return true;
        }
    }
    return false;
}

bool FileManager::has_bank(uint32_t bankNumber) const
{
    return _banks[bankNumber].is_valid();
}

void FileManager::scan_for_files()
{
    fs::DirectoryIterator dir = _fs.open_dir("/");
    FILINFO info;

    _reset_banks();

    while (dir.next(&info))
    {
        // Skip hidden or system files.
        if (info.fattrib & (AM_HID | AM_SYS))
        {
            continue;
        }
        // Skip files.
        if ((info.fattrib & AM_DIR) != AM_DIR)
        {
            continue;
        }

        char * dirName = info.lfname[0] ? info.lfname : info.fname;

        // Look for '[0-9].*' directories.
        if (!isdigit(dirName[0]))
        {
            continue;
        }

        uint32_t bankNumber = dirName[0] - '1';
        if (bankNumber < kMaxBankCount)
        {
            snprintf(_dirPath, sizeof(_dirPath), "/%s", dirName);
            _scan_bank_directory(bankNumber, _dirPath);
        }
    }
}

void FileManager::_scan_bank_directory(uint32_t bankNumber, const char * dirPath)
{
    fs::DirectoryIterator dir = _fs.open_dir(dirPath);
    FILINFO info;

    while (dir.next(&info))
    {
        // Skip directories and hidden or system files.
        if (info.fattrib & (AM_DIR | AM_HID | AM_SYS))
        {
            continue;
        }

        char * fileName = info.lfname[0] ? info.lfname : info.fname;
        uint32_t fileNameLength = strlen(fileName);

        // Look for '[0-9].*\.wav' files.
        if (isdigit(fileName[0])
            && fileName[fileNameLength - 4] == '.'
            && toupper(fileName[fileNameLength - 3]) == 'W'
            && toupper(fileName[fileNameLength - 2]) == 'A'
            && toupper(fileName[fileNameLength - 1]) == 'V'
            && info.fsize > 0)
        {
            uint32_t channel = fileName[0] - '1';
            if (channel >=0 && channel < kVoiceCount)
            {
                snprintf(_filePath, sizeof(_filePath), "%s/%s", dirPath, fileName);
                SampleBank::FilePath path(_filePath);

                _banks[bankNumber].set_sample_path(channel, path);
            }
        }
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
