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
#if !defined(_FILE_MANAGER_H_)
#define _FILE_MANAGER_H_

#include "file_system.h"
#include "singleton.h"
#include "simple_string.h"
#include "sample_bank.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

//! @brief Number of banks.
const uint32_t kMaxBankCount = kVoiceCount;

/*!
 * @brief Handles scanning filesystem to identify samples.
 */
class FileManager : public Singleton<FileManager>
{
public:
    FileManager();
    ~FileManager()=default;

    bool mount();
    void unmount();

    void scan_for_files();

    bool has_any_banks() const;
    bool has_bank(uint32_t bankNumber) const;
    SampleBank & get_bank(uint32_t bankNumber) { return _banks[bankNumber]; }

protected:
    fs::FileSystem _fs;
    SampleBank _banks[kMaxBankCount];
    fs::Path _path;

    void _reset_banks();
    void _scan_bank_directory(uint32_t bankNumber, const char * dirName);
    void _check_special_files(const char * name);
};

} // namespace slab

#endif // _FILE_MANAGER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
