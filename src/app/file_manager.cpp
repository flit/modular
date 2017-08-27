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
#include "wav_file.h"
#include "main.h"
#include <ctype.h>

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

FileManager::FileManager()
:   _fs()
{
}

void FileManager::mount()
{
    fs::error_t res = _fs.mount();

    if (res == fs::kSuccess)
    {
        scan_for_files();
    }
    else
    {
        DEBUG_PRINTF(ERROR_MASK, "fs init failed: %lu\r\n", res);
    }
}

void FileManager::unmount()
{
    _fs.unmount();
}

void FileManager::scan_for_files()
{
    fs::DirectoryIterator dir = _fs.open_dir("/");
    FILINFO info;

    while (dir.next(&info))
    {
        // Skip directories and hidden or system files.
        if (info.fattrib & (AM_DIR | AM_HID | AM_SYS))
        {
            continue;
        }

        // Look for '[0-9].wav' files.
        if (isdigit(info.fname[0]) && info.fname[1] == '.'
            && toupper(info.fname[2]) == 'W'
            && toupper(info.fname[3]) == 'A'
            && toupper(info.fname[4]) == 'V'
            && info.fsize > 0)
        {
            WaveFile wav(info.fname);

            bool inited = (wav.parse() == fs::kSuccess);

            // @todo check sample rate
            if (inited && wav.get_channels() <= 2)
            {
                uint32_t channel = info.fname[0] - '1';
                if (channel >= 0 && channel < kVoiceCount)
                {
                    g_voice[channel].set_file(wav);

                    uint32_t frameCount = g_voice[channel].get_audio_stream().get_frames();

                    DEBUG_PRINTF(INIT_MASK, "%s: %lu Hz; %lu bits; %lu ch; %lu bytes/frame; %lu frames\r\n",
                        info.fname,
                        wav.get_sample_rate(),
                        wav.get_sample_size(),
                        wav.get_channels(),
                        wav.get_frame_size(),
                        frameCount);
                }
            }
            else
            {
                DEBUG_PRINTF(ERROR_MASK, "Failed to parse %s\r\n", info.fname);
            }
        }
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
