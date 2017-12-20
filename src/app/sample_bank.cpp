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

#include "sample_bank.h"
#include "debug_log.h"
#include "wav_file.h"
#include "main.h"

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

SampleBank::SampleBank()
:   _isValid(false)
{
}

bool SampleBank::has_sample(uint32_t sampleNumber) const
{
    return _samplePaths[sampleNumber].get()[0] != 0;
}

const SampleBank::FilePath & SampleBank::get_sample_path(uint32_t sampleNumber) const
{
    return _samplePaths[sampleNumber];
}

void SampleBank::clear_sample_paths()
{
    _isValid = false;

    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        _samplePaths[0] = FilePath("");
    }
}

void SampleBank::set_sample_path(uint32_t sampleNumber, FilePath & path)
{
    _samplePaths[sampleNumber] = path;

    if (has_sample(sampleNumber))
    {
        _isValid = true;
    }
}

bool SampleBank::load_sample_to_voice(uint32_t sampleNumber, SamplerVoice & voice)
{
    if (!has_sample(sampleNumber))
    {
        return false;
    }

    // Open and parse .wav file.
    FilePath & path = _samplePaths[sampleNumber];
    WaveFile wav(path.get());

    fs::error_t err = wav.parse();
    if (err != fs::kSuccess)
    {
        DEBUG_PRINTF(ERROR_MASK, "Failed to parse %s\r\n", path.get());
        voice.clear_file();
        return false;
    }

    // Only support 48 kHz 16-bit format files with 1 or 2 channels.
    if (!(wav.get_channels() <= 2
        && wav.get_sample_rate() == 48000
        && wav.get_sample_size() == 16))
    {
        DEBUG_PRINTF(ERROR_MASK, "File %s is an unsupported format\r\n", path.get());
        voice.clear_file();
        return false;
    }

    // Set sample file in voice.
    voice.set_file(wav);

    DEBUG_PRINTF(INIT_MASK, "%s: %lu ch; %lu frames\r\n",
        path.get(),
        wav.get_channels(),
        voice.get_audio_stream().get_frames());

    return true;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
