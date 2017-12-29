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
#include "samplbaer.h"

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

SampleBank::SampleBank()
{
}

void SampleBank::reset()
{
    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        _samples[i].reset();
    }
}

bool SampleBank::is_valid() const
{
    uint32_t i;
    for (i = 0; i < kVoiceCount; ++i)
    {
        if (_samples[i].is_valid())
        {
            return true;
        }
    }
    return false;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

SampleBank::Sample::Sample()
:   _isValid(false),
    _params()
{
}

void SampleBank::Sample::reset()
{
    _isValid = false;
    _path = FilePath("");
    _params.reset();
}

void SampleBank::Sample::set_path(FilePath & path)
{
    _path = path;

    if (_path.get()[0] != 0)
    {
        _isValid = true;
    }
}

bool SampleBank::Sample::load_to_voice(SamplerVoice & voice)
{
    if (!_isValid)
    {
        voice.clear_file();
        return false;
    }

    // Open and parse .wav file.
    WaveFile wav(_path.get());

    fs::error_t err = wav.parse();
    if (err != fs::kSuccess)
    {
        DEBUG_PRINTF(ERROR_MASK, "Failed to parse %s\r\n", _path.get());
        voice.clear_file();
        return false;
    }

    // Only support 16-bit format files with 1 or 2 channels. If the sample rate is not
    // 48k then it will be played off-pitch.
    if (!(wav.get_channels() <= 2
        && wav.get_sample_size() == 16))
    {
        DEBUG_PRINTF(ERROR_MASK, "File %s is an unsupported format\r\n", _path.get());
        voice.clear_file();
        return false;
    }

    // Set sample file in voice.
    voice.set_file(wav);
    voice.set_params(_params);

    DEBUG_PRINTF(INIT_MASK, "%s: %lu ch; %lu frames\r\n",
        _path.get(),
        wav.get_channels(),
        voice.get_audio_stream().get_frames());

    return true;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
