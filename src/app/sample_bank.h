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
#if !defined(_SAMPLE_BANK_H_)
#define _SAMPLE_BANK_H_

#include "simple_string.h"
#include "audio_defs.h"
#include "sampler_voice.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Holds information about a bank of samples.
 */
class SampleBank
{
public:
    using FilePath = SimpleString<_MAX_LFN + 1>;

    /*!
     * @brief Wraps a sample in a bank.
     */
    class Sample
    {
    public:
        Sample();
        ~Sample()=default;

        void reset();

        bool is_valid() const { return _isValid; }

        const FilePath & get_path() const { return _path; }
        void set_path(FilePath & path);

        const VoiceParameters & get_params() const { return _params; }
        void set_params(const VoiceParameters & params) { _params = params; }

        bool load_to_voice(SamplerVoice & voice);

    protected:
        bool _isValid;
        FilePath _path;
        VoiceParameters _params;
    };

    SampleBank();
    ~SampleBank()=default;

    void reset();

    bool is_valid() const;
    bool has_sample(uint32_t sampleNumber) const { return _samples[sampleNumber].is_valid(); }

    Sample & get_sample(uint32_t sampleNumber) { return _samples[sampleNumber]; }

protected:
    Sample _samples[kVoiceCount];
};

} // namespace slab

#endif // _SAMPLE_BANK_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------