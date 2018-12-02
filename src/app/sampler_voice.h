/*
 * Copyright (c) 2017-2018 Immo Software
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
#if !defined(_SAMPLER_VOICE_H_)
#define _SAMPLER_VOICE_H_

#include "wav_file.h"
#include "voice_parameters.h"
#include "audio_defs.h"
#include "audio_buffer.h"
#include "asr_envelope.h"
#include "sample_buffer_manager.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Manages playback of a single sample file.
 */
class SamplerVoice
{
public:
    enum class TriggerMode
    {
        kTrigger,   //!< Once triggered, voice plays until end of sample.
        kGate,      //!< Voice plays starting with trigger until note off.
    };

    SamplerVoice();
    ~SamplerVoice()=default;

    void init(uint32_t n, int16_t * buffer);
    void set_file(WaveFile & file);
    void clear_file();

    uint32_t get_number() const { return _number; }

    //! @brief Whether a valid file has been set on the voice.
    bool is_valid() const { return _isValid; }

    //! @brief Whether the voice is able to play.
    bool is_ready() const { return _isValid && _isReady; }

    //! @brief Prepare voice for playing from start.
    void prime();

    void trigger();
    void note_off();
    bool is_playing() const { return _isPlaying; }

    void manager_ready_did_change(bool isReady) { _isReady = isReady; }
    void playing_did_finish();

    void set_trigger_mode(TriggerMode mode);
    void set_gain(float gain) { _params.gain = gain; }
    void set_base_octave_offset(float octave);
    void set_base_cents_offset(float cents) { _params.baseCentsOffset = cents; }
    void set_pitch_octave(float pitch) { _pitchOctave = pitch; }
    void set_sample_start(float start);
    void set_sample_end(float end);
    void set_playback_mode(VoiceParameters::PlaybackMode mode);
    void set_volume_env_attack(float seconds);
    void set_volume_env_release(float seconds);
    void set_volume_env_depth(float depth) { _params.volumeEnvDepth = depth; }
    void set_volume_env_mode(VoiceParameters::EnvMode mode);
    void set_pitch_env_attack(float seconds);
    void set_pitch_env_release(float seconds);
    void set_pitch_env_depth(float depth) { _params.pitchEnvDepth = depth; }
    void set_pitch_env_mode(VoiceParameters::EnvMode mode);

    //! @brief Restore a parameter to its default value.
    void reset_parameter(VoiceParameters::ParameterName which);

    void render(int16_t * data, uint32_t frameCount);

    WaveFile& get_wave_file() { return _wav; }
    WaveFile::AudioDataStream& get_audio_stream() { return _data; }
    float get_sample_length_in_seconds() const;

    SampleBufferManager& get_buffer_manager() { return _manager; }

    const VoiceParameters & get_params() const { return _params; }
    void set_params(const VoiceParameters & params);

    uint32_t get_buffered_microseconds() const;

protected:
    uint32_t _number;
    WaveFile _wav;
    WaveFile::AudioDataStream _data;
    SampleBufferManager _manager;
    bool _isValid;
    bool _isReady;
    bool _isPlaying;
    bool _doNoteOff;
    bool _doRetrigger;
    uint32_t _noteOffSamplesRemaining;
    float _readHead;    //!< Fractional read head within current buffer.
    float _pitchOctave;
    VoiceParameters _params;
    TriggerMode _triggerMode;
    ASREnvelope _volumeEnv;
    ASREnvelope _pitchEnv;
    uint32_t _triggerNoteOffSample;
    int32_t _voiceStatusRetriggerCounter;

    static float s_workBufferData[2][kAudioBufferSize];
    static AudioBuffer s_workBuffer;
    static AudioBuffer s_workBuffer2;

    float _interpolationBuffer[SampleBuffer::kInterpolationFrameCount]; //!< Last few samples from previous buffer.

    void _reset_voice();

    float _compute_playback_rate(float pitchModifier=0.0f, bool includePitchOctave=true) const;

    void _trace_buffered_time();
};

} // namespace slab

#endif // _SAMPLER_VOICE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
