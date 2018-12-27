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
#if !defined(_VOICE_PARAMETERS_H_)
#define _VOICE_PARAMETERS_H_

#include "serializer.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Set of parameters controlling voice playback.
 */
struct VoiceParameters
{
    //! @brief Voice parameter identifiers.
    enum ParameterName
    {
        kUnused,    //!< Special value for indicating an unused pot.
        kBaseOctave,
        kBaseCents,
        kSampleStart,
        kSampleEnd,
        kPlaybackMode,
        kVolumeEnvAttack,
        kVolumeEnvRelease,
        kVolumeEnvDepth,
        kVolumeEnvMode,
        kVolumeEnvLoopSpeed,
        kPitchEnvAttack,
        kPitchEnvRelease,
        kPitchEnvDepth,
        kPitchEnvMode,
        kPitchEnvLoopSpeed,
        kGain,
    };

    //! @brief Options for playback mode.
    enum PlaybackMode : uint8_t
    {
        kForwardPlayback = 0,
        kReversePlayback = 1,
    };

    //! @brief Available envelope modes.
    enum EnvMode : uint8_t
    {
        kOneShotEnv = 0,
        kLoopEnv = 1,
    };

    float gain;             //!< Range 0..1.
    float baseOctaveOffset; //!< Base +/- octave offset, nominal range -2..+3 octaves.
    float baseCentsOffset;  //!< Base +/- cents offset, nominal range -100..+100 cents.
    float startSample;      //!< Range 0..1.
    float endSample;        //!< Range 0..1.
    PlaybackMode playbackMode; //!< Forward or reverse setting.
    EnvMode volumeEnvMode;  //!< One-shot or loop mode for volume envelope.
    float volumeEnvLoopSpeed;   //!< Divisor to compress the a+r time when looping, range 1..n
    float volumeEnvAttack;  //!< Attack time in seconds, nominal range 0..(sample length in seconds).
    float volumeEnvRelease; //!< Release time in seconds, nominal range 0..(sample length in seconds).
    float volumeEnvDepth;    //!< +/- gain env depth, nominal range -1..+1.
    EnvMode pitchEnvMode;   //!< One-shot or loop mode for pitch envelope.
    float pitchEnvLoopSpeed;   //!< Divisor to compress the a+r time when looping, range 1..n
    float pitchEnvAttack;   //!< Attack time in seconds, nominal range 0..(sample length in seconds).
    float pitchEnvRelease;  //!< Release time in seconds, nominal range 0..(sample length in seconds).
    float pitchEnvDepth;    //!< +/- octave env depth, nominal range -2..+2 octaves.

    constexpr VoiceParameters()
    :   gain(1.0f),
        baseOctaveOffset(0.0f),
        baseCentsOffset(0.0f),
        startSample(0.0f),
        endSample(1.0f),
        playbackMode(kForwardPlayback),
        volumeEnvMode(kOneShotEnv),
        volumeEnvLoopSpeed(1.0f),
        volumeEnvAttack(0.0f),
        volumeEnvRelease(0.0f),
        volumeEnvDepth(0.0f),
        pitchEnvMode(kOneShotEnv),
        pitchEnvLoopSpeed(1.0f),
        pitchEnvAttack(0.0f),
        pitchEnvRelease(0.0f),
        pitchEnvDepth(0.0f)
    {
    }
    ~VoiceParameters()=default;
    VoiceParameters(const VoiceParameters & other)
    {
        this->operator=(other);
    }
    VoiceParameters & operator=(const VoiceParameters & other)
    {
        gain = other.gain;
        baseOctaveOffset = other.baseOctaveOffset;
        baseCentsOffset = other.baseCentsOffset;
        startSample = other.startSample;
        endSample = other.endSample;
        playbackMode = other.playbackMode;
        volumeEnvMode = other.volumeEnvMode;
        volumeEnvLoopSpeed = other.volumeEnvLoopSpeed;
        volumeEnvAttack = other.volumeEnvAttack;
        volumeEnvRelease = other.volumeEnvRelease;
        volumeEnvDepth = other.volumeEnvDepth;
        pitchEnvMode = other.pitchEnvMode;
        pitchEnvLoopSpeed = other.pitchEnvLoopSpeed;
        pitchEnvAttack = other.pitchEnvAttack;
        pitchEnvRelease = other.pitchEnvRelease;
        pitchEnvDepth = other.pitchEnvDepth;
        return *this;
    }

    void reset()
    {
        *this = VoiceParameters{};
    }

    bool load(Archive & settings)
    {
        return settings.read("gain", &gain)
            && settings.read("base_octave_offset", &baseOctaveOffset)
            && settings.read("base_cents_offset", &baseCentsOffset)
            && settings.read("start_sample", &startSample)
            && settings.read("end_sample", &endSample)
            && settings.read("playback_mode", &playbackMode)
            && settings.read("volume_env_mode", &volumeEnvMode)
            && settings.read("volume_env_loop_speed", &volumeEnvLoopSpeed)
            && settings.read("volume_env_attack", &volumeEnvAttack)
            && settings.read("volume_env_release", &volumeEnvRelease)
            && settings.read("volume_env_depth", &volumeEnvDepth)
            && settings.read("pitch_env_mode", &pitchEnvMode)
            && settings.read("pitch_env_loop_speed", &pitchEnvLoopSpeed)
            && settings.read("pitch_env_attack", &pitchEnvAttack)
            && settings.read("pitch_env_release", &pitchEnvRelease)
            && settings.read("pitch_env_depth", &pitchEnvDepth);
    }

    bool save(Archive & settings)
    {
        return settings.write("gain", gain)
            && settings.write("base_octave_offset", baseOctaveOffset)
            && settings.write("base_cents_offset", baseCentsOffset)
            && settings.write("start_sample", startSample)
            && settings.write("end_sample", endSample)
            && settings.write("playback_mode", playbackMode)
            && settings.write("volume_env_mode", volumeEnvMode)
            && settings.write("volume_env_loop_speed", volumeEnvLoopSpeed)
            && settings.write("volume_env_attack", volumeEnvAttack)
            && settings.write("volume_env_release", volumeEnvRelease)
            && settings.write("volume_env_depth", volumeEnvDepth)
            && settings.write("pitch_env_mode", pitchEnvMode)
            && settings.write("pitch_env_loop_speed", pitchEnvLoopSpeed)
            && settings.write("pitch_env_attack", pitchEnvAttack)
            && settings.write("pitch_env_release", pitchEnvRelease)
            && settings.write("pitch_env_depth", pitchEnvDepth);
    }
};

} // namespace slab

#endif // _VOICE_PARAMETERS_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
