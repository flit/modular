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

#include "argon/argon.h"
#include "wav_file.h"
#include "protected_queue.h"
#include "audio_defs.h"
#include "audio_buffer.h"
#include "serializer.h"
#include "asr_envelope.h"
#include <assert.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

class SamplerVoice;

/*!
 * @brief A buffer of sample data read from the file.
 */
struct SampleBuffer
{
    //! @brief Extra samples required for interpolation.
    static const uint32_t kInterpolationFrameCount = 3;

    //! @brief States of the buffer.
    enum class State : uint8_t
    {
        kUnused,
        kStartFile,
        kPlaying,
        kReady,
        kFree,
        kReading,
    };

    //! @brief Options for interpolation.
    enum class InterpolationMode
    {
        kLinear,
        kCubic,
        kHermite,
    };

    uint8_t number; //!< Sample buffer number, mostly for debugging.
    State state;    //!< Current state of the buffer.
    bool reread;    //!< Whether to auto re-queue the buffer for reading after current read is finished.
    int16_t * data; //!< Pointer to the buffer's data, post interpolation frames. The size is #kVoiceBufferSize.
    int16_t * dataWithInterpolationFrames;  //!< Pointer to interpolation frames and data that follows.
    uint32_t startFrame;    //!< Frame number within the source file for this buffer's first frame.
    uint32_t zeroSnapOffset;    //!< Offset frame snapped to zero. This is frame from which we should start reading. Only applies to the file start buffer (will be zero for all others).
    uint32_t frameCount;    //!< Number of valid frames within this buffer.

    //! @brief Set the buffer to unused state and clear variant members.
    void set_unused()
    {
        state = SampleBuffer::State::kUnused;
        startFrame = 0;
        zeroSnapOffset = 0;
        frameCount = 0;
        reread = false;
    }

    //! @brief Fill a buffer with interpolated samples.
    template <InterpolationMode mode>
    float read_into(float * buffer, uint32_t count, float fractionalFrame, float rate, const float * preBufferFrames)
    {
        assert(fractionalFrame + float(count) * rate < frameCount + 1);
        uint32_t n;
        for (n = 0; n < count; ++n)
        {
            *buffer++ = read<mode>(fractionalFrame, preBufferFrames);
            fractionalFrame += rate;
        }
        return fractionalFrame;
    }

    //! @brief Return one interpolated sample at a fractional position within this buffer.
    template <InterpolationMode mode>
    float read(float fractionalFrame, const float * preBufferFrames)
    {
        uint32_t intOffset = static_cast<uint32_t>(fractionalFrame);
        float fractOffset = fractionalFrame - static_cast<float>(intOffset);

        float x0;
        float x1;

        if (mode == InterpolationMode::kLinear)
        {
            // Linear interpolator.
            if (intOffset == 0)
            {
                x0 = preBufferFrames[2];
            }
            else
            {
                x0 = data[intOffset - 1];
            }
            x1 = data[intOffset];
            return (x0 + fractOffset * (x1 - x0));
        }
        else
        {
            // Read the 4 points used for interpolation.
            float xm1 = dataWithInterpolationFrames[intOffset];
            x0 = dataWithInterpolationFrames[intOffset + 1];
            x1 = dataWithInterpolationFrames[intOffset + 2];
            float x2 = dataWithInterpolationFrames[intOffset + 3];

            if (mode == InterpolationMode::kCubic)
            {
                // 4 point cubic interpolator.
                float a0 = x2 - x1 - xm1 + x0;
                float a1 = xm1 - x0 - a0;
                float a2 = x1 - xm1;
                return ((a0 * (fractOffset * fractOffset * fractOffset)) + (a1 * (fractOffset * fractOffset)) + (a2 * fractOffset) + x0);
            }
            else
            {
                // 4 point, 3rd order Hermite interpolator by Laurent de Soras.
                float c = (x1 - xm1) * 0.5f;
                float v = x0 - x1;
                float w = c + v;
                float a = w + v + (x2 - x0) * 0.5f;
                float wa = w + a;
                return ((((a * fractOffset) - wa) * fractOffset + c) * fractOffset + x0);
            }
        }
    }
};

/*!
 * @brief Manages the queue of sample data buffers and coordination with the reader thread.
 */
class SampleBufferManager
{
public:
    SampleBufferManager();
    ~SampleBufferManager()=default;

    void init(SamplerVoice * voice, int16_t * buffer);
    void set_file(uint32_t totalFrames);

    //! @brief Prepare for playback from start.
    void prime();

    //! @brief Modify the sample playback range.
    //!
    //! Out of range and out of order values are accepted. Start and end are limited so that start
    //! cannot be greater than end, and vice versa. The start and end are also limited to the valid
    //! total sample range of the data.
    //!
    //! @param start Sample number to start playing from, or -1 to use current value.
    //! @param start Sample number at which playback will cease. Pass -1 to use current value.
    void set_start_end_sample(int32_t start, int32_t end);

    bool is_ready() const { return _isReady; }
    SampleBuffer * get_current_buffer();
    SampleBuffer * get_empty_buffer();

    void enqueue_full_buffer(SampleBuffer * buffer);
    void retire_buffer(SampleBuffer * buffer);

    uint32_t get_total_samples() const { return _totalSamples; }
    uint32_t get_active_samples() const { return _endSample - _startSample; }
    uint32_t get_start_sample() const { return _startSample; }
    uint32_t get_end_sample() const { return _endSample; }
    uint32_t get_samples_played() const { return _samplesPlayed; }
    uint32_t get_samples_read() const { return _samplesRead; }
    uint32_t get_samples_queued() const { return _samplesQueued; }

    uint32_t get_buffered_samples() const;

protected:
    typedef SimpleQueue<SampleBuffer*, kVoiceBufferCount> BufferQueue;

    SamplerVoice * _voice;
    uint32_t _number;
    SampleBuffer _buffer[kVoiceBufferCount];
    BufferQueue _fullBuffers;
    BufferQueue _emptyBuffers;
    Ar::Mutex _primeMutex;
    SampleBuffer * _currentBuffer;
    uint32_t _activeBufferCount;
    uint32_t _totalSamples;
    uint32_t _startSample;
    uint32_t _endSample;
    uint32_t _samplesPlayed;
    uint32_t _samplesRead;
    uint32_t _samplesQueued;
    bool _didReadFileStart;
    bool _waitingForFileStart;
    bool _isReady;
    bool _snapToZeroStart;
    uint32_t _preppedCount;

    void _reset_buffers();

    void _queue_buffer_for_read(SampleBuffer * buffer);
    SampleBuffer * _dequeue_next_buffer();

    void _find_zero_crossing(SampleBuffer * buffer);

    void _trace_buffers();
};

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
        kPitchEnvAttack,
        kPitchEnvRelease,
        kPitchEnvDepth,
        kPitchEnvMode,
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
    float volumeEnvAttack;  //!< Attack time in seconds, nominal range 0..(sample length in seconds).
    float volumeEnvRelease; //!< Release time in seconds, nominal range 0..(sample length in seconds).
    float volumeEnvDepth;    //!< +/- gain env depth, nominal range -1..+1.
    EnvMode pitchEnvMode;   //!< One-shot or loop mode for pitch envelope.
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
        volumeEnvAttack(0.0f),
        volumeEnvRelease(0.0f),
        volumeEnvDepth(0.0f),
        pitchEnvMode(kOneShotEnv),
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
        volumeEnvAttack = other.volumeEnvAttack;
        volumeEnvRelease = other.volumeEnvRelease;
        volumeEnvDepth = other.volumeEnvDepth;
        pitchEnvMode = other.pitchEnvMode;
        pitchEnvAttack = other.pitchEnvAttack;
        pitchEnvRelease = other.pitchEnvRelease;
        pitchEnvDepth = other.pitchEnvDepth;
        return *this;
    }

    void reset()
    {
        gain = 1.0f;
        baseOctaveOffset = 0.0f;
        baseCentsOffset = 0.0f;
        startSample = 0.0f;
        endSample = 1.0f;
        playbackMode = kForwardPlayback;
        volumeEnvMode = kOneShotEnv;
        volumeEnvAttack = 0.0f;
        volumeEnvRelease = 0.0f;
        volumeEnvDepth = 0.0f;
        pitchEnvMode = kOneShotEnv;
        pitchEnvAttack = 0.0f;
        pitchEnvRelease = 0.0f;
        pitchEnvDepth = 0.0f;
    }

    bool load(Archive & settings)
    {
        return settings.read("base_octave_offset", &baseOctaveOffset)
            && settings.read("base_cents_offset", &baseCentsOffset)
            && settings.read("start_sample", &startSample)
            && settings.read("end_sample", &endSample)
            && settings.read("playback_mode", &playbackMode)
            && settings.read("volume_env_mode", &volumeEnvMode)
            && settings.read("volume_env_attack", &volumeEnvAttack)
            && settings.read("volume_env_release", &volumeEnvRelease)
            && settings.read("volume_env_depth", &volumeEnvDepth)
            && settings.read("pitch_env_mode", &pitchEnvMode)
            && settings.read("pitch_env_attack", &pitchEnvAttack)
            && settings.read("pitch_env_release", &pitchEnvRelease)
            && settings.read("pitch_env_depth", &pitchEnvDepth);
    }

    bool save(Archive & settings)
    {
        return settings.write("base_octave_offset", baseOctaveOffset)
            && settings.write("base_cents_offset", baseCentsOffset)
            && settings.write("start_sample", startSample)
            && settings.write("end_sample", endSample)
            && settings.write("playback_mode", playbackMode)
            && settings.write("volume_env_mode", &volumeEnvMode)
            && settings.write("volume_env_attack", volumeEnvAttack)
            && settings.write("volume_env_release", volumeEnvRelease)
            && settings.write("volume_env_depth", volumeEnvDepth)
            && settings.write("pitch_env_mode", pitchEnvMode)
            && settings.write("pitch_env_attack", pitchEnvAttack)
            && settings.write("pitch_env_release", pitchEnvRelease)
            && settings.write("pitch_env_depth", pitchEnvDepth);
    }
};

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
    void set_base_octave_offset(float octave) { _params.baseOctaveOffset = octave; }
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

    static float s_workBufferData[2][kAudioBufferSize];
    static AudioBuffer s_workBuffer;
    static AudioBuffer s_workBuffer2;

    float _interpolationBuffer[SampleBuffer::kInterpolationFrameCount]; //!< Last few samples from previous buffer.

    void _reset_voice();

    float _compute_playback_rate(float pitchModifier) const;

    void _trace_buffered_time();
};

} // namespace slab

#endif // _SAMPLER_VOICE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
