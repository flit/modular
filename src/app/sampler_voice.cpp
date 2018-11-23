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

#include "sampler_voice.h"
#include "ui.h"
#include "debug_log.h"
#include "itm_trace.h"
#include <cmath>
#include <algorithm>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

//! Set to 1 to cause the voice to automatically and repeatedly trigger.
#define ENABLE_TEST_LOOP_MODE (0)

//! Set to 1 to output buffered time via ITM.
#define ENABLE_BUFFERED_TIME_TRACE (1)

//! Number of samples to fade out.
const uint32_t kNoteOffSamples = 128;

static_assert(kNoteOffSamples % kAudioBufferSize == 0, "note off samples must evenly divide buffer size");

//! Number of cents per octave.
const float kCentsPerOctave = 1200.0f;

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

float SamplerVoice::s_workBufferData[2][kAudioBufferSize];
AudioBuffer SamplerVoice::s_workBuffer(s_workBufferData[0], kAudioBufferSize);
AudioBuffer SamplerVoice::s_workBuffer2(s_workBufferData[1], kAudioBufferSize);

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

SamplerVoice::SamplerVoice()
:   _wav(),
    _data(),
    _manager(),
    _isValid(false),
    _isReady(false),
    _isPlaying(false),
    _doNoteOff(false),
    _doRetrigger(false),
    _noteOffSamplesRemaining(0),
    _readHead(0.0f),
    _pitchOctave(0.0f),
    _params(),
    _triggerMode(TriggerMode::kTrigger),
    _volumeEnv(),
    _pitchEnv(),
    _triggerNoteOffSample(0)
{
}

void SamplerVoice::init(uint32_t n, int16_t * buffer)
{
    _number = n;

    _volumeEnv.set_sample_rate(kSampleRate);
    _volumeEnv.set_curve_type(ASREnvelope::kAttack, AudioRamp::kCubic);
    _volumeEnv.set_mode(ASREnvelope::kOneShotAR);
    _volumeEnv.set_peak(1.0f);

    _pitchEnv.set_sample_rate(kSampleRate);
    _pitchEnv.set_curve_type(ASREnvelope::kAttack, AudioRamp::kCubic);
    _pitchEnv.set_mode(ASREnvelope::kOneShotAR);
    _pitchEnv.set_peak(1.0f);

    _manager.init(this, buffer);
    clear_file();
}

void SamplerVoice::set_file(WaveFile& file)
{
    _isReady = false;
    _reset_voice();
    _params.reset();
    _pitchOctave = 0.0f;
    _wav = file;
    _data = _wav.get_audio_data();
    _isValid = true;

    // Prime the manager to start filling buffers.
    _manager.set_file(_data.get_frames());

#if ENABLE_TEST_LOOP_MODE
    _isPlaying = true;
#endif
}

void SamplerVoice::clear_file()
{
    _isValid = false;
    _isReady = false;
    _reset_voice();
    _params.reset();
    _pitchOctave = 0.0f;
    _wav = WaveFile();
    _data = WaveFile::AudioDataStream();
    _manager.set_file(0);
}

//! @brief Prepare voice to start playing from start sample.
void SamplerVoice::_reset_voice()
{
    _isPlaying = false;
    _doNoteOff = false;
    _doRetrigger = false;
    _noteOffSamplesRemaining = 0;
    _readHead = 0.0f;
    _volumeEnv.trigger();
    _pitchEnv.trigger();
    std::fill_n(&_interpolationBuffer[0], SampleBuffer::kInterpolationFrameCount, 0);
}

void SamplerVoice::prime()
{
    _reset_voice();
    _manager.prime();
}

void SamplerVoice::trigger()
{
    // Ignore the trigger if the manager isn't ready to play.
    if (!is_ready())
    {
        return;
    }

    // Handle re-triggering while sample is already playing.
    if (_isPlaying)
    {
        DEBUG_PRINTF(RETRIG_MASK, "V%lu: retrigger (@%lu)\r\n", _number, _manager.get_samples_played());

        // Start retrigger process.
        _doNoteOff = true;
        _doRetrigger = true;
        _noteOffSamplesRemaining = kNoteOffSamples;
    }
    else
    {
        UI::get().set_voice_playing(_number, true);
        _isPlaying = true;
    }
}

void SamplerVoice::note_off()
{
    // Shouldn't be getting note offs unless we're in gate mode.
    assert(_triggerMode == TriggerMode::kGate);

    // Ignore the note off event if the manager isn't ready to play.
    if (!is_ready())
    {
        return;
    }

    DEBUG_PRINTF(RETRIG_MASK, "V%lu: note off (@%lu)\r\n", _number, _manager.get_samples_played());

    if (_isPlaying)
    {
        // Start note off process.
        _volumeEnv.set_release_offset(0);
    }
}

void SamplerVoice::playing_did_finish()
{
    UI::get().set_voice_playing(_number, false);
    prime();

#if ENABLE_TEST_LOOP_MODE
    trigger();
#endif
}

void SamplerVoice::render(int16_t * data, uint32_t frameCount)
{
    DECLARE_ELAPSED_TIME(total);
    DECLARE_ELAPSED_TIME(render);
    DECLARE_ELAPSED_TIME(retire);
    DECLARE_ELAPSED_TIME(noteOff);

    uint32_t i;

    // Get the current buffer if playing.
    SampleBuffer * voiceBuffer = nullptr;
    if (is_ready() && is_playing())
    {
        voiceBuffer = _manager.get_current_buffer();
    }

    // If not playing, just fill with silence.
    if (!voiceBuffer)
    {
        for (i = 0; i < frameCount; ++i)
        {
            *data = 0;
            data += 2;
        }
        return;
    }

    // Compute the playback rate for this output buffer.
    float pitchModifier = _pitchEnv.next() * _params.pitchEnvDepth;
    float rate = _compute_playback_rate(pitchModifier);

    int16_t * bufferData = voiceBuffer->data;
    float readHead = _readHead;
    uint32_t bufferFrameCount = voiceBuffer->frameCount;
    uint32_t remainingFrames = frameCount;
    assert(readHead < bufferFrameCount);

    START_ELAPSED_TIME(total);

    // Skip ahead to the buffer's zero snap offset. This will only have an effect the file start.
    if (readHead == 0.0f)
    {
        readHead = voiceBuffer->zeroSnapOffset;
    }

    // Render into output buffer.
    while (remainingFrames)
    {
        assert(remainingFrames <= frameCount);
        uint32_t framesAtRate = std::ceil(remainingFrames * rate);
        uint32_t integerReadHead = readHead;
        uint32_t framesRemainingInBuffer = bufferFrameCount - integerReadHead;
        uint32_t outputFrames; // Number of frames we're outputting this time through the loop.

        // Handle case where there are fewer remaining frames in the buffer than
        // the playback rate, in which case we need to just move to the next buffer.
        if (framesRemainingInBuffer < rate)
        {
            readHead += rate;
        }
        else
        {
            if (framesAtRate > framesRemainingInBuffer)
            {
                outputFrames = min(uint32_t(std::floor(framesRemainingInBuffer / rate)), remainingFrames);
            }
            else
            {
                outputFrames = remainingFrames;
            }

            // Start volume env release phase for trigger mode.
            if (_triggerMode == TriggerMode::kTrigger)
            {
                // Will unsigned underflow when we're past the trigger note off sample, preventing
                // us from restarting the volume env release phase.
                uint32_t triggerNoteOffRemainingSamples = _triggerNoteOffSample - _manager.get_samples_played();

                if (triggerNoteOffRemainingSamples < outputFrames)
                {
                    outputFrames = triggerNoteOffRemainingSamples;
                    triggerNoteOffRemainingSamples = ~0UL;
                    _volumeEnv.set_release_offset(0);
                }
            }

            START_ELAPSED_TIME(render);

            // Render sample data into the output buffer at the specified playback rate.
            readHead = voiceBuffer->read_into<SampleBuffer::InterpolationMode::kHermite>(
                        s_workBuffer, outputFrames, readHead, rate);
            s_workBuffer.multiply_scalar(_params.gain, outputFrames);
            _volumeEnv.process(s_workBuffer2, outputFrames);
            s_workBuffer.multiply_vector(s_workBuffer2, outputFrames);
            s_workBuffer.copy_into(data, 2, outputFrames);

            END_ELAPSED_TIME(render);

            remainingFrames -= outputFrames;
            data += outputFrames * 2;
        }

        if (readHead > (bufferFrameCount - 1))
        {
            START_ELAPSED_TIME(retire);

            // Adjust read head.
            readHead -= bufferFrameCount;

            // Temporarily save last few frames from this buffer.
            std::copy_n(&bufferData[bufferFrameCount - SampleBuffer::kInterpolationFrameCount],
                        SampleBuffer::kInterpolationFrameCount,
                        _interpolationBuffer);

            _manager.retire_buffer(voiceBuffer);

            // If we're no longer playing, exit this loop.
            if (!_isPlaying)
            {
                // Set some variables used outside the loop below.
                readHead = 0.0f;
                break;
            }

            // Get the next buffer and update locals.
            voiceBuffer = _manager.get_current_buffer();
            if (!voiceBuffer)
            {
                break;
            }
            bufferData = voiceBuffer->data;
            bufferFrameCount = voiceBuffer->frameCount;

            // Copy saved frames to beginning of new buffer.
            std::copy_n(_interpolationBuffer,
                        SampleBuffer::kInterpolationFrameCount,
                        voiceBuffer->dataWithInterpolationFrames);

            END_ELAPSED_TIME(retire);
        }
        assert(readHead < bufferFrameCount);
    }

    // Save local read head.
    _readHead = readHead;

    // If the volume env has run out, we're done playing.
    if (_volumeEnv.is_finished())
    {
        _doNoteOff = true;
        _noteOffSamplesRemaining = 0;
    }

    // Handle end of note off and retriggering.
    if (_doNoteOff)
    {
        START_ELAPSED_TIME(noteOff);

        // Apply fade out.
        for (i = 0; i < min(_noteOffSamplesRemaining, frameCount); ++i)
        {
            int32_t sample = static_cast<int32_t>(data[i]);
            sample = sample * (_noteOffSamplesRemaining - i) / kNoteOffSamples;
            data[i] = static_cast<int16_t>(sample);
        }
        _noteOffSamplesRemaining -= i;

        if (_noteOffSamplesRemaining == 0)
        {
            // Prime will clear all flags for us, including _doNoteOff and _doRetrigger,
            // so save retrigger locally to set isPlaying after priming.
            bool savedRetrigger = _doRetrigger;
            prime();

            if (savedRetrigger)
            {
                _isPlaying = true;
                UI::get().indicate_voice_retriggered(_number);
            }
        }

        END_ELAPSED_TIME(noteOff);
    }

    END_ELAPSED_TIME(total);

    // Fill any leftover frames with silence.
    while (remainingFrames--)
    {
        *data = 0;
        data += 2;
    }

    // Did we finish this buffer?
    if (_readHead >= bufferFrameCount && voiceBuffer)
    {
        // Adjust read head.
        _readHead -= bufferFrameCount;

        // Temporarily save last few frames from this buffer.
        std::copy_n(&bufferData[bufferFrameCount - SampleBuffer::kInterpolationFrameCount],
                    SampleBuffer::kInterpolationFrameCount,
                    _interpolationBuffer);

        _manager.retire_buffer(voiceBuffer);

        // Copy saved frames to beginning of new buffer.
        voiceBuffer = _manager.get_current_buffer();
        if (voiceBuffer)
        {
            std::copy_n(_interpolationBuffer,
                        SampleBuffer::kInterpolationFrameCount,
                        voiceBuffer->dataWithInterpolationFrames);
        }
    }
    assert(_readHead < bufferFrameCount);

    // If we're no longer playing, tell the UI.
    if (!_isPlaying)
    {
        UI::get().set_voice_playing(_number, false);
    }

#if ENABLE_BUFFERED_TIME_TRACE
    _trace_buffered_time();
#endif
}

void SamplerVoice::set_sample_start(float start)
{
    // Stop playing and turn off LED.
    _reset_voice();
    UI::get().set_voice_playing(_number, false);

    _params.startSample = start;

    // Tell sample manager to set and load new start point.
    uint32_t sample = float(_manager.get_total_samples()) * start;
    _manager.set_start_end_sample(sample, -1);
}

void SamplerVoice::set_sample_end(float end)
{
    // Stop playing and turn off LED.
    _reset_voice();
    UI::get().set_voice_playing(_number, false);

    _params.endSample = end;

    uint32_t sample = float(_manager.get_total_samples()) * end;
    _manager.set_start_end_sample(-1, sample);
}

void SamplerVoice::set_playback_mode(VoiceParameters::PlaybackMode mode)
{
    _params.playbackMode = mode;
}

void SamplerVoice::set_trigger_mode(TriggerMode mode)
{
    _triggerMode = mode;

    if (mode == TriggerMode::kTrigger)
    {
        _volumeEnv.set_mode(ASREnvelope::kOneShotAR);
    }
    else
    {
        _volumeEnv.set_mode(ASREnvelope::kOneShotASR);
    }

    // Update volume env release time.
    set_volume_env_release(_params.volumeEnvRelease);
}

void SamplerVoice::set_volume_env_mode(VoiceParameters::EnvMode mode)
{
    _params.volumeEnvMode = mode;
    switch (mode)
    {
        case VoiceParameters::kOneShotEnv:
            // In one shot env mode, the volume env behaviour changes based
            // on whether the voice is in trigger or gate mode.
            switch (_triggerMode)
            {
                case TriggerMode::kTrigger:
                    _volumeEnv.set_mode(ASREnvelope::kOneShotAR);
                    break;
                case TriggerMode::kGate:
                    _volumeEnv.set_mode(ASREnvelope::kOneShotASR);
                    break;
            }
            break;
        case VoiceParameters::kLoopEnv:
            // In looping env mode, the volume env always loops regarless
            // of whether in trigger or gate mode.
            _volumeEnv.set_mode(ASREnvelope::kLoopingAR);
            break;
    }
}

void SamplerVoice::set_volume_env_attack(float seconds)
{
    _params.volumeEnvAttack = seconds;
    _volumeEnv.set_attack(seconds);
}

void SamplerVoice::set_volume_env_release(float seconds)
{
    _params.volumeEnvRelease = seconds;

    if (_triggerMode == TriggerMode::kTrigger)
    {
        _volumeEnv.set_release(_params.volumeEnvRelease);
    }
    else
    {
        _volumeEnv.set_release(max(_params.volumeEnvRelease, 128.0f / kSampleRate));
    }

    _triggerNoteOffSample = _data.get_frames() - static_cast<uint32_t>(seconds * kSampleRate);
}

void SamplerVoice::set_pitch_env_mode(VoiceParameters::EnvMode mode)
{
    _params.pitchEnvMode = mode;
    if (mode == VoiceParameters::kOneShotEnv)
    {
        _pitchEnv.set_mode(ASREnvelope::kOneShotAR);
    }
    else if (mode == VoiceParameters::kLoopEnv)
    {
        _pitchEnv.set_mode(ASREnvelope::kLoopingAR);
    }
}

void SamplerVoice::set_pitch_env_attack(float seconds)
{
    _params.pitchEnvAttack = seconds;

    // The pitch envelope update rate is once per audio buffer, so modify the attack time
    // to take this into account.
    _pitchEnv.set_attack(seconds / float(kAudioBufferSize));
}

void SamplerVoice::set_pitch_env_release(float seconds)
{
    _params.pitchEnvRelease = seconds;

    // The pitch envelope update rate is once per audio buffer, so modify the release time
    // to take this into account.
    _pitchEnv.set_release(seconds / float(kAudioBufferSize));
}

void SamplerVoice::set_params(const VoiceParameters & params)
{
    // Stop playing and turn off LED.
    _reset_voice();
    UI::get().set_voice_playing(_number, false);

    // Update params.
    _params = params;

    // Update params that require computation.
    set_volume_env_mode(_params.volumeEnvMode);
    set_volume_env_attack(_params.volumeEnvAttack);
    set_volume_env_release(_params.volumeEnvRelease);
    set_pitch_env_mode(_params.pitchEnvMode);
    set_pitch_env_attack(_params.pitchEnvAttack);
    set_pitch_env_release(_params.pitchEnvRelease);

    // Update playback range in SBM.
    float totalSamples = _manager.get_total_samples();
    uint32_t start = totalSamples * _params.startSample;
    uint32_t end = totalSamples * _params.endSample;
    _manager.set_start_end_sample(start, end);
}

//! @brief Restore a parameter to its default value.
void SamplerVoice::reset_parameter(VoiceParameters::ParameterName which)
{
    switch (which)
    {
        case VoiceParameters::kGain:
            set_gain(1.0f);
            break;
        case VoiceParameters::kBaseOctave:
            set_base_octave_offset(0.0f);
            break;
        case VoiceParameters::kBaseCents:
            set_base_cents_offset(0.0f);
            break;
        case VoiceParameters::kSampleStart:
            set_sample_start(0.0f);
            break;
        case VoiceParameters::kSampleEnd:
            set_sample_end(1.0f);
            break;
        case VoiceParameters::kPlaybackMode:
            set_playback_mode(VoiceParameters::kForwardPlayback);
            break;
        case VoiceParameters::kVolumeEnvAttack:
            set_volume_env_attack(0.0f);
            break;
        case VoiceParameters::kVolumeEnvRelease:
            set_volume_env_release(0.0f);
            break;
        case VoiceParameters::kVolumeEnvDepth:
            set_volume_env_depth(0.0f);
            break;
        case VoiceParameters::kVolumeEnvMode:
            set_volume_env_mode(VoiceParameters::kOneShotEnv);
            break;
        case VoiceParameters::kPitchEnvAttack:
            set_pitch_env_attack(0.0f);
            break;
        case VoiceParameters::kPitchEnvRelease:
            set_pitch_env_release(0.0f);
            break;
        case VoiceParameters::kPitchEnvDepth:
            set_pitch_env_depth(0.0f);
            break;
        case VoiceParameters::kPitchEnvMode:
            set_pitch_env_mode(VoiceParameters::kOneShotEnv);
            break;
        case VoiceParameters::kUnused:
        default:
            break;
    }
}

float SamplerVoice::get_sample_length_in_seconds() const
{
    return float(_data.get_frames()) / kSampleRate;
}

uint32_t SamplerVoice::get_buffered_microseconds() const
{
    float bufferedSamples = static_cast<float>(_manager.get_buffered_samples()) - _readHead;
    float rate = _compute_playback_rate(0.0f);
    float seconds = bufferedSamples / rate / kSampleRate;
    return static_cast<uint32_t>(seconds * 1000000.0f);
}

float SamplerVoice::_compute_playback_rate(float pitchModifier) const
{
    float octave = _params.baseOctaveOffset
                    + (_params.baseCentsOffset / kCentsPerOctave)
                    + _pitchOctave
                    + pitchModifier;
    constrain(octave, -4.0f, 4.0f);
    return powf(2.0f, octave);
}

void SamplerVoice::_trace_buffered_time()
{
    // Event data consists of:
    // [31:30] = 2-bit channel number
    // [29]    = playing flag
    // [28:0]  = 28-bits of buffered microseconds
    itm<kBufferedTimeChannel, uint32_t>::send((_number << 30)
                    | (static_cast<uint32_t>(_isPlaying) << 29)
                    | (get_buffered_microseconds() & 0x0fffffff));
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
