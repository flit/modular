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
:   _number(0),
    _listener(nullptr),
    _wav(),
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
    _triggerNoteOffSample(0),
    _voiceStatusRetriggerCounter(0)
{
}

void SamplerVoice::init(uint32_t n, int16_t * buffer)
{
    _number = n;

    _volumeEnv.set_sample_rate(kSampleRate);
    _volumeEnv.set_curve_type(ASREnvelope::kAttack, AudioRamp::kLinear);
    _volumeEnv.set_curve_type(ASREnvelope::kRelease, AudioRamp::kLinear);
    _volumeEnv.set_mode(ASREnvelope::kOneShotASR);
    _volumeEnv.set_peak(1.0f);
    _volumeEnv.recompute();

    _pitchEnv.set_sample_rate(kSampleRate);
    _pitchEnv.set_curve_type(ASREnvelope::kAttack, AudioRamp::kLinear);
    _pitchEnv.set_curve_type(ASREnvelope::kRelease, AudioRamp::kLinear);
    _pitchEnv.set_mode(ASREnvelope::kOneShotAR);
    _pitchEnv.set_peak(1.0f);
    _pitchEnv.recompute();

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
    if (_isPlaying)
    {
        if (_listener)
        {
            _listener->voice_did_change_playing_state(_number, false);
        }
    }
    _isPlaying = false;
    _doNoteOff = false;
    _doRetrigger = false;
    _noteOffSamplesRemaining = 0;
    _readHead = 0.0f;
    _volumeEnv.reset();
    _pitchEnv.reset();
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

        if (_listener)
        {
            _listener->voice_did_change_playing_state(_number, false);
        }

        _voiceStatusRetriggerCounter = 50.0f / 1000.0f * kSampleRate; // 50 ms
    }
    else
    {
        if (_listener)
        {
            _listener->voice_did_change_playing_state(_number, true);
        }
        _isPlaying = true;
        _voiceStatusRetriggerCounter = 0;
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
    // Prime will call _reset_voice() which will inform the listener.
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

    int16_t *initialData = data;
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

    // Compute volume envelope parameters.
    bool isVolumeEnvNegative = (_params.volumeEnvDepth < 0.0f);
    float adjustedVolumeEnvDepth = fabsf(_params.volumeEnvDepth);
    float volumeEnvOffset = 1.0f - adjustedVolumeEnvDepth;

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

    // Set volume env release phase offset for trigger mode if the env release
    // should start within this render buffer. This isn't necessary when the volume
    // env is in looping mode since it doesn't use sustain in that case.
    if (_triggerMode == TriggerMode::kTrigger
        && _params.volumeEnvMode != VoiceParameters::kLoopEnv)
    {
        // Will unsigned underflow when we're past the trigger note off sample, preventing
        // us from restarting the volume env release phase.
        uint32_t triggerNoteOffRemainingSamples = _triggerNoteOffSample - (_manager.get_samples_played() + uint32_t(readHead));

        if (triggerNoteOffRemainingSamples < frameCount)
        {
            _volumeEnv.set_release_offset(triggerNoteOffRemainingSamples);
        }
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

            // Create local work buffers for the number of output frames we need.
            AudioBuffer localWorkBuffer(s_workBuffer, outputFrames);
            AudioBuffer localWorkBuffer2(s_workBuffer2, outputFrames);

            START_ELAPSED_TIME(render);

            // Render sample data into the output buffer at the specified playback rate.
            readHead = voiceBuffer->read_into<SampleBuffer::InterpolationMode::kHermite>(
                        localWorkBuffer, outputFrames, readHead, rate);

            // Apply volume envelope and gain.
            _volumeEnv.process(localWorkBuffer2, outputFrames);
            localWorkBuffer2.multiply_scalar(adjustedVolumeEnvDepth);
            if (isVolumeEnvNegative)
            {
                localWorkBuffer2.negate();
                localWorkBuffer2.add_scalar(1.0f);
            }
            else
            {
                localWorkBuffer2.add_scalar(volumeEnvOffset);
            }
            localWorkBuffer.multiply_vector(s_workBuffer2);
            localWorkBuffer.multiply_scalar(_params.gain);

            // Convert from float to int16 while copying from work buffer to output buffer.
            localWorkBuffer.copy_into(data, 2, outputFrames);

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
    if (_volumeEnv.is_finished() && !_doNoteOff)
    {
        _doNoteOff = true;
        _noteOffSamplesRemaining = kNoteOffSamples;
    }

    // Handle end of note off and retriggering.
    if (_doNoteOff)
    {
        START_ELAPSED_TIME(noteOff);

        // Apply fade out.
        int16_t *fadeData = initialData;
        for (i = 0; i < min(_noteOffSamplesRemaining, frameCount); ++i)
        {
            int32_t sample = static_cast<int32_t>(*fadeData);
            sample = sample * (_noteOffSamplesRemaining - i) / kNoteOffSamples;
            *fadeData = static_cast<int16_t>(sample);
            fadeData += 2;
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
            }
        }

        END_ELAPSED_TIME(noteOff);
    }

    if (_voiceStatusRetriggerCounter)
    {
        _voiceStatusRetriggerCounter -= frameCount;
        if (_voiceStatusRetriggerCounter <= 0)
        {
            if (_listener)
            {
                _listener->voice_did_change_playing_state(_number, true);
            }
            _voiceStatusRetriggerCounter = 0;
        }
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
        if (_listener)
        {
            _listener->voice_did_change_playing_state(_number, false);
        }
    }

#if ENABLE_BUFFERED_TIME_TRACE
    _trace_buffered_time();
#endif
}

void SamplerVoice::set_base_octave_offset(float octave)
{
    _params.baseOctaveOffset = octave;

    // Update all envelope stages that are affected by base pitch.
    set_volume_env_attack(_params.volumeEnvAttack);
    set_volume_env_release(_params.volumeEnvRelease);
    set_pitch_env_attack(_params.pitchEnvAttack);
    set_pitch_env_release(_params.pitchEnvRelease);
}

void SamplerVoice::set_sample_start(float start)
{
    // Stop playing and turn off LED.
    _reset_voice();

    _params.startSample = start;

    // Tell sample manager to set and load new start point.
    uint32_t sample = float(_manager.get_total_samples()) * start;
    _manager.set_start_end_sample(sample, -1);
}

void SamplerVoice::set_sample_end(float end)
{
    // Stop playing and turn off LED.
    _reset_voice();

    _params.endSample = end;

    uint32_t sample = float(_manager.get_total_samples()) * end;
    _manager.set_start_end_sample(-1, sample);
}

void SamplerVoice::set_playback_mode(VoiceParameters::PlaybackMode mode)
{
    _params.playbackMode = mode;

    _reset_voice();

    switch (mode)
    {
        case VoiceParameters::kForwardPlayback:
            _manager.set_direction(SampleBufferManager::Direction::kForward);
            break;
        case VoiceParameters::kReversePlayback:
            _manager.set_direction(SampleBufferManager::Direction::kReverse);
            break;
    }
}

void SamplerVoice::set_trigger_mode(TriggerMode mode)
{
    _triggerMode = mode;

    // Update volume env release time.
    set_volume_env_release(_params.volumeEnvRelease);
}

void SamplerVoice::set_volume_env_mode(VoiceParameters::EnvMode mode)
{
    _params.volumeEnvMode = mode;
    switch (mode)
    {
        case VoiceParameters::kOneShotEnv:
            _volumeEnv.set_mode(ASREnvelope::kOneShotASR);
            break;
        case VoiceParameters::kLoopEnv:
            // In looping env mode, the volume env always loops regardless
            // of whether in trigger or gate mode.
            _volumeEnv.set_mode(ASREnvelope::kLoopingAR);
            break;
    }
}

void SamplerVoice::set_volume_env_loop_speed(float speed)
{
    _params.volumeEnvLoopSpeed = speed;

    if (_params.volumeEnvMode == VoiceParameters::kLoopEnv)
    {
        set_volume_env_attack(_params.volumeEnvAttack);
        set_volume_env_release(_params.volumeEnvRelease);
    }
}

void SamplerVoice::set_volume_env_attack(float seconds)
{
    _params.volumeEnvAttack = seconds;

    float rate = _compute_playback_rate(0.0f, false);
    float loopCompression = (_params.volumeEnvMode == VoiceParameters::kLoopEnv)
                            ? _params.volumeEnvLoopSpeed
                            : 1.0f;
    _volumeEnv.set_attack(seconds / rate / loopCompression);
    _volumeEnv.recompute();
}

void SamplerVoice::set_volume_env_release(float seconds)
{
    _params.volumeEnvRelease = seconds;

    float rate = _compute_playback_rate(0.0f, false);
    float loopCompression = (_params.volumeEnvMode == VoiceParameters::kLoopEnv)
                            ? _params.volumeEnvLoopSpeed
                            : 1.0f;
    _volumeEnv.set_release(seconds / rate / loopCompression);
    _volumeEnv.recompute();

    _triggerNoteOffSample = _data.get_frames()
                             - _volumeEnv.get_length_in_samples(ASREnvelope::kRelease);
}

void SamplerVoice::set_pitch_env_mode(VoiceParameters::EnvMode mode)
{
    _params.pitchEnvMode = mode;
    switch (mode)
    {
        case VoiceParameters::kOneShotEnv:
            _pitchEnv.set_mode(ASREnvelope::kOneShotAR);
            break;
        case VoiceParameters::kLoopEnv:
            _pitchEnv.set_mode(ASREnvelope::kLoopingAR);
            break;
    }
}

void SamplerVoice::set_pitch_env_loop_speed(float speed)
{
    _params.pitchEnvLoopSpeed = speed;

    if (_params.pitchEnvMode == VoiceParameters::kLoopEnv)
    {
        set_pitch_env_attack(_params.pitchEnvAttack);
        set_pitch_env_release(_params.pitchEnvRelease);
    }
}

void SamplerVoice::set_pitch_env_attack(float seconds)
{
    _params.pitchEnvAttack = seconds;

    float rate = _compute_playback_rate(0.0f, false);
    float loopCompression = (_params.pitchEnvMode == VoiceParameters::kLoopEnv)
                            ? _params.pitchEnvLoopSpeed
                            : 1.0f;
    // The pitch envelope update rate is once per audio buffer, so modify the attack time
    // to take this into account.
    _pitchEnv.set_attack(seconds / rate / loopCompression / float(kAudioBufferSize));
    _pitchEnv.recompute();
}

void SamplerVoice::set_pitch_env_release(float seconds)
{
    _params.pitchEnvRelease = seconds;

    float rate = _compute_playback_rate(0.0f, false);
    float loopCompression = (_params.pitchEnvMode == VoiceParameters::kLoopEnv)
                            ? _params.pitchEnvLoopSpeed
                            : 1.0f;
    // The pitch envelope update rate is once per audio buffer, so modify the release time
    // to take this into account.
    _pitchEnv.set_release(seconds / rate / loopCompression / float(kAudioBufferSize));
    _pitchEnv.recompute();
}

void SamplerVoice::set_params(const VoiceParameters & params)
{
    // Stop playing and turn off LED.
    _reset_voice();

    // Update params.
    _params = params;

    // Update params that require computation.
    set_volume_env_mode(_params.volumeEnvMode);
    set_pitch_env_mode(_params.pitchEnvMode);
    set_base_octave_offset(_params.baseOctaveOffset); // Also recomputes all env stages.

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
            set_volume_env_loop_speed(1.0f);
            break;
        case VoiceParameters::kVolumeEnvLoopSpeed:
            set_volume_env_loop_speed(1.0f);
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
            set_pitch_env_loop_speed(1.0f);
            break;
        case VoiceParameters::kPitchEnvLoopSpeed:
            set_pitch_env_loop_speed(1.0f);
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
    float rate = _compute_playback_rate();
    float seconds = bufferedSamples / rate / kSampleRate;
    return static_cast<uint32_t>(seconds * 1000000.0f);
}

float SamplerVoice::_compute_playback_rate(float pitchModifier, bool includePitchOctave) const
{
    float octave = _params.baseOctaveOffset
                    + (_params.baseCentsOffset / kCentsPerOctave)
                    + (includePitchOctave ? _pitchOctave : 0.0f)
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
