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
#include "reader_thread.h"
#include "ui.h"
#include "debug_log.h"
#include "utility.h"
#include <cmath>
#include <algorithm>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

//! Set to 1 to cause the voice to automatically and repeatedly trigger.
#define ENABLE_TEST_LOOP_MODE (0)

//! Number of samples to fade in when we can't find a zero crossing.
const uint32_t kFadeInSampleCount = 128;

//! Number of samples to fade out.
const uint32_t kNoteOffSamples = 128;

static_assert(kNoteOffSamples % kAudioBufferSize == 0, "note off samples must evenly divide buffer size");

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

float SamplerVoice::s_workBufferData[2][kAudioBufferSize];
AudioBuffer SamplerVoice::s_workBuffer(s_workBufferData[0], kAudioBufferSize);
AudioBuffer SamplerVoice::s_workBuffer2(s_workBufferData[1], kAudioBufferSize);

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

SampleBufferManager::SampleBufferManager()
:   _number(0),
    _fullBuffers(),
    _emptyBuffers(),
    _primeMutex(),
    _currentBuffer(nullptr),
    _activeBufferCount(0),
    _totalSamples(0),
    _startSample(0),
    _samplesPlayed(0),
    _samplesRead(0),
    _samplesQueued(0),
    _didReadFileStart(false),
    _waitingForFileStart(true),
    _isReady(false),
    _snapToZeroStart(false),
    _snapToZeroEnd(false),
    _preppedCount(0)
{
}

void SampleBufferManager::init(SamplerVoice * voice, int16_t * buffer)
{
    _voice = voice;
    _number = voice->get_number();
    _primeMutex.init("prime");

    // Init invariant buffer fields.
    uint32_t i;
    for (i = 0; i < kBufferCount; ++i)
    {
        _buffer[i].number = i;
        _buffer[i].data = &buffer[i * kBufferSize];
    }

    set_file(0);
}

//! @brief Reset all buffers to unused.
void SampleBufferManager::_reset_buffers()
{
    uint32_t i;
    for (i = 0; i < kBufferCount; ++i)
    {
        _buffer[i].set_unused();
    }
}

void SampleBufferManager::set_file(uint32_t totalFrames)
{
    _startSample = 0;
    _totalSamples = totalFrames;
    _endSample = totalFrames;
    _activeBufferCount = min(round_up_div(_totalSamples, kBufferSize), kBufferCount);
    _currentBuffer = nullptr;
    _samplesPlayed = 0;
    _samplesRead = 0;
    _samplesQueued = 0;
    _didReadFileStart = false;
    _waitingForFileStart = true;
    _isReady = false;
    _snapToZeroStart = false;
    _snapToZeroEnd = false;
    _preppedCount = 0;
    _reset_buffers();

    // Go ahead and prime.
    prime();
}

void SampleBufferManager::prime()
{
    Ar::Mutex::Guard guard(_primeMutex);
    DEBUG_PRINTF(QUEUE_MASK, "V%lu: start prime\r\n", _number);

    // Reset state.
    _samplesPlayed = _startSample;
    _samplesRead = _didReadFileStart ? min(_startSample + kBufferSize, _totalSamples) : _startSample;
    _samplesQueued = _samplesRead;

    // Clear buffers queues.
    ReaderThread::get().clear_voice_queue(_voice);
    _fullBuffers.clear();
    _emptyBuffers.clear();

    // Playing will start from the file start buffer.
    if (_activeBufferCount && _didReadFileStart)
    {
        _currentBuffer = &_buffer[0];
        _currentBuffer->state = SampleBuffer::State::kPlaying;
    }
    else
    {
        _currentBuffer = nullptr;
    }

    // Queue up the rest of the available buffers to be filled.
    uint32_t i = _didReadFileStart ? 1 : 0;
    for (; i < _activeBufferCount; ++i)
    {
        // If the buffer is currently being filled by the reader thread, then we can't touch
        // it. So just flag it for requeuing when it's finished. This will be handled in
        // enqueue_full_buffer().
        if (_buffer[i].state == SampleBuffer::State::kReading)
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%lu: prime: marking b%lu for reread\r\n", _number, i);
            _buffer[i].reread = true;
        }
        else
        {
            DEBUG_PRINTF(QUEUE_MASK, "V%lu: prime: queuing b%lu for read\r\n", _number, i);
            _queue_buffer_for_read(&_buffer[i]);
        }
    }
    DEBUG_PRINTF(QUEUE_MASK, "V%lu: end prime\r\n", _number);
}

SampleBuffer * SampleBufferManager::get_current_buffer()
{
    if (!_currentBuffer)
    {
        _currentBuffer = _dequeue_next_buffer();
    }
    return _currentBuffer;
}

void SampleBufferManager::_queue_buffer_for_read(SampleBuffer * buffer)
{
    buffer->startFrame = _samplesQueued;
    buffer->frameCount = min(kBufferSize, _endSample - _samplesQueued);
    buffer->reread = false;
    buffer->state = SampleBuffer::State::kFree;

    _emptyBuffers.put(buffer);
    _samplesQueued += buffer->frameCount;

    ReaderThread::get().enqueue(_voice);
}

SampleBuffer * SampleBufferManager::get_empty_buffer()
{
    Ar::Mutex::Guard guard(_primeMutex);

    SampleBuffer * buffer;
    if (_emptyBuffers.get(buffer))
    {
        assert(buffer->state == SampleBuffer::State::kFree);
        buffer->state = SampleBuffer::State::kReading;
        return buffer;
    }
    else
    {
        return nullptr;
    }
}

void SampleBufferManager::retire_buffer(SampleBuffer * buffer)
{
    Ar::Mutex::Guard guard(_primeMutex);

    assert(buffer->state == SampleBuffer::State::kPlaying);
    _samplesPlayed += buffer->frameCount;
    buffer->state = (buffer->number == 0) ? SampleBuffer::State::kReady : SampleBuffer::State::kFree;

    if (_samplesPlayed >= _endSample)
    {
        DEBUG_PRINTF(RETIRE_MASK, "V%lu: retiring b%d; played %lu (done)\r\n", _number, buffer->number, _samplesPlayed);

        _voice->playing_did_finish();
    }
    else
    {
        DEBUG_PRINTF(RETIRE_MASK, "V%lu: retiring b%d; played %lu\r\n", _number, buffer->number, _samplesPlayed);

        // Don't queue up file start buffer for reading, and don't queue more than the active
        // number of samples.
        if (buffer->number != 0 && _samplesQueued < _endSample)
        {
            DEBUG_PRINTF(RETIRE_MASK|QUEUE_MASK, "V%lu: retire: queue b%d to read @ %lu\r\n", _number, buffer->number, _samplesPlayed);
            _queue_buffer_for_read(buffer);
        }

        _dequeue_next_buffer();
    }
}

SampleBuffer * SampleBufferManager::_dequeue_next_buffer()
{
    Ar::Mutex::Guard guard(_primeMutex);

    SampleBuffer * buffer;
    if (_fullBuffers.get(buffer))
    {
        assert(buffer->state == SampleBuffer::State::kReady);
        DEBUG_PRINTF(CURBUF_MASK, "V%lu: current buffer = %d\r\n", _number, buffer->number);
        _currentBuffer = buffer;
        buffer->state = SampleBuffer::State::kPlaying;
    }
    else
    {
        DEBUG_PRINTF(ERROR_MASK, "V%lu: *** NO READY BUFFERS ***\r\n", _number);
//         Ar::_halt();
        _currentBuffer = nullptr;
        UI::get().indicate_voice_underflowed(_number);
    }
    return _currentBuffer;
}

//! @todo clean up
void SampleBufferManager::enqueue_full_buffer(SampleBuffer * buffer)
{
    Ar::Mutex::Guard guard(_primeMutex);

    assert(buffer);
    bool isBuffer0 = (buffer->number == 0);
    bool isOutOfOrder = (buffer->startFrame != _samplesRead);

    if (buffer->reread || isOutOfOrder)
    {
        assert(!(isOutOfOrder && !isBuffer0 && !buffer->reread));
        DEBUG_PRINTF(QUEUE_MASK, "V%lu: queuing b%d for reread\r\n", _number, buffer->number);

        if (isBuffer0)
        {
            // If the file start buffer has to be reread, just do another prime. Need to
            // set the buffer state to ready so prime doesn't cause another reread.
            buffer->state = SampleBuffer::State::kReady;
            _didReadFileStart = false;
            prime();
        }
        else
        {
            // Queue up this buffer to be re-read instead of putting it in the full buffers queue.
            // This call will set the buffer state appropriately.
            _queue_buffer_for_read(buffer);
        }
    }
    else
    {
        _samplesRead += buffer->frameCount;
        buffer->state = SampleBuffer::State::kReady;
        buffer->zeroSnapOffset = 0;

        DEBUG_PRINTF(QUEUE_MASK, "V%lu: queuing b%d for play\r\n", _number, buffer->number);
        _fullBuffers.put(buffer);

        if (isBuffer0)
        {
            _didReadFileStart = true;

            if (_snapToZeroStart)
            {
                _find_zero_crossing(buffer);
            }
        }

        // Wait until we get a full set of buffers before allowing play to start.
        if (_waitingForFileStart && isBuffer0)// && (++_preppedCount >= _activeBufferCount))
        {
            _waitingForFileStart = false;
            _isReady = true;
            _voice->manager_ready_did_change(true);
        }
    }
}

//! Finds the first zero crossing, meaning positive to negative or vice versa, or
//! an actual zero sample. If the sample is not zero, it sets the prior sample
//! to zero. The zero sample position is set in the buffer's zeroSnapOffset member.
//!
//! @note The buffer must have a frame count of at least 2, or this method will do nothing.
void SampleBufferManager::_find_zero_crossing(SampleBuffer * buffer)
{
    if (buffer->frameCount < 2)
    {
        return;
    }

    uint32_t i;
    int16_t previousSample = buffer->data[0];
    int16_t * sample = &buffer->data[1];
    for (i = 1; i < buffer->frameCount; ++i, ++sample)
    {
        int16_t thisSample = *sample;
        if ((thisSample <= 0 && previousSample >= 0)
            || (thisSample >= 0 && previousSample <= 0))
        {
            if (i > 0)
            {
                buffer->data[i - 1] = 0;
                buffer->zeroSnapOffset = i - 1;
            }
            else
            {
                *sample = 0;
                buffer->zeroSnapOffset = i;
            }
            return;
        }
        previousSample = thisSample;
    }

    // Failed to find a zero crossing, so apply a fade in.
    sample = &buffer->data[0];
    uint32_t fadeCount = min(buffer->frameCount, kFadeInSampleCount);
    for (i = 0; i < fadeCount; ++i, ++sample)
    {
        *sample = static_cast<int16_t>(static_cast<int32_t>(*sample) * i / fadeCount);
    }
}

void SampleBufferManager::set_start_end_sample(int32_t start, int32_t end)
{
    // Voice must not be playing.
    assert(!_voice->is_playing());

    // Handle sentinels to use current values.
    uint32_t ustart = (start == -1L) ? _startSample : start;
    uint32_t uend = (end == -1L) ? _endSample : end;
    DEBUG_PRINTF(MISC_MASK, "set_start_end: %lu - %lu\r\n", ustart, uend);

    uint32_t originalStart = _startSample;
    uint32_t originalEnd = _endSample;

    // Update parameters.
    _startSample = constrained(ustart, 0UL, _totalSamples);
    _endSample = constrained(uend, ustart, _totalSamples);

    // Update number of used buffers.
    _activeBufferCount = min(round_up_div(get_active_samples(), kBufferSize), kBufferCount);

    // Set any unused buffers to kUnused state.
    uint32_t i;
    for (i = _activeBufferCount; i < kBufferCount; ++i)
    {
        _buffer[i].set_unused();
    }

    // Reload start of the file if the start or end point changed.
    if (_startSample != originalStart || _endSample != originalEnd)
    {
        _isReady = false;
        _voice->manager_ready_did_change(false);
        _didReadFileStart = false;
        _waitingForFileStart = true;
        _snapToZeroStart = (_startSample != 0);
        _preppedCount = 0;
        prime();
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

SamplerVoice::SamplerVoice()
:   _wav(),
    _data(),
    _manager(),
    _isValid(false),
    _isReady(false),
    _isPlaying(false),
    _noteOffPending(false),
    _doNoteOff(false),
    _doRetrigger(false),
    _noteOffSamplesRemaining(0),
    _readHead(0.0f),
    _pitchOctave(0.0f),
    _params(),
    _volumeEnvMode(VolumeEnvMode::kTrigger),
    _volumeEnv(),
    _pitchEnv()
{
}

void SamplerVoice::init(uint32_t n, int16_t * buffer)
{
    _number = n;

    _volumeEnv.set_sample_rate(kSampleRate);
    _volumeEnv.set_peak(1.0f);
    _volumeEnv.enable_sustain(true);

    _pitchEnv.set_sample_rate(kSampleRate);
    _pitchEnv.set_peak(1.0f);
    _pitchEnv.enable_sustain(false);

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
    _noteOffPending = false;
    _doNoteOff = false;
    _doRetrigger = false;
    _noteOffSamplesRemaining = 0;
    _readHead = 0.0f;
    _volumeEnv.trigger();
    _pitchEnv.trigger();
    std::fill_n(&_interpolationBuffer[0], kInterpolationBufferLength, 0);
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
    assert(_volumeEnvMode == VolumeEnvMode::kGate);

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
        _noteOffPending = true;
        _noteOffSamplesRemaining = kNoteOffSamples;
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
    float octave = _params.baseOctaveOffset
                            + _pitchOctave
                            + (_params.baseCentsOffset / 1200.0f)
                            + pitchModifier;
    constrain(octave, -4.0f, 4.0f);
    float rate = powf(2.0f, octave);

    int16_t * bufferData = voiceBuffer->data;
    float readHead = _readHead;
    uint32_t bufferFrameCount = voiceBuffer->frameCount;
    uint32_t remainingFrames = frameCount;
    float s;

    START_ELAPSED_TIME(total);

    // Skip ahead to the buffer's zero snap offset. This will only apply to the file start.
    if (readHead == 0.0f)
    {
        readHead = voiceBuffer->zeroSnapOffset;
    }

    // Render into output buffer.
    assert(readHead < bufferFrameCount);
    while (remainingFrames)
    {
        assert(remainingFrames <= frameCount);
        uint32_t framesAtRate = std::ceil(remainingFrames * rate);
        uint32_t integerReadHead = readHead;
        uint32_t framesRemainingInBuffer = bufferFrameCount - integerReadHead;
        uint32_t outputFrames;

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
            if (_volumeEnvMode == VolumeEnvMode::kTrigger)
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
                        s_workBuffer, outputFrames, readHead, rate, _interpolationBuffer);
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

            readHead -= bufferFrameCount;
            std::copy_n(&bufferData[bufferFrameCount - kInterpolationBufferLength], kInterpolationBufferLength, _interpolationBuffer);

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

            END_ELAPSED_TIME(retire);
        }
        assert(readHead < bufferFrameCount);
    }

    // Save local read head.
    _readHead = readHead;

    // If the volume env has run out, we're done playing.
    if (_volumeEnv.is_finished())
    {
        _isPlaying = false;

        if (_noteOffPending)
        {
            _doNoteOff = true;
            _noteOffPending = false;
        }
    }

    // Handle end of note off and retriggering.
    if (_doNoteOff)
    {
        START_ELAPSED_TIME(noteOff);

        // Prime will clear all flags for us, so save retrigger locally to set
        // isPlaying after priming.
        bool savedRetrigger = _doRetrigger;
        prime();
        _isPlaying = savedRetrigger;

        if (_isPlaying)
        {
            UI::get().indicate_voice_retriggered(_number);
        }
        else
        {
            UI::get().set_voice_playing(_number, false);
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
        std::copy_n(&bufferData[bufferFrameCount - kInterpolationBufferLength], kInterpolationBufferLength, _interpolationBuffer);
        _manager.retire_buffer(voiceBuffer);
        _readHead -= bufferFrameCount;
    }
    assert(_readHead < bufferFrameCount);
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

void SamplerVoice::set_volume_env_mode(VolumeEnvMode mode)
{
    _volumeEnvMode = mode;

    if (mode == VolumeEnvMode::kTrigger)
    {
        _volumeEnv.set_release(_params.volumeEnvRelease);
    }
    else
    {
        _volumeEnv.set_release(max(_params.volumeEnvRelease, 128.0f / kSampleRate));
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
    set_volume_env_mode(_volumeEnvMode);

    _triggerNoteOffSample = _data.get_frames() - static_cast<uint32_t>(seconds * kSampleRate);
}

void SamplerVoice::set_pitch_env_attack(float seconds)
{
    _params.pitchEnvAttack = seconds;
    _pitchEnv.set_attack(seconds / float(kAudioBufferSize));
}

void SamplerVoice::set_pitch_env_release(float seconds)
{
    _params.pitchEnvRelease = seconds;
    _pitchEnv.set_release(seconds / float(kAudioBufferSize));
}

void SamplerVoice::set_params(const VoiceParameters & params)
{
    // Stop playing and turn off LED.
    _reset_voice();
    UI::get().set_voice_playing(_number, false);

    // Update params.
    _params = params;

    set_volume_env_attack(_params.volumeEnvAttack);
    set_volume_env_release(_params.volumeEnvRelease);
    set_pitch_env_attack(_params.pitchEnvAttack);
    set_pitch_env_release(_params.pitchEnvRelease);

    // Update playback range in SBM.
    float totalSamples = _manager.get_total_samples();
    uint32_t start = totalSamples * _params.startSample;
    uint32_t end = totalSamples * _params.endSample;
    _manager.set_start_end_sample(start, end);
}

float SamplerVoice::get_sample_length_in_seconds() const
{
    return float(_data.get_frames()) / kSampleRate;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
