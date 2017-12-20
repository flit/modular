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
#if !defined(_SAMPLBAER_H_)
#define _SAMPLBAER_H_

#include "sampler_voice.h"
#include "sampler_synth.h"
#include "channel_gate.h"
#include "channel_cv.h"
#include "card_manager.h"
#include "file_manager.h"
#include "audio_defs.h"
#include "ui.h"
#include "persistent_data_store.h"
#include "fsl_adc16.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

enum thread_priorties : uint8_t
{
    kAudioThreadPriority = 180,
    kReaderThreadPriority = 120,
    kCVThreadPriority = 80,
    kUIThreadPriority = 60,
    kInitThreadPriority = 40,
};

//! DMA channel numbers used by the application.
enum dma_channels : uint32_t
{
    kAudioPingDmaChannel = 0,
    kAudioPongDmaChannel = 1,
    kAdc0CommandDmaChannel = 2,
    kAdc0ReadDmaChannel = 3,
    kAdc1CommandDmaChannel = 4,
    kAdc1ReadDmaChannel = 5,
    kAllocatedDmaChannelCount = 6,  //!< Number of DMA channels used by the application.
};

namespace persistent_data {

//! Keys for persistent data values.
enum data_keys : uint32_t
{
    kCalibrationDataKey = 'cal_',
    kLastSelectedBankKey = 'lbnk',
    kLastVoiceMode = 'vmod',
};

extern PersistentData<kLastSelectedBankKey, uint32_t> g_lastSelectedBank;
extern PersistentData<kLastVoiceMode, VoiceMode> g_lastVoiceMode;

}

extern SamplerSynth g_sampler;
extern SamplerVoice g_voice[kVoiceCount];
extern ChannelGate g_gates[kVoiceCount];
extern ChannelCV g_cvs[kVoiceCount];
extern Pot g_pots[kVoiceCount];
extern CardManager g_cardManager;
extern FileManager g_fileManager;
extern adc16_config_t g_adcConfig;

} // namespace slab

#endif // _SAMPLBAER_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
