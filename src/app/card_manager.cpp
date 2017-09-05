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

#include "card_manager.h"
#include "fsl_sd.h"
#include "fsl_sd_disk.h"
#include "fsl_sdmmc_common.h"

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

CardManager::CardManager()
:   _isCardPresent(false)
{
}

void CardManager::init()
{
    // Configure SD host.
    static const sdmmchost_detect_card_t cd = {
        .cdType = kSDMMCHOST_DetectCardByHostDATA3,
        .cdTimeOut_ms = (~0U),
        .cardInserted = nullptr,
        .cardRemoved = nullptr,
        };

    g_sd.host.base = SDHC;
    g_sd.host.sourceClock_Hz = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    g_sd.usrParam.cd = &cd;

    SD_HostInit(&g_sd);
}

//!
bool CardManager::check_presence()
{
    if (_isCardPresent)
    {
        // Skip this check if a transfer is in progress so we don't interrupt it.
        // Read and write transfers continue automaticaly until a StopTransmission
        // command is sent. So sending a SendStatus command during a transfer can
        // cause the response to get mixed up.
        if (!SD_IsTransferring(&g_sd))
        {
            bool response = get_card_status();
            _isCardPresent = response;
        }
    }
    else
    {
        // Attempt to init SD.
        status_t status = SD_CardInit(&g_sd);
        if (status == kStatus_Success)
        {
            _isCardPresent = true;
        }
    }

    return _isCardPresent;
}

bool CardManager::get_card_status()
{
    SDMMCHOST_COMMAND command{0};
    SDMMCHOST_TRANSFER content{0};

    command.index = kSDMMC_SendStatus;
    command.argument = g_sd.relativeAddress << 16U;
    command.responseType = kCARD_ResponseTypeR1;
    command.responseErrorFlags = 0;

    content.command = &command;
    content.data = NULL;

    status_t status = g_sd.host.transfer(g_sd.host.base, &content);

    return (status == kStatus_Success);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
