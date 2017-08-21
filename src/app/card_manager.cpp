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
#include "file_manager.h"
#include "debug_log.h"
#include "fsl_sd.h"
#include "fsl_sd_disk.h"
#include "fsl_sdmmc_common.h"
#include "fsl_port.h"

using namespace slab;

void card_detect_handler(PORT_Type * port, uint32_t pin, void * userData);

bool g_cardPresent = false;

// namespace slab {
// extern FileManager g_fileManager;
// }

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void card_detect_handler(PORT_Type * port, uint32_t pin, void * userData)
{
//     DEBUG_PRINTF(BUTTON_MASK, "card detect\r\n");

    if (!g_cardPresent)
    {
        DEBUG_PRINTF(BUTTON_MASK, "card inserted\r\n");
//         PORT_SetPinMux(PIN_SDHC_D3_PORT, PIN_SDHC_D3_BIT, kPORT_MuxAlt4);
        PORT_SetPinInterruptConfig(PIN_SDHC_D3_PORT, PIN_SDHC_D3_BIT, kPORT_InterruptFallingEdge);
    }
    else
    {
        DEBUG_PRINTF(BUTTON_MASK, "card removed\r\n");
        PORT_SetPinMux(PIN_SDHC_D3_PORT, PIN_SDHC_D3, kPORT_MuxAsGpio);
        PORT_SetPinInterruptConfig(PIN_SDHC_D3_PORT, PIN_SDHC_D3_BIT, kPORT_InterruptRisingEdge);
    }

    g_cardPresent = !g_cardPresent;
}

CardManager::CardManager()
:   _isCardPresent(false)
{
}

void CardManager::init()
{
    // Configure SD host.
    static sdmmchost_detect_card_t cd = {
        .cdType = kSDMMCHOST_DetectCardByHostDATA3,
        .cdTimeOut_ms = (~0U),
        .cardInserted = NULL,
        .cardRemoved = NULL,
        };

    g_sd.host.base = SDHC;
    g_sd.host.sourceClock_Hz = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    g_sd.usrParam.cd = &cd;

    SD_HostInit(&g_sd);

//     PinIrqManager::get().connect(PIN_SDHC_D3_PORT, PIN_SDHC_D3_BIT, card_detect_handler, NULL);
}

bool CardManager::check_presence()
{
//     g_fileManager.lock();

    if (_isCardPresent)
    {
        // Send the select command.
        status_t err = SDMMC_SelectCard(g_sd.host.base, g_sd.host.transfer, g_sd.relativeAddress, true);
        if (err)
        {
            _isCardPresent = false;
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

//     g_fileManager.unlock();

    return _isCardPresent;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
