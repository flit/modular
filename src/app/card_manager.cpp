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

#include "card_manager.h"
#include "samplbaer.h"
#include "fsl_sd.h"
#include "fsl_sd_disk.h"
#include "fsl_sdmmc_common.h"

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

//! Interval for checking SD card presence.
const uint32_t kCardDetectInterval_ms = 500;

//! Debounce delay for checking SD card presence.
const uint32_t kCardDetectDebounceDelay_ms = 50;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

CardManager::CardManager()
:   _thread(),
    _isCardPresent(false),
    _isCardInited(false),
    _debounceCardDetect(false),
    _isImmediateCheckPending(false)
{
}

void CardManager::init()
{
    // Configure SD host.
    // Setting card detect to gpio even though we're using command polling to
    // effectively disable the driver's card detect support, which is unreliable.
    static const sdmmchost_detect_card_t cd = {
        .cdType = kSDMMCHOST_DetectCardByGpioCD,
        .cdTimeOut_ms = (~0U),
        .cardInserted = nullptr,
        .cardRemoved = nullptr,
        };

    g_sd.host.base = SDHC;
    g_sd.host.sourceClock_Hz = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    g_sd.usrParam.cd = &cd;

    SD_HostInit(&g_sd);

    // Create and start up card manager thread.
    _thread.init("card", this, &CardManager::card_thread, kCardThreadPriority, kArSuspendThread);
    _runloop.init("card", &_thread);

    // Set up card detection timer.
    _cardDetectTimer.init("card-detect", this, &CardManager::handle_card_detect_timer, kArPeriodicTimer, kCardDetectInterval_ms);
    _runloop.addTimer(&_cardDetectTimer);
    _cardDetectTimer.start();

    _thread.resume();
}

void CardManager::card_thread()
{
    while (true)
    {
        _runloop.run(kArInfiniteTimeout, nullptr);
    }
}


void CardManager::handle_card_detect_timer(Ar::Timer * timer)
{
    // Detect change in card presence. If a change is detected, set the debounce flag
    // and exit. We'll process the debounce next time this timer fires.
    bool isPresent = check_presence();
    if (!_debounceCardDetect)
    {
        // Set debounce flag if a change in presence is detected.
        _debounceCardDetect = (isPresent != _isCardPresent);
        if (_debounceCardDetect)
        {
            _cardDetectTimer.setDelay(kCardDetectDebounceDelay_ms);
        }
    }
    else
    {
        _debounceCardDetect = false;
        _cardDetectTimer.setDelay(kCardDetectInterval_ms);

        // Card inserted.
        if (isPresent && !_isCardPresent)
        {
            _isCardPresent = true;
            UI::get().send_event(UIEvent(kCardInserted));
        }
        // Card removed.
        else if (!isPresent && _isCardPresent)
        {
            _isCardPresent = false;
            UI::get().send_event(UIEvent(kCardRemoved));
        }
    }
}

void CardManager::report_card_error()
{
    // Perform the check on the card detect thread so it's serialized with the detect timer.
    if (!_isImmediateCheckPending)
    {
        _isImmediateCheckPending = true;
        _runloop.perform(check_card_removed_stub, this);
    }
}

void CardManager::check_card_removed()
{
    if (!check_presence() && _isCardPresent)
    {
        // Force immediate debounce.
        _debounceCardDetect = true;
        _cardDetectTimer.setDelay(kCardDetectDebounceDelay_ms);
    }

    _isImmediateCheckPending = false;
}

void CardManager::check_card_removed_stub(void * param)
{
    CardManager * _this = reinterpret_cast<CardManager *>(param);
    assert(_this);
    _this->check_card_removed();
}

bool CardManager::check_presence()
{
    if (_isCardInited)
    {
        // Skip this check if a transfer is in progress so we don't interrupt it.
        // Read and write transfers continue automaticaly until a StopTransmission
        // command is sent. So sending a SendStatus command during a transfer can
        // cause the response to get mixed up.
        if (!SD_IsTransferring(&g_sd))
        {
            bool response = get_card_status();
            _isCardInited = response;

            if (!response)
            {
                SD_CardDeinit(&g_sd);
                SD_HostReset(&g_sd.host);
            }
        }
    }
    else
    {
        // Attempt to init SD.
        status_t status = SD_CardInit(&g_sd);
        if (status == kStatus_Success)
        {
            _isCardInited = true;
        }
    }

    return _isCardInited;
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
