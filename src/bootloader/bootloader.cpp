/*
 * Copyright (c) 2016-2017 Chris Reed
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

#include "argon/argon.h"
#include "board.h"
#include "file_system.h"
#include "card_manager.h"
#include "utility.h"
#include "led.h"
#include "microseconds.h"
#include "debug_log.h"
#include "fsl_flash.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

//! App starts at 128kB offset.
#define APP_START_ADDR (0x20000)

//! Name of the firmware update file.
#define FW_UPDATE_FILENAME "/firmware.bin"

//! Value of a word of flash that is erased.
#define ERASED_WORD (0xffffffff)

//! Unique ID for the app. (sbär)
#define APP_SIGNATURE (0x72e46273)

//! Time in milliseconds LEDs are on when flashing.
#define LED_FLASH_TIME_MS (50)

//! @brief Start of the app's vector table.
struct AppVectors
{
    uint32_t initialStack;
    uint32_t resetHandler;
    uint32_t nmiHandler;
    uint32_t hardFaultHandler;
    uint32_t memManageHandler;
    uint32_t busFaultHandler;
    uint32_t usageFaultHandler;
    uint32_t signature;
    uint32_t crc32;
    uint32_t appSize;
};

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void toggle_leds();
void start_app(volatile AppVectors * vectors);
void perform_update(fs::File& updateFile);
void check_update();
bool have_valid_app();
void bootloader_thread(void * arg);
int main(void);

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

namespace slab {

//! @brief The app's vectors.
volatile AppVectors * g_app = reinterpret_cast<volatile AppVectors *>(APP_START_ADDR);

fs::FileSystem g_fs;
CardManager g_cardManager;

Ar::ThreadWithStack<4096> g_bootloaderThread("bootloader", bootloader_thread, 0, 100);

LED<PIN_CH1_LED_GPIO_BASE, PIN_CH1_LED_BIT> g_ch1Led;
LED<PIN_CH2_LED_GPIO_BASE, PIN_CH2_LED_BIT> g_ch2Led;
LED<PIN_CH3_LED_GPIO_BASE, PIN_CH3_LED_BIT> g_ch3Led;
LED<PIN_CH4_LED_GPIO_BASE, PIN_CH4_LED_BIT> g_ch4Led;
LEDBase * g_channelLeds[] = { &g_ch1Led, &g_ch2Led, &g_ch3Led, &g_ch4Led};
LED<PIN_BUTTON1_LED_GPIO_BASE, PIN_BUTTON1_LED_BIT> g_button1Led;

//! Buffer to store data read from firmware update file.
uint32_t g_sectorBuffer[FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE / sizeof(uint32_t)];

}

DEFINE_DEBUG_LOG

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void toggle_leds()
{
    // Assume all LEDs are in the same state.
    bool state = !g_channelLeds[0]->is_on();
    int which;
    for (which = 0; which < 4; ++which)
    {
        g_channelLeds[which]->set(state);
    }
    g_button1Led.set(state);
}

void start_app(volatile AppVectors * vectors)
{
    // Disable interrupts, then switch to the app's vector table.
    __disable_irq();
    SCB->VTOR = reinterpret_cast<uint32_t>(vectors);

    // Load variables used in inline asm below.
    uint32_t z = 0;
    uint32_t stack = vectors->initialStack;
    uint32_t entry = vectors->resetHandler;

    // Switch to MSP, set MSP to the app's initial SP, then jump to app.
    asm volatile (  "msr    control, %[z]   \n\t"
                    "isb                    \n\t"
                    "mov    sp, %[stack]    \n\t"
                    "bx     %[entry]"
                    :
                    :   [z] "l" (z),
                        [stack] "l" (stack),
                        [entry] "l" (entry)
                    );
}

void perform_update(fs::File& updateFile)
{
    const uint32_t kSectorSize = FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE;

    uint32_t remainingBytes = updateFile.get_size();
    if (remainingBytes < kSectorSize)
    {
        DEBUG_PRINTF(ERROR_MASK, "update image is too small (only %d bytes)\r\n", remainingBytes);
        return;
    }

    // Init flash driver.
    flash_config_t flash = {0};
    status_t status = FLASH_Init(&flash);
    if (status != kStatus_Success)
    {
        DEBUG_PRINTF(ERROR_MASK, "failed to init flash driver (err=%d)\r\n", status);
        return;
    }

    uint32_t sectorAddress = APP_START_ADDR;

    DEBUG_PRINTF(INIT_MASK, "erasing sector @ 0x%x\r\n", APP_START_ADDR);

    // Erase app's vector table sector without programming it.
    __disable_irq();
    status = FLASH_Erase(&flash, sectorAddress, kSectorSize, kFLASH_ApiEraseKey);
    __enable_irq();
    if (status != kStatus_Success)
    {
        DEBUG_PRINTF(ERROR_MASK, "failed to erase first sector (err=%d)\r\n", status);
        return;
    }
    sectorAddress += kSectorSize;
    remainingBytes -= kSectorSize;
    updateFile.seek(kSectorSize);

    while (remainingBytes)
    {
        uint32_t bytesToRead = min(remainingBytes, kSectorSize);

        DEBUG_PRINTF(MISC_MASK, "reading sector @ 0x%x\r\n", sectorAddress);

        // Read data from update file.
        uint32_t bytesRead;
        fs::error_t err = updateFile.read(bytesToRead, g_sectorBuffer, &bytesRead);
        if (err || bytesRead != bytesToRead)
        {
            DEBUG_PRINTF(ERROR_MASK, "failed to read data from file\r\n");
            return;
        }

        // Fill trailing portion of the sector with fs.
        if (bytesRead < kSectorSize)
        {
            memset(&((uint8_t *)g_sectorBuffer)[bytesRead], 0xff, kSectorSize - bytesRead);
        }

        DEBUG_PRINTF(MISC_MASK, "erasing sector @ 0x%x\r\n", sectorAddress);

        // Erase this sector.
        __disable_irq();
        status = FLASH_Erase(&flash, sectorAddress, kSectorSize, kFLASH_ApiEraseKey);
        __enable_irq();
        if (status != kStatus_Success)
        {
            DEBUG_PRINTF(ERROR_MASK, "failed to erase sector @ 0x%x (err=%d)\r\n", sectorAddress, status);
            return;
        }

        DEBUG_PRINTF(MISC_MASK, "writing sector @ 0x%x\r\n", sectorAddress);

        // Program the whole sector.
        __disable_irq();
        status = FLASH_ProgramSection(&flash, sectorAddress, g_sectorBuffer, kSectorSize);
        __enable_irq();
        if (status != kStatus_Success)
        {
            DEBUG_PRINTF(ERROR_MASK, "failed to program sector @ 0x%x (err=%d)\r\n", sectorAddress, status);
            return;
        }

        DEBUG_PRINTF(MISC_MASK, "verifying sector @ 0x%x\r\n", sectorAddress);

        // Read the programmed sector to verify its contents.
        if (memcmp(g_sectorBuffer, (uint8_t *)sectorAddress, kSectorSize) != 0)
        {
            DEBUG_PRINTF(ERROR_MASK, "verify failed for sector @ 0x%x\r\n", sectorAddress);
            return;
        }

        sectorAddress += kSectorSize;
        remainingBytes -= bytesRead;
    }

    DEBUG_PRINTF(INIT_MASK, "reading sector @ 0x%x\r\n", APP_START_ADDR);

    // Now that the rest of the image is successfully programmed, we can program the vector table sector.
    updateFile.seek(0);

    // Read data from update file.
    uint32_t bytesRead;
    fs::error_t err = updateFile.read(kSectorSize, g_sectorBuffer, &bytesRead);
    if (err || bytesRead != kSectorSize)
    {
        DEBUG_PRINTF(ERROR_MASK, "failed to read first sector from file\r\n");
        return;
    }

    DEBUG_PRINTF(INIT_MASK, "writing sector @ 0x%x\r\n", APP_START_ADDR);

    // Program the whole sector.
    __disable_irq();
    status = FLASH_ProgramSection(&flash, APP_START_ADDR, g_sectorBuffer, kSectorSize);
    __enable_irq();
    if (status != kStatus_Success)
    {
        DEBUG_PRINTF(ERROR_MASK, "failed to program first app sector (err=%d)\r\n", status);
        return;
    }

    DEBUG_PRINTF(INIT_MASK, "verifying sector @ 0x%x\r\n", APP_START_ADDR);

    // Read the programmed sector to verify its contents.
    if (memcmp(g_sectorBuffer, (uint8_t *)APP_START_ADDR, kSectorSize) != 0)
    {
        DEBUG_PRINTF(ERROR_MASK, "verify failed for first sector, erasing to prevent boot\r\n");

        // Verify failed, so erase app's vector table to prevent it from booting.
        __disable_irq();
        FLASH_Erase(&flash, APP_START_ADDR, kSectorSize, kFLASH_ApiEraseKey);
        __enable_irq();

        return;
    }

    // Delete the firmware update file since it was successfully programmed.
    DEBUG_PRINTF(INIT_MASK, "update complete; deleting %s\r\n", FW_UPDATE_FILENAME);
    updateFile.remove();
}

void check_update()
{
    fs::File updateFile(FW_UPDATE_FILENAME);
    fs::error_t err = updateFile.open();
    if (err)
    {
        DEBUG_PRINTF(ERROR_MASK, "failed to open update file\r\n");
        return;
    }
    DEBUG_PRINTF(INIT_MASK, "opened firmware update file\r\n");

    // Read the vector table from the update file.
    AppVectors header;
    uint32_t bytesRead;
    err = updateFile.read(sizeof(header), &header, &bytesRead);
    if (err || bytesRead != sizeof(header))
    {
        DEBUG_PRINTF(ERROR_MASK, "unable to read file header\r\n");
        return;
    }

    // Check signature.
    if (header.signature != APP_SIGNATURE)
    {
        DEBUG_PRINTF(ERROR_MASK, "invalid signature in update file\r\n");
        return;
    }

    // Check file size matches the size in the header.
    uint32_t fileSize = updateFile.get_size();
    if (fileSize != header.appSize)
    {
        DEBUG_PRINTF(ERROR_MASK, "mismatch between file size (%d) and size in header (%d)\r\n", fileSize, header.appSize);
        return;
    }

    // Update if either of:
    // - Update's CRC does not match existing app's CRC.
    // - There is no existing app (update CRC is highly unlikely to match 0xffffffff, but
    //   it doesn't hurt to have an explicit check).
    bool doUpdate = (header.crc32 != g_app->crc32)
                    || (g_app->crc32 == ERASED_WORD);

    if (doUpdate)
    {
        perform_update(updateFile);
    }
    else
    {
        DEBUG_PRINTF(INIT_MASK, "same crc; ignoring update\r\n");
    }
}

bool have_valid_app()
{
    return (g_app->initialStack != ERASED_WORD && g_app->resetHandler != ERASED_WORD);
}

void bootloader_thread(void * arg)
{
    DEBUG_PRINTF(INIT_MASK, "samplbaer bootloader initializing...\r\n");

    // Configure channel LED color.
    GPIO_WritePinOutput(PIN_CHLEDN_GPIO, PIN_CHLEDN_BIT, 0);
    GPIO_WritePinOutput(PIN_CHLEDP_GPIO, PIN_CHLEDP_BIT, 1);

    // Invert polarity of channel LEDs.
    g_channelLeds[0]->set_polarity(true);
    g_channelLeds[1]->set_polarity(true);
    g_channelLeds[2]->set_polarity(true);
    g_channelLeds[3]->set_polarity(true);

    // Init SD card manager.
    g_cardManager.init();
    g_cardManager.check_presence();

    // If a card is present:
    //  - check for and process a firmware update.
    //  - jump to app if there is a valid vector table.
    //  - if no app, wait until card is removed
    //
    // If no card:
    //  - if valid app: jump to it
    //  - if no app: wait for card to be inserted

    while (true)
    {
        if (g_cardManager.is_card_present())
        {
            // Look for a firmware update file.
            fs::error_t result = g_fs.init();
            if (result == fs::kSuccess)
            {
                check_update();
            }
            else
            {
                DEBUG_PRINTF(ERROR_MASK, "fs init failed: %d\r\n", result);
            }

            // Now jump to the app if there is one.
            if (have_valid_app())
            {
                DEBUG_PRINTF(INIT_MASK, "Launching app...\r\n");

                // Jump to the app.
                start_app(g_app);
            }
            else
            {
                DEBUG_PRINTF(ERROR_MASK, "no app; halting.\r\n", result);

                // Wait for card to be removed.
                while (g_cardManager.check_presence())
                {
                    toggle_leds();
                    Ar::Thread::sleep(LED_FLASH_TIME_MS);
                }
            }
        }
        else
        {
            // Now jump to the app if there is one.
            if (have_valid_app())
            {
                DEBUG_PRINTF(INIT_MASK, "Launching app...\r\n");

                // Jump to the app.
                start_app(g_app);
            }

            // Wait for card to be inserted.
            while (!g_cardManager.check_presence())
            {
                toggle_leds();
                Ar::Thread::sleep(LED_FLASH_TIME_MS);
            }
        }
    }
}

int main(void)
{
#if DEBUG
    // Disable write buffer to make bus faults precise.
    SCnSCB->ACTLR = SCnSCB_ACTLR_DISDEFWBUF_Msk;
#endif

    init_board();
    Microseconds::init();
    ar_kernel_run();
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
