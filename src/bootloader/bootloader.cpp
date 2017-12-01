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
#include "channel_led.h"
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
#define LED_FLASH_TIME_MS (100)

//! Time in milliseconds to delay between card presence checks.
#define CARD_CHECK_TIME_MS (250)

//! Number of channel LEDs.
#define CHANNEL_LED_COUNT (4)

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

//! @brief Manages LED flashing.
class LEDFlasher
{
public:
    LEDFlasher();
    ~LEDFlasher()=default;

    void init();
    void set_flashing(bool isFlashing);

protected:
    Ar::ThreadWithStack<4096> _thread;
    volatile bool _flashing;
    bool _state;

    void flasher_thread();
    void set_all_leds(bool state);
};

//! @brief Encapsulation of the bootloader.
//!
//! The bootloader thread will automatically start once Argon runs.
class Bootloader
{
public:
    Bootloader();
    ~Bootloader()=default;

protected:
    static const uint32_t kSectorSize = FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE;

    Ar::ThreadWithStack<4096> _thread;

    //! @brief The app's vectors.
    volatile AppVectors * _app;

    fs::FileSystem _fs;
    CardManager _cardManager;
    fs::File _updateFile;

    //! Buffer to store data read from firmware update file.
    uint32_t _sectorBuffer[kSectorSize / sizeof(uint32_t)];

    flash_config_t _flash;

    bool erase_sector(uint32_t sectorAddress);
    bool program_sector(uint32_t sectorAddress);
    bool load_sector_data(uint32_t bytesToRead);

    void start_app();
    void perform_update();
    bool check_update();
    bool have_valid_app();
    void bootloader_thread();
};

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

int main(void);

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

namespace slab {

LEDFlasher g_flasher;
Bootloader g_bootloader;

ChannelLEDManager g_channelLedManager;
LED<PIN_BUTTON1_LED_GPIO_BASE, PIN_BUTTON1_LED_BIT> g_button1Led;

}

DEFINE_DEBUG_LOG

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

uint64_t ar_get_microseconds()
{
    return Microseconds::get();
}

LEDFlasher::LEDFlasher()
:   _thread("leds", this, &LEDFlasher::flasher_thread, 50, kArSuspendThread),
    _flashing(false),
    _state(false)
{
}

void LEDFlasher::init()
{
    set_all_leds(false);

    _thread.resume();
}

void LEDFlasher::set_flashing(bool isFlashing)
{
    _flashing = isFlashing;

    // Make sure LEDs are turned off if disabling flashing.
    if (!_flashing && _state)
    {
        set_all_leds(false);
        _state = false;
    }
}

void LEDFlasher::set_all_leds(bool state)
{
    int which;
    for (which = 0; which < CHANNEL_LED_COUNT; ++which)
    {
        g_channelLedManager.set_channel_state(which, state ? ChannelLEDManager::kRed : ChannelLEDManager::kOff);
    }
    g_channelLedManager.flush();
    g_button1Led.set(state);
}

void LEDFlasher::flasher_thread()
{
    while (true)
    {
        if (_flashing)
        {
            _state = !_state;
            set_all_leds(_state);
        }
        Ar::Thread::sleep(LED_FLASH_TIME_MS);
    }
}

Bootloader::Bootloader()
:   _thread("bootloader", this, &Bootloader::bootloader_thread, 100, kArStartThread),
    _app(reinterpret_cast<volatile AppVectors *>(APP_START_ADDR))
{
}

void Bootloader::start_app()
{
    // Disable interrupts, then switch to the app's vector table.
    __disable_irq();
    SCB->VTOR = reinterpret_cast<uint32_t>(_app);

    // Load variables used in inline asm below.
    uint32_t z = 0;
    uint32_t stack = _app->initialStack;
    uint32_t entry = _app->resetHandler;

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

//! @brief Erases the specified sector of flash.
bool Bootloader::erase_sector(uint32_t sectorAddress)
{
    DEBUG_PRINTF(MISC_MASK, "erasing sector @ 0x%x\r\n", sectorAddress);

    __disable_irq();
    status_t status = FLASH_Erase(&_flash, sectorAddress, kSectorSize, kFLASH_ApiEraseKey);
    __enable_irq();
    if (status != kStatus_Success)
    {
        DEBUG_PRINTF(ERROR_MASK, "failed to erase sector @ 0x%x (err=%d)\r\n", sectorAddress, status);
        return false;
    }
    return true;
}

//! @brief Programs flash and performs readback verify.
bool Bootloader::program_sector(uint32_t sectorAddress)
{
    DEBUG_PRINTF(MISC_MASK, "writing sector @ 0x%x\r\n", sectorAddress);

    // Program the whole sector.
    __disable_irq();
    status_t status = FLASH_ProgramSection(&_flash, sectorAddress, _sectorBuffer, kSectorSize);
    __enable_irq();
    if (status != kStatus_Success)
    {
        DEBUG_PRINTF(ERROR_MASK, "failed to program sector @ 0x%x (err=%d)\r\n", sectorAddress, status);
        return false;
    }

    DEBUG_PRINTF(MISC_MASK, "verifying sector @ 0x%x\r\n", sectorAddress);

    // Read the programmed sector to verify its contents.
    if (memcmp(_sectorBuffer, (uint8_t *)sectorAddress, kSectorSize) != 0)
    {
        DEBUG_PRINTF(ERROR_MASK, "verify failed for sector @ 0x%x\r\n", sectorAddress);
        return false;
    }

    return true;
}

//! @brief Reads the next @a bytesToRead bytes from the update file.
bool Bootloader::load_sector_data(uint32_t bytesToRead)
{
    // Read data from update file.
    uint32_t bytesRead;
    fs::error_t err = _updateFile.read(bytesToRead, _sectorBuffer, &bytesRead);
    if (err || bytesRead != bytesToRead)
    {
        DEBUG_PRINTF(ERROR_MASK, "failed to read data from file\r\n");
        return false;
    }

    // Fill trailing portion of the sector with fs.
    if (bytesRead < kSectorSize)
    {
        memset(&((uint8_t *)_sectorBuffer)[bytesRead], 0xff, kSectorSize - bytesRead);
    }

    return true;
}

//! @brief Copy the firmware update to flash.
//!
//! The first step is to erase the app's vector table sector. This sector will not be
//! programmed until all other sectors are successfully programmed, so that the app is
//! not made bootable until it is fully ready to go.
//!
//! For each sector, the data is read from the file into a temporary buffer. Then the
//! flash sector is erased and programmed. After programming, the flash contents are
//! verified against the file data still in the temporary buffer.
//!
//! The last step is to read, program, and verify the vector table sector that was erased
//! in the first step. If the verify fails, then the sector is erased again so that we
//! don't try to boot a corrupted app.
void Bootloader::perform_update()
{
    uint32_t remainingBytes = _updateFile.get_size();
    if (remainingBytes < kSectorSize)
    {
        DEBUG_PRINTF(ERROR_MASK, "update image is too small (only %d bytes)\r\n", remainingBytes);
        return;
    }

    // Init flash driver.
    memset(&_flash, 0, sizeof(_flash));
    status_t status = FLASH_Init(&_flash);
    if (status != kStatus_Success)
    {
        DEBUG_PRINTF(ERROR_MASK, "failed to init flash driver (err=%d)\r\n", status);
        return;
    }

    // First erase app's vector table sector without programming it.
    if (!erase_sector(APP_START_ADDR))
    {
        return;
    }

    uint32_t sectorAddress = APP_START_ADDR + kSectorSize;
    remainingBytes -= kSectorSize;
    _updateFile.seek(kSectorSize);

    while (remainingBytes)
    {
        uint32_t bytesToRead = min(remainingBytes, kSectorSize);

        // Read data from update file.
        if (!load_sector_data(bytesToRead))
        {
            return;
        }

        // Erase this sector.
        if (!erase_sector(sectorAddress))
        {
            return;
        }

        // Program and verify the sector.
        if (!program_sector(sectorAddress))
        {
            return;
        }

        sectorAddress += kSectorSize;
        remainingBytes -= bytesToRead;
    }

    DEBUG_PRINTF(INIT_MASK, "reading sector @ 0x%x\r\n", APP_START_ADDR);

    // Now that the rest of the image is successfully programmed, we can program the vector table sector.
    _updateFile.seek(0);

    // Read data from update file.
    if (!load_sector_data(kSectorSize))
    {
        return;
    }

    // Program and verify the first sector.
    if (!program_sector(APP_START_ADDR))
    {
        // Verify failed, so erase app's vector table to prevent it from booting.
        erase_sector(APP_START_ADDR);

        return;
    }

    // Delete the firmware update file since it was successfully programmed.
    DEBUG_PRINTF(INIT_MASK, "update complete; deleting %s\r\n", FW_UPDATE_FILENAME);
    _updateFile.remove();
}

//! @brief Check if we should perform an update.
//!
//! First an attempt is made to open the firmware update file. If the file is present,
//! then the app signature in its header is checked. The size of the file is also compared
//! against the size stored in the header, to make sure all data is present. If those
//! checks pass, the CRC stored in the file's header is compared against the CRC in the
//! header of the current version of the app resident in flash.
//!
//! Note that filesystem is assumed to already be mounted when this method is called.
bool Bootloader::check_update()
{
    _updateFile = fs::File(FW_UPDATE_FILENAME);
    fs::error_t err = _updateFile.open();
    if (err)
    {
        DEBUG_PRINTF(ERROR_MASK, "failed to open update file\r\n");
        return false;
    }
    DEBUG_PRINTF(INIT_MASK, "opened firmware update file\r\n");

    // Read the vector table from the update file.
    AppVectors header;
    uint32_t bytesRead;
    err = _updateFile.read(sizeof(header), &header, &bytesRead);
    if (err || bytesRead != sizeof(header))
    {
        DEBUG_PRINTF(ERROR_MASK, "unable to read file header\r\n");
        return false;
    }

    // Check signature.
    if (header.signature != APP_SIGNATURE)
    {
        DEBUG_PRINTF(ERROR_MASK, "invalid signature in update file\r\n");
        return false;
    }

    // Check file size matches the size in the header.
    uint32_t fileSize = _updateFile.get_size();
    if (fileSize != header.appSize)
    {
        DEBUG_PRINTF(ERROR_MASK, "mismatch between file size (%d) and size in header (%d)\r\n", fileSize, header.appSize);
        return false;
    }

    // Update if either of:
    // - Update's CRC does not match existing app's CRC.
    // - There is no existing app (update CRC is highly unlikely to match 0xffffffff, but
    //   it doesn't hurt to have an explicit check).
    bool doUpdate = (header.crc32 != _app->crc32)
                    || (_app->crc32 == ERASED_WORD);
    return doUpdate;
}

bool Bootloader::have_valid_app()
{
    return (_app->initialStack != ERASED_WORD
            && _app->resetHandler != ERASED_WORD
            && _app->signature == APP_SIGNATURE);
}

void Bootloader::bootloader_thread()
{
    DEBUG_PRINTF(INIT_MASK, "samplbaer bootloader initializing...\r\n");

    // Init LED flasher thread.
    g_channelLedManager.init();
    g_flasher.init();

    // Init SD card manager.
    _cardManager.init();
    _cardManager.check_presence();

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
        if (_cardManager.is_card_present())
        {
            // Look for a firmware update file.
            fs::error_t result = _fs.mount();
            if (result == fs::kSuccess)
            {
                if (check_update())
                {
                    perform_update();
                }
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
                start_app();
            }
            else
            {
                DEBUG_PRINTF(ERROR_MASK, "no app; halting.\r\n", result);

                // Unmount the file system until a card is reinserted.
                _fs.unmount();

                // Wait for card to be removed.
                g_flasher.set_flashing(true);
                while (_cardManager.check_presence())
                {
                    Ar::Thread::sleep(CARD_CHECK_TIME_MS);
                }
                g_flasher.set_flashing(false);
            }
        }
        else
        {
            // Now jump to the app if there is one.
            if (have_valid_app())
            {
                DEBUG_PRINTF(INIT_MASK, "Launching app...\r\n");

                // Jump to the app.
                start_app();
            }

            // Wait for card to be inserted.
            g_flasher.set_flashing(true);
            while (!_cardManager.check_presence())
            {
                Ar::Thread::sleep(CARD_CHECK_TIME_MS);
            }
            g_flasher.set_flashing(false);
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
