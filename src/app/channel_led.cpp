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
#include "channel_led.h"
#include "fsl_gpio.h"
#include "board.h"

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

// static void clkdelay(uint32_t n)
// {
//     while (--n)
//     {
//         __NOP();
//     }
// }

ChannelLEDManager::ChannelLEDManager()
:   _editBuffer(0),
    _transferBuffer(0)
{
}

void ChannelLEDManager::init()
{
    dspi_master_config_t masterConfig;
    DSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.ctarConfig.baudRate = 10000; // 10 kHz
    masterConfig.ctarConfig.bitsPerFrame = 8;
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 50000;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 50000;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 50000;
    masterConfig.whichPcs = kDSPI_Pcs0;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint = kDSPI_SckToSin0Clock;
    DSPI_MasterInit(SPI0, &masterConfig, CLOCK_GetBusClkFreq());

    DSPI_MasterTransferCreateHandle(SPI0, &_spiHandle, _transfer_callback, this);

    // Enable output.
    GPIO_PinWrite(PIN_CH_LED_OE_N_GPIO, PIN_CH_LED_OE_N_BIT, 0);
}

void ChannelLEDManager::set_channel_state(uint32_t channel, ChannelLedState state)
{
    uint32_t channelBitOffset = channel * 2;
    _editBuffer = (_editBuffer & ~(0x3 << channelBitOffset))
                    | (static_cast<uint8_t>(state) << channelBitOffset);
}

void ChannelLEDManager::flush()
{
    // Copy edit buffer to transfer buffer.
    _transferBuffer = _editBuffer;

    // Reset latch.
    GPIO_PinWrite(PIN_CH_LED_LATCH_GPIO, PIN_CH_LED_LATCH_BIT, 0);

    // Initiate transfer.
    dspi_transfer_t transfer = {0};
    transfer.txData = &_transferBuffer;
    transfer.rxData = NULL;
    transfer.dataSize = 1;
    transfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0;
    DSPI_MasterTransferNonBlocking(SPI0, &_spiHandle, &transfer);
}

void ChannelLEDManager::_transfer_callback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
    // Latch. Can stay high until next flush.
    GPIO_PinWrite(PIN_CH_LED_LATCH_GPIO, PIN_CH_LED_LATCH_BIT, 1);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
