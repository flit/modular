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
#include "ui.h"

using namespace slab;

//! @brief The SPI peripheral instance used for channel LEDs.
#define CHANNEL_LED_SPI (SPI0)

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

ChannelLEDManager::ChannelLEDManager()
:   _buffer(0)
{
}

void ChannelLEDManager::init()
{
    // Init and configure the SPI peripheral.
    dspi_master_config_t masterConfig;
    DSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.ctarConfig.baudRate = 10000000; // 10 MHz
    masterConfig.ctarConfig.bitsPerFrame = 8;
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 10000;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 20000;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 0;
    masterConfig.whichPcs = kDSPI_Pcs3;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint = kDSPI_SckToSin0Clock;
    DSPI_MasterInit(CHANNEL_LED_SPI, &masterConfig, CLOCK_GetBusClkFreq());

    DSPI_SetFifoEnable(CHANNEL_LED_SPI, true, false); // tx fifo enabled, rx fifo disabled
    DSPI_Enable(CHANNEL_LED_SPI, true);
    DSPI_StartTransfer(CHANNEL_LED_SPI);

    // Enable output.
    GPIO_PinWrite(PIN_CH_LED_OE_N_GPIO, PIN_CH_LED_OE_N_BIT, 0);
}

bool ChannelLEDManager::flush()
{
    // Check flags and ignore flush request if the FIFO is full.
    uint32_t flags = DSPI_GetStatusFlags(CHANNEL_LED_SPI);
    if ((flags & kDSPI_TxFifoFillRequestFlag) == 0)
    {
        return false;
    }

    // Initiate transfer.
    dspi_command_data_config_t commandConfig;
    commandConfig.isPcsContinuous = false;
    commandConfig.whichCtar = kDSPI_Ctar0;
    commandConfig.whichPcs = kDSPI_Pcs3;
    commandConfig.clearTransferCount = false;
    commandConfig.isEndOfQueue = false;
    DSPI_MasterWriteData(CHANNEL_LED_SPI, &commandConfig, _buffer);

    return true;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
