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

ChannelLEDManager::ChannelLEDManager()
:   _editBuffer(0),
    _isTransferring(false)
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

    DSPI_EnableInterrupts(SPI0, kDSPI_TxCompleteInterruptEnable);
    DSPI_SetFifoEnable(SPI0, true, false); // tx fifo enabled, rx fifo disabled
    DSPI_Enable(SPI0, true);
    DSPI_StartTransfer(SPI0);
    NVIC_EnableIRQ(SPI0_IRQn);

    // Enable output.
    GPIO_PinWrite(PIN_CH_LED_OE_N_GPIO, PIN_CH_LED_OE_N_BIT, 0);
}

void ChannelLEDManager::flush()
{
    _isTransferring = true;

    // Reset latch.
    GPIO_PinWrite(PIN_CH_LED_LATCH_GPIO, PIN_CH_LED_LATCH_BIT, 0);

    // Initiate transfer.
    dspi_command_data_config_t commandConfig;
    commandConfig.isPcsContinuous = false;
    commandConfig.whichCtar = kDSPI_Ctar0;
    commandConfig.whichPcs = kDSPI_Pcs0;
    commandConfig.clearTransferCount = false;
    commandConfig.isEndOfQueue = false;
    DSPI_MasterWriteData(SPI0, &commandConfig, _editBuffer);
}

extern "C" void SPI0_IRQHandler(void)
{
    GPIO_PinWrite(PIN_CH_LED_LATCH_GPIO, PIN_CH_LED_LATCH_BIT, 1);
    ChannelLEDManager::get().clear_is_transferring();
    DSPI_ClearStatusFlags(SPI0, kDSPI_TxCompleteFlag);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
