/*
 * Copyright (c) 2016 Chris Reed
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

#include "board.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void init_debug_console()
{
    // Set PTB16 and PTB17
    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinMux(PORTB, 16, kPORT_MuxAlt3);
    PORT_SetPinMux(PORTB, 17, kPORT_MuxAlt3);
    DbgConsole_Init((uint32_t)UART0, 115200, DEBUG_CONSOLE_DEVICE_TYPE_UART, CLOCK_GetCoreSysClkFreq());
}

void init_board()
{
    // Set crystal frequencies.
    CLOCK_SetXtal0Freq(kXtal0Freq);
    CLOCK_SetXtal32Freq(kXtal32Freq);

    // Disable MPU.
    if (SIM->SCGC7 & SIM_SCGC7_MPU_MASK)
    {
        SYSMPU->CESR = 0;
        SIM->SCGC7 &= ~SIM_SCGC7_MPU_MASK;
    }

    BOARD_BootClockRUN();

    init_debug_console();

    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortE);

#if DEBUG
    // SWO pin
    PORT_SetPinMux(PIN_SWO_PORT, PIN_SWO_BIT, kPORT_MuxAlt7);
#endif

    // SAI pins
    PORT_SetPinMux(PIN_I2S_MCLK_PORT, PIN_I2S_MCLK_BIT, PIN_I2S_MCLK_MUX);
    PORT_SetPinMux(PIN_I2S_TXD0_PORT, PIN_I2S_TXD0_BIT, PIN_I2S_TXD0_MUX);
    PORT_SetPinMux(PIN_I2S_TXD1_PORT, PIN_I2S_TXD1_BIT, PIN_I2S_TXD1_MUX);
    PORT_SetPinMux(PIN_I2S_TX_WCLK_PORT, PIN_I2S_TX_WCLK_BIT, PIN_I2S_TX_WCLK_MUX);
    PORT_SetPinMux(PIN_I2S_TX_BCLK_PORT, PIN_I2S_TX_BCLK_BIT, PIN_I2S_TX_BCLK_MUX);

    // SDHC pins
    const port_pin_config_t sdhcPinConfig = {
        .pullSelect = kPORT_PullUp,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .openDrainEnable = kPORT_OpenDrainDisable,
        .driveStrength = kPORT_HighDriveStrength,
        .mux = kPORT_MuxAlt4,
    };
    PORT_SetMultiplePinsConfig(PIN_SDHC_CLK_PORT,
        PIN_SDHC_CLK | PIN_SDHC_CMD | PIN_SDHC_D0 | PIN_SDHC_D1 | PIN_SDHC_D2 | PIN_SDHC_D3,
        &sdhcPinConfig);

    // LED pins
    PORT_SetPinMux(PIN_CH_LED_DIN_PORT, PIN_CH_LED_DIN_BIT, kPORT_MuxAlt2); // Alt2 = SPI0_SOUT
    PORT_SetPinMux(PIN_CH_LED_CLK_PORT, PIN_CH_LED_CLK_BIT, kPORT_MuxAlt2); // Alt2 = SPI0_CLK
    PORT_SetPinMux(PIN_CH_LED_LATCH_PORT, PIN_CH_LED_LATCH_BIT, kPORT_MuxAlt2); // Alt2 = SPI0_PCS0
    PORT_SetPinMux(PIN_CH_LED_OE_N_PORT, PIN_CH_LED_OE_N_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_BUTTON1_LED_PORT, PIN_BUTTON1_LED_BIT, kPORT_MuxAlt4); // Alt4 = FTM3_CH3

    const gpio_pin_config_t gpioOut1 = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1,
    };
    const gpio_pin_config_t gpioOut0 = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0,
    };

    // LEDs off by default.
    GPIO_PinInit(PIN_CH_LED_OE_N_GPIO, PIN_CH_LED_OE_N_BIT, &gpioOut1);

    // Buttons
    PORT_SetPinMux(PIN_BUTTON1_PORT, PIN_BUTTON1_BIT, kPORT_MuxAsGpio);
    PORT_SetPinMux(PIN_BUTTON2_PORT, PIN_BUTTON2_BIT, kPORT_MuxAsGpio);

    const gpio_pin_config_t gpioIn = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0,
    };
    GPIO_PinInit(PIN_BUTTON1_GPIO, PIN_BUTTON1_BIT, &gpioIn);
    GPIO_PinInit(PIN_BUTTON2_GPIO, PIN_BUTTON2_BIT, &gpioIn);

    PORT_SetPinInterruptConfig(PIN_BUTTON1_PORT, PIN_BUTTON1_BIT, kPORT_InterruptEitherEdge);
    PORT_SetPinInterruptConfig(PIN_BUTTON2_PORT, PIN_BUTTON2_BIT, kPORT_InterruptEitherEdge);

    PORT_EnablePinsDigitalFilter(PIN_BUTTON1_PORT, PIN_BUTTON1 | PIN_BUTTON2, true);

    // Set up digital filtering on button port for 20 ms.
    port_digital_filter_config_t filterConfig = {
        .digitalFilterWidth = 20,
        .clockSource = kPORT_LpoClock, // 1 kHz
    };
    PORT_SetDigitalFilterConfig(PIN_BUTTON1_PORT, &filterConfig);

    // DAC_RESET pin
    PORT_SetPinMux(PIN_DAC_RESET_PORT, PIN_DAC_RESET_BIT, kPORT_MuxAsGpio);
    GPIO_PinInit(PIN_DAC_RESET_GPIO, PIN_DAC_RESET_BIT, &gpioOut0);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
