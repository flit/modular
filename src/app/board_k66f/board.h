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
#if !defined(__BOARD_H__)
#define __BOARD_H__

#include "fsl_device_registers.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

// SDHC base address, clock and card detection pin
#define BOARD_SDHC_BASEADDR SDHC
#define BOARD_SDHC_CLKSRC kCLOCK_CoreSysClk
#define BOARD_SDHC_CLK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)
#define BOARD_SDHC_IRQ SDHC_IRQn
#define BOARD_SDHC_CD_GPIO_BASE GPIOD
#define BOARD_SDHC_CD_GPIO_PIN 10U
#define BOARD_SDHC_CD_PORT_BASE PORTD
#define BOARD_SDHC_CD_PORT_IRQ PORTD_IRQn
#define BOARD_SDHC_CD_PORT_IRQ_HANDLER PORTD_IRQHandler
#define BOARD_SDHC_CD_LOGIC_RISING

// Device I2C ports and addresses
#define BOARD_CODEC_I2C_BASE (I2C1)
#define BOARD_CODEC_I2C_ADDR (0x1a)

// I2C0 (accelerometer and gyro)

// I2C0 SDA
#define PIN_I2C0_SDA_PORT   (PORTD)
#define PIN_I2C0_SDA_GPIO   (GPIOD)
#define PIN_I2C0_SDA_BIT    (8)
#define PIN_I2C0_SDA        (1 << PIN_I2C0_SDA_BIT)

// I2C0 SCL
#define PIN_I2C0_SCL_PORT   (PORTD)
#define PIN_I2C0_SCL_GPIO   (GPIOD)
#define PIN_I2C0_SCL_BIT    (9)
#define PIN_I2C0_SCL        (1 << PIN_I2C0_SCL_BIT)

// I2C1 (audio codec and proximity sensor)

// I2C1 SDA
#define PIN_I2C1_SDA_PORT   (PORTC)
#define PIN_I2C1_SDA_GPIO   (GPIOC)
#define PIN_I2C1_SDA_BIT    (11)
#define PIN_I2C1_SDA        (1 << PIN_I2C1_SDA_BIT)

// I2C1 SCL
#define PIN_I2C1_SCL_PORT   (PORTC)
#define PIN_I2C1_SCL_GPIO   (GPIOC)
#define PIN_I2C1_SCL_BIT    (10)
#define PIN_I2C1_SCL        (1 << PIN_I2C1_SCL_BIT)

// RGB LED
#define PIN_RED_LED_PORT    (PORTC)
#define PIN_RED_LED_GPIO    (GPIOC)
#define PIN_RED_LED_BIT     (9)
#define PIN_RED_LED         (1 << PIN_RED_LED_BIT)

#define PIN_GREEN_LED_PORT  (PORTE)
#define PIN_GREEN_LED_GPIO  (GPIOE)
#define PIN_GREEN_LED_BIT   (6)
#define PIN_GREEN_LED       (1 << PIN_GREEN_LED_BIT)

#define PIN_BLUE_LED_PORT   (PORTA)
#define PIN_BLUE_LED_GPIO   (GPIOA)
#define PIN_BLUE_LED_BIT    (11)
#define PIN_BLUE_LED        (1 << PIN_BLUE_LED_BIT)

// Buttons
#define PIN_BTN1_PORT       (PORTD)
#define PIN_BTN1_GPIO       (GPIOD)
#define PIN_BTN1_BIT        (13)
#define PIN_BTN1            (1 << PIN_BTN1_BIT)

// SAI
#define PIN_I2S_MCLK_PORT   (PORTC)
#define PIN_I2S_MCLK_GPIO   (GPIOC)
#define PIN_I2S_MCLK_BIT    (6)
#define PIN_I2S_MCLK        (1 << PIN_I2S_MCLK_BIT)

#define PIN_I2S_TX_WCLK_PORT   (PORTE)
#define PIN_I2S_TX_WCLK_GPIO   (GPIOE)
#define PIN_I2S_TX_WCLK_BIT    (11)
#define PIN_I2S_TX_WCLK        (1 << PIN_I2S_TX_WCLK_BIT)

#define PIN_I2S_TX_BCLK_PORT   (PORTE)
#define PIN_I2S_TX_BCLK_GPIO   (GPIOE)
#define PIN_I2S_TX_BCLK_BIT    (12)
#define PIN_I2S_TX_BCLK        (1 << PIN_I2S_TX_BCLK_BIT)

#define PIN_I2S_TXD_PORT   (PORTC)
#define PIN_I2S_TXD_GPIO   (GPIOC)
#define PIN_I2S_TXD_BIT    (1)
#define PIN_I2S_TXD        (1 << PIN_I2S_TXD_BIT)

#define PIN_I2S_RX_WCLK_PORT   (PORTE)
#define PIN_I2S_RX_WCLK_GPIO   (GPIOE)
#define PIN_I2S_RX_WCLK_BIT    (8)
#define PIN_I2S_RX_WCLK        (1 << PIN_I2S_RX_WCLK_BIT)

#define PIN_I2S_RX_BCLK_PORT   (PORTE)
#define PIN_I2S_RX_BCLK_GPIO   (GPIOE)
#define PIN_I2S_RX_BCLK_BIT    (9)
#define PIN_I2S_RX_BCLK        (1 << PIN_I2S_RX_BCLK_BIT)

#define PIN_I2S_RXD_PORT   (PORTE)
#define PIN_I2S_RXD_GPIO   (GPIOE)
#define PIN_I2S_RXD_BIT    (7)
#define PIN_I2S_RXD        (1 << PIN_I2S_RXD_BIT)

// SDHC
// D[3:0], CLK, CMD
#define PIN_SDHC_CLK_PORT   (PORTE)
#define PIN_SDHC_CLK_GPIO   (GPIOE)
#define PIN_SDHC_CLK_BIT    (2)
#define PIN_SDHC_CLK        (1 << PIN_SDHC_CLK_BIT)

#define PIN_SDHC_CMD_PORT   (PORTE)
#define PIN_SDHC_CMD_GPIO   (GPIOE)
#define PIN_SDHC_CMD_BIT    (3)
#define PIN_SDHC_CMD        (1 << PIN_SDHC_CMD_BIT)

#define PIN_SDHC_D0_PORT   (PORTE)
#define PIN_SDHC_D0_GPIO   (GPIOE)
#define PIN_SDHC_D0_BIT    (1)
#define PIN_SDHC_D0        (1 << PIN_SDHC_D0_BIT)

#define PIN_SDHC_D1_PORT   (PORTE)
#define PIN_SDHC_D1_GPIO   (GPIOE)
#define PIN_SDHC_D1_BIT    (0)
#define PIN_SDHC_D1        (1 << PIN_SDHC_D1_BIT)

#define PIN_SDHC_D2_PORT   (PORTE)
#define PIN_SDHC_D2_GPIO   (GPIOE)
#define PIN_SDHC_D2_BIT    (5)
#define PIN_SDHC_D2        (1 << PIN_SDHC_D2_BIT)

#define PIN_SDHC_D3_PORT   (PORTE)
#define PIN_SDHC_D3_GPIO   (GPIOE)
#define PIN_SDHC_D3_BIT    (4)
#define PIN_SDHC_D3        (1 << PIN_SDHC_D3_BIT)


// ADC inputs

// Channel 1 CV/Gate = ADC1_SE16
#define CH1_CV_ADC            (1)
#define CH1_CV_CHANNEL        (16)

// Channel 2 CV/Gate = ADC1_SE13 (PTB7)
#define CH2_CV_ADC            (1)
#define CH2_CV_CHANNEL        (13)


//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void init_debug_console();
void init_board();

#endif // __BOARD_H__
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
