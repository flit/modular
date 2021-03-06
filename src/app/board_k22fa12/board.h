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

#if __cplusplus
const uint32_t kXtal0Freq = 12000000;
const uint32_t kXtal32Freq = 32768;
#endif

// Persistent data store address in flash.
#define DATA_STORE_BASE_ADDR (FSL_FEATURE_FLASH_FLEX_NVM_START_ADDRESS)
#define DATA_STORE_PAGE_SIZE (FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_SECTOR_SIZE)
#define DATA_STORE_WRITE_SIZE (FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_WRITE_UNIT_SIZE)
#define DATA_STORE_PAGE_COUNT (4)

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
#define BOARD_SD_DETECT_TYPE kHOST_DetectCardByHostCD

// SWO
#define PIN_SWO_PORT            (PORTA)
#define PIN_SWO_GPIO            (GPIOA)
#define PIN_SWO_BIT             (2)
#define PIN_SWO                 (1 << PIN_SWO_BIT)

// SWO
#define PIN_DAC_RESET_PORT      (PORTC)
#define PIN_DAC_RESET_GPIO      (GPIOC)
#define PIN_DAC_RESET_BIT       (5)
#define PIN_DAC_RESET           (1 << PIN_DAC_RESET_BIT)

// LEDs
#define PIN_CH_LED_LATCH_PORT       (PORTA)
#define PIN_CH_LED_LATCH_GPIO       (GPIOA)
#define PIN_CH_LED_LATCH_GPIO_BASE  (GPIOA_BASE)
#define PIN_CH_LED_LATCH_BIT        (14)
#define PIN_CH_LED_LATCH            (1 << PIN_CH_LED_LATCH_BIT)
#define PIN_CH_LED_LATCH_SPI_PCS    (kDSPI_Pcs0)

#define PIN_CH_LED_DIN_PORT         (PORTA)
#define PIN_CH_LED_DIN_GPIO         (GPIOA)
#define PIN_CH_LED_DIN_GPIO_BASE    (GPIOA_BASE)
#define PIN_CH_LED_DIN_BIT          (16)
#define PIN_CH_LED_DIN              (1 << PIN_CH_LED_DIN_BIT)

#define PIN_CH_LED_CLK_PORT         (PORTA)
#define PIN_CH_LED_CLK_GPIO         (GPIOA)
#define PIN_CH_LED_CLK_GPIO_BASE    (GPIOA_BASE)
#define PIN_CH_LED_CLK_BIT          (15)
#define PIN_CH_LED_CLK              (1 << PIN_CH_LED_CLK_BIT)

#define PIN_CH_LED_OE_N_PORT        (PORTA)
#define PIN_CH_LED_OE_N_GPIO        (GPIOA)
#define PIN_CH_LED_OE_N_GPIO_BASE   (GPIOA_BASE)
#define PIN_CH_LED_OE_N_BIT         (17)
#define PIN_CH_LED_OE_N             (1 << PIN_CH_LED_OE_N_BIT)

#define PIN_BUTTON1_LED_PORT    (PORTD)
#define PIN_BUTTON1_LED_GPIO    (GPIOD)
#define PIN_BUTTON1_LED_GPIO_BASE (GPIOD_BASE)
#define PIN_BUTTON1_LED_BIT     (3)
#define PIN_BUTTON1_LED         (1 << PIN_BUTTON1_LED_BIT)

#define BUTTON1_LED_FTM_BASE    (FTM3_BASE)
#define BUTTON1_LED_FTM_CHANNEL (kFTM_Chnl_3)

// Buttons
#define PIN_BUTTON1_PORT        (PORTD)
#define PIN_BUTTON1_GPIO        (GPIOD)
#define PIN_BUTTON1_BIT         (2)
#define PIN_BUTTON1             (1 << PIN_BUTTON1_BIT)

#define PIN_BUTTON2_PORT        (PORTD)
#define PIN_BUTTON2_GPIO        (GPIOD)
#define PIN_BUTTON2_BIT         (1)
#define PIN_BUTTON2             (1 << PIN_BUTTON2_BIT)

// SAI
#define PIN_I2S_MCLK_PORT       (PORTC)
#define PIN_I2S_MCLK_GPIO       (GPIOC)
#define PIN_I2S_MCLK_BIT        (6)
#define PIN_I2S_MCLK            (1 << PIN_I2S_MCLK_BIT)
#define PIN_I2S_MCLK_MUX        (kPORT_MuxAlt6)

#define PIN_I2S_TX_WCLK_PORT    (PORTC)
#define PIN_I2S_TX_WCLK_GPIO    (GPIOC)
#define PIN_I2S_TX_WCLK_BIT     (2)
#define PIN_I2S_TX_WCLK         (1 << PIN_I2S_TX_WCLK_BIT)
#define PIN_I2S_TX_WCLK_MUX     (kPORT_MuxAlt6)

#define PIN_I2S_TX_BCLK_PORT    (PORTC)
#define PIN_I2S_TX_BCLK_GPIO    (GPIOC)
#define PIN_I2S_TX_BCLK_BIT     (3)
#define PIN_I2S_TX_BCLK         (1 << PIN_I2S_TX_BCLK_BIT)
#define PIN_I2S_TX_BCLK_MUX     (kPORT_MuxAlt6)

#define PIN_I2S_TXD0_PORT       (PORTC)
#define PIN_I2S_TXD0_GPIO       (GPIOC)
#define PIN_I2S_TXD0_BIT        (1)
#define PIN_I2S_TXD0            (1 << PIN_I2S_TXD0_BIT)
#define PIN_I2S_TXD0_MUX        (kPORT_MuxAlt6)

#define PIN_I2S_TXD1_PORT       (PORTC)
#define PIN_I2S_TXD1_GPIO       (GPIOC)
#define PIN_I2S_TXD1_BIT        (0)
#define PIN_I2S_TXD1            (1 << PIN_I2S_TXD1_BIT)
#define PIN_I2S_TXD1_MUX        (kPORT_MuxAlt6)

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
//
// ADC0:
//  3  -> CV3
//  12 -> Pot1
//  13 -> Pot2
//  23 -> CV2
//
// ADC1
//  14 -> Pot3
//  15 -> Pot4
//  18 -> CV1
//  19 -> CV4

// Channel 1 CV/Gate = ADC1_SE18
#define CH1_CV_ADC            (1)
#define CH1_CV_CHANNEL        (18)
#define CH1_CV_CHANNEL_MASK   (1 << CH1_CV_CHANNEL)

// Channel 2 CV/Gate = ADC0_SE23
#define CH2_CV_ADC            (0)
#define CH2_CV_CHANNEL        (23)
#define CH2_CV_CHANNEL_MASK   (1 << CH2_CV_CHANNEL)

// Channel 3 CV/Gate = ADC0_DP3
#define CH3_CV_ADC            (0)
#define CH3_CV_CHANNEL        (3)
#define CH3_CV_CHANNEL_MASK   (1 << CH3_CV_CHANNEL)

// Channel 4 CV/Gate = ADC1_DM0
#define CH4_CV_ADC            (1)
#define CH4_CV_CHANNEL        (19)
#define CH4_CV_CHANNEL_MASK   (1 << CH4_CV_CHANNEL)

// Channel 1 Pot = PTB2/ADC0_SE12
#define CH1_POT_ADC            (0)
#define CH1_POT_CHANNEL        (12)
#define CH1_POT_CHANNEL_MASK   (1 << CH1_POT_CHANNEL)

// Channel 2 Pot = PTB3/ADC0_SE13
#define CH2_POT_ADC            (0)
#define CH2_POT_CHANNEL        (13)
#define CH2_POT_CHANNEL_MASK   (1 << CH2_POT_CHANNEL)

// Channel 3 Pot = PTB10/ADC1_SE14
#define CH3_POT_ADC            (1)
#define CH3_POT_CHANNEL        (14)
#define CH3_POT_CHANNEL_MASK   (1 << CH3_POT_CHANNEL)

// Channel 4 Pot = PTB11/ADC1_SE15
#define CH4_POT_ADC            (1)
#define CH4_POT_CHANNEL        (15)
#define CH4_POT_CHANNEL_MASK   (1 << CH4_POT_CHANNEL)


//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void init_debug_console();
void init_board();

#endif // __BOARD_H__
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
