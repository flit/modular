/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
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

#include "fsl_host.h"
#include "fsl_sdhc.h"
#include "event.h"
#include "board.h"
#include "fsl_port.h"
#include "argon/argon.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief host controller error recovery.
 * @param host base address.
 */
static void Host_ErrorRecovery(HOST_TYPE *hostBase);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static sdhc_handle_t g_sdhcHandle;
static uint32_t g_sdhcAdmaTable[SDHC_ADMA_TABLE_WORDS];
static volatile bool g_sdhcTransferSuccessFlag = true;
/*! @brief Card detect flag. */
static volatile uint32_t g_sdInsertedFlag;
/*******************************************************************************
 * Code
 ******************************************************************************/

/* Delay some time united in milliseconds. */
static void Delay(uint32_t milliseconds)
{
    uint32_t i;
    uint32_t j;

    for (i = 0; i < milliseconds; i++)
    {
        for (j = 0; j < 20000U; j++)
        {
            __asm("NOP");
        }
    }
}

// static void HOST_DetectCardByGpio(void)
// {
//     if (GPIO_ReadPinInput(BOARD_SDHC_CD_GPIO_BASE, BOARD_SDHC_CD_GPIO_PIN))
// #if defined BOARD_SDHC_CD_LOGIC_RISING
//     {
//         g_sdInsertedFlag = 1U;
//     }
//     else
//     {
//         g_sdInsertedFlag = 0U;
//     }
// #else
//     {
//         g_sdInsertedFlag = 0U;
//     }
//     else
//     {
//         g_sdInsertedFlag = 1U;
//     }
// #endif
// }

static void HOST_DetectCardInsertByHost(HOST_TYPE *hostBase)
{
    g_sdInsertedFlag = 1U;
    EVENT_Notify(kEVENT_CardDetect);
    HOST_CARD_DETECT_INSERT_INTERRUPT_DISABLE(hostBase);
}

static void HOST_DetectCardRemoveByHost(HOST_TYPE *hostBase)
{
    g_sdInsertedFlag = 0U;
    EVENT_Notify(kEVENT_CardDetect);
}

#if 0
/* Card detect. */
status_t HOST_DetectCard(HOST_TYPE *hostBase, host_detect_card_type_t cd, bool isHostReady)
{
    if (!EVENT_Create(kEVENT_CardDetect))
    {
        return kStatus_Fail;
    }

    if (cd == kHOST_DetectCardByGpioCD)
    {
        /* Card detection pin will generate interrupt on either eage */
//         PORT_SetPinInterruptConfig(BOARD_SDHC_CD_PORT_BASE, BOARD_SDHC_CD_GPIO_PIN, kPORT_InterruptEitherEdge);
//         /* Open card detection pin NVIC. */
//         NVIC_EnableIRQ(HOST_CARD_DETECT_IRQ);
//         /* set IRQ priority */
//         NVIC_SetPriority(HOST_CARD_DETECT_IRQ, 6U);
//         /* check card detect status */
//         HOST_DetectCardByGpio();
    }
    else if (cd == kHOST_DetectCardByHostDATA3)
    {
        if (!isHostReady)
        {
            return kStatus_Fail;
        }
        /* enable card detect through DATA3 */
        HOST_CARD_DETECT_DATA3_ENABLE(hostBase, true);
        /* enable card detect interrupt */
        HOST_CARD_DETECT_INSERT_ENABLE(hostBase);
        HOST_CARD_DETECT_INSERT_INTERRUPT_ENABLE(hostBase);
    }
    else
    {
        /* SDHC do not support detect card through CD */
        return kStatus_Fail;
    }

    if (!g_sdInsertedFlag)
    {
        /* Wait card inserted. */
        do
        {
            if (!EVENT_Wait(kEVENT_CardDetect, EVENT_TIMEOUT_CARD_DETECT))
            {
                return kStatus_Fail;
            }
        } while (!g_sdInsertedFlag);
    }

    EVENT_Delete(kEVENT_CardDetect);

    /* Delat some time to make card stable. */
    Delay(1000U);

    return kStatus_Success;
}
#endif

/* Card detect pin port interrupt handler. */
// void HOST_CARD_DETECT_INTERRUPT_HANDLER(void)
// {
//     if (PORT_GetPinsInterruptFlags(BOARD_SDHC_CD_PORT_BASE) == (1U << BOARD_SDHC_CD_GPIO_PIN))
//     {
//         HOST_DetectCardByGpio();
//     }
//     /* Clear interrupt flag.*/
//     PORT_ClearPinsInterruptFlags(BOARD_SDHC_CD_PORT_BASE, ~0U);
//     EVENT_Notify(kEVENT_CardDetect);
// }

void CardInsertedCallback(void)
{
//     __BKPT(0);
//     PORTE->PCR[4] |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK; // Enable pull-up.
    SDHC_DisableInterruptSignal(SDHC, kSDHC_CardInsertionFlag);
    SDHC_ClearInterruptStatusFlags(SDHC, kSDHC_CardInsertionFlag);
    SDHC_EnableInterruptSignal(SDHC, kSDHC_CardRemovalFlag);
//     SD_InitCard();
}

void CardRemovedCallback(void)
{
//     __BKPT(0);
//     PORTE->PCR[4] &= ~PORT_PCR_PE_MASK; // Disable pull-up.
    SDHC_DisableInterruptSignal(SDHC, kSDHC_CardRemovalFlag);
    SDHC_ClearInterruptStatusFlags(SDHC, kSDHC_CardRemovalFlag);
    SDHC_EnableInterruptSignal(SDHC, kSDHC_CardInsertionFlag);
}

/* SDHC transfer complete callback function. */
static void SDHC_TransferCompleteCallback(SDHC_Type *base, sdhc_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_Success)
    {
        g_sdhcTransferSuccessFlag = true;
    }
    else
    {
        g_sdhcTransferSuccessFlag = false;
    }

    EVENT_Notify(kEVENT_TransferComplete);
}

/* User defined transfer function. */
static status_t SDHC_TransferFunction(SDHC_Type *base, sdhc_transfer_t *content)
{
    status_t error = kStatus_Success;

    do
    {
        error = SDHC_TransferNonBlocking(base, &g_sdhcHandle, g_sdhcAdmaTable, SDHC_ADMA_TABLE_WORDS, content);
    } while (error == kStatus_SDHC_BusyTransferring);

    if ((error != kStatus_Success) || (false == EVENT_Wait(kEVENT_TransferComplete, EVENT_TIMEOUT_TRANSFER_COMPLETE)) ||
        (!g_sdhcTransferSuccessFlag))
    {
        error = kStatus_Fail;
        /* host error recovery */
        Host_ErrorRecovery(base);
    }

    return error;
}

static void Host_ErrorRecovery(HOST_TYPE *hostBase)
{
    uint32_t status = 0U;
    /* get host present status */
    status = SDHC_GetPresentStatusFlags(hostBase);
    /* check command inhibit status flag */
    if ((status & kSDHC_CommandInhibitFlag) != 0U)
    {
        /* reset command line */
        SDHC_Reset(hostBase, kSDHC_ResetCommand, 100U);
    }
    /* check data inhibit status flag */
    if ((status & kSDHC_DataInhibitFlag) != 0U)
    {
        /* reset data line */
        SDHC_Reset(hostBase, kSDHC_ResetData, 100U);
    }
}

status_t HOST_Init(void *host)
{
    sdhc_transfer_callback_t sdhcCallback = {0};
    sdhc_host_t *sdhcHost = (sdhc_host_t *)host;

    /* Initializes SDHC. */
    sdhcHost->config.cardDetectDat3 = false;
    sdhcHost->config.endianMode = SDHC_ENDIAN_MODE;
    sdhcHost->config.dmaMode = SDHC_DMA_MODE;
    sdhcHost->config.readWatermarkLevel = SDHC_READ_WATERMARK_LEVEL;
    sdhcHost->config.writeWatermarkLevel = SDHC_WRITE_WATERMARK_LEVEL;
    SDHC_Init(sdhcHost->base, &(sdhcHost->config));

    /* Create handle for SDHC driver */
    sdhcCallback.TransferComplete = SDHC_TransferCompleteCallback;
    sdhcCallback.CardInserted = HOST_DetectCardInsertByHost;
    sdhcCallback.CardRemoved = HOST_DetectCardRemoveByHost;
    SDHC_TransferCreateHandle(sdhcHost->base, &g_sdhcHandle, &sdhcCallback, NULL);

    /* Create transfer complete event. */
    if (false == EVENT_Create(kEVENT_TransferComplete))
    {
        return kStatus_Fail;
    }

    /* Define transfer function. */
    sdhcHost->transfer = SDHC_TransferFunction;

    return kStatus_Success;
}

void HOST_Reset(HOST_TYPE *hostBase)
{
    /* reserved for future */
}

void HOST_Deinit(void *host)
{
    sdhc_host_t *sdhcHost = (sdhc_host_t *)host;
    SDHC_Deinit(sdhcHost->base);
    EVENT_Delete(kEVENT_TransferComplete);
}
