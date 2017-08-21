/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#ifndef _FSL_MMC_H_
#define _FSL_MMC_H_

#include "fsl_sdmmc_common.h"

/*!
 * @addtogroup MMCCARD
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief MMC card flags */
enum _mmc_card_flag
{
    kMMC_SupportHighSpeed26MHZFlag = (1U << 0U),           /*!< Support high speed 26MHZ */
    kMMC_SupportHighSpeed52MHZFlag = (1U << 1U),           /*!< Support high speed 52MHZ */
    kMMC_SupportHighSpeedDDR52MHZ180V300VFlag = (1 << 2U), /*!< ddr 52MHZ 1.8V or 3.0V */
    kMMC_SupportHighSpeedDDR52MHZ120VFlag = (1 << 3U),     /*!< DDR 52MHZ 1.2V */
    kMMC_SupportHS200200MHZ180VFlag = (1 << 4U),           /*!< HS200 ,200MHZ,1.8V */
    kMMC_SupportHS200200MHZ120VFlag = (1 << 5U),           /*!< HS200, 200MHZ, 1.2V */
    kMMC_SupportHS400DDR200MHZ180VFlag = (1 << 6U),        /*!< HS400, DDR, 200MHZ,1.8V */
    kMMC_SupportHS400DDR200MHZ120VFlag = (1 << 7U),        /*!< HS400, DDR, 200MHZ,1.2V */
    kMMC_SupportHighCapacityFlag = (1U << 8U),             /*!< Support high capacity */
    kMMC_SupportAlternateBootFlag = (1U << 9U),            /*!< Support alternate boot */
    kMMC_SupportDDRBootFlag = (1U << 10U),                 /*!< support DDR boot flag*/
    kMMC_SupportHighSpeedBootFlag = (1U << 11U),           /*!< support high speed boot flag*/

    kMMC_DataBusWidth4BitFlag = (1U << 12U), /*!< current data bus is 4 bit mode*/
    kMMC_DataBusWidth8BitFlag = (1U << 13U), /*!< current data bus is 8 bit mode*/
    kMMC_DataBusWidth1BitFlag = (1U << 14U), /*!< current data bus is 1 bit mode */

};

/*!
 * @brief mmc card state
 *
 * Define the card structure including the necessary fields to identify and describe the card.
 */
typedef struct _mmc_card
{
    SDMMCHOST_CONFIG host; /*!< Host information */

    bool isHostReady;                                     /*!< use this flag to indicate if need host re-init or not*/
    uint32_t busClock_Hz;                                 /*!< MMC bus clock united in Hz */
    uint32_t relativeAddress;                             /*!< Relative address of the card */
    bool enablePreDefinedBlockCount;                      /*!< Enable PRE-DEFINED block count when read/write */
    uint32_t flags;                                       /*!< Capability flag in _mmc_card_flag */
    uint32_t rawCid[4U];                                  /*!< Raw CID content */
    uint32_t rawCsd[4U];                                  /*!< Raw CSD content */
    uint32_t rawExtendedCsd[MMC_EXTENDED_CSD_BYTES / 4U]; /*!< Raw MMC Extended CSD content */
    uint32_t ocr;                                         /*!< Raw OCR content */
    mmc_cid_t cid;                                        /*!< CID */
    mmc_csd_t csd;                                        /*!< CSD */
    mmc_extended_csd_t extendedCsd;                       /*!< Extended CSD */
    uint32_t blockSize;                                   /*!< Card block size */
    uint32_t userPartitionBlocks;                         /*!< Card total block number in user partition */
    uint32_t bootPartitionBlocks;                         /*!< Boot partition size united as block size */
    uint32_t eraseGroupBlocks;                            /*!< Erase group size united as block size */
    mmc_access_partition_t currentPartition;              /*!< Current access partition */
    mmc_voltage_window_t hostVoltageWindowVCCQ;           /*!< Host IO voltage window */
    mmc_voltage_window_t hostVoltageWindowVCC; /*!< application must set this value according to board specific */
    mmc_high_speed_timing_t currentTiming;     /*!< indicate the current host timing mode*/

} mmc_card_t;

/*! @brief MMC card boot configuration definition. */
typedef struct _mmc_boot_config
{
    bool enableBootAck;                        /*!< Enable boot ACK */
    mmc_boot_partition_enable_t bootPartition; /*!< Boot partition */
    bool retainBootBusWidth;                   /*!< If retain boot bus width */
    mmc_data_bus_width_t bootDataBusWidth;     /*!< Boot data bus width */
} mmc_boot_config_t;

/*************************************************************************************************
 * API
 ************************************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name MMCCARD Function
 * @{
 */

/*!
 * @brief Initializes the MMC card and host.
 *
 * @param card Card descriptor.
 *
 * @retval kStatus_SDMMC_HostNotReady host is not ready.
 * @retval kStatus_SDMMC_GoIdleFailed Go idle failed.
 * @retval kStatus_SDMMC_SendOperationConditionFailed Send operation condition failed.
 * @retval kStatus_SDMMC_AllSendCidFailed Send CID failed.
 * @retval kStatus_SDMMC_SetRelativeAddressFailed Set relative address failed.
 * @retval kStatus_SDMMC_SendCsdFailed Send CSD failed.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_SelectCardFailed Send SELECT_CARD command failed.
 * @retval kStatus_SDMMC_SendExtendedCsdFailed Send EXT_CSD failed.
 * @retval kStatus_SDMMC_SetBusWidthFailed Set bus width failed.
 * @retval kStatus_SDMMC_SwitchHighSpeedFailed Switch high speed failed.
 * @retval kStatus_SDMMC_SetCardBlockSizeFailed Set card block size failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t MMC_Init(mmc_card_t *card);

/*!
 * @brief Deinitializes the card and host.
 *
 * @param card Card descriptor.
 */
void MMC_Deinit(mmc_card_t *card);

/*!
 * @brief intialize the card.
 *
 * @param card Card descriptor.
 *
 * @retval kStatus_SDMMC_HostNotReady host is not ready.
 * @retval kStatus_SDMMC_GoIdleFailed Go idle failed.
 * @retval kStatus_SDMMC_SendOperationConditionFailed Send operation condition failed.
 * @retval kStatus_SDMMC_AllSendCidFailed Send CID failed.
 * @retval kStatus_SDMMC_SetRelativeAddressFailed Set relative address failed.
 * @retval kStatus_SDMMC_SendCsdFailed Send CSD failed.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_SelectCardFailed Send SELECT_CARD command failed.
 * @retval kStatus_SDMMC_SendExtendedCsdFailed Send EXT_CSD failed.
 * @retval kStatus_SDMMC_SetBusWidthFailed Set bus width failed.
 * @retval kStatus_SDMMC_SwitchHighSpeedFailed Switch high speed failed.
 * @retval kStatus_SDMMC_SetCardBlockSizeFailed Set card block size failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t MMC_CardInit(mmc_card_t *card);

/*!
 * @brief Deinitializes the card.
 *
 * @param card Card descriptor.
 */
void MMC_CardDeinit(mmc_card_t *card);

/*!
 * @brief initialize the host.
 *
 * This function deinitializes the specific host.
 *
 * @param card Card descriptor.
 */
status_t MMC_HostInit(mmc_card_t *card);

/*!
 * @brief Deinitializes the host.
 *
 * This function deinitializes the host.
 *
 * @param card Card descriptor.
 */
void MMC_HostDeinit(mmc_card_t *card);

/*!
 * @brief reset the host.
 *
 * This function reset the specific host.
 *
 * @param host host descriptor.
 */
void MMC_HostReset(SDMMCHOST_CONFIG *host);

/*!
 * @brief Checks if the card is read-only.
 *
 * @param card Card descriptor.
 * @retval true Card is read only.
 * @retval false Card isn't read only.
 */
bool MMC_CheckReadOnly(mmc_card_t *card);

/*!
 * @brief Reads data blocks from the card.
 *
 * @param card Card descriptor.
 * @param buffer The buffer to save data.
 * @param startBlock The start block index.
 * @param blockCount The number of blocks to read.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_SetBlockCountFailed Set block count failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t MMC_ReadBlocks(mmc_card_t *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief Writes data blocks to the card.
 *
 * @param card Card descriptor.
 * @param buffer The buffer to save data blocks.
 * @param startBlock Start block number to write.
 * @param blockCount Block count.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_SetBlockCountFailed Set block count failed.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t MMC_WriteBlocks(mmc_card_t *card, const uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);

/*!
 * @brief Erases groups of the card.
 *
 * Erase group is the smallest erase unit in MMC card. The erase range is [startGroup, endGroup].
 *
 * @param  card Card descriptor.
 * @param  startGroup Start group number.
 * @param  endGroup End group number.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t MMC_EraseGroups(mmc_card_t *card, uint32_t startGroup, uint32_t endGroup);

/*!
 * @brief Selects the partition to access.
 *
 * @param card Card descriptor.
 * @param partitionNumber The partition number.
 * @retval kStatus_SDMMC_ConfigureExtendedCsdFailed Configure EXT_CSD failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t MMC_SelectPartition(mmc_card_t *card, mmc_access_partition_t partitionNumber);

/*!
 * @brief Configures the boot activity of the card.
 *
 * @param card Card descriptor.
 * @param config Boot configuration structure.
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_ConfigureExtendedCsdFailed Configure EXT_CSD failed.
 * @retval kStatus_SDMMC_ConfigureBootFailed Configure boot failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t MMC_SetBootConfig(mmc_card_t *card, const mmc_boot_config_t *config);

/* @} */
#if defined(__cplusplus)
}
#endif
/*! @} */
#endif /* _FSL_MMC_H_*/
