/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * Copyright (c) 2017-2019 Immo Software
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdbool.h>
#include "argon/argon.h"
#include "fsl_sdmmc_event.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Get event instance.
 * @param eventType The event type
 * @return The event instance's pointer.
 */
static ar_semaphore_t *SDMMCEVENT_GetInstance(sdmmc_event_t eventType);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Transfer complete event. */
static ar_semaphore_t g_eventTransferComplete;
/*! @brief Card detect event. */
static ar_semaphore_t g_eventCardDetect;

/*******************************************************************************
 * Code
 ******************************************************************************/

static ar_semaphore_t *SDMMCEVENT_GetInstance(sdmmc_event_t eventType)
{
    ar_semaphore_t *event;

    switch (eventType)
    {
        case kSDMMCEVENT_TransferComplete:
            event = &g_eventTransferComplete;
            break;
        case kSDMMCEVENT_CardDetect:
            event = &g_eventCardDetect;
            break;
        default:
            event = NULL;
            break;
    }

    return event;
}

bool SDMMCEVENT_Create(sdmmc_event_t eventType)
{
    ar_semaphore_t *event = SDMMCEVENT_GetInstance(eventType);

    if (event)
    {
        ar_semaphore_create(event, (eventType == kSDMMCEVENT_TransferComplete) ? "sdtx" : "sdcd", 0);

        return true;
    }
    else
    {
        return false;
    }
}

bool SDMMCEVENT_Wait(sdmmc_event_t eventType, uint32_t timeoutMilliseconds)
{
    ar_semaphore_t *event = SDMMCEVENT_GetInstance(eventType);

    if (timeoutMilliseconds && event)
    {
        if (ar_semaphore_get(event, timeoutMilliseconds) != kArSuccess)
        {
            return false; /* timeout */
        }
        else
        {
            return true; /* event taken */
        }
    }
    else
    {
        return false;
    }
}

bool SDMMCEVENT_Notify(sdmmc_event_t eventType)
{
    ar_semaphore_t *event = SDMMCEVENT_GetInstance(eventType);

    if (event)
    {
        if (ar_semaphore_put(event) == kArSuccess)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

void SDMMCEVENT_Delete(sdmmc_event_t eventType)
{
    ar_semaphore_t *event = SDMMCEVENT_GetInstance(eventType);

    if (event)
    {
        ar_semaphore_delete(event);
    }
}

void SDMMCEVENT_Delay(uint32_t milliseconds)
{
    ar_thread_sleep(milliseconds);
}
