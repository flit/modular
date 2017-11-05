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
#if !defined(_DEBUG_LOG_H_)
#define _DEBUG_LOG_H_

#include "simple_string.h"
#include "ring_buffer.h"
#include "microseconds.h"
#include <string.h>
#include <stdio.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

#define ERROR_MASK (1<<0)
#define INIT_MASK (1<<1)
#define TRIG_MASK (1<<2)
#define RETRIG_MASK (1<<3)
#define QUEUE_MASK (1<<4)
#define RETIRE_MASK (1<<5)
#define CURBUF_MASK (1<<6)
#define FILL_MASK (1<<7)
#define TIME_MASK (1<<8)
#define BUTTON_MASK (1<<9)
#define MISC_MASK (1<<31)

#if !defined(ENABLE_DEBUG_PRINTF)
#define ENABLE_DEBUG_PRINTF (DEBUG)
#endif

#if !defined(DEBUG_PRINTF_TO_UART)
#define DEBUG_PRINTF_TO_UART (0)
#endif

#if !defined(DEBUG_PRINTF_MASK)
#define DEBUG_PRINTF_MASK (ERROR_MASK|INIT_MASK|BUTTON_MASK|TRIG_MASK|RETRIG_MASK|FILL_MASK|TIME_MASK|QUEUE_MASK)
#endif

#define DEBUG_LOG_MSG_SIZE (48)
#define DEBUG_LOG_ENTRY_COUNT (50)

#if ENABLE_DEBUG_PRINTF && !DEBUG_PRINTF_TO_UART

extern char g_debugLogBuffer[DEBUG_LOG_MSG_SIZE];
extern slab::RingBuffer<slab::SimpleString<DEBUG_LOG_MSG_SIZE>, DEBUG_LOG_ENTRY_COUNT> g_debugLog;

#define DEBUG_PRINTF(f, m, ...) \
    do { if ((f) & DEBUG_PRINTF_MASK) { \
        snprintf(g_debugLogBuffer, sizeof(g_debugLogBuffer), "[%lu] " m, Microseconds::get(), ##__VA_ARGS__); \
        slab::SimpleString<DEBUG_LOG_MSG_SIZE> _ds(g_debugLogBuffer); \
        g_debugLog.put(_ds); \
    } } while (0)

#define DEFINE_DEBUG_LOG \
    char g_debugLogBuffer[DEBUG_LOG_MSG_SIZE]; \
    RingBuffer<SimpleString<DEBUG_LOG_MSG_SIZE>, DEBUG_LOG_ENTRY_COUNT> g_debugLog;

#elif ENABLE_DEBUG_PRINTF && DEBUG_PRINTF_TO_UART

#define DEBUG_PRINTF(f, m, ...) do { if ((f) & DEBUG_PRINTF_MASK) { \
        printf("[%lu] " m, Microseconds::get(), ##__VA_ARGS__); \
    } } while (0)
#define DEFINE_DEBUG_LOG

#else // !ENABLE_DEBUG_PRINTF

#define DEBUG_PRINTF(f, m, ...)
#define DEFINE_DEBUG_LOG

#endif // !ENABLE_DEBUG_PRINTF

#endif // _DEBUG_LOG_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
