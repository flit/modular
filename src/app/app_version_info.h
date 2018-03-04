/*
 * Copyright (c) 2018 Immo Software
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
#if !defined(_APP_VERSION_INFO_H_)
#define _APP_VERSION_INFO_H_

#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

#if STANDALONE_BUILD

    //! App starts at address 0 in a standalone build.
    #define APP_BASE_ADDR (0)

#else // STANDALONE_BUILD

    //! App starts at 64kB offset.
    #define APP_BASE_ADDR (0x10000)

    //! Bootloader starts at address 0.
    #define BOOTLOADER_BASE_ADDR (0x0)

#endif // STANDALONE_BUILD

/*!
 * @brief Version information.
 */
struct Version
{
    uint8_t major;
    uint8_t minor;
    uint16_t bugfix;
};

/*!
 * @brief General information about the application.
 */
struct AppVersionInfo
{
    uint32_t signature; //!< Four char code signature for the application.
    Version version;    //!< Current version of the app.
    const char * name;  //!< Long name of the app.
    const char * versionString; //!< Long form of the version as a string.
    const char * sha;   //!< Git SHA-1.
};

/*!
 * @brief Start of the app's vector table.
 */
struct AppVectors
{
    uint32_t initialStack;
    uint32_t resetHandler;
    uint32_t nmiHandler;
    uint32_t hardFaultHandler;
    uint32_t memManageHandler;
    uint32_t busFaultHandler;
    uint32_t usageFaultHandler;
    uint32_t signature;
    uint32_t crc32;
    uint32_t appSize;
    AppVersionInfo * appVersionInfo;
};

} // namespace slab

//! Global app version info instance.
extern const slab::AppVersionInfo g_appVersionInfo;

#endif // _APP_VERSION_INFO_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
