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

#include "persistent_data_store.h"
#include "crc16.h"
#include "utility.h"
#include <cassert>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const uint32_t kErasedWord = 0xffffffff;

const uint32_t PersistentDataStore::kMaxPageDataOffset = DATA_STORE_PAGE_SIZE - FieldHeader::get_length();

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

PersistentDataStore::PersistentDataStore()
:   _activePage(kErasedWord),
    _nextWriteOffset(0),
    _nextPageVersion(0),
    _firstKey(nullptr)
{
}

void PersistentDataStore::init()
{
    FLASH_Init(&_flash);

    _load_pages();

    // If there is no active page, i.e. all pages are erased, then init a page.
    if (_activePage == kErasedWord)
    {
        _activePage = 0;
        _nextPageVersion = 1;
        _init_page(_activePage);
    }

    _nextWriteOffset = _scan_page(_activePage);
}

void PersistentDataStore::reset()
{
    uint32_t i;
    for (i = 0; i < DATA_STORE_PAGE_COUNT; ++i)
    {
        uint32_t pageAddress = DATA_STORE_BASE_ADDR + i * DATA_STORE_PAGE_SIZE;
        _erase_page(pageAddress);
    }
}

void PersistentDataStore::add_key(KeyInfo * info)
{
    info->next = _firstKey;
    _firstKey = info;
}

bool PersistentDataStore::read_key(KeyInfo * info, uint8_t * data, uint32_t length)
{
    if (info->newest == nullptr)
    {
        info->newest = _find_newest_field(info->key, _activePage);
        if (info->newest == nullptr)
        {
            return false;
        }
    }

    memcpy(data, info->newest->get_data(), length);

    return true;
}

bool PersistentDataStore::write_key(KeyInfo * info, const uint8_t * data, uint32_t length)
{
    if (info->newest == nullptr)
    {
        info->newest = _find_newest_field(info->key, _activePage);
    }

    // Check if this write will fit in the active page.
    uint32_t fieldLength = align_up<kWriteAlignment>(FieldHeader::get_length() + length);
    if (_nextWriteOffset + fieldLength > DATA_STORE_PAGE_SIZE)
    {
        // We need to merge fields into a new page.
        if (!_merge_fields())
        {
            return false;
        }
    }

    // Construct the field's header.
    uint32_t nextVersion = (info->newest != nullptr) ? (info->newest->fieldVersion + 1) : 1;

    FieldHeader newHeader;
    newHeader.key = info->key;
    newHeader.dataLength = length;
    newHeader.fieldLength = fieldLength;
    newHeader.fieldVersion = nextVersion;
    newHeader.crc = Crc16().compute(data, length);

    // Write to the active page.
    FieldHeader * writtenHeader = _write_field(&newHeader, data);
    if (!writtenHeader)
    {
        return false;
    }
    info->newest = writtenHeader;

    return true;
}

bool PersistentDataStore::_init_page(uint32_t page)
{
    PageInfo & info = _pages[page];

    // Erase page if it is not already.
    if (info.state != kErased)
    {
        if (!_erase_page(info.address))
        {
            return false;
        }
    }

    // Write the page's header with the next page data version.
    PageHeader header;
    header.magic = kMagicNumber;
    header.formatVersion = kFormatVersion;
    header.dataVersion = _nextPageVersion;
    if (!_write_data(info.address, &header, sizeof(header)))
    {
        return false;
    }

    info.state = kValid;
    info.version = _nextPageVersion;
    ++_nextPageVersion;

    return true;
}

bool PersistentDataStore::_merge_fields()
{
    uint32_t previousActivePage = _activePage;
    _activePage = _find_next_page();
    if (_activePage == kErasedWord)
    {
        return false;
    }

    // Erase page if it is not already.
    PageInfo & info = _pages[_activePage];
    if (info.state != kErased)
    {
        if (!_erase_page(info.address))
        {
            return false;
        }
    }

    // Set the page state to merging.
    info.state = kMerging;

    // Start writing after the header, which will not be written until all keys are merged.
    _nextWriteOffset = PageHeader::get_length();

    // Copy the latest version of each key into the new page.
    KeyInfo * key = _firstKey;
    while (key)
    {
        // Load key if necessary.
        if (!key->newest)
        {
            key->newest = _find_newest_field(key->key, previousActivePage);
        }

        // Copy key to new page. newest may be null if the key wasn't ever written.
        if (key->newest)
        {
            FieldHeader * newHeader = _write_field(key->newest, key->newest->get_data());
            if (newHeader == nullptr)
            {
                return false;
            }

            key->newest = newHeader;
        }

        key = key->next;
    }

    // Write the page's header with the next page data version.
    PageHeader header;
    header.magic = kMagicNumber;
    header.formatVersion = kFormatVersion;
    header.dataVersion = _nextPageVersion;
    if (!_write_data(info.address, &header, sizeof(header)))
    {
        return false;
    }

    info.state = kActive;
    info.version = _nextPageVersion;
    ++_nextPageVersion;

    // Erase previous page.
    _erase_page(_pages[previousActivePage].address);
    _pages[previousActivePage].state = kErased;

    return true;
}

PersistentDataStore::FieldHeader * PersistentDataStore::_write_field(const FieldHeader * header, const uint8_t * data)
{
    assert(_nextWriteOffset <= kMaxPageDataOffset);
    uint32_t writeAddress = _pages[_activePage].address + _nextWriteOffset;
    if (!_write_data(writeAddress, header, sizeof(FieldHeader)))
    {
        return nullptr;
    }
    if (!_write_data(writeAddress + FieldHeader::get_length(), data, header->dataLength))
    {
        return nullptr;
    }

    _nextWriteOffset += header->fieldLength;

    return reinterpret_cast<FieldHeader *>(writeAddress);
}

//! @brief Select the next page to write into.
//!
//! Returns either an erased page or an out of data valid page, which will have to be erased
//! before it can be used.
uint32_t PersistentDataStore::_find_next_page()
{
    uint32_t i;
    uint32_t activePageVersion = (_activePage != kErasedWord) ? _pages[_activePage].version : kErasedWord;
    for (i = 0; i < DATA_STORE_PAGE_COUNT; ++i)
    {
        if (_pages[i].state == kErased
            || (_pages[i].state == kValid && _pages[i].version < activePageVersion))
        {
            return i;
        }
    }
    return kErasedWord;
}

void PersistentDataStore::_load_pages()
{
    uint32_t i;
    uint32_t newestVersion = 0;

    _activePage = kErasedWord;

    for (i = 0; i < DATA_STORE_PAGE_COUNT; ++i)
    {
        uint32_t pageAddress = DATA_STORE_BASE_ADDR + i * DATA_STORE_PAGE_SIZE;
        PageHeader * header = reinterpret_cast<PageHeader *>(pageAddress);

        PageInfo info = { pageAddress, kErased, 0 };
        if (header->magic == kMagicNumber && header->formatVersion == kFormatVersion)
        {
            info.state = kValid;
            info.version = header->dataVersion;
        }
        else if (header->magic == kErasedWord && _is_page_erased(pageAddress))
        {
            info.state = kErased;
        }
        else
        {
            // Page is corrupted, attempt to erase it.
            if (_erase_page(pageAddress))
            {
                info.state = kErased;
            }
            else
            {
                info.state = kCorrupted;
            }
        }

        _pages[i] = info;

        if (info.version > newestVersion)
        {
            _activePage = i;
            newestVersion = info.version;
        }
    }

    if (_activePage != kErasedWord)
    {
        _pages[_activePage].state = kActive;
    }
    _nextPageVersion = newestVersion + 1;
}

uint32_t PersistentDataStore::_scan_page(uint32_t page)
{
    uint32_t offset = PageHeader::get_length();

    while (offset < kMaxPageDataOffset)
    {
        FieldHeader * field = reinterpret_cast<FieldHeader *>(_pages[page].address + offset);
        if (field->key == kErasedWord)
        {
            break;
        }

        offset += field->fieldLength;
    }

    return offset;
}

PersistentDataStore::FieldHeader * PersistentDataStore::_find_newest_field(uint32_t key, uint32_t page)
{
    uint32_t offset = PageHeader::get_length();
    uint32_t newestVersion = 0;
    FieldHeader * newestField = nullptr;

    while (offset < kMaxPageDataOffset)
    {
        FieldHeader * field = reinterpret_cast<FieldHeader *>(_pages[page].address + offset);

        if (field->key == kErasedWord)
        {
            break;
        }

        if (field->key == key)
        {
            // Check CRC.
            uint16_t crc = Crc16().compute(field->get_data(), field->dataLength);
            if (crc == field->crc
                && field->fieldVersion > newestVersion)
            {
                newestField = field;
                newestVersion = field->fieldVersion;
            }
        }

        offset += field->fieldLength;
    }

    return newestField;
}

bool PersistentDataStore::_is_page_erased(uint32_t address)
{
    uint32_t * word = reinterpret_cast<uint32_t *>(address);
    uint32_t * pageEnd = word + (DATA_STORE_PAGE_SIZE / sizeof(uint32_t));
    for (; word < pageEnd; ++word)
    {
        if (*word != kErasedWord)
        {
            return false;
        }
    }
    return true;
}

bool PersistentDataStore::_erase_page(uint32_t address)
{
    status_t status;
    status = FLASH_Erase(&_flash, address, DATA_STORE_PAGE_SIZE, kFLASH_ApiEraseKey);
    if (status != kStatus_Success)
    {
        return false;
    }

    status = FLASH_VerifyErase(&_flash, address, DATA_STORE_PAGE_SIZE, kFLASH_MarginValueUser);
    if (status != kStatus_Success)
    {
        return false;
    }

    return true;
}

//! @brief Write data to flash.
//!
//! The start address of the write must be correctly aligned, but the length can be less
//! than the minimum flash write size represented by #kWriteAlignment. If the length is
//! short, a temporary buffer is used to
bool PersistentDataStore::_write_data(uint32_t address, const void * data, uint32_t length)
{
    assert((address & (kWriteAlignment - 1)) == 0);

    bool result = true;

    // Write the aligned part of the data, in case the length is larger than the minimum
    // flash write size.
    uint32_t alignedLength = align_down<kWriteAlignment>(length);
    if (alignedLength > 0)
    {
        result = _write_and_verify(address, data, alignedLength);
    }

    // Write unaligned part of the data by copying into a temp buffer.
    if (alignedLength < length && result)
    {
        memset(_writeBuffer, 0xff, sizeof(_writeBuffer));
        memcpy(_writeBuffer, reinterpret_cast<const uint8_t *>(data) + alignedLength, length - alignedLength);

        result = _write_and_verify(address + alignedLength, _writeBuffer, sizeof(_writeBuffer));
    }

    return result;
}

//! @brief Write strictly aligned and sized data to flash and verify the write.
//!
//! Both the start address and data length must be aligned to the minimum flash write size.
bool PersistentDataStore::_write_and_verify(uint32_t address, const void * data, uint32_t length)
{
    uint32_t * writeData = reinterpret_cast<uint32_t *>(const_cast<void *>(data));

    status_t status;
    status = FLASH_Program(&_flash, address, writeData, length);
    if (status != kStatus_Success)
    {
        return false;
    }

    uint32_t failedAddress;
    uint32_t failedData;
    status = FLASH_VerifyProgram(&_flash, address, length, writeData, kFLASH_MarginValueUser, &failedAddress, &failedData);
    if (status != kStatus_Success)
    {
        return false;
    }

    return true;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
