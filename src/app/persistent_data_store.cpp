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
#include <cassert>

using namespace slab;

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

const uint32_t kErasedWord = 0xffffffff;

const uint32_t PersistentDataStore::FieldHeader::kHeaderCrcDataLength = sizeof(FieldHeader) - sizeof(uint16_t);

const uint32_t PersistentDataStore::Page::kMaxDataOffset = DATA_STORE_PAGE_SIZE - PersistentDataStore::FieldHeader::get_length();

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

PersistentDataStore::PersistentDataStore()
:   _activePage(nullptr),
    _nextPageVersion(0),
    _firstKey(nullptr)
{
}

void PersistentDataStore::init()
{
    FLASH_Init(&_flash);

    _load_pages();

    // If there is no active page, i.e. all pages are erased, then init a page.
    if (!_activePage)
    {
        uint32_t activePageNumber = _find_next_page();
        assert(activePageNumber != kErasedWord);
        _activePage = &_pages[activePageNumber];
        _nextPageVersion = 1;
        _activePage->format_page();
    }
}

void PersistentDataStore::reset()
{
    uint32_t i;
    for (i = 0; i < DATA_STORE_PAGE_COUNT; ++i)
    {
        _pages[i].erase();
    }
}

void PersistentDataStore::add_key(KeyInfo * info)
{
    assert(info->key != kErasedWord && "key must not be 0xffffffff");
    info->next = _firstKey;
    _firstKey = info;
}

bool PersistentDataStore::read_key(KeyInfo * info, uint8_t * data, uint32_t length)
{
    if (info->newest == nullptr)
    {
        info->newest = _activePage->find_newest_field(info->key);
        if (info->newest == nullptr)
        {
            return false;
        }
    }

    if (data)
    {
        memcpy(data, info->newest->get_data(), length);
    }

    return true;
}

bool PersistentDataStore::write_key(KeyInfo * info, const uint8_t * data, uint32_t length)
{
    if (info->newest == nullptr)
    {
        info->newest = _activePage->find_newest_field(info->key);
    }

    // If the data is not changing, then we don't really need to write the field.
    if (info->newest
        && (length == info->newest->dataLength)
        && (memcmp(info->newest->get_data(), data, length) == 0))
    {
        return true;
    }

    // Check if this write will fit in the active page.
    uint32_t fieldLength = align_up<kWriteAlignment>(FieldHeader::get_length() + length);
    if (!_activePage->can_write_field(fieldLength))
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
    newHeader.crc = newHeader.compute_crc(data);

    // Write to the active page.
    FieldHeader * writtenHeader = _activePage->write_field(&newHeader, data);
    if (!writtenHeader)
    {
        return false;
    }
    info->newest = writtenHeader;

    return true;
}

bool PersistentDataStore::_merge_fields()
{
    uint32_t nextPage = _find_next_page();
    if (nextPage == kErasedWord)
    {
        return false;
    }

    // Start merging. The page header will not be written until we're done merging, so
    // the page will be identified as corrupt if we get interrupted somehow.
    Page & page = _pages[nextPage];
    if (!page.start_merge())
    {
        return false;
    }

    // Copy the latest version of each key into the new page.
    KeyInfo * key = _firstKey;
    while (key)
    {
        // Load key from page we're merging from if necessary.
        if (!key->newest)
        {
            key->newest = _activePage->find_newest_field(key->key);
        }

        // Copy key to new page. newest may be null if the key wasn't ever written.
        if (key->newest)
        {
            // Copy the header and reset the version.
            FieldHeader updatedHeader = *(key->newest);
            updatedHeader.fieldVersion = 1;

            // Write the updated header and data to the new page.
            FieldHeader * newHeader = page.write_field(&updatedHeader, key->newest->get_data());
            if (newHeader == nullptr)
            {
                return false;
            }

            key->newest = newHeader;
        }

        key = key->next;
    }

    // Complete the merge. This writes the page header and sets the page state to kActive.
    if (!page.finish_merge())
    {
        return false;
    }

    // Erase previous active page.
    _activePage->erase();

    // Update active page pointer.
    _activePage = &_pages[nextPage];

    return true;
}

//! @brief Select the next page to write into.
//!
//! Returns either an erased page or an out of date valid page, which will have to be erased
//! before it can be used.
uint32_t PersistentDataStore::_find_next_page()
{
    uint32_t i;
    uint32_t activePageVersion = kErasedWord;
    uint32_t startPage = 0;

    // Start searching from the page after the active page.
    if (_activePage)
    {
        activePageVersion = _activePage->get_version();
        startPage = ((reinterpret_cast<uint32_t>(_activePage) - reinterpret_cast<uint32_t>(&_pages[0])) / sizeof(Page) + 1) % DATA_STORE_PAGE_COUNT;
    }
    for (i = startPage; i < DATA_STORE_PAGE_COUNT; ++i)
    {
        if (_pages[i].get_state() == Page::kErased
            || (_pages[i].get_state() == Page::kValid && _pages[i].get_version() < activePageVersion))
        {
            return i;
        }
    }
    for (i = 0; i < startPage; ++i)
    {
        if (_pages[i].get_state() == Page::kErased
            || (_pages[i].get_state() == Page::kValid && _pages[i].get_version() < activePageVersion))
        {
            return i;
        }
    }

    // Failed to find a page we can write into!
    return kErasedWord;
}

void PersistentDataStore::_load_pages()
{
    uint32_t i;
    uint32_t newestVersion = 0;
    uint32_t activePageNumber = kErasedWord;

    for (i = 0; i < DATA_STORE_PAGE_COUNT; ++i)
    {
        Page & page = _pages[i];
        uint32_t pageAddress = DATA_STORE_BASE_ADDR + i * DATA_STORE_PAGE_SIZE;
        page.init(pageAddress);

        if (page.get_state() == Page::kValid)
        {
            uint32_t pageVersion = page.get_version();
            if (pageVersion > newestVersion)
            {
                activePageNumber = i;
                newestVersion = pageVersion;
            }
        }
    }

    if (activePageNumber != kErasedWord)
    {
        _activePage = &_pages[activePageNumber];
        _activePage->set_state(Page::kActive);
    }
    _nextPageVersion = newestVersion + 1;
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

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

PersistentDataStore::Page::Page()
:   _address(0),
    _header(nullptr),
    _state(kCorrupted),
    _nextWriteOffset(0)
{
}

void PersistentDataStore::Page::init(uint32_t address)
{
    _address = address;
    _header = reinterpret_cast<PageHeader *>(_address);

    if (_header->magic == PageHeader::kMagicNumber
        && _header->formatVersion == PageHeader::kFormatVersion)
    {
        _state = kValid;
        _scan();
    }
    else if (_header->magic == kErasedWord && is_erased())
    {
        _state = kErased;
    }
    // Page is corrupted, attempt to erase it.
    else if (erase())
    {
        _state = kErased;
    }
    else
    {
        _state = kCorrupted;
    }
}

void PersistentDataStore::Page::_scan()
{
    uint32_t offset = PageHeader::get_length();

    while (offset < kMaxDataOffset)
    {
        FieldHeader * field = reinterpret_cast<FieldHeader *>(_address + offset);
        if (field->key == kErasedWord)
        {
            break;
        }

        offset += field->fieldLength;
    }

    _nextWriteOffset = offset;
}

PersistentDataStore::FieldHeader * PersistentDataStore::Page::find_newest_field(uint32_t key)
{
    uint32_t offset = PageHeader::get_length();
    uint32_t newestVersion = 0;
    FieldHeader * newestField = nullptr;

    while (offset < kMaxDataOffset)
    {
        FieldHeader * field = reinterpret_cast<FieldHeader *>(_address + offset);

        if (field->key == kErasedWord)
        {
            break;
        }

        if (field->key == key)
        {
            // Check CRC.
            if (field->compute_crc(field->get_data()) == field->crc
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

bool PersistentDataStore::Page::is_erased()
{
    uint32_t * word = reinterpret_cast<uint32_t *>(_address);
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

bool PersistentDataStore::Page::erase()
{
    flash_config_t * flash = &PersistentDataStore::get()._flash;
    status_t status = FLASH_Erase(flash, _address, DATA_STORE_PAGE_SIZE, kFLASH_ApiEraseKey);
    if (status != kStatus_Success)
    {
        return false;
    }

    status = FLASH_VerifyErase(flash, _address, DATA_STORE_PAGE_SIZE, kFLASH_MarginValueUser);
    if (status != kStatus_Success)
    {
        return false;
    }

    // Clear state variables.
    _state = kErased;
    _nextWriteOffset = 0;

    return true;
}

bool PersistentDataStore::Page::start_merge()
{
    // Erase this page if it's not already erased.
    if (!_ensure_erased())
    {
        return false;
    }

    // Set the page state to merging.
    _state = kMerging;

    // Start writing after the header, which will not be written until all keys are merged.
    _nextWriteOffset = PageHeader::get_length();

    return true;
}

bool PersistentDataStore::Page::finish_merge()
{
    assert(_state == kMerging);

    // Write the page's header with the next page data version.
    if (!_write_header())
    {
        return false;
    }

    _state = kActive;

    return true;
}

bool PersistentDataStore::Page::format_page()
{
    // Erase page if it is not already.
    if (!_ensure_erased())
    {
        return false;
    }

    // Place the page's header.
    if (!_write_header())
    {
        return false;
    }

    _nextWriteOffset = PageHeader::get_length();

    return true;
}

bool PersistentDataStore::Page::_ensure_erased()
{
    // Erase page if it is not already.
    return (_state == kErased) || erase();
}

bool PersistentDataStore::Page::_write_header()
{
    // Write the page's header with the next page data version.
    PageHeader header;
    header.magic = PageHeader::kMagicNumber;
    header.formatVersion = PageHeader::kFormatVersion;
    header.dataVersion =  PersistentDataStore::get()._get_next_page_version();
    if (!PersistentDataStore::get()._write_data(_address, &header, sizeof(header)))
    {
        return false;
    }

    _state = kValid;

    return true;
}

bool PersistentDataStore::Page::can_write_field(uint32_t length)
{
    return (_nextWriteOffset + length < DATA_STORE_PAGE_SIZE);
}

PersistentDataStore::FieldHeader * PersistentDataStore::Page::write_field(const FieldHeader * header, const uint8_t * data)
{
    assert((_nextWriteOffset + header->fieldLength) < DATA_STORE_PAGE_SIZE);
    PersistentDataStore & store = PersistentDataStore::get();
    uint32_t writeAddress = _address + _nextWriteOffset;
    if (!store._write_data(writeAddress, header, sizeof(FieldHeader)))
    {
        return nullptr;
    }
    if (!store._write_data(writeAddress + FieldHeader::get_length(), data, header->dataLength))
    {
        return nullptr;
    }

    _nextWriteOffset += header->fieldLength;

    return reinterpret_cast<FieldHeader *>(writeAddress);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
