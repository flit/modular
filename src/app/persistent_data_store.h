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
#if !defined(_PERSISTENT_DATA_STORE_H_)
#define _PERSISTENT_DATA_STORE_H_

#include <stdint.h>
#include "singleton.h"
#include "board.h"
#include "utility.h"
#include "fsl_flash.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Manages storage of key value pairs in flash.
 *
 * The sum of the sizes of all fields plus headers must be less than the size of a page.
 *
 * Several configuration macros are used:
 * - #DATA_STORE_BASE_ADDR - Address of first page in flash.
 * - #DATA_STORE_PAGE_SIZE - Size in bytes of each flash page.
 * - #DATA_STORE_WRITE_SIZE - Flash write alignment and minimum byte count.
 * - #DATA_STORE_PAGE_COUNT - Number of pages allocated to the data store.
 */
class PersistentDataStore : public Singleton<PersistentDataStore>
{
protected:

    //! @brief Minimum alignment of the start of each individual flash write.
    static const uint32_t kWriteAlignment = DATA_STORE_WRITE_SIZE;

public:

    // Must have at least two pages.
    static_assert(DATA_STORE_PAGE_COUNT >= 2);

    /*!
     * @brief Header for each field entry in a page.
     *
     * The actual size of the header is rounded up to the minimum flash write size,
     * the #kWriteAlignment constant.
     */
    struct FieldHeader
    {
        uint32_t key;
        uint16_t dataLength;
        uint16_t fieldLength;
        uint16_t fieldVersion;
        uint16_t crc;

        static constexpr uint32_t get_length() { return align_up<kWriteAlignment>(sizeof(FieldHeader)); }

        uint8_t * get_data()
        {
            return reinterpret_cast<uint8_t *>(reinterpret_cast<uint32_t>(this) + get_length());
        }

        FieldHeader * get_next_field()
        {
            return reinterpret_cast<FieldHeader *>(reinterpret_cast<uint32_t>(this) + fieldLength);
        }
    };

    struct KeyInfo
    {
        uint32_t key;
        FieldHeader * newest;
        KeyInfo * next;

        KeyInfo(uint32_t theKey) : key(theKey), newest(nullptr), next(nullptr) {}
    };

    PersistentDataStore();
    ~PersistentDataStore()=default;

    void init();
    void add_key(KeyInfo * info);

    bool read_key(KeyInfo * info, uint8_t * data, uint32_t length);
    bool write_key(KeyInfo * info, const uint8_t * data, uint32_t length);

    void reset();

protected:

    static const uint32_t kMagicNumber = 'pdat';
    static const uint32_t kFormatVersion = 1;

    static const uint32_t kMaxPageDataOffset;

    struct PageHeader
    {
        uint32_t magic;
        uint32_t formatVersion;
        uint32_t dataVersion;

        static constexpr uint32_t get_length() { return align_up<kWriteAlignment>(sizeof(PageHeader)); }
    };

    enum PageState : uint32_t
    {
        kErased,
        kValid,
        kActive,
        kMerging,
        kCorrupted,
    };

    struct PageInfo
    {
        uint32_t address;
        PageState state;
        uint32_t version;
    };

    flash_config_t _flash;
    PageInfo _pages[DATA_STORE_PAGE_COUNT];
    uint32_t _activePage;
    uint32_t _nextWriteOffset;
    uint32_t _nextPageVersion;
    KeyInfo * _firstKey;
    uint8_t _writeBuffer[kWriteAlignment] __attribute__ ((aligned(4)));

    void _load_pages();
    uint32_t _scan_page(uint32_t page);
    FieldHeader * _find_newest_field(uint32_t key, uint32_t page);
    bool _init_page(uint32_t page);
    bool _merge_fields();
    FieldHeader * _write_field(const FieldHeader * header, const uint8_t * data);
    uint32_t _find_next_page();

    bool _is_page_erased(uint32_t address);
    bool _erase_page(uint32_t address);
    bool _write_data(uint32_t address, const void * data, uint32_t length);
    bool _write_and_verify(uint32_t address, const void * data, uint32_t length);

};

/*!
 * @brief A value saved in the persistent data store.
 */
template <uint32_t Key, typename Data>
class PersistentData : public PersistentDataStore::KeyInfo
{
public:
    PersistentData() : PersistentDataStore::KeyInfo(Key), _cached(nullptr), _isCacheValid(false) {}
    ~PersistentData()=default;

    void init()
    {
        PersistentDataStore::get().add_key(this);
    }

    bool read(Data * data)
    {
        if (_isCacheValid)
        {
            memcpy(data, &_cached, sizeof(Data));
            return true;
        }
        else
        {
            bool result = PersistentDataStore::get().read_key(this, reinterpret_cast<uint8_t *>(data), sizeof(Data));
            if (result)
            {
                _cached = newest->get_data();
                _isCacheValid = true;
            }
            return result;
        }
    }

    bool write(const Data * data)
    {
        bool ok = PersistentDataStore::get().write_key(this, reinterpret_cast<const uint8_t *>(data), sizeof(Data));
        if (ok)
        {
            _cached = reinterpret_cast<Data *>(newest->get_data());
            _isCacheValid = true;
        }
        else
        {
            _isCacheValid = false;
        }
        return ok;
    }

    Data read()
    {
        uint32_t data;
        read(&data);
        return data;
    }

    bool write(Data data)
    {
        return write(&data);
    }

    operator Data () const
    {
        return read();
    }

protected:
    Data * _cached;
    bool _isCacheValid;

};

} // namespace slab

#endif // _PERSISTENT_DATA_STORE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
