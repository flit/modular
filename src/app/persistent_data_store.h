/*
 * Copyright (c) 2017-2018 Immo Software
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
#include "crc32.h"
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
     * The complete field layout is composed of three regions:
     *
     *  |-----------------------|
     *  |                       |
     *  |        header         |
     *  |                       |
     *  |-----------------------|  <-- get_header_length()
     *  |                       |
     *  |                       |
     *  |        data           |
     *  |        ...            |
     *  |                       |
     *  |-----------------------|  <-- compute_crc_offset()
     *  |        crc32          |
     *  |-----------------------|  <-- compute_field_length()
     *
     * The actual size of each field region is rounded up to the flash minimum write size,
     * the #kWriteAlignment constant. There are static members to compute the sizes of each
     * region.
     */
    struct FieldHeader
    {
        uint32_t key;           //!< Unique identifier for this value.
        uint16_t dataLength;    //!< Actual data length, not rounded up.
        uint16_t fieldLength;   //!< Total size of the field starting from _key_ and including CRC.
        uint16_t fieldVersion;  //!< Version number for this instance of the field within the page.
        uint16_t _padding;      //!< (padding to make the struct word aligned)

        //! @brief Constructor to zero all fields.
        FieldHeader()
        :   key(0),
            dataLength(0),
            fieldLength(0),
            fieldVersion(0),
            _padding(0)
        {
        }

        //! @name Size and offset
        //@{
        //! @brief Size of the header rounded up to minimum write size.
        static constexpr uint32_t get_header_length() { return align_up<kWriteAlignment>(sizeof(FieldHeader)); }

        //! @brief Size of data on media.
        static constexpr uint32_t get_data_length(uint32_t length) { return align_up<kWriteAlignment>(length); }

        //! @brief Size of the CRC32 on media.
        static constexpr uint32_t get_crc_length() { return align_up<kWriteAlignment>(sizeof(uint32_t)); }

        //! @brief Offset from the start of the field to the CRC32.
        static uint32_t compute_crc_offset(uint32_t dataLen)
        {
            return get_header_length() + get_data_length(dataLen);
        }

        //! @brief Computes the total length of the field on media.
        static uint32_t compute_field_length(uint32_t dataLen)
        {
            return compute_crc_offset(dataLen) + get_crc_length();
        }
        //@}

        //! @brief Get a pointer to data from the start of the field.
        uint8_t * get_data()
        {
            return reinterpret_cast<uint8_t *>(reinterpret_cast<uint32_t>(this) + get_header_length());
        }

        //! @brief Get a pointer to the CRC from the start of the field.
        uint32_t * get_crc()
        {
            return reinterpret_cast<uint32_t *>(reinterpret_cast<uint32_t>(this) + compute_crc_offset(dataLength));
        }

        //! @brief Calculate the CRC from provided data.
        //!
        //! The field header (this struct) must have already been filled in.
        uint32_t compute_crc(const uint8_t * data) const
        {
            return Crc32()
                    .compute(this, sizeof(FieldHeader))
                    .compute(data, dataLength)
                    .get();
        }
    };

    /*!
     * @brief Linked list node to keep track of all data store keys.
     */
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
    /*!
     * @brief Header of each valid data store page.
     *
     * The actual size of the header is rounded up to the minimum flash write size.
     */
    struct PageHeader
    {
        static const uint32_t kMagicNumber = 'pdat';
        static const uint32_t kFormatVersion = 2;

        uint32_t magic;
        uint32_t formatVersion;
        uint32_t dataVersion;

        static constexpr uint32_t get_length() { return align_up<kWriteAlignment>(sizeof(PageHeader)); }
    };

    /*!
     * @brief Represents one page of the data store.
     */
    class Page
    {
    public:
        static const uint32_t kMaxDataOffset;

        //! @brief Possible states of a page.
        enum PageState : uint32_t
        {
            kErased,
            kValid,
            kActive,
            kMerging,
            kCorrupted,
        };

        Page();
        ~Page()=default;

        void init(uint32_t address);

        PageState get_state() const { return _state; }
        void set_state(PageState newState) { _state = newState; }
        uint32_t get_version() const { return _header->dataVersion; }

        FieldHeader * find_newest_field(uint32_t key);

        bool is_erased();
        bool erase();

        bool start_merge();
        bool finish_merge();

        bool format_page();
        bool can_write_field(uint32_t length);
        FieldHeader * write_field(const FieldHeader * header, const uint8_t * data);

    protected:
        uint32_t _address;
        PageHeader * _header;
        PageState _state;
        uint32_t _nextWriteOffset;

        void _scan();
        bool _ensure_erased();
        bool _write_header();
    };

    flash_config_t _flash;
    Page _pages[DATA_STORE_PAGE_COUNT];
    Page * _activePage;
    uint32_t _nextPageVersion;
    KeyInfo * _firstKey;
    uint8_t _writeBuffer[kWriteAlignment] __attribute__ ((aligned(4)));

    void _load_pages();
    bool _merge_fields();
    uint32_t _find_next_page();

    uint32_t _get_next_page_version() { return _nextPageVersion++; }

    bool _write_data(uint32_t address, const void * data, uint32_t length);
    bool _write_and_verify(uint32_t address, const void * data, uint32_t length);

    friend class Page;
};

/*!
 * @brief A value saved in the persistent data store.
 */
template <uint32_t Key, typename Data>
class PersistentData : public PersistentDataStore::KeyInfo
{
public:
    PersistentData() : PersistentDataStore::KeyInfo(Key), _cached(nullptr) {}
    PersistentData(const PersistentData<Key, Data> & other)=delete;
    PersistentData & operator = (const PersistentData<Key, Data> & other)=delete;
    ~PersistentData()=default;

    void init()
    {
        PersistentDataStore::get().add_key(this);
    }

    bool is_present()
    {
        return newest || PersistentDataStore::get().read_key(this, nullptr, 0);
    }

    bool read(Data * data)
    {
        if (_cached)
        {
            memcpy(data, &_cached, sizeof(Data));
            return true;
        }
        else
        {
            bool result = PersistentDataStore::get().read_key(this, reinterpret_cast<uint8_t *>(data), sizeof(Data));
            if (result)
            {
                _cached = reinterpret_cast<Data *>(newest->get_data());
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
        }
        else
        {
            _cached = nullptr;
        }
        return ok;
    }

    const Data & read()
    {
        if (!_cached)
        {
            bool result = PersistentDataStore::get().read_key(this, nullptr, 0);
            if (result)
            {
                _cached = reinterpret_cast<Data *>(newest->get_data());
            }
        }
        return *_cached;
    }

    bool write(const Data & data)
    {
        return write(&data);
    }

    operator Data ()
    {
        return read();
    }

protected:
    const Data * _cached;

};

} // namespace slab

#endif // _PERSISTENT_DATA_STORE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
