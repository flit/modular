/*
 * Copyright (c) 2016 Immo Software
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
#if !defined(_WAV_FILE_H_)
#define _WAV_FILE_H_

#include "file_system.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

//! @brief Macro to build a four-char code in little-endian format.
#define FOUR_CHAR_CODE(a,b,c,d) ((a) | ((b) << 8) | ((c) << 16) | ((d) << 24))

namespace slab {

/*!
 * @brief Decodes a .wav file.
 */
class WaveFile : public File
{
public:

    //! @brief Stream of a wav file's audio data.
    //!
    //! This class is really just a simple wrapper to make calculations of the audio
    //! data size and file offset easier.
    class AudioDataStream : public Stream
    {
    public:
        AudioDataStream() : _file(nullptr) {}
        AudioDataStream(WaveFile * wave) : Stream(), _file(wave) {}
        AudioDataStream(const AudioDataStream& other)=default;
        AudioDataStream& operator = (const AudioDataStream& other)=default;

        virtual uint32_t read(uint32_t count, void * data) override;
        virtual uint32_t write(uint32_t count, const void * data) override { return 0; }
        virtual bool seek(uint32_t offset) override;

        virtual uint32_t get_size() const override;
        virtual uint32_t get_offset() const override;

        uint32_t get_frames() const;

    protected:
        WaveFile * _file;
    };

    WaveFile();
    WaveFile(const char * path);
    virtual ~WaveFile() {}

    WaveFile& operator = (const WaveFile& other);

    bool is_valid() const { return _dataSize > 0; }

    //! @brief Parse file to read format and find data.
    bool parse();

    AudioDataStream get_audio_data() { return AudioDataStream(this); }

    uint32_t get_sample_rate() const { return _format.dwSamplesPerSec; }
    uint32_t get_sample_size() const { return _format.wBitsPerSample; }
    uint32_t get_channels() const { return _format.wChannels; }
    uint32_t get_frame_size() const { return _format.wBlockAlign; }

protected:
    friend class AudioDataStream;

    //! @brief Four-char codes used in WAV files.
    enum ChunkIDs : uint32_t
    {
        kRIFFGroupID = FOUR_CHAR_CODE('R', 'I', 'F', 'F'),
        kWaveFormatID = FOUR_CHAR_CODE('W', 'A', 'V', 'E'),
        kFormatChunkID = FOUR_CHAR_CODE('f', 'm', 't', ' '),
        kDataChunkID = FOUR_CHAR_CODE('d', 'a', 't', 'a'),
    };

    //! @brief Header for every chunk.
    struct ChunkHeader {
        uint32_t chunkID;
        int32_t chunkSize;
    };

    //! @brief Wave file header.
    struct RIFFFileHeader {
        uint32_t groupID;
        uint32_t fileSize;
        uint32_t formatID;
    };

    //! @brief Values for wFormatTag in the format chunk.
    enum : int16_t
    {
        kUncompressedWaveFormat = 1,
    };

    //! @brief Structure for the 'fmt ' chunk.
    //!
    //! The actual structure in the file is prefixed with the ChunkHeader. The header is missing
    //! from this struct definition because it is read separately while searching for the format
    //! chunk.
    //!
    //! Additional fields may follow those defined here, depending upon wFormatTag. The actual size
    //! of the chunk is specified in the chunk header.
    struct FormatChunk {
        // ChunkHeader header;
        int16_t wFormatTag;
        uint16_t wChannels;
        uint32_t dwSamplesPerSec;
        uint32_t dwAvgBytesPerSec;
        uint16_t wBlockAlign;
        uint16_t wBitsPerSample;
    };

    FormatChunk _format;    //!< The file's audio format header.
    uint32_t _dataOffset;   //!< Byte offset of the start of the audio data.
    uint32_t _dataSize;     //!< Byte count of the audio data chunk.

    //! @brief Search for a chunk from the current position.
    bool find_chunk(uint32_t chunkID, ChunkHeader * header);
};

} // namespace slab

#endif // _WAV_FILE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
