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

#include "wav_file.h"
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

WaveFile::WaveFile()
:   File(),
    _dataOffset(0),
    _dataSize(0)
{
}

WaveFile::WaveFile(const char * path)
:   File(path),
    _dataOffset(0),
    _dataSize(0)
{
}

WaveFile& WaveFile::operator = (const WaveFile& other)
{
    File::operator = (other);
    _format = other._format;
    _dataOffset = other._dataOffset;
    _dataSize = other._dataSize;
    return *this;
}

fs::error_t WaveFile::parse()
{
    error_t status;
    uint32_t bytesRead;

    // Make sure we're open.
    status = open();
    if (status != fs::kSuccess)
    {
        return status;
    }

    // Move to start of the file.
    if (!seek(0))
    {
        return fs::kGenericError;
    }

    // Read and validate file header.
    RIFFFileHeader fileHeader;
    status = read(sizeof(fileHeader), &fileHeader, &bytesRead);
    if (status != fs::kSuccess)
    {
        return status;
    }
    if (bytesRead != sizeof(fileHeader))
    {
        return kParseError;
    }

    if (fileHeader.groupID != kRIFFGroupID || fileHeader.formatID != kWaveFormatID)
    {
        return kParseError;
    }

    ChunkHeader chunkHeader;

    // Scan for format chunk.
    if (!find_chunk(kFormatChunkID, &chunkHeader))
    {
        return kParseError;
    }

    // Save offset of the start of the format chunk data.
    uint32_t currentOffset = get_offset();

    // Read format chunk.
    status = read(sizeof(_format), &_format, &bytesRead);
    if (status != fs::kSuccess)
    {
        return status;
    }
    if (bytesRead != sizeof(_format))
    {
        return kParseError;
    }

    // Make sure the file is uncompressed.
    if (_format.wFormatTag != kUncompressedWaveFormat)
    {
        return kParseError;
    }

    // Skip over any extra data in the format chunk.
    seek(currentOffset + chunkHeader.chunkSize);

    // Scan for data chunk.
    if (!find_chunk(kDataChunkID, &chunkHeader))
    {
        return kParseError;
    }

    // We're now positioned to read audio data.
    _dataOffset = get_offset();
    _dataSize = chunkHeader.chunkSize;
    return fs::kSuccess;
}

bool WaveFile::find_chunk(uint32_t chunkID, ChunkHeader * header)
{
    uint32_t bytesRead;

    // Continue scanning unless we hit the end of the file.
    while (get_offset() <= get_size())
    {
        // Read the header at the current offset.
        fs::error_t status = read(sizeof(ChunkHeader), header, &bytesRead);
        if (status != fs::kSuccess || bytesRead != sizeof(ChunkHeader))
        {
            return false;
        }

        // Is this the requested chunk?
        if (header->chunkID == chunkID)
        {
            return true;
        }

        // Skip over this chunk's data.
        uint32_t newOffset = get_offset() + header->chunkSize;
        if (!seek(newOffset))
        {
            return false;
        }

        // Make sure we didn't hit the end of the file or had some other error.
        if (get_offset() != newOffset)
        {
            return false;
        }
    }

    return false;
}

fs::error_t WaveFile::AudioDataStream::read(uint32_t count, void * data, uint32_t * actualCount)
{
    assert(_file);
    return _file->read(count, data, actualCount);
}

fs::error_t WaveFile::AudioDataStream::seek(uint32_t offset)
{
    assert(_file);
    return _file->seek(_file->_dataOffset + offset);
}

uint32_t WaveFile::AudioDataStream::get_size() const
{
    assert(_file);
    return _file->_dataSize;
}

uint32_t WaveFile::AudioDataStream::get_offset() const
{
    assert(_file);
    return _file->get_offset() - _file->_dataOffset;
}

uint32_t WaveFile::AudioDataStream::get_frames() const
{
    assert(_file);
    return _file->_dataSize / _file->_format.wBlockAlign;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
