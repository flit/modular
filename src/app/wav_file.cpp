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

bool WaveFile::parse()
{
    uint32_t bytesRead;

    // Make sure we're open.
    if (!open())
    {
        return false;
    }

    // Move to start of the file.
    seek(0);

    // Read and validate file header.
    RIFFFileHeader fileHeader;
    bytesRead = read(sizeof(fileHeader), &fileHeader);
    if (bytesRead != sizeof(fileHeader))
    {
        return false;
    }

    if (fileHeader.groupID != kRIFFGroupID || fileHeader.formatID != kWaveFormatID)
    {
        return false;
    }

    ChunkHeader chunkHeader;

    // Scan for format chunk.
    if (!find_chunk(kFormatChunkID, &chunkHeader))
    {
        return false;
    }

    // Save offset of the start of the format chunk data.
    uint32_t currentOffset = get_offset();

    // Read format chunk.
    bytesRead = read(sizeof(_format), &_format);
    if (bytesRead != sizeof(_format))
    {
        return false;
    }

    // Make sure the file is uncompressed.
    if (_format.wFormatTag != kUncompressedWaveFormat)
    {
        return false;
    }

    // Skip over any extra data in the format chunk.
    seek(currentOffset + chunkHeader.chunkSize);

    // Scan for data chunk.
    if (!find_chunk(kDataChunkID, &chunkHeader))
    {
        return false;
    }

    // We're now positioned to read audio data.
    _dataOffset = get_offset();
    _dataSize = chunkHeader.chunkSize;
    return true;
}

bool WaveFile::find_chunk(uint32_t chunkID, ChunkHeader * header)
{
    uint32_t bytesRead;

    // Continue scanning unless we hit the end of the file.
    while (get_offset() <= get_size())
    {
        // Read the header at the current offset.
        bytesRead = read(sizeof(ChunkHeader), header);
        if (bytesRead != sizeof(ChunkHeader))
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
        seek(newOffset);

        // Make sure we didn't hit the end of the file or had some other error.
        if (get_offset() != newOffset)
        {
            return false;
        }
    }

    return false;
}

uint32_t WaveFile::AudioDataStream::read(uint32_t count, void * data)
{
    assert(_file);
    return _file->read(count, data);
}

bool WaveFile::AudioDataStream::seek(uint32_t offset)
{
    assert(_file);
    _file->seek(_file->_dataOffset + offset);
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
