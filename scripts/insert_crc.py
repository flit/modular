#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import print_function

import argparse
import subprocess
import binascii
import struct

SIG_OFFSET = 0x1c
CRC_OFFSET = 0x20
LEN_OFFSET = 0x24

APP_SIGNATURE = 0x72e46273 # sbär

def insert_le32(data, offset, value):
    data[offset + 0] = (value >> 0) & 0xFF
    data[offset + 1] = (value >> 8) & 0xFF
    data[offset + 2] = (value >> 16) & 0xFF
    data[offset + 3] = (value >> 24) & 0xFF

def main():
    parser = argparse.ArgumentParser(description='CRC generator')
    parser.add_argument("input", type=str, help="ELF input file.")
    parser.add_argument("output", type=str, help="Output file name.")
    args = parser.parse_args()

    binFile = args.input.split('.')[0] + '.bin'
    cmd = ["arm-none-eabi-objcopy", "-O", "binary", args.input, binFile]
    subprocess.call(cmd)

    with open(binFile, "rb") as inputFile:
        data = bytearray(inputFile.read())

    crc = binascii.crc32(data) & 0xffffffff
    insert_le32(data, SIG_OFFSET, APP_SIGNATURE)
    insert_le32(data, CRC_OFFSET, crc)
    insert_le32(data, LEN_OFFSET, len(data))

    with open(args.output, "wb") as outputFile:
        outputFile.write(data)

    print("Wrote %d bytes to %s (CRC32 = 0x%08x)" % (len(data), args.output, crc))

if __name__ == "__main__":
    main()

