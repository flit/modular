#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy

INPUT_RANGE = 99       # Input integer size
OUTPUT_RANGE = 99      # Output integer size
TAB_WIDTH = 4
OUT_RANGE_WIDTH = len(str(OUTPUT_RANGE))

HEADER = """
#include <stdint.h>

namespace slab {{

//! @brief CIE1931 map
extern const uint8_t kCie1931[{}] = {{"""

FOOTER = """
};

}

"""

def cie1931(L):
    L *= 100.0
    return (L / 902.3) if (L <= 8) else (((L + 16.0) / 116.0) ** 3)

table = [int(round(cie1931(float(L) / INPUT_RANGE) * OUTPUT_RANGE)) for L in range(0, INPUT_RANGE + 1)]

with open('cie1931.cpp', 'w') as f:
    f.write(HEADER.format(INPUT_RANGE + 1))

    for i, L in enumerate(table):
        if i % 10 == 0:
            f.write('\n' + ' ' * (TAB_WIDTH - 1))
        f.write(' {:{w}d},'.format(L, w=OUT_RANGE_WIDTH))

    f.write(FOOTER)



