
#include <stdint.h>

namespace slab {

//! @brief CIE1931 map
extern const uint8_t kCie1931[100] = {
     0,  0,  0,  0,  0,  1,  1,  1,  1,  1,
     1,  1,  1,  2,  2,  2,  2,  2,  3,  3,
     3,  3,  4,  4,  4,  4,  5,  5,  6,  6,
     6,  7,  7,  8,  8,  9,  9, 10, 10, 11,
    11, 12, 13, 13, 14, 15, 15, 16, 17, 18,
    19, 20, 20, 21, 22, 23, 24, 25, 26, 27,
    29, 30, 31, 32, 33, 35, 36, 37, 39, 40,
    41, 43, 44, 46, 47, 49, 51, 52, 54, 56,
    58, 59, 61, 63, 65, 67, 69, 71, 73, 75,
    78, 80, 82, 84, 87, 89, 91, 94, 96, 99,
};

}
