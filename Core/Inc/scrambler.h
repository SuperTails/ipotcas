// Additive scrambler from DVB

#pragma once

#include <stdint.h>

#define SCRAMBLE 1

typedef uint32_t scrambler_t;

inline static uint32_t scrambler_reset(void) {
    return 0x00A9;
}

inline static uint32_t scrambler_step(scrambler_t *s) {
#if SCRAMBLE
    uint32_t new_bit = ((*s >> 14) ^ (*s >> 13)) & 1;
    *s = (*s << 1) | new_bit;
    return new_bit;
#else
    return 0;
#endif
}

inline static uint8_t scrambler_step_byte(scrambler_t *s) {
    uint8_t result = 0;
    for (int i = 0; i < 8; ++i) {
        result |= scrambler_step(s) << i;
    }
    return result;
}
