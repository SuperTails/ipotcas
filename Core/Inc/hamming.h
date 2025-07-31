#pragma once

#include <stdint.h>

static inline uint16_t hamming_decode_15_11(uint16_t chunk, int *error_cnt) {
    int p0 = __builtin_popcount(chunk & 0x5555) & 1;
    int p1 = __builtin_popcount(chunk & 0x6666) & 1;
    int p2 = __builtin_popcount(chunk & 0x7878) & 1;
    int p3 = __builtin_popcount(chunk & 0x7F80) & 1;

    int p = ((p3 << 3) | (p2 << 2) | (p1 << 1) | (p0 << 0));
    if (p != 0) {
        chunk ^= 1 << (p - 1);
        if (error_cnt) ++(*error_cnt);
    }
    
    return (
        ((chunk >> 4) & 0x7F0) |
        ((chunk >> 3) & 0x00E) |
        ((chunk >> 2) & 0x001)
    );
}

static inline uint16_t hamming_encode_15_11(uint16_t val) {
    uint16_t chunk = (
        ((val & 0x001) << 2) |
        ((val & 0x00E) << 3) |
        ((val & 0x7F0) << 4)
    );

    int p0 = __builtin_popcount(chunk & 0x5555) & 1;
    int p1 = __builtin_popcount(chunk & 0x6666) & 1;
    int p2 = __builtin_popcount(chunk & 0x7878) & 1;
    int p3 = __builtin_popcount(chunk & 0x7F80) & 1;
    chunk ^= p0 << 0;
    chunk ^= p1 << 1;
    chunk ^= p2 << 3;
    chunk ^= p3 << 7;

    return chunk;
}

static inline uint16_t hamming_decode_7_4(uint16_t chunk, int *error_cnt) {
    int p0 = __builtin_popcount(chunk & 0x5555) & 1;
    int p1 = __builtin_popcount(chunk & 0x6666) & 1;
    int p2 = __builtin_popcount(chunk & 0x7878) & 1;

    int p = ((p2 << 2) | (p1 << 1) | (p0 << 0));
    if (p != 0) {
        chunk ^= 1 << (p - 1);
        if (error_cnt) ++(*error_cnt);
    }
    
    return (
        ((chunk >> 3) & 0x00E) |
        ((chunk >> 2) & 0x001)
    );
}

static inline uint16_t hamming_encode_7_4(uint16_t val) {
    uint16_t chunk = (
        ((val & 0x001) << 2) |
        ((val & 0x00E) << 3)
    );

    int p0 = __builtin_popcount(chunk & 0x5555) & 1;
    int p1 = __builtin_popcount(chunk & 0x6666) & 1;
    int p2 = __builtin_popcount(chunk & 0x7878) & 1;
    chunk ^= p0 << 0;
    chunk ^= p1 << 1;
    chunk ^= p2 << 3;

    return chunk;
}
