#pragma once
#include <stdint.h>

//#define SAMPLE_PERIOD_NS 62500 // 16khz
#define SAMPLE_PERIOD_NS 31250 // 32khz

#define CARRIERS 3

static const int CARRIER_PERIODS_NS[CARRIERS] = {
  125000, // 8 kHz
  166666, // 6 kHz
  250000  // 4 kHz
};

//#define CARRIER_PERIOD_NS 250000 // 4khz
//#define CARRIER_PERIOD_NS 166666 // 6khz
//#define CARRIER_FREQUENCY (1.0e9 / CARRIER_PERIOD_NS)

#define SYMBOL_PERIOD_US 5000
#define SYMBOL_GAP_US (8 * SYMBOL_PERIOD_US)
#define TRAIN_PERIOD_US (2 * SYMBOL_PERIOD_US)

#define SAMPLES_PER_SYMBOL (SYMBOL_PERIOD_US * 1000 / SAMPLE_PERIOD_NS)

#define BITS_PER_SYMBOL 4

static const int8_t CONSTELLATION_TO_SYMBOL[9][9] = {
  { -1, -1, -1, 28, -1, 27, -1, -1, -1 },
  { -1, -1, 14, -1, 1, -1, 12, -1, -1 },
  { -1, 16, -1, 23, -1, 17, -1, 22, -1 },
  { 7, -1, 9, -1, 6, -1, 11, -1, 4 },
  { -1, 25, -1, 30, -1, 26, -1, 29, -1 },
  { 0, -1, 15, -1, 2, -1, 13, -1, 3 },
  { -1, 18, -1, 21, -1, 19, -1, 20, -1 },
  { -1, -1, 8, -1, 5, -1, 10, -1, -1 },
  { -1, -1, -1, 31, -1, 24, -1, -1, -1 },
};

static const int8_t SYMBOL_TO_CONSTELLATION_I[] = { -4,  0, 0, 4,  4, 0,  0, -4, -2, -2, 2,  2,  2, 2, -2, -2, -3,  1, -3, 1, 3, -1,  3, -1, 1, -3, 1,  1, -1, 3, -1, -1 };
static const int8_t SYMBOL_TO_CONSTELLATION_Q[] = {  1, -3, 1, 1, -1, 3, -1, -1,  3, -1, 3, -1, -3, 1, -3,  1, -2, -2,  2, 2, 2,  2, -2, -2, 4,  0, 0, -4, -4, 0,  0,  4 };
