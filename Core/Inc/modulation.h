#pragma once
#include <stdint.h>

//#define SAMPLE_PERIOD_NS 62500 // 16khz
#define SAMPLE_PERIOD_NS 31250 // 32khz

#define CARRIERS 12

static const int CARRIER_FREQUENCIES_HZ[CARRIERS] = {
   8000,
   8500,
   9000,
   9500,
  10000,
  10500,
  11000,
  11500,
  12000,
  12500,
  13000,
  13500,
};

static const int CARRIER_PERIODS_NS[CARRIERS] = {
  //1000000, // 1 kHz
  //666666, // 1.5 kHz
  //500000, // 2.0 kHz
  //400000, // 2.5 kHz
  //333333, // 3 kHz
  //285714, // 3.5 kHz (GOOD)

  /*250000, // 4 kHz
  222222, // 4.5 kHz
  200000, // 5 kHz
  181818, // 5.5 kHz (AWFUL??)
  166666, // 6 kHz
  153846, // 6.5 kHz
  142857, // 7 kHz
  133333, // 7.5 kHz*/
  125000, // 8 kHz
  117647, // 8.5 kHz
  111111, // 9 kHz
  105263, // 9.5 kHz
  100000, // 10 kHz
  95238, // 10.5 kHz
  90909, // 11 kHz
  86957, // 11.5 kHz
  83333, // 12 kHz
  80000, // 12.5 kHz
  76923, // 13 kHz
  74074, // 13.5 kHz
};

//#define CARRIER_PERIOD_NS 250000 // 4khz
//#define CARRIER_PERIOD_NS 166666 // 6khz
//#define CARRIER_FREQUENCY (1.0e9 / CARRIER_PERIOD_NS)

#define SYMBOL_PERIOD_US 5000
#define SYMBOL_GAP_US (8 * SYMBOL_PERIOD_US)
#define TRAIN_PERIOD_US (2 * SYMBOL_PERIOD_US)

#define SAMPLES_PER_SYMBOL (SYMBOL_PERIOD_US * 1000 / SAMPLE_PERIOD_NS)

#define DAC_PERIOD_US 10

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
