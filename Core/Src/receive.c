#include "receive.h"
#include "fast_math.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_cortex.h"
#include "stm32f7xx_hal_sai.h"
#include "tusb.h"
#include "hamming.h"
#include "modulation.h"
#include "ethernet.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define WINDOW_SIZE 96

#define SAMPLE_BUF_SZ (WINDOW_SIZE)

#define RAD_PER_SAMPLE(i) (2.0 * M_PI * SAMPLE_PERIOD_NS / CARRIER_PERIODS_NS[i])

#define TRELLIS_HISTORY 16
#define CC_STATES 8


int diff_dec(int y_prev, int y_new) {
  const int8_t TABLE[4][4] = {
    {0, 1, 2, 3},
    {1, 0, 3, 2},
    {3, 2, 0, 1},
    {2, 3, 1, 0}
  };

  return TABLE[y_prev][y_new];
}

int y12_for_transition(int prev_state, int cur_state) {
  const int8_t TABLE[8][8] = {
    { 0, 3, 2, 1, -1, -1, -1, -1  },
    { -1, -1, -1, -1, 0, 1, 3, 2 },
    { 3, 0, 1, 2, -1, -1, -1, -1 },
    { -1, -1, -1, -1, 2, 3, 1, 0 },
    { 2, 1, 0, 3, -1, -1, -1, -1 },
    { -1, -1, -1, -1, 3, 2, 0, 1 },
    { 1, 2, 3, 0, -1, -1, -1, -1 },
    { -1, -1, -1, -1, 1, 0, 2, 3 }
  };

  return TABLE[prev_state][cur_state];
}

typedef struct {
    float path_metric[TRELLIS_HISTORY][CC_STATES];
    int prev_states[TRELLIS_HISTORY][CC_STATES];
    int q34[TRELLIS_HISTORY][CC_STATES];
  
    int y12;
  
    int head;
    int tail;
} trellis_t;

//const char *expected_data = "Somebody once told me the world is gonna roll me / I ain't the sharpest tool in the shed / She was looking kind of dumb with her finger and her thumb / In the shape of an \"L\" on her forehead / Well, the years start comin' and they don't stop comin' / Fed to the rules and I hit the ground runnin' / Didn't make sense not to live for fun / Your brain gets smart but your head gets dumb / So much to do, so much to see / So what's wrong with taking the backstreets? / You'll never know if you don't go / You'll never shine if you don't glow / Hey now, you're an all star / Get your game on, go play / Hey now, you're a rock star / Get your show on, get paid / (And all that glitters is gold) / Only shootin' stars break the mold / It's a cool place, and they say it gets colder / You're bundled up now, wait 'til you get older / But the meteor men beg to differ / Judging by the hole in the satellite picture / The ice we skate is gettin' pretty thin / The waters gettin' warm so you might as well swim / My world's on fire, how 'bout yours? / That's the way I like it and I'll never get bored / Hey now, you're an all star / Get your game on, go play / Hey now, you're a rock star / Get your show on, get paid / (All that glitters is gold) / Only shootin' stars break the mold / Go for the moon / G-g-g-go for the moon / Go for the moon / Go-go-go for the moon / Hey now, you're an all star / Get your game on, go play / Hey now, you're a rock star / Get your show on, get paid / (All that glitters is gold).";
const char expected_data[] = { 0x1E,0x7C,0x0B,0xE0,0x69,0xFD,0x67,0xE6,0x59,0x6B,0x96,0x99,0xFD,0x67,0xAA,0x19,0x93,0x06,0xC8,0xFC,0x67,0xF8,0x99,0x67,0xD6,0x9A,0x01,0x32,0x2A,0xDA,0x7F,0x16,0x9E,0xA9,0x66,0x80,0x8C,0x79,0xD6,0x9A,0x01,0x32,0x2A,0xDA,0x72,0xD6,0x9A,0x01,0x32,0x34,0xDA,0x7F,0x96,0xA1,0x85,0x67,0xAA,0x19,0x20,0xC3,0x9C,0x79,0x68,0x80,0x0C,0x6D,0xF6,0x9F,0xE1,0x67,0xF8,0xD9,0x61,0x06,0xC8,0x64,0x68,0xFF,0x59,0x78,0x16,0x9E,0x01,0x32,0xE6,0x59,0x6B,0x06,0xC8,0xFC,0x33,0x80,0x0C,0x53,0x05,0xC8,0x1C,0x66,0xCC,0x19,0x7E,0x46,0xCB,0xA8,0x68,0x80,0x8C,0x8A,0xB6,0x9C,0xB5,0x66,0x80,0x8C,0x87,0xB6,0x9C,0x1D,0x66,0x19,0x1A,0x80,0xD6,0x9A,0x79,0x68,0x2A,0x1A,0x20,0xA3,0xA2,0xFD,0x67,0xFF,0x59,0x78,0x06,0xC8,0x30,0x67,0xF8,0x19,0x20,0xA3,0xA2,0x2D,0x67,0xAD,0x19,0x20,0xE3,0xA1,0x2D,0x67,0xAD,0x99,0x6A,0x06,0xC8,0xFC,0x33,0x80,0x8C,0xA7,0xB5,0x9C,0xB5,0x66,0x80,0x0C,0x8D,0x76,0x98,0x79,0x68,0x80,0x4C,0x78,0xF6,0x9F,0xFD,0x67,0xD5,0x19,0x73,0x86,0x9F,0xD1,0x66,0x80,0x4C,0x75,0xC6,0x9C,0xE1,0x67,0xAA,0x19,0x20,0xF3,0x9F,0xCD,0x66,0x80,0x8C,0x6A,0xD6,0xA2,0x99,0x67,0x99,0x19,0x20,0x43,0xA3,0x31,0x67,0x2A,0xDA,0x72,0x06,0xC8,0x2C,0x67,0xAD,0x59,0x86,0x06,0xC8,0xCC,0x66,0xCC,0x19,0x7E,0x46,0x9B,0xB5,0x66,0x19,0x1A,0x20,0x73,0x98,0xE1,0x67,0xAA,0x19,0x20,0xB3,0x9C,0xB5,0x66,0x19,0x1A,0x20,0xA3,0xA2,0x2D,0x67,0x2D,0x9A,0x79,0x96,0x99,0x01,0x32,0xFF,0x0C,0x20,0xC3,0x54,0xE1,0x67,0x80,0x8C,0x8A,0xB6,0x9C,0xB5,0x66,0x80,0x8C,0x87,0xB6,0x9C,0x1D,0x66,0x00,0x5A,0x6B,0x06,0xC8,0xFC,0x67,0xB3,0x19,0x20,0x73,0x98,0xE1,0x67,0x80,0x4C,0x26,0x13,0x56,0x65,0x32,0x80,0xCC,0x7F,0x86,0x9F,0x01,0x32,0xCB,0x59,0x6B,0x96,0xA1,0x01,0x32,0xB3,0xD9,0x7F,0x96,0xA1,0xB5,0x66,0xCB,0x59,0x6B,0x76,0x98,0xA9,0x66,0x80,0xCC,0x3F,0x03,0xC8,0xD0,0x5A,0xAD,0x59,0x78,0x16,0x9E,0x85,0x33,0x80,0x8C,0x8A,0xB6,0x9C,0xB5,0x66,0x80,0x0C,0x93,0xD6,0x9A,0x1D,0x66,0x19,0x9A,0x87,0x06,0xC8,0x78,0x68,0x2A,0xDA,0x61,0x96,0xA1,0xA9,0x68,0x80,0x8C,0x67,0xF6,0x9F,0x99,0x67,0xCC,0x19,0x7E,0x46,0xCB,0x00,0x32,0x87,0x19,0x7E,0xA6,0x9A,0x01,0x32,0x2A,0xDA,0x72,0xD6,0x9A,0x31,0x69,0x80,0x8C,0x6A,0xF6,0x9F,0xE1,0x67,0xB4,0x8C,0x8A,0x06,0xC8,0x78,0x68,0x2A,0xDA,0x7F,0x06,0xA0,0x01,0x32,0x9E,0xD9,0x7F,0x66,0x9E,0x31,0x67,0xF8,0x19,0x2D,0x03,0xC8,0xFC,0x33,0x80,0xCC,0x4C,0xD5,0x9A,0xA9,0x66,0x80,0x8C,0x8A,0xF6,0x9F,0x01,0x32,0x2A,0xDA,0x72,0xD6,0x9A,0x01,0x32,0x19,0x5A,0x8B,0x16,0x9E,0xB5,0x66,0x1E,0x1A,0x20,0x73,0x98,0xE1,0x67,0xAA,0x19,0x20,0xC3,0x54,0x01,0x32,0xCB,0x19,0x73,0xA6,0xA2,0x01,0x32,0x2A,0xDA,0x72,0xD6,0x9A,0x01,0x32,0xB4,0x59,0x86,0xF6,0x9F,0xB5,0x68,0xF8,0x99,0x6A,0x06,0xC8,0x64,0x68,0x2D,0x1A,0x7E,0x86,0x9F,0x31,0x67,0xF8,0x19,0x2D,0x03,0xC8,0xFC,0x33,0x80,0x8C,0x4A,0xC5,0x9C,0xA9,0x66,0xF8,0x19,0x2D,0xA3,0xA2,0x01,0x32,0xE6,0xD9,0x61,0x56,0x9D,0xB5,0x66,0x80,0x8C,0x87,0xD6,0x9A,0xE1,0x67,0x1E,0x5A,0x6B,0x06,0xC8,0xE0,0x67,0xFF,0x99,0x8A,0x06,0xC8,0xA8,0x68,0xFF,0x19,0x20,0x13,0x9E,0x31,0x67,0x33,0x5A,0x6B,0x06,0xC8,0xCC,0x66,0xFF,0x59,0x86,0x06,0xC8,0xCC,0x66,0x2D,0x1A,0x7E,0x06,0xC8,0xFC,0x33,0x80,0x0C,0xB3,0xF5,0x9F,0xB5,0x68,0x19,0x1A,0x20,0x93,0x99,0x65,0x68,0x87,0x19,0x73,0x86,0x9F,0x01,0x32,0xB4,0x59,0x6B,0xA6,0xA2,0x79,0x68,0x80,0x8C,0x87,0x66,0x9E,0x1D,0x66,0x19,0x9A,0x8A,0x06,0xC8,0x64,0x66,0x2D,0x9A,0x8A,0x06,0xC8,0x30,0x69,0xFF,0x59,0x8B,0x96,0xA1,0x01,0x32,0xCB,0x59,0x6B,0x76,0x98,0xA9,0x66,0x80,0x0C,0x6D,0xD6,0x9A,0xA9,0x68,0x1E,0x1A,0x20,0xA3,0x9A,0xB5,0x68,0xE6,0x59,0x66,0x06,0xC8,0xFC,0x33,0x80,0x8C,0xA7,0xF5,0x9F,0x01,0x32,0xE6,0x59,0x8B,0xE6,0x99,0x2D,0x67,0x80,0x8C,0x8A,0xF6,0x9F,0x01,0x32,0xAA,0xD9,0x7F,0x16,0xCE,0x00,0x32,0x1E,0xDA,0x7F,0x06,0xC8,0x98,0x67,0x2D,0x9A,0x67,0xB6,0x9C,0x01,0x32,0x2A,0xDA,0x7F,0x06,0xC8,0x78,0x68,0xAD,0x59,0x6B,0x06,0xC8,0xFC,0x33,0x80,0x8C,0xA7,0xF5,0x9F,0x01,0x32,0x34,0xDA,0x72,0x76,0x98,0xA9,0x68,0xB4,0x8C,0x87,0x06,0xC8,0xD0,0x68,0x19,0xDA,0x7F,0x86,0x9F,0xD1,0x66,0x80,0x0C,0x8D,0xC6,0x9C,0xA9,0x68,0xCB,0x19,0x20,0xA3,0xA2,0x1D,0x66,0xD5,0x19,0x73,0x86,0x9F,0xD1,0x66,0x80,0x8C,0x8A,0xB6,0x9C,0xB5,0x66,0x80,0x4C,0x66,0x76,0x98,0x79,0x66,0xD5,0x99,0x87,0xA6,0xA2,0x65,0x68,0xAD,0x59,0x6B,0xA6,0xA2,0x79,0x68,0x7F,0x0F,0x20,0xF3,0xCF,0x00,0x32,0xCC,0xD6,0x7F,0xD6,0xA2,0xD1,0x32,0xE1,0x59,0x78,0x06,0xC8,0xE0,0x67,0xAD,0xD9,0x8C,0xD6,0x9A,0x65,0x68,0x80,0x4C,0x75,0x86,0x9F,0xFD,0x67,0x34,0x1A,0x20,0xC3,0x9C,0xCD,0x66,0x80,0x0C,0x93,0xF6,0x9F,0xB5,0x68,0x80,0x8C,0x6A,0xF6,0x9F,0xE1,0x67,0xB4,0x8C,0x8A,0x06,0xC8,0xD0,0x66,0xFF,0x19,0x20,0xF3,0xCF,0x00,0x32,0xCC,0xD6,0x7F,0xD6,0xA2,0xD1,0x32,0xE1,0x59,0x78,0x06,0xC8,0xE0,0x67,0xAD,0xD9,0x8C,0xD6,0x9A,0x65,0x68,0x80,0x8C,0x87,0xB6,0x9C,0x31,0x67,0xF8,0x59,0x6B,0x06,0xC8,0x30,0x67,0xB3,0x19,0x20,0xC3,0xA4,0xFD,0x67,0x2D,0x1A,0x20,0xA3,0x9A,0xFD,0x67,0xF8,0x19,0x2D,0xA3,0xA2,0x01,0x32,0xB4,0x59,0x78,0xF6,0x9F,0xD1,0x68,0x80,0xCC,0x3F,0x03,0xC8,0x2C,0x55,0xAD,0x19,0x93,0x06,0xC8,0xE0,0x67,0xFF,0x19,0x8D,0x16,0xCE,0x00,0x32,0x4C,0xDA,0x7F,0xD6,0xA2,0xD1,0x32,0x19,0x5A,0x6B,0x06,0xC8,0x1C,0x66,0xF8,0x19,0x20,0x73,0x98,0x85,0x67,0xE1,0x19,0x20,0xE3,0xA1,0xA9,0x68,0x87,0x59,0x86,0x06,0xC8,0xFC,0x33,0x80,0x0C,0x4D,0xD5,0x9A,0xA9,0x68,0x80,0x0C,0x93,0xF6,0x9F,0xB5,0x68,0x19,0x1A,0x20,0x43,0x9B,0x1D,0x66,0xE6,0x59,0x6B,0x06,0xC8,0xFC,0x67,0xF8,0x59,0x38,0x03,0xC8,0xD0,0x66,0xFF,0x19,0x20,0x03,0xA0,0x85,0x67,0x87,0x19,0x93,0x06,0xC8,0xFC,0x33,0x80,0xCC,0x52,0xD5,0x9A,0x31,0x69,0x80,0x0C,0x7E,0xF6,0x9F,0xD1,0x68,0xE1,0x0C,0x20,0xC3,0xA4,0xFD,0x67,0x2D,0x1A,0x2D,0x93,0xA1,0xB5,0x66,0x80,0xCC,0x61,0x06,0xC8,0x64,0x68,0xFF,0x99,0x67,0x56,0x9D,0x01,0x32,0x1E,0x9A,0x8A,0x76,0x98,0x65,0x68,0x80,0xCC,0x3F,0x03,0xC8,0xD0,0x54,0xAD,0x99,0x8A,0x06,0xC8,0x30,0x69,0xFF,0x59,0x8B,0x96,0xA1,0x01,0x32,0x1E,0xDA,0x72,0xF6,0x9F,0xD1,0x68,0x80,0xCC,0x7F,0x86,0x9F,0x85,0x33,0x80,0x0C,0x6D,0xD6,0x9A,0xA9,0x68,0x80,0x0C,0x80,0x76,0x98,0x31,0x67,0xAA,0x19,0x20,0xF3,0xCF,0x00,0x32,0xCB,0xCC,0x41,0x85,0x9F,0xA9,0x66,0x80,0xCC,0x61,0x16,0x9E,0x85,0x67,0x80,0x8C,0x8A,0xB6,0x9C,0x1D,0x66,0x2A,0x1A,0x20,0x43,0x9B,0x85,0x67,0xCC,0x99,0x8A,0xA6,0xA2,0xB5,0x66,0x19,0x9A,0x87,0x06,0xC8,0x30,0x67,0x1E,0x1A,0x20,0x43,0x9B,0xFD,0x67,0xE1,0x99,0x6A,0xC6,0xCC,0x00,0x32,0xFF,0x0C,0x20,0xF3,0x57,0xE1,0x67,0xE1,0x19,0x93,0x06,0xC8,0x78,0x68,0xCB,0xD9,0x7F,0xF6,0x9F,0xA9,0x68,0xCC,0x19,0x7E,0x46,0xCB,0x00,0x32,0x1E,0x9A,0x8A,0x76,0x98,0x65,0x68,0x1E,0x1A,0x20,0x93,0x99,0x65,0x68,0xAD,0xD9,0x61,0x56,0x9D,0x01,0x32,0x2A,0xDA,0x72,0xD6,0x9A,0x01,0x32,0xE6,0xD9,0x7F,0x16,0x9E,0xA9,0x66,0x80,0xCC,0x3F,0x03,0xC8,0x30,0x55,0x2A,0x1A,0x2D,0xE3,0xA1,0x01,0x32,0x87,0x19,0x20,0xE3,0x99,0xFD,0x67,0xFF,0x59,0x78,0x06,0xC8,0x00,0x68,0xE1,0xD9,0x61,0xE6,0x99,0xB5,0x66,0xE1,0x0C,0x20,0x73,0x98,0xE1,0x67,0xAA,0x19,0x20,0xA3,0xA2,0x2D,0x67,0xAD,0x19,0x93,0x06,0xC8,0x78,0x68,0x87,0x19,0x93,0x06,0xC8,0x30,0x67,0x2A,0x1A,0x20,0x43,0x9B,0xB5,0x66,0x2A,0x9A,0x87,0x06,0xC8,0x78,0x66,0xFF,0x59,0x78,0xA6,0x9A,0xB5,0x66,0x19,0x1A,0x20,0xF3,0xCF,0x00,0x32,0xCC,0xD6,0x7F,0xD6,0xA2,0xD1,0x32,0x19,0x5A,0x6B,0x06,0xC8,0x64,0x66,0x2D,0x1A,0x7E,0xA6,0x9A,0x85,0x67,0xAD,0x99,0x6A,0x06,0xC8,0xB4,0x68,0x00,0x1A,0x20,0x83,0x9F,0xFD,0x67,0x34,0x5A,0x38,0x03,0xC8,0xD0,0x68,0x87,0x19,0x73,0xA6,0xA2,0x01,0x32,0xB4,0x8C,0x8A,0xC6,0x9C,0x85,0x67,0x80,0x0C,0x93,0xF6,0x9F,0xB5,0x68,0x80,0x0C,0x6D,0xD6,0x9A,0xA9,0x68,0x80,0xCC,0x7F,0x16,0x9E,0xA9,0x66,0xAD,0x59,0x86,0x06,0xC8,0xFC,0x33,0x80,0x4C,0x46,0xD5,0xA2,0xA9,0x68,0x80,0x8C,0x8A,0xB6,0x9C,0xB5,0x66,0x80,0x8C,0x79,0xD6,0x9A,0xA9,0x68,0xAD,0xD9,0x7F,0x96,0xA1,0x01,0x32,0xE6,0x59,0x6B,0x86,0x9F,0x01,0x32,0x99,0x59,0x6B,0x46,0x9B,0x01,0x32,0x2A,0xDA,0x7F,0x06,0xC8,0xA8,0x66,0xCC,0xD9,0x6C,0x36,0x9B,0xB5,0x66,0x19,0x1A,0x20,0xF3,0xCF,0x00,0x32,0x52,0x55,0x8B,0xA6,0x9A,0xD1,0x66,0xCC,0x19,0x7E,0x46,0x9B,0x01,0x32,0x99,0x19,0x93,0x06,0xC8,0xA8,0x68,0xCB,0x59,0x6B,0x06,0xC8,0x2C,0x67,0xFF,0x59,0x78,0xD6,0x9A,0x01,0x32,0xCC,0x19,0x7E,0x06,0xC8,0xA8,0x68,0xCB,0x59,0x6B,0x06,0xC8,0x78,0x68,0x87,0x99,0x8A,0xD6,0x9A,0x85,0x67,0xE1,0x19,0x73,0xA6,0xA2,0xB5,0x66,0x80,0x0C,0x80,0xC6,0x9C,0x79,0x66,0x2A,0x5A,0x8B,0x96,0xA1,0xB5,0x66,0x80,0xCC,0x3F,0x03,0xC8,0xA8,0x5A,0xCB,0x59,0x6B,0x06,0xC8,0x30,0x67,0x9E,0x59,0x6B,0x06,0xC8,0xD0,0x68,0xAD,0x19,0x20,0xE3,0xA1,0x55,0x67,0x87,0x99,0x8A,0xD6,0x9A,0x01,0x32,0xCC,0x99,0x87,0x06,0xC8,0xD0,0x66,0xAD,0x99,0x8A,0xA6,0xA2,0x31,0x67,0xF8,0x19,0x2D,0x03,0xC8,0x00,0x68,0x19,0x5A,0x6B,0xA6,0xA2,0xA9,0x68,0x4C,0x1A,0x20,0xA3,0xA2,0x2D,0x67,0xCC,0x19,0x7E,0x06,0xC8,0xFC,0x33,0x80,0x8C,0xAA,0xB5,0x9C,0xB5,0x66,0x80,0x0C,0x8D,0x76,0x98,0xA9,0x68,0xAD,0x59,0x86,0xE6,0xA1,0x01,0x32,0xB4,0x59,0x6B,0xA6,0xA2,0xA9,0x68,0xCC,0x19,0x7E,0x46,0xCB,0x00,0x32,0x34,0xDA,0x61,0x96,0xA1,0x99,0x67,0x80,0x8C,0x87,0xF6,0x9F,0x01,0x32,0x4C,0xDA,0x7F,0xD6,0xA2,0x01,0x32,0xE6,0x19,0x73,0x46,0x9B,0x2D,0x67,0x2A,0x1A,0x20,0x73,0x98,0x79,0x68,0x80,0x0C,0x8D,0xD6,0x9A,0x85,0x67,0xE1,0x19,0x20,0xE3,0xA1,0xD1,0x68,0xCC,0x99,0x79,0x06,0xC8,0xFC,0x33,0x80,0x8C,0x59,0xC5,0xA4,0x01,0x32,0x34,0xDA,0x7F,0x96,0xA1,0x85,0x67,0xAA,0x19,0x2D,0xE3,0xA1,0x01,0x32,0xFF,0x19,0x7E,0x06,0xC8,0xCC,0x66,0xCC,0x59,0x86,0xD6,0x9A,0x85,0x33,0x80,0xCC,0x72,0xF6,0x9F,0xD1,0x68,0x80,0x0C,0x2D,0x93,0x99,0xFD,0x67,0x2D,0x9A,0x8A,0x06,0xC8,0x30,0x69,0xFF,0x59,0x8B,0x96,0xA1,0x79,0x68,0x7F,0x0F,0x20,0xF3,0xCF,0x00,0x32,0xAA,0xD6,0x72,0x76,0x98,0xA9,0x68,0xB4,0x8C,0x87,0x06,0xC8,0xA8,0x68,0xCB,0x59,0x6B,0x06,0xC8,0xD0,0x68,0x87,0x19,0x93,0x06,0xC8,0x30,0x55,0x80,0x4C,0x78,0xC6,0x9C,0x55,0x67,0xAD,0x19,0x20,0xC3,0x9C,0xA9,0x68,0x80,0xCC,0x61,0x86,0x9F,0xA9,0x66,0x80,0x0C,0x53,0x45,0xCB,0x84,0x67,0xE1,0x19,0x20,0x83,0x9F,0xB5,0x66,0x33,0x5A,0x6B,0x96,0xA1,0x01,0x32,0xB4,0x59,0x6B,0xA6,0xA2,0x01,0x32,0x99,0xD9,0x7F,0x96,0xA1,0xB5,0x66,0xAA,0x19,0x20,0xF3,0xCF,0x00,0x32,0x4B,0x55,0x6B,0xC6,0xA4,0x01,0x32,0xF8,0xD9,0x7F,0x46,0xA3,0x85,0x33,0x80,0x0C,0x93,0xF6,0x9F,0xB5,0x68,0xB4,0x4C,0x86,0xD6,0x9A,0x01,0x32,0x87,0x19,0x7E,0x06,0xC8,0x1C,0x66,0xE1,0x59,0x78,0x06,0xC8,0x78,0x68,0x2A,0xDA,0x61,0x96,0xA1,0x01,0x32,0xFF,0x0C,0x20,0x43,0x53,0xB5,0x66,0x2A,0x1A,0x20,0xC3,0xA4,0xFD,0x67,0x2D,0x5A,0x86,0x06,0xC8,0xD0,0x66,0x87,0x99,0x79,0xD6,0x9A,0x01,0x32,0xFF,0x19,0x7E,0x16,0xCE,0x00,0x32,0xB4,0xD9,0x7F,0x06,0xC8,0x00,0x68,0xE1,0xD9,0x61,0xC6,0xA4,0x01,0x32,0xFF,0x0C,0x20,0xB3,0x54,0xB5,0x66,0x4C,0x1A,0x20,0x83,0x9F,0xFD,0x67,0x34,0x5A,0x38,0x03,0xC8,0x30,0x69,0xFF,0x59,0x8B,0x46,0xCB,0x64,0x68,0xAD,0x19,0x20,0x73,0x98,0x01,0x32,0x19,0xDA,0x7F,0xE6,0x99,0x55,0x67,0x80,0x8C,0x87,0xA6,0xA2,0x1D,0x66,0x19,0x1A,0x20,0xF3,0xCF,0x00,0x32,0x34,0x55,0x6B,0xA6,0xA2,0x01,0x32,0x4C,0xDA,0x7F,0xD6,0xA2,0x65,0x68,0x80,0x8C,0x87,0xB6,0x9C,0xFD,0x67,0x34,0x1A,0x20,0xF3,0x9F,0xE1,0x67,0xE1,0x0C,0x20,0x43,0x9B,0xB5,0x66,0x2A,0x1A,0x20,0x03,0xA0,0x1D,0x66,0xCC,0x99,0x6A,0x06,0xC8,0xFC,0x33,0x80,0xCC,0x32,0x73,0x50,0x85,0x67,0xE1,0x19,0x20,0xA3,0xA2,0x2D,0x67,0x87,0x99,0x8A,0x06,0xC8,0xD0,0x66,0xE1,0x19,0x73,0xA6,0xA2,0xA9,0x68,0xAD,0x59,0x86,0xE6,0xA1,0x01,0x32,0xCC,0x99,0x87,0x06,0xC8,0xD0,0x66,0xFF,0x59,0x78,0xA6,0x9A,0x31,0x33,0x80,0xCC,0x3F,0x03,0xC8,0xFC,0x55,0xF8,0x59,0x78,0xC6,0xA4,0x01,0x32,0x1E,0xDA,0x72,0xF6,0x9F,0xFD,0x67,0x2A,0x1A,0x73,0x86,0x9F,0xD1,0x32,0x80,0x8C,0x87,0xA6,0xA2,0x1D,0x66,0x19,0x9A,0x87,0x06,0xC8,0x64,0x66,0x19,0x5A,0x6B,0x76,0x98,0x55,0x67,0x80,0x8C,0x8A,0xB6,0x9C,0xB5,0x66,0x80,0x8C,0x79,0xF6,0x9F,0x85,0x67,0xAA,0x19,0x20,0xF3,0xCF,0x00,0x32,0x34,0xD5,0x7F,0x06,0xC8,0xCC,0x66,0xFF,0x59,0x86,0x06,0xC8,0xA8,0x68,0xCB,0x59,0x6B,0x06,0xC8,0x98,0x67,0xFF,0xD9,0x7F,0x86,0x9F,0x01,0x32,0xFF,0x0C,0x20,0x43,0x53,0x99,0x33,0xB4,0x99,0x39,0x43,0x9B,0x99,0x33,0xB4,0xD9,0x7F,0x06,0xC8,0xCC,0x66,0xFF,0x59,0x86,0x06,0xC8,0xA8,0x68,0xCB,0x59,0x6B,0x06,0xC8,0x98,0x67,0xFF,0xD9,0x7F,0x86,0x9F,0x01,0x32,0xFF,0x0C,0x20,0x43,0x53,0xFD,0x67,0x80,0xCC,0x6C,0xF6,0x9F,0x65,0x68,0x80,0x8C,0x8A,0xB6,0x9C,0xB5,0x66,0x80,0x8C,0x79,0xF6,0x9F,0xFD,0x67,0xF8,0x19,0x20,0xF3,0xCF,0x00,0x32,0x34,0xD5,0x7F,0x66,0xCE,0xD0,0x66,0xFF,0x99,0x39,0x43,0x9B,0xFD,0x67,0x80,0xCC,0x6C,0xF6,0x9F,0x65,0x68,0x80,0x8C,0x8A,0xB6,0x9C,0xB5,0x66,0x80,0x8C,0x79,0xF6,0x9F,0xFD,0x67,0xF8,0x19,0x20,0xF3,0xCF,0x00,0x32,0x4B,0x55,0x6B,0xC6,0xA4,0x01,0x32,0xF8,0xD9,0x7F,0x46,0xA3,0x85,0x33,0x80,0x0C,0x93,0xF6,0x9F,0xB5,0x68,0xB4,0x4C,0x86,0xD6,0x9A,0x01,0x32,0x87,0x19,0x7E,0x06,0xC8,0x1C,0x66,0xE1,0x59,0x78,0x06,0xC8,0x78,0x68,0x2A,0xDA,0x61,0x96,0xA1,0x01,0x32,0xFF,0x0C,0x20,0x43,0x53,0xB5,0x66,0x2A,0x1A,0x20,0xC3,0xA4,0xFD,0x67,0x2D,0x5A,0x86,0x06,0xC8,0xD0,0x66,0x87,0x99,0x79,0xD6,0x9A,0x01,0x32,0xFF,0x19,0x7E,0x16,0xCE,0x00,0x32,0xB4,0xD9,0x7F,0x06,0xC8,0x00,0x68,0xE1,0xD9,0x61,0xC6,0xA4,0x01,0x32,0xFF,0x0C,0x20,0xB3,0x54,0xB5,0x66,0x4C,0x1A,0x20,0x83,0x9F,0xFD,0x67,0x34,0x5A,0x38,0x03,0xC8,0x30,0x69,0xFF,0x59,0x8B,0x46,0xCB,0x64,0x68,0xAD,0x19,0x20,0x73,0x98,0x01,0x32,0x19,0xDA,0x7F,0xE6,0x99,0x55,0x67,0x80,0x8C,0x87,0xA6,0xA2,0x1D,0x66,0x19,0x1A,0x20,0xF3,0xCF,0x00,0x32,0x34,0x55,0x6B,0xA6,0xA2,0x01,0x32,0x4C,0xDA,0x7F,0xD6,0xA2,0x65,0x68,0x80,0x8C,0x87,0xB6,0x9C,0xFD,0x67,0x34,0x1A,0x20,0xF3,0x9F,0xE1,0x67,0xE1,0x0C,0x20,0x43,0x9B,0xB5,0x66,0x2A,0x1A,0x20,0x03,0xA0,0x1D,0x66,0xCC,0x99,0x6A,0x06,0xC8,0xFC,0x33,0x80,0xCC,0x32,0x73,0x50,0x85,0x67,0xE1,0x19,0x20,0xA3,0xA2,0x2D,0x67,0x87,0x99,0x8A,0x06,0xC8,0xD0,0x66,0xE1,0x19,0x73,0xA6,0xA2,0xA9,0x68,0xAD,0x59,0x86,0xE6,0xA1,0x01,0x32,0xCC,0x99,0x87,0x06,0xC8,0xD0,0x66,0xFF,0x59,0x78,0xA6,0x9A,0x31,0x33,0xF8,0x0C};
const size_t expected_len = sizeof(expected_data) - 1;

extern const char TEST_DATA[];
const size_t TEST_DATA_LEN = 1507;

size_t expected_cnt = 0;


void trellis_reset(trellis_t *t) {
    memset(t, 0, sizeof(*t));

    // only the initial state is possible
    for (int i = 0; i < CC_STATES; ++i) {
        t->path_metric[t->head][i] = INFINITY;
    }
    t->path_metric[t->head][0] = 0.0f;

    for (int t1 = 0; t1 < TRELLIS_HISTORY; ++t1) {
        for (int s = 0; s < CC_STATES; ++s) {
            t->prev_states[t1][s] = -1;
        }
    }

    t->y12 = 0;

    t->head = 1;
}
  
int trellis_pop_tail(trellis_t *t) {
    if (((t->tail + 1) % TRELLIS_HISTORY) == t->head) {
        return -1;
    }
  
    int idx = (t->head - 1 + TRELLIS_HISTORY) % TRELLIS_HISTORY;
  
    // First, find most likely current state
    int best_state = 0;
    float best_state_score = INFINITY;
    for (int s = 0; s < CC_STATES; ++s) {
        if (best_state_score > t->path_metric[idx][s]) {
            best_state_score = t->path_metric[idx][s];
            best_state = s;
        }
    }

    int cur_state = best_state;
    while (1) {
        if (idx == ((t->tail + 1) % TRELLIS_HISTORY)) {
            // This is the last transition
            break;
        }

        cur_state = t->prev_states[idx][cur_state];
        idx = (idx - 1 + TRELLIS_HISTORY) % TRELLIS_HISTORY;
    }

    int result_y12 = y12_for_transition(t->prev_states[idx][cur_state], cur_state);
    int result_q34 = t->q34[idx][cur_state];

    int q12 = diff_dec(t->y12, result_y12);
    t->y12 = result_y12;

    ++t->tail;
    if (t->tail >= TRELLIS_HISTORY) t->tail = 0;

    return (q12 << 2) | result_q34;
}
  
int trellis_push_head(trellis_t *t, float i_pt, float q_pt) {
    int result = -1;
    if (t->head == t->tail) {
        result = trellis_pop_tail(t);
    }

    for (int s = 0; s < CC_STATES; ++s) {
        t->prev_states[t->head][s] = -1;
        t->path_metric[t->head][s] = INFINITY;
    }

    for (int prev_state = 0; prev_state < CC_STATES; ++prev_state) {
        for (int cur_state = 0; cur_state < CC_STATES; ++cur_state) {
            int y12_trans = y12_for_transition(prev_state, cur_state);
            if (y12_trans < 0) {
                // This transition is impossible
                continue;
            }

            float branch_metric = INFINITY;
            int likely_q34 = 0;
            for (int guessed_q34 = 0; guessed_q34 < 4; ++guessed_q34) {
                float i_exp = SYMBOL_TO_CONSTELLATION_I[((cur_state & 1) << 4) | (y12_trans << 2) | guessed_q34];
                float q_exp = SYMBOL_TO_CONSTELLATION_Q[((cur_state & 1) << 4) | (y12_trans << 2) | guessed_q34];
                float guessed_bm = (i_pt-i_exp)*(i_pt-i_exp) + (q_pt-q_exp)*(q_pt-q_exp);
                if (branch_metric > guessed_bm) {
                    branch_metric = guessed_bm;
                    likely_q34 = guessed_q34;
                }
            }

            int prev_idx = (t->head + TRELLIS_HISTORY - 1) % TRELLIS_HISTORY;

            float new_metric = t->path_metric[prev_idx][prev_state] + branch_metric;
            if (t->path_metric[t->head][cur_state] > new_metric) {
                t->path_metric[t->head][cur_state] = new_metric;
                t->prev_states[t->head][cur_state] = prev_state;
                t->q34[t->head][cur_state] = likely_q34;
            }
        }
    }

    ++t->head;
    if (t->head >= TRELLIS_HISTORY) t->head = 0;

    return result;
}

typedef struct {
    trellis_t trellis;
    float mag_adj;
    float phase_adj;
    complexf_t adj;
} carrier_t;

carrier_t carriers[CARRIERS];

int bit_errors[CARRIERS] = { 0 };
  
int16_t sample_buf[SAMPLE_BUF_SZ] __ALIGNED(4) = { 0 };
// +4 bytes for header
int sample_buf_head = 0;
int quiet_time = 0;
int samples = 0;
int next_symbol_time = 0;

extern DAC_HandleTypeDef hdac;

#define RX_STREAM_SIZE 1520


typedef struct {
    size_t bits;
    size_t len;
    int next_byte;
    uint8_t data[RX_STREAM_SIZE];
} rx_stream_t;

static void rx_push_bitstring(rx_stream_t *rxs, int value, int num_bits) {
    rxs->next_byte |= (value << rxs->bits);
    rxs->bits += num_bits;
    while (rxs->bits >= 8) {
        if (rxs->len < RX_STREAM_SIZE) {
            rxs->data[rxs->len++] = rxs->next_byte & 0xFF;
        }
        rxs->next_byte >>= 8;
        rxs->bits -= 8;
    }

}

static void rx_stream_push_block(rx_stream_t *rxs, int symbols[CARRIERS]) {
    for (int b = 0; b < 4; ++b) {
        for (int c = 0; c < CARRIERS; ++c) {
            rx_push_bitstring(rxs, (symbols[c] >> b) & 0x1, 1);
        }
    }
}

static void rx_stream_push(rx_stream_t *rxs, int symbol) {
    rxs->next_byte |= (symbol << rxs->bits);
    rxs->bits += BITS_PER_SYMBOL;
    if (rxs->bits >= 8) {
        if (rxs->len < RX_STREAM_SIZE) {
            rxs->data[rxs->len++] = rxs->next_byte & 0xFF;
        }
        rxs->next_byte >>= 8;
        rxs->bits -= 8;
    }
}

static void rx_stream_reset(rx_stream_t *rxs) {
    rxs->bits = 0;
    rxs->len = 0;
    rxs->next_byte = 0;
}

typedef struct {
    size_t bits;
    uint32_t next_chunk;
    size_t check_bits;
    uint32_t next_check_chunk;
    size_t exp_bits;
    uint32_t next_exp;
} rx_hamming_stream_t;

static void rxh_push_bitstring(rx_stream_t *rxs, rx_hamming_stream_t *rxh, int value, int num_bits) {
    rxh->next_chunk |= (value << rxh->bits);
    rxh->bits += num_bits;
    while (rxh->bits >= 7) {
        uint16_t chunk = rxh->next_chunk & 0x7F;
        rxh->next_chunk >>= 7;
        rxh->bits -= 7;

        rx_push_bitstring(rxs, hamming_decode_7_4(chunk), 4);
    }

    rxh->next_check_chunk |= (value << rxh->check_bits);
    rxh->check_bits += num_bits;
    while (rxh->check_bits >= CARRIERS) {
        while (expected_cnt < expected_len && rxh->exp_bits < CARRIERS) {
            rxh->next_exp |= (expected_data[expected_cnt++] << rxh->exp_bits);
            rxh->exp_bits += 8;
        }

        uint16_t chunk = rxh->next_check_chunk;
        rxh->next_check_chunk >>= CARRIERS;
        rxh->check_bits -= CARRIERS;

        if (rxh->exp_bits >= CARRIERS) {
            int err = (chunk ^ rxh->next_exp) & ((1 << CARRIERS) - 1);
            rxh->next_exp >>= CARRIERS;
            rxh->exp_bits -= CARRIERS;
            for (int c = 0; c < CARRIERS; ++c) {
                if ((err >> c) & 1) {
                    ++bit_errors[c];
                }
            }
        }
    }
}

static void rxh_push_block(rx_stream_t *rxs, rx_hamming_stream_t *rxh, int symbols[CARRIERS]) {
    for (int b = 0; b < 4; ++b) {
        for (int c = 0; c < CARRIERS; ++c) {
            rxh_push_bitstring(rxs, rxh, (symbols[c] >> b) & 0x1, 1);
        }
    }
}

static void rxh_reset(rx_hamming_stream_t *rxh) {
    rxh->bits = 0;
    rxh->next_chunk = 0;
    rxh->check_bits = 0;
    rxh->next_check_chunk = 0;
    rxh->exp_bits = 0;
    rxh->next_exp = 0;
}

rx_stream_t rxs = { 0 };
rx_hamming_stream_t rxh = { 0 };

extern ADC_HandleTypeDef hadc1;

float signed_angle_diff(float lhs, float rhs) {
    return fmod(lhs - rhs + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
}

bool frame_started = false;
bool frame_finished = false;
bool data_sample_ready = false;
float data_b_i[CARRIERS];
float data_b_q[CARRIERS];

extern SAI_HandleTypeDef hsai_BlockB1;

static void SAI_Receive_IT16Bit_2(SAI_HandleTypeDef *hsai);

static uint32_t SAI_InterruptFlag2(const SAI_HandleTypeDef *hsai)
{
  uint32_t tmpIT = SAI_IT_OVRUDR;

  tmpIT |= SAI_IT_FREQ;

  if ((hsai->Init.Protocol == SAI_AC97_PROTOCOL) &&
      ((hsai->Init.AudioMode == SAI_MODESLAVE_RX) || (hsai->Init.AudioMode == SAI_MODEMASTER_RX)))
  {
    tmpIT |= SAI_IT_CNRDY;
  }

  if ((hsai->Init.AudioMode == SAI_MODESLAVE_RX) || (hsai->Init.AudioMode == SAI_MODESLAVE_TX))
  {
    tmpIT |= SAI_IT_AFSDET | SAI_IT_LFSDET;
  }
  else
  {
    /* hsai has been configured in master mode */
    tmpIT |= SAI_IT_WCKCFG;
  }
  return tmpIT;
}

int receive_init(void) {
    for (int c = 0; c < CARRIERS; ++c) {
        carriers[c].mag_adj = 10.0;
    }

    SAI_HandleTypeDef *hsai = &hsai_BlockB1;

    if (hsai->State != HAL_SAI_STATE_READY) {
        printf("sai state is %d\n", hsai->State);
        return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hsai);

    hsai->pBuffPtr = NULL;
    hsai->XferSize = 0;
    hsai->XferCount = 0;
    hsai->ErrorCode = HAL_SAI_ERROR_NONE;
    hsai->State = HAL_SAI_STATE_BUSY_RX;

    hsai->InterruptServiceRoutine = SAI_Receive_IT16Bit_2;

    /* Enable TXE and OVRUDR interrupts */
    __HAL_SAI_ENABLE_IT(hsai, SAI_InterruptFlag2(hsai));

    /* Check if the SAI is already enabled */
    if ((hsai->Instance->CR1 & SAI_xCR1_SAIEN) == RESET) {
        /* Enable SAI peripheral */
        __HAL_SAI_ENABLE(hsai);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hsai);

    return HAL_OK;
}

static void find_best_iq(float b_i, float b_q, int *best_i, int *best_q) {
    float best_dist = INFINITY;
    for (int i = 0; i < 32; ++i) { 
        float i_dist = b_i - SYMBOL_TO_CONSTELLATION_I[i];
        float q_dist = b_q - SYMBOL_TO_CONSTELLATION_Q[i];
        float dist = i_dist * i_dist + q_dist * q_dist;
        if (dist < best_dist) {
            best_dist = dist;
            *best_i = SYMBOL_TO_CONSTELLATION_I[i];
            *best_q = SYMBOL_TO_CONSTELLATION_Q[i];
        }
    }
}

void receive_task(void) {
    //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 1);

    if (frame_started) {
        frame_started = false;
        rx_stream_reset(&rxs);
        for (int i = 0; i < CARRIERS; ++i) {
            trellis_reset(&carriers[i].trellis);
        }
    }


    if (data_sample_ready) {
        data_sample_ready = false;

        int16_t cons_points[CARRIERS * 2];
        int symbols[CARRIERS] = { 0 };
        bool symbols_valid = false;
        float avg_drift = 0.0f;
        complexf_t diff2;
        for (int c = 0; c < CARRIERS; ++c) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 1);

            complexf_t b_raw = { data_b_i[c], data_b_q[c] };
            complexf_t b_adj = complexf_mul(b_raw, carriers[c].adj);

            const float mu = 1e-2;
            float mag = sqrtf(b_adj.re * b_adj.re + b_adj.im * b_adj.im);
            carriers[c].mag_adj += mu * (3.0f - mag);

            cons_points[c*2+0] = (int16_t)(b_adj.re * 4096.0f);
            cons_points[c*2+1] = (int16_t)(b_adj.im * 4096.0f);

            /*let closest = closest_symbol(samp_adj);
            if matches!((closest.re, closest.im), (1, 0) | (-1, 0) | (0, 1) | (0, -1)) {
                self.phase_adj[c] -= 0.1 * (samp_adj / Complex::new(closest.re as f32, closest.im as f32)).arg();
            }

            self.iq_adj[c] = self.mag_adj[c] * C32::new(self.phase_adj[c].cos(), self.phase_adj[c].sin());*/

            symbols[c] = trellis_push_head(&carriers[c].trellis, b_adj.re, b_adj.im);

            int i_int, q_int;
            find_best_iq(b_adj.re, b_adj.im, &i_int, &q_int);
            complexf_t best_iq = { i_int, q_int };
            complexf_t diff = complexf_div(b_adj, best_iq);

            diff2 = diff;

            avg_drift += -atan2f(diff.im, diff.re) / (CARRIER_FREQUENCIES_HZ[c] * CARRIERS);

            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);
        }

        for (int c = 0; c < CARRIERS; ++c) {
            carriers[c].phase_adj += avg_drift * CARRIER_FREQUENCIES_HZ[c];
            carriers[c].adj.re = carriers[c].mag_adj * cosf(carriers[c].phase_adj);
            carriers[c].adj.im = carriers[c].mag_adj * sinf(carriers[c].phase_adj);
        }

        /*if (rxs.len == 0) {
            printf("dr %d %d %d\n", (int)(avg_drift * 1e6), (int)(diff2.re * 1e6), (int)(diff2.im * 1e6));
        }*/

        if (symbols[0] >= 0) {
            rxh_push_block(&rxs, &rxh, symbols);
        }

        //printf("MAG: %d\n", (int)(log10f(m) * 1000.0f));

        int16_t header[] = { 0x55FF, 0x00AA, 0x0002, sizeof(cons_points) };
        __disable_irq();
        tud_cdc_write(header, sizeof(header));
        tud_cdc_write(cons_points, sizeof(cons_points));
        __enable_irq();
    }

    if (frame_finished) {
        frame_finished = false;

        while (1) {
            int symbols[CARRIERS] = { 0 };
            for (int c = 0; c < CARRIERS; ++c) {
                symbols[c] = trellis_pop_tail(&carriers[c].trellis);
            }

            if (symbols[0] < 0) {
                break;
            }

            rxh_push_block(&rxs, &rxh, symbols);
        }

        if (rxs.len > 0) {
            int exp_len = (rxs.data[1] << 8) | rxs.data[0];
            int len_diff = exp_len - (int)rxs.len - 2;
            if (-40 <= len_diff && len_diff <= 40) {
                rxs.len = exp_len + 2;
                ethernet_send_packet(rxs.data+2, exp_len);
                printf("RX (%u %u): ", exp_len, rxs.len - 2);
            } else {
                printf("BAD LEN");
            }

            /*if (rxs.len < exp_len + 2 + 8) {
                rxs.len = exp_len + 2;
            }*/

            for (size_t i = 2; i < rxs.len; ++i) {
                printf("%c", rxs.data[i]);
            }
            printf("\n");
            printf("m/p: %d %d\n", (int)(log10f(carriers[0].mag_adj) * 1000.0f), 0);
            int l = (rxs.len < expected_len) ? rxs.len : expected_len;
            printf("%d: ", 2*l/CARRIERS);
            for (int i = 0; i < CARRIERS; ++i) {
                printf("%d ", bit_errors[i]);
                bit_errors[i] = 0;
            }
            expected_cnt = 0;
            printf("\n");
            if (rxs.len-2 == TEST_DATA_LEN) {
                for (int i = 0; i < TEST_DATA_LEN; ++i) {
                    if (TEST_DATA[i] != rxs.data[i+2]) {
                        printf("ERROR AT %d\n", i);
                        break;
                    }
                }
            } else {
                printf("AAA %u %u\n", rxs.len-2, TEST_DATA_LEN);
            }
            /*if (byt_errors > 0) {
                printf("byt errors: %d\n", byt_errors);
            }*/
            //printf("phase adj: %d %d %d\n", (int)(1000.0 * carriers[0].phase_adj), (int)(1000.0 * carriers[1].phase_adj), (int)(1000.0 * (carriers[1].phase_adj - carriers[0].phase_adj)));
            rx_stream_reset(&rxs);
            rxh_reset(&rxh);
        }
    }

    //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 0);
}

static int32_t b_i_accum[CARRIERS] = { 0 };
static int32_t b_q_accum[CARRIERS] = { 0 };

static float filt_power = 0.0;

static void handle_sample_interrupt(int16_t adc_val) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);

    ++samples;

    for (int c = 0; c < CARRIERS; ++c) {
        int ang = sample_buf_head * (SAMPLE_PERIOD_NS_NUM * (CARRIER_FREQUENCIES_HZ[c] / 100)) / (10000000 / 256 * SAMPLE_PERIOD_NS_DEN);
        b_i_accum[c] += icos(ang) * (adc_val - sample_buf[sample_buf_head]);
        b_q_accum[c] += isin(ang) * (adc_val - sample_buf[sample_buf_head]);
    }
    sample_buf[sample_buf_head++] = adc_val;

    bool data_sample = quiet_time < SAMPLES_PER_SYMBOL / 4 && (samples > 2 * SAMPLES_PER_SYMBOL) && (samples >= next_symbol_time);
    if (data_sample) {
        data_sample_ready = true;
    }

    if (sample_buf_head >= SAMPLE_BUF_SZ) {
        sample_buf_head = 0;
        int16_t header[4] __ALIGNED(4) = { 0x55FF, 0x00AA, 0x0001, sizeof(sample_buf) }; 
        tud_cdc_write(header, sizeof(header));
        tud_cdc_write(sample_buf, sizeof(sample_buf));
    }

    bool training_sample = (samples == SAMPLES_PER_SYMBOL * 3 / 2);

    float total_power = 0.0f;
    for (int c = 0; c < CARRIERS; ++c) {
        float b_i = b_i_accum[c] * (3.3 / (256 * 4096 * WINDOW_SIZE));
        float b_q = b_q_accum[c] * (3.3 / (256 * 4096 * WINDOW_SIZE));

        float b_mag_sq = (b_i * b_i + b_q * b_q);
        total_power += b_mag_sq;

        if (training_sample) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 1);
            carriers[c].phase_adj = -atan2f(b_q, b_i);
            carriers[c].adj.re = carriers[c].mag_adj * cosf(carriers[c].phase_adj);
            carriers[c].adj.im = carriers[c].mag_adj * sinf(carriers[c].phase_adj);
            //carriers[c].adj.re =  4.0f * b_i / b_mag_sq;
            //carriers[c].adj.im = -4.0f * b_q / b_mag_sq;
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);
        }

        if (data_sample) {
            data_b_i[c] = b_i;
            data_b_q[c] = b_q;
        }
    }
    filt_power = filt_power * 0.5 + total_power * 0.5;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)(total_power * 1e5));

    if (quiet_time > SAMPLES_PER_SYMBOL / 2) {
        frame_finished = true;
    }

    if ((quiet_time < SAMPLES_PER_SYMBOL / 2) && samples > 2 * SAMPLES_PER_SYMBOL) {
        if (samples >= next_symbol_time) {
            next_symbol_time += SAMPLES_PER_SYMBOL;
        }
    }

    if (filt_power < 2.0e-3) {
        ++quiet_time;
    } else {
        if (quiet_time > SAMPLES_PER_SYMBOL / 2) {
            frame_started = true;
            // we just started a new transmission, sample as late as possible
            samples = 0;
            // TODO: Don't hardcode this to expect 2 symbol period training
            next_symbol_time = (SAMPLES_PER_SYMBOL * 23 / 8);
        }
        quiet_time = 0;
    }

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0);
}

static void SAI_Receive_IT16Bit_2(SAI_HandleTypeDef *hsai) {
    int16_t val = hsai->Instance->DR;

    static int which_slot = 0;
    if (!which_slot) {
        handle_sample_interrupt(val);
    }
    which_slot = !which_slot;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    (void)hadc;
    int16_t adc_val = HAL_ADC_GetValue(&hadc1);

    handle_sample_interrupt(adc_val);
}
