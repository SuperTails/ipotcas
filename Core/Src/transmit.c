#include "transmit.h"
#include "modulation.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim3;

uint64_t micros() {
    static uint16_t prev = 0;
    static uint64_t offset = 0;
    uint16_t cur = __HAL_TIM_GET_COUNTER(&htim3);
    if (prev > cur) {
        // account for wrapping
        offset += 65536;
    }
    prev = cur;
    return offset + cur;
}

static int diff_enc(int y_prev, int q) {
  int8_t TABLE[4][4] = {
    {0, 1, 2, 3},
    {1, 0, 3, 2},
    {2, 3, 1, 0},
    {3, 2, 0, 1}
  };

  return TABLE[y_prev][q];
}

static int cc_step(int *state, int y12) {
  int a = (*state >> 2) & 1;
  int b = (*state >> 1) & 1;
  int c = (*state >> 0) & 1;

  int y1 = (y12 >> 1) & 1;
  int y2 = y12 & 1;

  int ap = c;
  int bp = a ^ y1 ^ y2 ^ (c * (b ^ y2));
  int cp = b ^ y2 ^ (y1 * c);

  *state = (ap << 2) | (bp << 1) | cp;

  return cp;
}

typedef struct {
    int y12;
    int cc;
} encoder_t;

static void encoder_reset(encoder_t *enc) {
    memset(enc, 0, sizeof(*enc));
}

static void encoder_enc(encoder_t *enc, int symbol, int *i, int *q) {
    int q34 = symbol & 0x3;
    int q12 = symbol >> 2;

    enc->y12 = diff_enc(enc->y12, q12);
    int y0 = cc_step(&enc->cc, enc->y12);

    *i = SYMBOL_TO_CONSTELLATION_I[(y0 << 4) | (enc->y12 << 2) | q34];
    *q = SYMBOL_TO_CONSTELLATION_Q[(y0 << 4) | (enc->y12 << 2) | q34];
}

static int current_dac_value(int mod_i, int mod_q) {
    double rad = micros() * (2.0 * M_PI * CARRIER_FREQUENCY / 1.0e6);
    double amp = (mod_i * cos(rad) + mod_q * sin(rad));
    // scale -8 to 8 range into 0 to 1, then convert to int
    return (int)(((amp + 8.0) / 16.0) * 4096.0);
}

typedef enum {
    TX_GAP,
    TX_TRAIN,
    TX_BYTE_LO,
    TX_BYTE_HI,
} tx_state_t;

typedef struct {
    uint64_t next_event;
    int mod_i, mod_q;
    int idx;
    encoder_t enc;
    tx_state_t state;
} transmitter_t;

static void tx_reset(transmitter_t *tx) {
    memset(tx, 0, sizeof(*tx));
    tx->next_event = micros();
}

const char sym_data[] = "Somebody once told me the world is gonna roll me / I ain't the sharpest tool in the shed / She was looking kind of dumb with her finger and her thumb / In the shape of an \"L\" on her forehead / Well, the years start comin' and they don't stop comin' / Fed to the rules and I hit the ground runnin' / Didn't make sense not to live for fun / Your brain gets smart but your head gets dumb / So much to do, so much to see / So what's wrong with taking the backstreets? / You'll never know if you don't go / You'll never shine if you don't glow";

// returns the current value that should be fed to the DAC
static int tx_update(transmitter_t *tx) {
    if (micros() > tx->next_event) {
        if (tx->state == TX_GAP) {
            tx->next_event += 2 * TRAIN_PERIOD_US;
            tx->mod_i = 4; tx->mod_q = 0;
            tx->idx = 0;
            encoder_reset(&tx->enc);
            tx->state = TX_TRAIN;
        } else if ((tx->state == TX_TRAIN) || (tx->state == TX_BYTE_HI && tx->idx < sizeof(sym_data))) {
            int symbol = sym_data[tx->idx] & 0xF;

            tx->next_event += SYMBOL_PERIOD_US;
            encoder_enc(&tx->enc, symbol, &tx->mod_i, &tx->mod_q);
            tx->state = TX_BYTE_LO;
        } else if (tx->state == TX_BYTE_LO) {
            int symbol = (sym_data[tx->idx++] >> 4) & 0xF;

            tx->next_event += SYMBOL_PERIOD_US;
            encoder_enc(&tx->enc, symbol, &tx->mod_i, &tx->mod_q);
            tx->state = TX_BYTE_HI;
        } else if (tx->state == TX_BYTE_HI) {
            tx->next_event += SYMBOL_GAP_US;
            tx->mod_i = 0; tx->mod_q = 0;
            tx->state = TX_GAP;
        }
    }

    return current_dac_value(tx->mod_i, tx->mod_q);
}

transmitter_t TRANSMITTER;

void transmit_init() {
    tx_reset(&TRANSMITTER);
}

void transmit_task(DAC_HandleTypeDef *hdac) {
    int value = tx_update(&TRANSMITTER);
    
    //int value = current_dac_value(4, 0);
    HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);
}