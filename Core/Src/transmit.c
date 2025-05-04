#include "transmit.h"
#include "modulation.h"
#include "ethernet.h"
#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <sys/wait.h>

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

double fast_cos(double rad) {
    //rad = fmod(rad + M_PI, 2.0f * M_PI) - M_PI;

    // from https://stackoverflow.com/questions/18662261/fastest-implementation-of-sine-cosine-and-square-root-in-c-doesnt-need-to-b
    double tp = 1./(2.*M_PI);
    rad *= tp;
    rad -= 0.25 + floor(rad + 0.25);
    rad *= 16.0 * (fabs(rad) - 0.5);

    /*float rad_2 = rad * rad;
    float rad_4 = rad_2 * rad_2;

    return 1.0 - rad_2 / 2.0 + rad_4 / 24.0;*/
    return rad;
}

double fast_sin(double rad) {
    return fast_cos(rad - M_PI_2);
}

const void *current_packet = NULL;
//const uint8_t *sym_data = NULL;
//size_t sym_len = 0;
const char *sym_data = "Somebody once told me the world is gonna roll me / I ain't the sharpest tool in the shed / She was looking kind of dumb with her finger and her thumb / In the shape of an \"L\" on her forehead / Well, the years start comin' and they don't stop comin' / Fed to the rules and I hit the ground runnin' / Didn't make sense not to live for fun / Your brain gets smart but your head gets dumb / So much to do, so much to see / So what's wrong with taking the backstreets? / You'll never know if you don't go / You'll never shine if you don't glow";
size_t sym_len = 539;

typedef enum {
    TX_GAP,
    TX_TRAIN,
    TX_SEND
} tx_state_t;

typedef struct {
    uint64_t next_event;
    int mod_i[CARRIERS];
    int mod_q[CARRIERS];
    int idx;
    int hi;
    encoder_t enc[CARRIERS];
    tx_state_t state;
} transmitter_t;

void tx_reload_data(transmitter_t *tx) {
    tx->idx = 0;
    tx->hi = 0;
}

bool tx_data_exhausted(const transmitter_t *tx) {
    return tx->idx >= sym_len;
}

int tx_munch_symbol(transmitter_t *tx) {
    if (tx->idx >= sym_len) {
        return 0;
    } else if (tx->hi) {
        tx->hi = false;
        return sym_data[tx->idx++];
    } else {
        tx->hi = true;
        return sym_data[tx->idx];
    }
}

static int current_dac_value(transmitter_t *t) {
    double amp = 0.0;
    for (int i = 0; i < CARRIERS; ++i) {
        double rad = micros() * (2.0 * M_PI * 1e3 / CARRIER_PERIODS_NS[i]);
        amp += 1.5 * (t->mod_i[i] * fast_cos(rad) + t->mod_q[i] * fast_sin(rad)) / CARRIERS;
    }
    // scale -8 to 8 range into 0 to 1, then convert to int
    return (int)(((amp + 8.0) / 16.0) * 4096.0);
}

static void tx_reset(transmitter_t *tx) {
    memset(tx, 0, sizeof(*tx));
    tx->next_event = micros();
}

#if 0
extern UART_HandleTypeDef huart2;

volatile bool dma_ready = true;
uint8_t dma_buffer[1524+3];

bool transmit_ready(size_t len) {
    return dma_ready;
}

void transmit_send(const uint8_t *data, size_t len) {
    dma_buffer[0] = '%';
    dma_buffer[1] = len & 0xFF;
    dma_buffer[2] = (len >> 8) & 0xFF;
    memcpy(dma_buffer+3, data, len);
    HAL_UART_Transmit_DMA(&huart2, dma_buffer, len+3);
    dma_ready = false;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        dma_ready = true;
    }
}
#else


bool transmit_ready(size_t len) {
    return current_packet == NULL;
}

void transmit_send(const void *pkt, const uint8_t *data, size_t len) {
    if (current_packet) {
        return;
    }

    current_packet = pkt;
    sym_data = data;
    sym_len = len;
}
#endif


//const char sym_data[] = "Hello, world!\n";

// returns the current value that should be fed to the DAC
static int tx_update(transmitter_t *tx) {
    if (tx->state == TX_GAP && sym_len == 0) {
        return 4096 / 2;
    }

    if (micros() > tx->next_event) {
        if (tx->state == TX_GAP) {
            tx->next_event += TRAIN_PERIOD_US;
            tx_reload_data(tx);
            for (int i = 0; i < CARRIERS; ++i) {
                tx->mod_i[i] = 4; tx->mod_q[i] = 0;
                encoder_reset(&tx->enc[i]);
            }
            tx->state = TX_TRAIN;
        } else if ((tx->state == TX_TRAIN) || (tx->state == TX_SEND && !tx_data_exhausted(tx))) {
            tx->next_event += SYMBOL_PERIOD_US;
            for (int i = 0; i < CARRIERS; ++i) {
                int symbol = tx_munch_symbol(tx);
                encoder_enc(&tx->enc[i], symbol, &tx->mod_i[i], &tx->mod_q[i]);
            }
            tx->state = TX_SEND;
        } else if (tx->state == TX_SEND) {
            if (current_packet) {
                ethernet_free_rx_buffer(current_packet);
                current_packet = NULL;
                sym_data = NULL;
                sym_len = 0;
            }

            tx->next_event += SYMBOL_GAP_US;
            for (int i = 0; i < CARRIERS; ++i) {
                tx->mod_i[i] = 0; tx->mod_q[i] = 0;
            }
            tx->state = TX_GAP;
        }
    }

    return current_dac_value(tx);
}

transmitter_t TRANSMITTER;

void transmit_task(DAC_HandleTypeDef *hdac) {
    int value = tx_update(&TRANSMITTER);
    
    //int value = current_dac_value(4, 0);
    //static int i = 0;
    //int value = micros() % 4096;
    HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);
}

void transmit_init() {
    tx_reset(&TRANSMITTER);
}