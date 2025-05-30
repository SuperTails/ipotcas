#include "transmit.h"
#include "fast_math.h"
#include "modulation.h"
#include "ethernet.h"
#include "hamming.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_dma.h"
#include "tusb.h"
#include "stm32f7xx_hal_dac.h"
#include "stm32f7xx_hal_dma_ex.h"
#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>

extern TIM_HandleTypeDef htim3;

int64_t micros(void) {
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

static void encoder_enc(encoder_t *enc, int symbol, int16_t *i, int16_t *q) {
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
const char *sym_data =
    "Somebody once told me the world is gonna roll me / "
    "I ain't the sharpest tool in the shed / "
    "She was looking kind of dumb with her finger and her thumb / "
    "In the shape of an \"L\" on her forehead / "
    "Well, the years start comin' and they don't stop comin' / "
    "Fed to the rules and I hit the ground runnin' / "
    "Didn't make sense not to live for fun / "
    "Your brain gets smart but your head gets dumb / "
    "So much to do, so much to see / "
    "So what's wrong with taking the backstreets? / "
    "You'll never know if you don't go / "
    "You'll never shine if you don't glow / "
    "Hey now, you're an all star / "
    "Get your game on, go play / "
    "Hey now, you're a rock star / "
    "Get your show on, get paid / "
    "(And all that glitters is gold) / "
    "Only shootin' stars break the mold / "
    "It's a cool place, and they say it gets colder / "
    "You're bundled up now, wait 'til you get older / "
    "But the meteor men beg to differ / "
    "Judging by the hole in the satellite picture / "
    "The ice we skate is gettin' pretty thin / "
    "The waters gettin' warm so you might as well swim / "
    "My world's on fire, how 'bout yours? / "
    "That's the way I like it and I'll never get bored / "
    "Hey now, you're an all star / "
    "Get your game on, go play / "
    "Hey now, you're a rock star / "
    "Get your show on, get paid / "
    "(All that glitters is gold) / "
    "Only shootin' stars break the mold / "
    "Go for the moon / "
    "G-g-g-go for the moon / "
    "Go for the moon / "
    "Go-go-go for the moon / "
    "Hey now, you're an all star / "
    "Get your game on, go play / "
    "Hey now, you're a rock star / "
    "Get your show on, get paid / "
    "(All that glitters is gold)";

size_t sym_len = 1507;
//const char *sym_data = "Hello, world!\n";
//size_t sym_len = 15;

typedef struct {
    size_t idx;
    uint32_t buf;
    int bits;
} tx_bitstream_t;

void txbs_reload_data(tx_bitstream_t *s) {
    s->idx = 0;
    s->buf = sym_len;
    s->bits = 16;
}

bool txbs_data_exhausted(const tx_bitstream_t *s) {
    return s->idx >= sym_len && s->bits == 0;
}

int txbs_munch_bits(tx_bitstream_t *s, int num_bits) {
    while (num_bits > s->bits && s->idx < sym_len) {
        s->buf |= sym_data[s->idx++] << s->bits;
        s->bits += 8;
    }

    int result = s->buf & ((1 << num_bits) - 1);
    s->buf >>= num_bits;
    s->bits = (s->bits > num_bits) ? (s->bits - num_bits) : 0;
    return result;
}

typedef struct {
    uint32_t buf;
    int bits;
} tx_hamming_t;

static void txhs_reload_data(tx_hamming_t *txh) {
    txh->buf = 0;
    txh->bits = 0;
}

static bool txhs_data_exhausted(const tx_bitstream_t *s, const tx_hamming_t *txh) {
    return txbs_data_exhausted(s) && (txh->bits == 0);
}

static int txhs_munch_bits(tx_bitstream_t *s, tx_hamming_t *txh, int num_bits) {
    while (num_bits > txh->bits) {
        uint16_t val = txbs_munch_bits(s, 4);
        uint16_t chunk = hamming_encode_7_4(val);
        txh->buf |= chunk << txh->bits;
        txh->bits += 7;
    }

    int result = txh->buf & ((1 << num_bits) - 1);
    txh->buf >>= num_bits;
    txh->bits = (txh->bits > num_bits) ? (txh->bits - num_bits) : 0;
    return result;
}

typedef struct {
    tx_bitstream_t bs;
    tx_hamming_t hs;
    encoder_t enc[CARRIERS];
    int train_periods;
    int gap_periods;
} tx_cstream_t;

typedef struct __ALIGNED(4) {
    int16_t i; 
    int16_t q;
} mod_t;

static void txcs_reload_data(tx_cstream_t *tx) {
    txbs_reload_data(&tx->bs);
    txhs_reload_data(&tx->hs);
    for (int i = 0; i < CARRIERS; ++i) {
        encoder_reset(&tx->enc[i]);
    }
    // TODO: Maybe rewrite the constants as multiples of a period?
    tx->train_periods = TRAIN_PERIOD_US / SYMBOL_PERIOD_US;
    tx->gap_periods = SYMBOL_GAP_US / SYMBOL_PERIOD_US;
}

static bool txcs_data_exhausted(const tx_cstream_t *tx) {
    return tx->gap_periods == 0;
}

static void txcs_munch_period(tx_cstream_t *tx, mod_t mod[CARRIERS]) {
    if (tx->train_periods > 0) {
        for (int i = 0; i < CARRIERS; ++i) {
            mod[i].i = 4; mod[i].q = 0;
        }
        --tx->train_periods;
    } else if (txhs_data_exhausted(&tx->bs, &tx->hs)) {
        if (tx->gap_periods > 0) --tx->gap_periods;
        for (int i = 0; i < CARRIERS; ++i) {
            mod[i].i = 0; mod[i].q = 0;
        }
    } else {
        int symbols[CARRIERS] = { 0 };

        for (int b = 0; b < 4; ++b) {
            int stripe = txhs_munch_bits(&tx->bs, &tx->hs, CARRIERS);
            for (int c = 0; c < CARRIERS; ++c) {
                symbols[c] |= ((stripe >> c) & 1) << b;
            }
        }

        for (int i = 0; i < CARRIERS; ++i) {
            encoder_enc(&tx->enc[i], symbols[i], &mod[i].i, &mod[i].q);
        }
    }
}

#define HISTORY 1

typedef struct {
    tx_cstream_t cs;
    int64_t cur_symbol_center;
    mod_t mod[HISTORY][CARRIERS];
    int cur_symbol; // history index of the current symbol
    int64_t current_micros;
} transmitter_t;

static double raised_cos_filt_impulse(double t, double rolloff) {
    if (fabs(t) < 1e-3) {
        return 1.0;
    }

    /*if (fabs(t) < 0.5) {
        //return 1.0 - fabs(t);
        return 1.0;
    } else {
        return 0.0;
    }*/

    double half_inv_rolloff = 0.5 / rolloff;
    if (rolloff > 1e-3 && fabs(fabs(t) - half_inv_rolloff) < 1e-3) {
        return rolloff * 0.5 * fast_sin(M_PI * half_inv_rolloff);
    }

    double tmp = 2.0 * rolloff * t;

    double numer = fast_sin(M_PI * t) * fast_cos(M_PI * rolloff * t);
    double denom = M_PI * t * (1.0 - tmp * tmp);

    return numer / denom;
}

static int32_t raised_cos_filt_impulse_2(int32_t t) {
    if (abs(t) < 32) {
        return 1 << 12; // 1.0
    }

    if (abs(t-2048) < 32 || abs(t+2048) < 32) {
        return 1 << 11; // 0.5
    }

    const int32_t ang_mask = (1 << 12) - 1;
    int32_t numer = isin((t & ang_mask) >> 4); // << 8
    int32_t denom = (t * ((1<<12) - ((4 * t * t) >> 12))) >> 12; // << 12
    denom = (denom * 12867) >> 12; // denom *= M_PI

    return (numer << 16) / denom; // << 12
}

int current_dac_value(transmitter_t *t) {
    if (0) {
        int64_t u = t->current_micros;

        double amp = 0.0;
        static int last_step = 0;
        int freq_step = (u / 20000) % 2000;
        double t = freq_step / 2000.0;
        // 100 Hz to 100kHz
        double freq = exp(4.6 + (9.9 - 4.6) * t);
        if (freq_step != last_step) {
            printf("freq %d\n", (int)(freq));
            HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_0);
            last_step = freq_step;
        }
        double rad = freq * u * (2.0 * M_PI / 1e6);
        amp += fast_cos(rad) / 2.0;
        //int i = (micros() / 100000) - ((micros() - 50) / 100000);
        //amp = (i * 8.0) - 4.0;
        // scale -8 to 8 range into 0 to 1, then convert to int
        return (int)(((amp + 8.0) / 16.0) * 4096.0);

    } else {
        int u = t->current_micros % 2000;

        int ip0 = t->cur_symbol;

        int32_t amp = 0;

        // TODO: Manually const-fold math on CARRIER_FREQUENCIES_HZ

        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 1);
        for (int i = 0; i < CARRIERS; ++i) {
            int ang = u * (256 * CARRIER_FREQUENCIES_HZ[i] / 100) / 10000;
            int32_t tmp = smlad(*(int32_t *)&t->mod[ip0][i], cos_sin_table[ang & 0xFF], 0);
            if (i == 2 || i == 6 || i == 7) { amp += tmp * 2; } else { amp += tmp; }
        }

        int pilot_ang = u * (256 * PILOT_FREQUENCY_HZ / 100) / 10000;

        #if 0
        bool tx_active = false;
        for (int i = 0; i < CARRIERS; ++i) {
            if (t->mod[ip0][i].i != 0 || t->mod[ip0][i].q != 0) {
                tx_active = true;
                break;
            }
        }
        if (tx_active) {
            amp += icos(pilot_ang);
        }
        #endif

        // amp is now in 12.20 fixed-point

        // *2:     increase amplitude
        // *4096:  12-bit DAC
        // /128:   convert cos/sin to -1 to 1
        // /16:    scale down (2*) -4 to 4 range
        amp *= 1 << 8;
        amp /= 128 * CARRIERS;
        amp += 2048;
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);

        return amp;
    }
}

static int tx_update(transmitter_t *tx) {
    if (tx->current_micros >= tx->cur_symbol_center + SYMBOL_PERIOD_US / 2) {
        tx->cur_symbol_center += SYMBOL_PERIOD_US;
        ++tx->cur_symbol; if (tx->cur_symbol >= HISTORY) tx->cur_symbol = 0;

        if (txcs_data_exhausted(&tx->cs)) {
            if (current_packet) {
                ethernet_free_rx_buffer(current_packet);
                current_packet = NULL;
                sym_data = NULL;
                sym_len = 0;
            }
        } else {
            txcs_munch_period(&tx->cs, tx->mod[tx->cur_symbol]);
        }
    }

    tx->current_micros += DAC_PERIOD_US;

    return current_dac_value(tx);
}

static void tx_reset(transmitter_t *tx) {
    memset(tx, 0, sizeof(*tx));
    tx->cur_symbol_center = 0;
    tx->current_micros = 0;
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

static transmitter_t TRANSMITTER;

bool transmit_ready(size_t len) {
    (void)len;
    return current_packet == NULL;
}

void transmit_send(const void *pkt, const uint8_t *data, size_t len) {
    if (current_packet) {
        return;
    }

    current_packet = pkt;
    sym_data = data;
    sym_len = len;

    txcs_reload_data(&TRANSMITTER.cs);
}
#endif

#define TX_SAMPLES_PER_SYMBOL (SYMBOL_PERIOD_US / DAC_PERIOD_US)

static uint16_t dac_dma_buf_a[TX_SAMPLES_PER_SYMBOL] __ALIGNED(8);
static uint16_t dac_dma_buf_b[TX_SAMPLES_PER_SYMBOL] __ALIGNED(8);


void DMA_DAC_M0_Cplt(struct __DMA_HandleTypeDef *dma) {
    (void)dma;

    // buffer A just finished being read from, refill it
    for (int i = 0; i < TX_SAMPLES_PER_SYMBOL; ++i) {
        uint16_t value = tx_update(&TRANSMITTER);
        dac_dma_buf_a[i] = value;
    }
    tud_cdc_n_write(1, dac_dma_buf_a, sizeof(dac_dma_buf_a));
}

void DMA_DAC_M1_Cplt(struct __DMA_HandleTypeDef *dma) {
    (void)dma;

    // buffer B just finished being read from, refill it
    for (int i = 0; i < TX_SAMPLES_PER_SYMBOL; ++i) {
        uint16_t value = tx_update(&TRANSMITTER);
        dac_dma_buf_b[i] = value;
    }
    tud_cdc_n_write(1, dac_dma_buf_b, sizeof(dac_dma_buf_b));
}

void transmit_task(DAC_HandleTypeDef *hdac) {
}

HAL_StatusTypeDef transmit_init(DAC_HandleTypeDef *hdac) {
    tx_reset(&TRANSMITTER);

    HAL_StatusTypeDef status;
    uint32_t tmpreg;

    /* Process locked */
    __HAL_LOCK(hdac);

    /* Change DAC state */
    hdac->State = HAL_DAC_STATE_BUSY;

    /* Set the DMA transfer complete callback for channel1 */
    hdac->DMA_Handle1->XferCpltCallback = DMA_DAC_M0_Cplt;
    hdac->DMA_Handle1->XferM1CpltCallback = DMA_DAC_M1_Cplt;

    /* Enable the selected DAC channel1 DMA request */
    SET_BIT(hdac->Instance->CR, DAC_CR_DMAEN1);

    /* Get DHR12R1 address */
    tmpreg = (uint32_t)&hdac->Instance->DHR12R1;

    /* Enable the DAC DMA underrun interrupt */
    __HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR1);

    /* Enable the DMA Stream */
    status = HAL_DMAEx_MultiBufferStart_IT(hdac->DMA_Handle1, (uint32_t)dac_dma_buf_a, tmpreg, (uint32_t)dac_dma_buf_b, TX_SAMPLES_PER_SYMBOL);

    /* Process Unlocked */
    __HAL_UNLOCK(hdac);

    if (status == HAL_OK) {
        /* Enable the Peripheral */
        __HAL_DAC_ENABLE(hdac, DAC_CHANNEL_1);
    } else {
        hdac->ErrorCode |= HAL_DAC_ERROR_DMA;
    }

    return HAL_OK;
}
