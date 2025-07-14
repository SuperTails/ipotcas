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
const char *sym_data = NULL;
size_t sym_len = 0;

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

// 48 kHz -> 125/6 microseconds
#define TICK_NUM 125
#define TICK_DEN 6

typedef struct {
    tx_cstream_t cs;
    int64_t cur_symbol_center;
    mod_t mod[HISTORY][CARRIERS];
    int cur_symbol; // history index of the current symbol
    int64_t current_micros;
    int tick_err;
} transmitter_t;

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
            //if (i == 2 || i > 6) { amp += tmp * 2; } else { amp += tmp; }
            amp += tmp;
            //if (i == 7) { amp += tmp * 2; }
        }

        // amp is now in 12.20 fixed-point

        // *2:     increase amplitude
        // *4096:  12-bit DAC
        // /128:   convert cos/sin to -1 to 1
        // /16:    scale down (2*) -4 to 4 range
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);

        return amp;
    }
}

static bool transmit_pop_next(void);

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

            transmit_pop_next();
        } else {
            txcs_munch_period(&tx->cs, tx->mod[tx->cur_symbol]);
        }
    }

    tx->tick_err += TICK_NUM;
    tx->current_micros += tx->tick_err / TICK_DEN;
    tx->tick_err %= TICK_DEN;

    return current_dac_value(tx);
}

static void tx_reset(transmitter_t *tx) {
    memset(tx, 0, sizeof(*tx));
    tx->cur_symbol_center = 0;
    tx->current_micros = 0;
}

static transmitter_t TRANSMITTER;

typedef struct {
    const void *pkt;
    const uint8_t *data;
    size_t len;
} tx_packet_t;

#define TX_PACKET_QUEUE_DEPTH 4

struct {
    tx_packet_t buf[TX_PACKET_QUEUE_DEPTH];
    int head;
    int tail;
    int len;
} TX_PACKET_QUEUE;

bool transmit_ready(size_t len) {
    (void)len;
    return TX_PACKET_QUEUE.len < TX_PACKET_QUEUE_DEPTH;
}

void transmit_send(const void *pkt, const uint8_t *data, size_t len) {
    tx_packet_t *entry = &TX_PACKET_QUEUE.buf[TX_PACKET_QUEUE.head++];
    if (TX_PACKET_QUEUE.head >= TX_PACKET_QUEUE_DEPTH) TX_PACKET_QUEUE.head = 0;
    entry->pkt = pkt;
    entry->data = data;
    entry->len = len;
    ++TX_PACKET_QUEUE.len;
}

static bool transmit_pop_next(void) {
    if (TX_PACKET_QUEUE.len == 0) {
        return false;
    }

    tx_packet_t *entry = &TX_PACKET_QUEUE.buf[TX_PACKET_QUEUE.tail++];
    if (TX_PACKET_QUEUE.tail >= TX_PACKET_QUEUE_DEPTH) TX_PACKET_QUEUE.tail = 0;
    current_packet = entry->pkt;
    sym_data = entry->data;
    sym_len = entry->len;
    txcs_reload_data(&TRANSMITTER.cs);
    --TX_PACKET_QUEUE.len;
    return true;
}

#define TX_SAMPLES_PER_SYMBOL (SYMBOL_PERIOD_US * 48 / 1000)
#define DMA_BUF_SZ (TX_SAMPLES_PER_SYMBOL * 2)

static int16_t dac_dma_buf_a[DMA_BUF_SZ] __ALIGNED(8);
static int16_t dac_dma_buf_b[DMA_BUF_SZ] __ALIGNED(8);

static int16_t usb_buf[TX_SAMPLES_PER_SYMBOL] __ALIGNED(8);


void DMA_DAC_M0_Cplt(struct __DMA_HandleTypeDef *dma) {
    (void)dma;

    // buffer A just finished being read from, refill it
    for (int i = 0; i < TX_SAMPLES_PER_SYMBOL; ++i) {
        int16_t value = (int16_t)tx_update(&TRANSMITTER);
        dac_dma_buf_a[2*i+0] = value;
        dac_dma_buf_a[2*i+1] = value;
        usb_buf[i] = value;
    }
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
    tud_cdc_n_write(1, usb_buf, sizeof(usb_buf));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 0);
}

void DMA_DAC_M1_Cplt(struct __DMA_HandleTypeDef *dma) {
    (void)dma;

    // buffer B just finished being read from, refill it
    for (int i = 0; i < TX_SAMPLES_PER_SYMBOL; ++i) {
        int16_t value = (int16_t)tx_update(&TRANSMITTER);
        dac_dma_buf_b[2*i+0] = value;
        dac_dma_buf_b[2*i+1] = value;
        usb_buf[i] = value;
    }
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
    tud_cdc_n_write(1, usb_buf, sizeof(usb_buf));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 0);
}

void transmit_task(DAC_HandleTypeDef *hdac) {
}

static uint32_t SAI_InterruptFlag2(const SAI_HandleTypeDef *hsai)
{
  uint32_t tmpIT = SAI_IT_OVRUDR;

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

#define SAI_LONG_TIMEOUT      1000U

extern SAI_HandleTypeDef hsai_BlockA1;

HAL_StatusTypeDef transmit_init(DAC_HandleTypeDef *hdac) {
    tx_reset(&TRANSMITTER);

    SAI_HandleTypeDef *hsai = &hsai_BlockA1;

    uint32_t tickstart = HAL_GetTick();

    if (hsai->State != HAL_SAI_STATE_READY) {
        return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hsai);

    hsai->pBuffPtr = (uint8_t *)dac_dma_buf_a;
    hsai->XferSize = TX_SAMPLES_PER_SYMBOL;
    hsai->XferCount = TX_SAMPLES_PER_SYMBOL;
    hsai->ErrorCode = HAL_SAI_ERROR_NONE;
    hsai->State = HAL_SAI_STATE_BUSY_TX;

    /* Set the SAI Tx DMA Half transfer complete callback */
    //hsai->hdmatx->XferHalfCpltCallback = SAI_DMATxHalfCplt;
    //hsai->hdmatx->XferErrorCallback = SAI_DMAError;
    //hsai->hdmatx->XferAbortCallback = NULL;

    /* Set the SAI TxDMA transfer complete callback */
    hsai->hdmatx->XferCpltCallback   = DMA_DAC_M0_Cplt;
    hsai->hdmatx->XferM1CpltCallback = DMA_DAC_M1_Cplt;

    /* Enable the Tx DMA Stream */
    int dma_ok = HAL_DMAEx_MultiBufferStart_IT(hsai->hdmatx, (uint32_t)dac_dma_buf_a, (uint32_t)&hsai->Instance->DR, (uint32_t)dac_dma_buf_b, DMA_BUF_SZ);
    if (dma_ok != HAL_OK) {
      __HAL_UNLOCK(hsai);
      printf("tx dma init failed %d\n", dma_ok);
      return  HAL_ERROR;
    }

    /* Enable the interrupts for error handling */
    __HAL_SAI_ENABLE_IT(hsai, SAI_InterruptFlag2(hsai));

    /* Enable SAI Tx DMA Request */
    hsai->Instance->CR1 |= SAI_xCR1_DMAEN;

    /* Wait until FIFO is not empty */
    while ((hsai->Instance->SR & SAI_xSR_FLVL) == SAI_FIFOSTATUS_EMPTY) {
      /* Check for the Timeout */
      if ((HAL_GetTick() - tickstart) > SAI_LONG_TIMEOUT)
      {
        /* Update error code */
        hsai->ErrorCode |= HAL_SAI_ERROR_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hsai);

        return HAL_TIMEOUT;
      }
    }

    /* Check if the SAI is already enabled */
    if ((hsai->Instance->CR1 & SAI_xCR1_SAIEN) == 0U) {
      /* Enable SAI peripheral */
      __HAL_SAI_ENABLE(hsai);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hsai);

    return HAL_OK;
}
