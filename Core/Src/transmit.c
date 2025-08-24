#include "transmit.h"
#include "fast_math.h"
#include "modulation.h"
#include "ethernet.h"
#include "hamming.h"
#include "scrambler.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_dma.h"
#include "tusb.h"
#include "main.h"
#include "stm32f7xx_hal_dac.h"
#include "stm32f7xx_hal_dma_ex.h"
#include <complex.h>
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
    int q12 = (symbol >> 2) & 0x3;

#if TRELLIS_CODING
    enc->y12 = diff_enc(enc->y12, q12);
    int y0 = cc_step(&enc->cc, enc->y12);
#else
    enc->y12 = diff_enc(enc->y12, q12);
    int y0 = (symbol >> 4) & 0x1;
#endif

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
    scrambler_t scrambler;
    uint8_t checksum_acc;
    uint8_t checksum_cum;
    int checksum_done;
} tx_bitstream_t;

void txbs_reload_data(tx_bitstream_t *s) {
    s->idx = 0;
    s->buf = sym_len;
    s->bits = 16;
    s->checksum_acc = 0;
    s->checksum_cum = 0;

    s->scrambler = scrambler_reset();
    s->buf ^= scrambler_step_byte(&s->scrambler);
    s->buf ^= scrambler_step_byte(&s->scrambler) << 8;
}

bool txbs_data_exhausted(const tx_bitstream_t *s) {
    return s->idx == (sym_len + 2) && s->bits == 0;
}

int txbs_munch_bits(tx_bitstream_t *s, int num_bits) {
    while (num_bits > s->bits && s->idx < sym_len + 2) {
        if (s->idx < sym_len) {
            s->buf |= (sym_data[s->idx] ^ scrambler_step_byte(&s->scrambler)) << s->bits;
            s->checksum_acc += sym_data[s->idx];
            s->checksum_cum += s->checksum_acc;
            s->bits += 8;
            s->idx++;
        } else if (s->idx == sym_len) {
            s->buf |= (s->checksum_acc ^ scrambler_step_byte(&s->scrambler)) << s->bits;
            s->bits += 8;
            s->idx++;
        } else if (s->idx == sym_len + 1) {
            s->buf |= (s->checksum_cum ^ scrambler_step_byte(&s->scrambler)) << s->bits;
            s->bits += 8;
            s->idx++;
        }
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
        uint16_t val = txbs_munch_bits(s, HAMMING_MESSAGE_SIZE);
        uint16_t chunk = HAMMING_ENCODE(val);
        txh->buf |= chunk << txh->bits;
        txh->bits += HAMMING_BLOCK_SIZE;
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

        for (int b = 0; b < BITS_PER_SYMBOL; ++b) {
            int stripe = txhs_munch_bits(&tx->bs, &tx->hs, CARRIERS);
            for (int c = 0; c < CARRIERS; ++c) {
                symbols[c] |= ((stripe >> c) & 1) << b;
            }
        }

        #if MODULATION_ASK
        for (int c = 0; c < CARRIERS; ++c) {
            mod[c].i = 1 + (symbols[c]);
            mod[c].q = 0;
        }
        #elif MODULATION_QAM
        for (int i = 0; i < CARRIERS; ++i) {
            encoder_enc(&tx->enc[i], symbols[i], &mod[i].i, &mod[i].q);
        }
        #endif
    }
}

#define HISTORY 1

typedef struct {
    tx_cstream_t cs;
    int64_t cur_symbol_center;
    mod_t mod[HISTORY][CARRIERS];
    int cur_symbol; // history index of the current symbol
    int32_t ticks; // elapsed ticks modulo SAMPLES_PER_SYMBOL
} transmitter_t;

int current_dac_value(transmitter_t *t) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 1);

    int ip0 = t->cur_symbol;

    #if 0
    int any = 0;
    static unsigned sum = 0;
    if (t->ticks == 0) {
        sum >>= 2;
        if (sum == 0) sum = 0xF29393EF;
    }
    for (int i = 0; i < CARRIERS; ++i) {
        if (t->mod[ip0][i].i != 0 || t->mod[ip0][i].q != 0) {
            any = 1;
        }
        t->mod[ip0][i].i = 0;
        t->mod[ip0][i].q = 0;
    }

    if (any) {
        // FSK:
        //if (g) { t->mod[ip0][0].i = 4; } else { t->mod[ip0][11].i = 4; }
        // 2-ASK:
        t->mod[ip0][0].i = 1 + (sum & 0x3);
        // BPSK:
        //t->mod[ip0][0].i = (sum % 2) ? 4 : -4;
    }
    #endif

    int32_t amp = 0;
    for (int i = 0; i < CARRIERS; ++i) {
        int ang = t->ticks * (256 * CARRIER_FREQUENCIES_HZ[i]) / SAMPLE_RATE_HZ;
        amp = smlad(*(int32_t *)&t->mod[ip0][i], cos_sin_table[ang & 0xFF], amp);
    }

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);
    return amp;
}

static bool transmit_pop_next(void);

static int tx_update(transmitter_t *tx) {
    if (++tx->ticks >= SAMPLES_PER_SYMBOL) {
        tx->ticks = 0;
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

    return current_dac_value(tx);
}

static void tx_reset(transmitter_t *tx) {
    memset(tx, 0, sizeof(*tx));
    tx->cur_symbol_center = 0;
    tx->ticks = 0;
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

#if TX_RAW_STREAM
static volatile int16_t next_tx_stream[TX_SAMPLES_PER_SYMBOL] = { 0 };
static volatile int next_tx_stream_ready = 0;

void submit_raw_stream(int16_t *samples) { 
    for (int i = 0; i < TX_SAMPLES_PER_SYMBOL; ++i) {
        next_tx_stream[i] = samples[i];
    }
    next_tx_stream_ready = 1;
}
#endif

void DMA_DAC_M0_Cplt(struct __DMA_HandleTypeDef *dma) {
    (void)dma;


#if TX_RAW_STREAM

    if (next_tx_stream_ready) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);

        for (int i = 0; i < TX_SAMPLES_PER_SYMBOL; ++i) {
            int16_t value = next_tx_stream[i];
            dac_dma_buf_a[2*i+0] = value;
            dac_dma_buf_a[2*i+1] = value;
        }
        next_tx_stream_ready = 0;

        usb_msg_header_t header = { USB_MSG_HEADER_LO, USB_MSG_HEADER_HI, USB_MSG_TX_AUDIO_ACK, 0 };
        tud_cdc_write((const void *)&header, sizeof(header));
        tud_cdc_write_flush();
    }

#else // normal operation

    // buffer A just finished being read from, refill it
    for (int i = 0; i < TX_SAMPLES_PER_SYMBOL; ++i) {
        int16_t value = (int16_t)tx_update(&TRANSMITTER);
        dac_dma_buf_a[2*i+0] = value;
        dac_dma_buf_a[2*i+1] = value;
        usb_buf[i] = value;
    }

#endif

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 0);

}

void DMA_DAC_M1_Cplt(struct __DMA_HandleTypeDef *dma) {
    (void)dma;


#if TX_RAW_STREAM

    if (next_tx_stream_ready) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);

        for (int i = 0; i < TX_SAMPLES_PER_SYMBOL; ++i) {
            int16_t value = next_tx_stream[i];
            dac_dma_buf_b[2*i+0] = value;
            dac_dma_buf_b[2*i+1] = value;
        }
        next_tx_stream_ready = 0;

        usb_msg_header_t header = { USB_MSG_HEADER_LO, USB_MSG_HEADER_HI, USB_MSG_TX_AUDIO_ACK, 0 };
        tud_cdc_write((const void *)&header, sizeof(header));
        tud_cdc_write_flush();

    }

#else // normal operation

    // buffer B just finished being read from, refill it
    for (int i = 0; i < TX_SAMPLES_PER_SYMBOL; ++i) {
        int16_t value = (int16_t)tx_update(&TRANSMITTER);
        dac_dma_buf_b[2*i+0] = value;
        dac_dma_buf_b[2*i+1] = value;
        usb_buf[i] = value;
    }

#endif

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 0);
}

void transmit_task(DAC_HandleTypeDef *hdac) {
}

// magic flag nonsense from the STM HAL
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

// Initialize DMA to transfer generated samples to the audio codec
// and call the DAC_DMA_M0_Cplt/DAC_DMA_M1_Cplt callbacks whenever we need more samples
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
