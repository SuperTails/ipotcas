#include "receive.h"
#include "fast_math.h"
#include "stm32f7xx_hal.h"
#include "tusb.h"
#include "hamming.h"
#include "modulation.h"
#include "ethernet.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define WINDOW_SIZE 64

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

const char *expected_data = "Somebody once told me the world is gonna roll me / I ain't the sharpest tool in the shed / She was looking kind of dumb with her finger and her thumb / In the shape of an \"L\" on her forehead / Well, the years start comin' and they don't stop comin' / Fed to the rules and I hit the ground runnin' / Didn't make sense not to live for fun / Your brain gets smart but your head gets dumb / So much to do, so much to see / So what's wrong with taking the backstreets? / You'll never know if you don't go / You'll never shine if you don't glow";
size_t expected_len = 539;


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
    complexf_t adj;
} carrier_t;

carrier_t carriers[CARRIERS];
  
int16_t sample_buf[SAMPLE_BUF_SZ] __ALIGNED(4) = { 0 };
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

void receive_init(void) {
    HAL_ADC_Start_IT(&hadc1);
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
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 1);

    if (frame_started) {
        frame_started = false;
        rx_stream_reset(&rxs);
        for (int i = 0; i < CARRIERS; ++i) {
            trellis_reset(&carriers[i].trellis);
        }
    }

    if (data_sample_ready) {
        data_sample_ready = false;
        int symbols[CARRIERS] = { 0 };
        bool symbols_valid = false;
        float avg_drift = 0.0f;
        complexf_t diff2;
        for (int c = 0; c < CARRIERS; ++c) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 1);

            complexf_t b_raw = { data_b_i[c], data_b_q[c] };
            complexf_t b_adj = complexf_mul(b_raw, carriers[c].adj);

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
            complexf_t phase_adj;
            phase_adj.re = cosf(avg_drift * CARRIER_FREQUENCIES_HZ[c]);
            phase_adj.im = sinf(avg_drift * CARRIER_FREQUENCIES_HZ[c]);
            carriers[c].adj = complexf_mul(carriers[c].adj, phase_adj);
        }

        /*if (rxs.len == 0) {
            printf("dr %d %d %d\n", (int)(avg_drift * 1e6), (int)(diff2.re * 1e6), (int)(diff2.im * 1e6));
        }*/

        if (symbols[0] >= 0) {
            rxh_push_block(&rxs, &rxh, symbols);
        }
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
            uint16_t exp_len = (rxs.data[1] << 8) | rxs.data[0];
            printf("RX (%u %u): ", exp_len, rxs.len - 2);
            /*if (rxs.len < exp_len + 2 + 8) {
                rxs.len = exp_len + 2;
            }*/

            for (size_t i = 2; i < rxs.len; ++i) {
                printf("%c", rxs.data[i]);
            }
            printf("\n");
            int l = (rxs.len < expected_len) ? rxs.len : expected_len;
            int byt_errors = 0;
            int first_err = -1;
            int err_dist[CARRIERS] = { 0 };
            for (int i = 4; i < 2*l; ++i) {
                int mask = (0xF << (4 * (i % 2)));
                int errs = (rxs.data[i/2] ^ expected_data[(i-4)/2]) & mask;
                if (errs) {
                    if (first_err == -1) {
                        first_err = i;
                    }
                    ++err_dist[i % CARRIERS];
                    ++byt_errors;
                }
            }
            printf("%d %d: ", 2*l/CARRIERS, first_err);
            for (int i = 0; i < CARRIERS; ++i) {
                printf("%d ", err_dist[i]);
            }
            printf("\n");
            /*if (byt_errors > 0) {
                printf("byt errors: %d\n", byt_errors);
            }*/
            //printf("phase adj: %d %d %d\n", (int)(1000.0 * carriers[0].phase_adj), (int)(1000.0 * carriers[1].phase_adj), (int)(1000.0 * (carriers[1].phase_adj - carriers[0].phase_adj)));
            ethernet_send_packet(rxs.data, rxs.len);
            rx_stream_reset(&rxs);
            rxh_reset(&rxh);
        }
    }

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, 0);
}

static int32_t b_i_accum[CARRIERS] = { 0 };
static int32_t b_q_accum[CARRIERS] = { 0 };

static float filt_power = 0.0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);

    (void)hadc;
    int16_t adc_val = HAL_ADC_GetValue(&hadc1);
    ++samples;

    for (int c = 0; c < CARRIERS; ++c) {
        int ang = sample_buf_head * (SAMPLE_PERIOD_NS * (CARRIER_FREQUENCIES_HZ[c] / 100)) / (10000000 / 256);
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
            carriers[c].adj.re =  4.0f * b_i / b_mag_sq;
            carriers[c].adj.im = -4.0f * b_q / b_mag_sq;
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

    if (filt_power < 0.5e-3) {
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
