#include "receive.h"
#include "fast_math.h"
#include "stm32f7xx_hal.h"
#include "modulation.h"
#include "ethernet.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define SAMPLE_BUF_SZ 64

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
    int16_t cos_buf[SAMPLE_BUF_SZ] __ALIGNED(4);
    int16_t sin_buf[SAMPLE_BUF_SZ] __ALIGNED(4);
    trellis_t trellis;
    float phase_adj;
    float mag_adj;
} carrier_t;

carrier_t carriers[CARRIERS];
  
int16_t sample_buf[SAMPLE_BUF_SZ] __ALIGNED(4);
int sample_buf_head = 0;
int quiet_time = 0;
int samples = 0;
int next_symbol_time = 0;
bool new_data = false;

extern DAC_HandleTypeDef hdac;

#define RX_STREAM_SIZE 1520

typedef struct {
    size_t bits;
    size_t len;
    int next_byte;
    uint8_t data[RX_STREAM_SIZE];
} rx_stream_t;

void rx_stream_push(rx_stream_t *rxs, int symbol) {
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

void rx_stream_reset(rx_stream_t *rxs) {
    rxs->bits = 0;
    rxs->len = 0;
    rxs->next_byte = 0;
}

rx_stream_t rxs;

extern ADC_HandleTypeDef hadc1;

float signed_angle_diff(float lhs, float rhs) {
    return fmod(lhs - rhs + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
}


void init_coeff_buf(carrier_t *c, int i) {
    for (int k = 0; k < SAMPLE_BUF_SZ; ++k) {
        c->cos_buf[k] = cos(RAD_PER_SAMPLE(i) * k + c->phase_adj) * 2048;
        c->sin_buf[k] = sin(RAD_PER_SAMPLE(i) * k + c->phase_adj) * 2048;
    }
}

void receive_init(void) {
    HAL_ADC_Start_IT(&hadc1);
    for (int i = 0; i < CARRIERS; ++i) {
        init_coeff_buf(&carriers[i], i);
    }
}

float min_power = INFINITY;
float filt_power = 0.0;

void receive_task(void) {
    if (!new_data) {
        return;
    }

    new_data = false;

    static int abc = 0;
    ++abc;

    /*if (abc % 1000 == 0) {
        printf("logf %d\n", (int)(logf(filt_power) * 100.0));
    }*/

    bool training_sample = (samples == SAMPLES_PER_SYMBOL * 3 / 2);
    bool data_sample = (quiet_time < SAMPLES_PER_SYMBOL / 4 && (samples > 2 * SAMPLES_PER_SYMBOL) && (samples >= next_symbol_time));

    float total_power = 0.0;

    int32_t sample_buf2[SAMPLE_BUF_SZ/2];
    memcpy(sample_buf2, sample_buf, sizeof(sample_buf));

    for (int c = 0; c < CARRIERS; ++c) {
        // this block takes like 10us
        int32_t b_i_int = 0;
        int32_t b_q_int = 0;

        for (int k = 0; k < SAMPLE_BUF_SZ; k += 2) {
            b_i_int = smlad(*(int32_t *)&carriers[c].cos_buf[k], sample_buf2[k/2], b_i_int);
            b_q_int = smlad(*(int32_t *)&carriers[c].sin_buf[k], sample_buf2[k/2], b_q_int);
        }

        float b_i = b_i_int * (3.3 / (2048 * 4096 * SAMPLE_BUF_SZ));
        float b_q = b_q_int * (3.3 / (2048 * 4096 * SAMPLE_BUF_SZ));

        float b_mag_sq = (b_i * b_i + b_q * b_q);
        if (training_sample) {
            //gpio_put(1, true);
            carriers[c].phase_adj -= atan2(b_q, b_i);
            init_coeff_buf(&carriers[c], c);
            carriers[c].mag_adj = 4.0 / sqrt(b_mag_sq);
            //gpio_put(1, false);
        }

        total_power += b_mag_sq;

        b_i *= carriers[c].mag_adj;
        b_q *= carriers[c].mag_adj;

        /*if (abc % 10000 == 0) {
            printf("%d %d\n", b_i_int, b_q_int);
        }*/

        if (data_sample) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 1);

            int best_i = 0;
            int best_q = 0;
            float best_dist = INFINITY;
            for (int i = 0; i < 32; ++i) { 
                float i_dist = b_i - SYMBOL_TO_CONSTELLATION_I[i];
                float q_dist = b_q - SYMBOL_TO_CONSTELLATION_Q[i];
                float dist = i_dist * i_dist + q_dist * q_dist;
                if (dist < best_dist) {
                    best_dist = dist;
                    best_i = SYMBOL_TO_CONSTELLATION_I[i];
                    best_q = SYMBOL_TO_CONSTELLATION_Q[i];
                }
            }

            bool phase_changed = false;
            if (best_i == 1 && best_q == 0) {
                carriers[c].phase_adj -= signed_angle_diff(atan2(b_q, b_i), 0.0f) * 0.2f;
                phase_changed = true;
            } else if (best_i == 0 && best_q == 1) {
                carriers[c].phase_adj -= signed_angle_diff(atan2(b_q, b_i), M_PI_2) * 0.2f;
                phase_changed = true;
            } else if (best_i == -1 && best_q == 0) {
                carriers[c].phase_adj -= signed_angle_diff(atan2(b_q, b_i), M_PI) * 0.2f;
                phase_changed = true;
            } else if (best_i == 0 && best_q == -1) {
                carriers[c].phase_adj -= signed_angle_diff(atan2(b_q, b_i), -M_PI_2) * 0.2f;
                phase_changed = true;
            }

            if (phase_changed) {
                init_coeff_buf(&carriers[c], c);
                //printf("phase adj: %d %d\n", (int)(1000.0 * carriers[0].phase_adj), (int)(1000.0 * carriers[1].phase_adj));
            }


            //printf("%d: %2d %2d %d %d\n", c, best_i, best_q, (int)(b_i * 1000), (int)(b_q * 1000));
            //my_printf("%f,%f\n", mag_adj * b_i1, mag_adj * b_q1);

            int s = trellis_push_head(&carriers[c].trellis, b_i, b_q);
            if (s >= 0) {
                rx_stream_push(&rxs, s);
            }

            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);
        }
    }

    filt_power = filt_power * 0.5 + total_power * 0.5;

    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)(logf(filt_power) * 100.0 + 4096));
    //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)(logf(filt_power) * 300.0 + 2048));


    if ((quiet_time < SAMPLES_PER_SYMBOL / 2) && samples > 2 * SAMPLES_PER_SYMBOL) {
        if (samples >= next_symbol_time) {
            next_symbol_time += SAMPLES_PER_SYMBOL;
        }
    }

    if (quiet_time > SAMPLES_PER_SYMBOL / 2) {
        int done = 0;
        while (!done) {
            for (int c = 0; c < CARRIERS; ++c) {
                int s = trellis_pop_tail(&carriers[c].trellis);
                if (s < 0) {
                    done = 1;
                    break;
                }

                rx_stream_push(&rxs, s);
            }
        }

        if (rxs.len > 0) {
            printf("RX: ");
            for (int i = 0; i < rxs.len; ++i) {
                printf("%c", rxs.data[i]);
            }
            printf("\n");
            int l = (rxs.len < expected_len) ? rxs.len : expected_len;
            int byt_errors = 0;
            int err_dist[CARRIERS] = { 0 };
            for (int i = 0; i < 2*l; ++i) {
                if ((rxs.data[i/2] ^ expected_data[i/2]) & (0xF << (4 * (i % 2)))) {
                    ++err_dist[i % CARRIERS];
                    ++byt_errors;
                }
            }
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
        }
    }

    if (filt_power < 1e-3) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
        ++quiet_time;
    } else {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0);
        if (quiet_time > SAMPLES_PER_SYMBOL / 2) {
            // we just started a new transmission, sample as late as possible
            samples = 0;
            // TODO: Don't hardcode this to expect 2 symbol period training
            next_symbol_time = (SAMPLES_PER_SYMBOL * 23 / 8);
            rx_stream_reset(&rxs);
            printf("START\n");
            for (int i = 0; i < CARRIERS; ++i) {
                trellis_reset(&carriers[i].trellis);
            }
        }
        quiet_time = 0;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    (void)hadc;
    uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
    sample_buf[sample_buf_head++] = adc_val;
    if (sample_buf_head >= SAMPLE_BUF_SZ) sample_buf_head = 0;
    new_data = true;
    ++samples;
    //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, adc_val);
}

