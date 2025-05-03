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

#define SAMPLE_BUF_SZ 32

#define RAD_PER_SAMPLE (2.0 * M_PI * SAMPLE_PERIOD_NS / CARRIER_PERIOD_NS)

#define TRELLIS_HISTORY 16
#define CC_STATES 8

int diff_dec(int y_prev, int y_new) {
  const int TABLE[4][4] = {
    {0, 1, 2, 3},
    {1, 0, 3, 2},
    {3, 2, 0, 1},
    {2, 3, 1, 0}
  };

  return TABLE[y_prev][y_new];
}

int y12_for_transition(int prev_state, int cur_state) {
  int TABLE[8][8] = {
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



struct {
    float path_metric[TRELLIS_HISTORY][CC_STATES];
    int prev_states[TRELLIS_HISTORY][CC_STATES];
    int q34[TRELLIS_HISTORY][CC_STATES];
  
    int y12;
  
    int head;
    int tail;
} trellis;
  
void trellis_reset() {
    memset(&trellis, 0, sizeof(trellis));

    // only the initial state is possible
    for (int i = 0; i < CC_STATES; ++i) {
        trellis.path_metric[trellis.head][i] = INFINITY;
    }
    trellis.path_metric[trellis.head][0] = 0.0f;

    for (int t = 0; t < TRELLIS_HISTORY; ++t) {
        for (int s = 0; s < CC_STATES; ++s) {
            trellis.prev_states[t][s] = -1;
        }
    }

    trellis.y12 = 0;

    trellis.head = 1;
}
  
int trellis_pop_tail() {
    if (((trellis.tail + 1) % TRELLIS_HISTORY) == trellis.head) {
        return -1;
    }
  
    int idx = (trellis.head - 1 + TRELLIS_HISTORY) % TRELLIS_HISTORY;
  
    // First, find most likely current state
    int best_state = 0;
    float best_state_score = INFINITY;
    for (int s = 0; s < CC_STATES; ++s) {
        if (best_state_score > trellis.path_metric[idx][s]) {
            best_state_score = trellis.path_metric[idx][s];
            best_state = s;
        }
    }

    int cur_state = best_state;
    while (1) {
        if (idx == ((trellis.tail + 1) % TRELLIS_HISTORY)) {
            // This is the last transition
            break;
        }

        cur_state = trellis.prev_states[idx][cur_state];
        idx = (idx - 1 + TRELLIS_HISTORY) % TRELLIS_HISTORY;
    }

    int result_y12 = y12_for_transition(trellis.prev_states[idx][cur_state], cur_state);
    int result_q34 = trellis.q34[idx][cur_state];

    int q12 = diff_dec(trellis.y12, result_y12);
    trellis.y12 = result_y12;

    ++trellis.tail;
    if (trellis.tail >= TRELLIS_HISTORY) trellis.tail = 0;

    return (q12 << 2) | result_q34;
}
  
int trellis_push_head(float i_pt, float q_pt) {
    int result = -1;
    if (trellis.head == trellis.tail) {
        result = trellis_pop_tail();
    }

    for (int s = 0; s < CC_STATES; ++s) {
        trellis.prev_states[trellis.head][s] = -1;
        trellis.path_metric[trellis.head][s] = INFINITY;
    }

    for (int prev_state = 0; prev_state < CC_STATES; ++prev_state) {
        for (int cur_state = 0; cur_state < CC_STATES; ++cur_state) {
            int y12_trans = y12_for_transition(prev_state, cur_state);
            if (y12_trans < 0) {
                // This transition is impossible
                continue;
            }

            float branch_metric = INFINITY;
            int likely_q34;
            for (int guessed_q34 = 0; guessed_q34 < 4; ++guessed_q34) {
                float i_exp = SYMBOL_TO_CONSTELLATION_I[((cur_state & 1) << 4) | (y12_trans << 2) | guessed_q34];
                float q_exp = SYMBOL_TO_CONSTELLATION_Q[((cur_state & 1) << 4) | (y12_trans << 2) | guessed_q34];
                float guessed_bm = (i_pt-i_exp)*(i_pt-i_exp) + (q_pt-q_exp)*(q_pt-q_exp);
                if (branch_metric > guessed_bm) {
                    branch_metric = guessed_bm;
                    likely_q34 = guessed_q34;
                }
            }

            int prev_idx = (trellis.head + TRELLIS_HISTORY - 1) % TRELLIS_HISTORY;

            float new_metric = trellis.path_metric[prev_idx][prev_state] + branch_metric;
            if (trellis.path_metric[trellis.head][cur_state] > new_metric) {
                trellis.path_metric[trellis.head][cur_state] = new_metric;
                trellis.prev_states[trellis.head][cur_state] = prev_state;
                trellis.q34[trellis.head][cur_state] = likely_q34;
            }
        }
    }

    ++trellis.head;
    if (trellis.head >= TRELLIS_HISTORY) trellis.head = 0;

    return result;
}
  
float cos_buf[SAMPLE_BUF_SZ];
float sin_buf[SAMPLE_BUF_SZ];
float sample_buf[SAMPLE_BUF_SZ];
int sample_buf_head = 0;
int quiet_time = 0;
int samples = 0;
int next_symbol_time = 0;
int y12 = 0;
int by = 0;
bool new_data = false;
bool half = false;

#define RX_SIZE 1524

uint8_t received_data[RX_SIZE];
int received_data_len = 0;

float phase_adj = 0.0f;
float mag_adj = 0.0f;

extern ADC_HandleTypeDef hadc1;

float signed_angle_diff(float lhs, float rhs) {
    return fmod(lhs - rhs + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
}


void init_coeff_buf() {
    for (int k = 0; k < SAMPLE_BUF_SZ; ++k) {
        cos_buf[k] = cos(RAD_PER_SAMPLE * k + phase_adj) * (3.3 / (4096 * SAMPLE_BUF_SZ));
        sin_buf[k] = sin(RAD_PER_SAMPLE * k + phase_adj) * (3.3 / (4096 * SAMPLE_BUF_SZ));
    }
}

void receive_init(void) {
    HAL_ADC_Start_IT(&hadc1);
    init_coeff_buf();
}


void receive_task(void) {
    if (!new_data) {
        return;
    }

    new_data = false;

    //printf("%d\n", (int)sample_buf[(sample_buf_head + SAMPLE_BUF_SZ - 1) % SAMPLE_BUF_SZ]);

    float b_i = 0.0f;
    float b_q = 0.0f;
    for (int k = 0; k < SAMPLE_BUF_SZ; ++k) {
        b_i += cos_buf[k] * sample_buf[k];
        b_q += sin_buf[k] * sample_buf[k];
    }

    float b_mag_sq = (b_i * b_i + b_q * b_q);
    if (samples == SAMPLES_PER_SYMBOL * 3 / 2) {
        //gpio_put(1, true);
        phase_adj -= atan2(b_q, b_i);
        init_coeff_buf();
        mag_adj = 4.0 / sqrt(b_mag_sq);
        //gpio_put(1, false);
    }

    if (quiet_time > SAMPLES_PER_SYMBOL / 2) {
        while (1) {
          int s = trellis_pop_tail();
          if (s < 0) {
            break;
          }

          if (half) {
            by |= s << 4;
            half = false;
            if (received_data_len < RX_SIZE) {
                received_data[received_data_len++] = by;
            }
          } else {
            by = s;
            half = true;
          }
        }

        if (received_data_len > 0) {
            printf("RX: ");
            for (int i = 0; i < received_data_len; ++i) {
                printf("%c", received_data[i]);
            }
            printf("\n");
            received_data_len = 0;
            ethernet_send_packet(received_data, received_data_len);
        }
    }


    if (b_mag_sq < 500e-6) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
        ++quiet_time;
    } else {
        if (quiet_time > SAMPLES_PER_SYMBOL / 2) {
            // we just started a new transmission, sample as late as possible
            half = false;
            samples = 0;
            // TODO: Don't hardcode this to expect 2 symbol period training
            next_symbol_time = (SAMPLES_PER_SYMBOL * 11 / 4);
            received_data_len = 0;
            printf("START\n");
            trellis_reset();
        }
        quiet_time = 0;
    }

    b_i *= mag_adj;
    b_q *= mag_adj;

    if ((quiet_time < (SAMPLES_PER_SYMBOL / 2)) && samples > 2 * SAMPLES_PER_SYMBOL) {
        if (samples >= next_symbol_time) {
            next_symbol_time += SAMPLES_PER_SYMBOL;
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
                phase_adj -= signed_angle_diff(atan2(b_q, b_i), 0.0f) * 0.2f;
                phase_changed = true;
            } else if (best_i == 0 && best_q == 1) {
                phase_adj -= signed_angle_diff(atan2(b_q, b_i), M_PI_2) * 0.2f;
                phase_changed = true;
            } else if (best_i == -1 && best_q == 0) {
                phase_adj -= signed_angle_diff(atan2(b_q, b_i), M_PI) * 0.2f;
                phase_changed = true;
            } else if (best_i == 0 && best_q == -1) {
                phase_adj -= signed_angle_diff(atan2(b_q, b_i), -M_PI_2) * 0.2f;
                phase_changed = true;
            }

            if (phase_changed) {
                init_coeff_buf();
            }

            //printf("%2d %2d %d %d\n", best_i, best_q, (int)(b_i * 1000), (int)(b_q * 1000));
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);
            //my_printf("%f,%f\n", mag_adj * b_i1, mag_adj * b_q1);

            int s = trellis_push_head(b_i, b_q);
            if (s != -1) {
                if (half) {
                    by |= s << 4;
                    half = false;
                    if (received_data_len < RX_SIZE) {
                        received_data[received_data_len++] = by;
                    }
                } else {
                    by = s;
                    half = true;
                }
            }
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    sample_buf[sample_buf_head++] = HAL_ADC_GetValue(&hadc1);
    if (sample_buf_head >= SAMPLE_BUF_SZ) sample_buf_head = 0;
    new_data = true;
    ++samples;
}

