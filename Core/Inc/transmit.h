#pragma once

#include "stm32f7xx_hal.h"
#include "modulation.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

HAL_StatusTypeDef transmit_init(DAC_HandleTypeDef *hdac);
bool transmit_ready(size_t len);
void transmit_send(const void *packet, const uint8_t *data, size_t len);
void transmit_task(DAC_HandleTypeDef *hdac);

#if TX_RAW_STREAM

#define TX_QUEUE_SIZE 16

void submit_raw_stream(uint8_t *samples);
bool need_new_raw_samples(void);
#endif
