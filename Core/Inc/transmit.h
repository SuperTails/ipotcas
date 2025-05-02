#pragma once

#include "stm32f7xx_hal.h"

void transmit_init(void);
void transmit_task(DAC_HandleTypeDef *hdac);