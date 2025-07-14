#pragma once

#include "fixed.h"

#define RS_BLOCK_SIZE 255
#define RS_MSG_SIZE 223

void encode_suffix(data_t *data);

void encode_rs_8_c(const data_t *data, data_t *parity, int pad);

int decode_rs_8(data_t *data, int pad);
