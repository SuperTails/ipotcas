#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// returns nonzero if ethernet OK
int ethernet_init(void);

bool ethernet_can_transmit(size_t len);

void ethernet_send_packet(const uint8_t *data, size_t len);

void ethernet_free_rx_buffer(const void *buf);

void ethernet_task(void);