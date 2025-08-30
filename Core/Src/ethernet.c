// Most of the code in this file is from:
// https://github.com/stm32-hotspot/CKB-STM32-HAL-Ethernet-BareMetal

#include "ethernet.h"
#include "transmit.h"
#include "stm32f7xx_hal.h"
#include "lan8742.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern ETH_HandleTypeDef heth;
extern ETH_TxPacketConfig TxConfig;

int32_t ETH_PHY_INTERFACE_Init(void);
int32_t ETH_PHY_INTERFACE_DeInit (void);
int32_t ETH_PHY_INTERFACE_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_INTERFACE_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_INTERFACE_GetTick(void);

lan8742_Object_t LAN8742;
lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_INTERFACE_Init,
                                  ETH_PHY_INTERFACE_DeInit,
                                  ETH_PHY_INTERFACE_WriteReg,
                                  ETH_PHY_INTERFACE_ReadReg,
                                  ETH_PHY_INTERFACE_GetTick};

int32_t ETH_PHY_INTERFACE_Init(void) {
  /* Configure the MDIO Clock */
  HAL_ETH_SetMDIOClockRange(&heth);
  return 0;
}

int32_t ETH_PHY_INTERFACE_DeInit(void) {
  return 0;
}

int32_t ETH_PHY_INTERFACE_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal) {
  if (HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) != HAL_OK) {
    return -1;
  }
  return 0;
}

int32_t ETH_PHY_INTERFACE_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal) {
  if(HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) != HAL_OK) {
    return -1;
  }
  return 0;
}

int32_t ETH_PHY_INTERFACE_GetTick(void) {
  return HAL_GetTick();
}

int ETH_StartLink(void) {
  ETH_MACConfigTypeDef MACConf = {0};
  int32_t PHYLinkState = 0U;
  uint32_t linkchanged = 0U, speed = 0U, duplex =0U;
  PHYLinkState = LAN8742_GetLinkState(&LAN8742);
  if(PHYLinkState <= LAN8742_STATUS_LINK_DOWN) {
    HAL_ETH_Stop(&heth);
  } else if (PHYLinkState > LAN8742_STATUS_LINK_DOWN) {
    switch (PHYLinkState) {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_100MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_10MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_10MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    default:
      break;
    }
    if(linkchanged) {
      HAL_ETH_GetMACConfig(&heth, &MACConf);
      MACConf.DuplexMode = duplex;
      MACConf.Speed = speed;
      MACConf.DropTCPIPChecksumErrorPacket = DISABLE;
      MACConf.ForwardRxUndersizedGoodPacket = ENABLE;
      HAL_ETH_SetMACConfig(&heth, &MACConf);
      HAL_ETH_Start_IT(&heth);  
      printf("LINK STARTED %ld %ld %ld\n", PHYLinkState, duplex, speed);
    }
    return 1;
  }
  return 0;
}

typedef struct {
  ETH_BufferTypeDef AppBuff;
  uint8_t buffer[1524] __ALIGNED(32);
} ETH_AppBuff;

#define POOL_ENTRIES 8

int pool_taken[POOL_ENTRIES] = { 0 };
ETH_AppBuff rx_pool[POOL_ENTRIES];
int buffer_uses[POOL_ENTRIES] = { 0 };

void ethernet_free_rx_buffer(const void *buf) {
  int idx = ((intptr_t)buf - (intptr_t)&rx_pool[0]) / sizeof(rx_pool[0]);
  pool_taken[idx] = 0;
  ++buffer_uses[idx];
  //printf("freed %d\n", idx);
}


void HAL_ETH_RxAllocateCallback(uint8_t **buff) {
  for (int i = 0; i < POOL_ENTRIES; ++i) {
    if (!pool_taken[i]) {
      pool_taken[i] = 1;
      ETH_AppBuff *p = &rx_pool[i];
      *buff = &p->buffer[0];
      p->AppBuff.next = NULL;
      p->AppBuff.len = 1524;
      return;
    }
  }
  *buff = NULL;
  //printf("alloc fail\n");
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length) {
  ETH_BufferTypeDef **ppStart = (ETH_BufferTypeDef **)pStart;
  ETH_BufferTypeDef **ppEnd = (ETH_BufferTypeDef **)pEnd;
  ETH_BufferTypeDef *p = NULL;
  p = (ETH_BufferTypeDef *)(buff - offsetof(ETH_AppBuff, buffer));
  p->next = NULL;
  p->len = Length;
  //printf("link %p %p %p\n", *ppStart, *ppEnd, p);
  if (!*ppStart) {
    *ppStart = p;
  } else {
    (*ppEnd)->next = p;
  }
  *ppEnd = p;
}

volatile bool buffer_available = true;

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef * heth) {
  buffer_available = true;
}


void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef * heth) {
  ETH_AppBuff *frame_Rx;
  if (HAL_ETH_ReadData(heth, (void **)&frame_Rx) != HAL_OK) {
    //printf("eth rx err\n");
    return;
  }

  #if 0
  if (tud_network_can_xmit(frame_Rx->AppBuff.len)) {
    tud_network_xmit(frame_Rx->buffer, frame_Rx->AppBuff.len);
    printf("eth rx %d ok\n", frame_Rx->AppBuff.len);
  } else {
    printf("eth rx %d bad\n", frame_Rx->AppBuff.len);
    ethernet_free_rx_buffer(frame_Rx);
  }
  #else
  if (transmit_ready(frame_Rx->AppBuff.len)) {
    //printf("eth rx %lu\n", frame_Rx->AppBuff.len);
    transmit_send(frame_Rx, frame_Rx->buffer, frame_Rx->AppBuff.len);
  } else {
    //printf("eth ov %lu\n", frame_Rx->AppBuff.len);
    ethernet_free_rx_buffer(frame_Rx);
  }
  #endif
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth) {
  printf("DMA ERR\n");
}

extern UART_HandleTypeDef huart2;

#define RXBUFSZ 1024

int uart_rx_tail = 0; // index that is being popped off
uint8_t uart_rx_buf[RXBUFSZ] __ALIGNED(32);

uint8_t next_packet_buf[1524];
int next_packet_idx;
int next_packet_len;
enum {
  ST_HEADER,
  ST_LEN_LO,
  ST_LEN_HI,
  ST_BODY
} next_packet_state = ST_HEADER;

uint8_t get_next_uart(void) {
  uint8_t d = uart_rx_buf[uart_rx_tail++];
  if (uart_rx_tail >= RXBUFSZ) uart_rx_tail = 0;
  return d;
}

int ethernet_init(void) {
  /* Set PHY IO functions */
  LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
  /* Initialize the LAN8742 ETH PHY */
  LAN8742_Init(&LAN8742);
  /* Initialize link speed negotiation and start Ethernet peripheral */
  return ETH_StartLink();
}

typedef struct {
  uint8_t dest_mac[6];
  uint8_t src_mac[6];
  uint8_t type[2];
  uint8_t payload[100];
} ethernet_frame_t;

void ETH_ConstructEthernetFrame(
  ethernet_frame_t *frame,
  const uint8_t *dest_mac,
  const uint8_t *src_mac,
  const uint8_t *type,
  const uint8_t *payload,
  uint16_t payload_len
) {
  // Copy the destination MAC address
  memcpy(frame->dest_mac, dest_mac, 6);
  // Copy the source MAC address
  memcpy(frame->src_mac, src_mac, 6);
  // Set the Ethernet type field
  memcpy(frame->type, type, 2);
  // Copy the payload data
  memcpy(frame->payload, payload, payload_len);
}

uint8_t tx_buffer[1524];
ETH_BufferTypeDef TxBuffer;
ethernet_frame_t frame;

//const uint8_t dest_mac[] = {0x00, 0xE0, 0x4C, 0x68, 0x02, 0xEA}; // Destination MAC Address
const uint8_t dest_mac[] = {0x00, 0x80, 0xE1, 0x00, 0x00, 0x00};  // Source MAC Address
const uint8_t src_mac[] = {0x00, 0x80, 0xE1, 0x00, 0x00, 0x00};  // Source MAC Address
const uint8_t type[] = {0x08,0x00 }; // EtherType set to IPV4 packet
const uint8_t payload[] = {0x54,0x65,0x73,0x74,0x69,0x6e,0x67,0x20,0x45,0x74,0x68,0x65,0x72,0x6e,0x65,0x74,0x20,0x6f,0x6e,0x20,0x53,0x54,0x4d,0x33,0x32};
const uint16_t payload_len = sizeof(payload);

bool ethernet_can_transmit(size_t len) {
  return (len <= 1524) && buffer_available;
}

void ethernet_send_packet(const uint8_t *data, size_t len) {
  memcpy(tx_buffer, data, len);

  /*if (memcmp(dst, "\0\0\0\0\0\0", 6) == 0) {
    memcpy(dst+6, tud_network_mac_address, 6); // rewrite source address to be from our device :)
  }*/

  TxBuffer.buffer = tx_buffer;
  TxBuffer.len = len;
  TxConfig.TxBuffer = &TxBuffer;

  /*ETH_ConstructEthernetFrame(&frame, dest_mac, src_mac, type, payload, payload_len);
  TxBuffer.buffer = (uint8_t *)&frame;
  TxBuffer.len = sizeof(dest_mac) + sizeof(src_mac) + sizeof(type) + payload_len;
  TxBuffer.next = NULL;
  TxConfig.TxBuffer = &TxBuffer;*/

  buffer_available = false;

  HAL_ETH_Transmit_IT(&heth, &TxConfig);
  HAL_ETH_ReleaseTxPacket(&heth);
}
