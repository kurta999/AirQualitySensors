/*
 * Copyright (C) Effman s.r.o. - All rights reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

/**
 * @file usart_driver.h
 * @author Attila Kiss <attila@effman.eu>
 * @date 2019. febr. 22.
 * @brief This file is responsible for USART driver with DMA
 */

#ifndef USART_DRIVER_H_
#define USART_DRIVER_H_

#include "device_specific.h"
#include "cmsis_os.h"

#define USART_DRIVER_SOURCE_ESP32       0
#define USART_DRIVER_SOURCE_M95         1

#define USART_DRIVER_DMA_RX             0
#define USART_DRIVER_DMA_TX             1

#define UART_SEND_TIMEOUT             300

typedef struct
{
  uint16_t old_dma_pos;
  osSemaphoreId tx_semaphore;
  uint8_t dma_buffer[1024];  /**< RX buffer for ESP32 RX DMA */
  USART_TypeDef* usart;
  DMA_TypeDef* rx_dma;
  DMA_Channel_TypeDef* rx_dma_channel;
  uint8_t rx_dma_channel_id;
  DMA_TypeDef* tx_dma;
  DMA_Channel_TypeDef* tx_dma_channel;
  uint8_t tx_dma_channel_id;
} UsartDriverHandle;  /**< USART Driver handle */

typedef struct
{
  uint8_t instance;
  UsartDriverHandle* driver_instance;
} UsartDriversHandle[6];

void UsartDriver_Init(void);
uint8_t UsartDriver_Install(UsartDriverHandle* handle, USART_TypeDef* usart,
    DMA_TypeDef* rx_dma, DMA_Channel_TypeDef* rx_dma_channel, uint8_t rx_dma_channel_id,
    DMA_TypeDef* tx_dma, DMA_Channel_TypeDef* tx_dma_channel, uint8_t tx_dma_channel_id);
void UsartDriver_Transmit(uint8_t source_type, char* data, uint16_t len);
void UsartDriver_UartIrqHandler(uint8_t source_type);
void UsartDriver_DmaIrqHandler(uint8_t source_type, uint8_t is_tx);
UsartDriverHandle* UsartDriver_GetDriver(uint8_t source_type);

#endif /* USART_DRIVER_H_ */
