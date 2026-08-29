#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include <stdint.h>

#include "cmsis_os.h"
#include "device_specific.h"

#define USART_DRIVER_SOURCE_ESP32 0U
#define USART_DRIVER_DMA_RX       0U
#define USART_DRIVER_DMA_TX       1U
#define UART_SEND_TIMEOUT         300U

typedef struct
{
    uint16_t old_dma_pos;
    osSemaphoreId tx_semaphore;
    /**
     * Result of the last DMA transmission, written by the DMA ISR and read by
     * UsartDriver_Transmit() after the completion semaphore is taken. The
     * semaphore alone cannot carry this: the error and transfer-complete
     * interrupts both release it, so without a separate flag a failed
     * transfer is indistinguishable from a successful one.
     */
    volatile uint8_t tx_error;
    uint8_t dma_buffer[1024];
    USART_TypeDef* usart;
    DMA_TypeDef* rx_dma;
    DMA_Channel_TypeDef* rx_dma_channel;
    uint8_t rx_dma_channel_id;
    DMA_TypeDef* tx_dma;
    DMA_Channel_TypeDef* tx_dma_channel;
    uint8_t tx_dma_channel_id;
} UsartDriverHandle;

typedef struct
{
    uint8_t instance;
    UsartDriverHandle* driver_instance;
} UsartDriverEntry;

void UsartDriver_Init(void);
uint8_t UsartDriver_Install(
    UsartDriverHandle* handle,
    USART_TypeDef* usart,
    DMA_TypeDef* rx_dma,
    DMA_Channel_TypeDef* rx_dma_channel,
    uint8_t rx_dma_channel_id,
    DMA_TypeDef* tx_dma,
    DMA_Channel_TypeDef* tx_dma_channel,
    uint8_t tx_dma_channel_id);
HAL_StatusTypeDef UsartDriver_Transmit(uint8_t source_type, const char* data, uint16_t length);
void UsartDriver_UartIrqHandler(uint8_t source_type);
void UsartDriver_DmaIrqHandler(uint8_t source_type, uint8_t is_tx);
void UsartDriver_Task(void const* argument);
UsartDriverHandle* UsartDriver_GetDriver(uint8_t source_type);

#endif /* USART_DRIVER_H */
