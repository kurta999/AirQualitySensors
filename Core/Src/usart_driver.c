#include "usart_driver.h"
#include "device_specific.h"
#include "cmsis_os.h"
#include "stm32l4xx_ll_usart.h"
#include "main.h"
#include <string.h>
#include "wifi.h"

/* Private macro -------------------------------------------------------------*/
#define USART_DRIVER_SOURCE_ESP32      0

/* Private variables ---------------------------------------------------------*/
static osMessageQId usart_rx_dma_queue_id;  /**< Queue for special task which processes data received from UART */
#if M95_USE_UART_RX_DMA == 1
static osMessageQId usart_rx_dma_queue_id2;  /**< Queue for special task which processes data received from UART */
#endif
static UsartDriversHandle drivers;  /**< USART drivers map */
static uint8_t usart_driver_counter;  /**< Internal counter for USART drivers */
static const uint8_t dma_channel_offsets[] =
{
  (uint8_t)(DMA1_Channel1_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel2_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel3_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel4_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel5_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel6_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel7_BASE - DMA1_BASE)
};
__STATIC_INLINE uint32_t UD_DMA_IsEnabledIT_TC(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return ((READ_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + dma_channel_offsets[Channel - 1U])))->CCR,
                   DMA_CCR_TCIE) == (DMA_CCR_TCIE)) ? 1UL : 0UL);
}
__STATIC_INLINE uint32_t UD_DMA_IsEnabledIT_HT(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return ((READ_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + dma_channel_offsets[Channel - 1U])))->CCR,
                   DMA_CCR_HTIE) == (DMA_CCR_HTIE)) ? 1UL : 0UL);
}
__STATIC_INLINE uint32_t UD_DMA_IsEnabledIT_TE(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return ((READ_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + dma_channel_offsets[Channel - 1U])))->CCR,
                   DMA_CCR_TEIE) == (DMA_CCR_TEIE)) ? 1UL : 0UL);
}
__STATIC_INLINE void UD_DMA_EnableIT_TC(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel - 1U])))->CCR, DMA_CCR_TCIE);
}
__STATIC_INLINE void UD_DMA_EnableIT_HT(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel - 1U])))->CCR, DMA_CCR_HTIE);
}
__STATIC_INLINE void UD_DMA_EnableIT_TE(DMA_TypeDef *DMAx, uint32_t Channel)
{
  SET_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel - 1U])))->CCR, DMA_CCR_TEIE);
}
__STATIC_INLINE void UD_DMA_DisableIT_TC(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel - 1U])))->CCR, DMA_CCR_TCIE);
}
__STATIC_INLINE void UD_DMA_DisableIT_HT(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel - 1U])))->CCR, DMA_CCR_HTIE);
}
__STATIC_INLINE void UD_DMA_DisableIT_TE(DMA_TypeDef *DMAx, uint32_t Channel)
{
  CLEAR_BIT(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel - 1U])))->CCR, DMA_CCR_TEIE);
}


UsartDriverHandle esp32_usart;  /**< USART driver for ESP32 */
#if M95_USE_UART_RX_DMA == 1
UsartDriverHandle m95_usart;  /**< USART driver for M95 */
#endif

uint8_t UsartDriver_Install(UsartDriverHandle* handle, USART_TypeDef* usart,
    DMA_TypeDef* rx_dma, DMA_Channel_TypeDef* rx_dma_channel, uint8_t rx_dma_channel_id,
    DMA_TypeDef* tx_dma, DMA_Channel_TypeDef* tx_dma_channel, uint8_t tx_dma_channel_id)
{
#ifdef DEBUG
  assert(usart_driver_counter < ARRAY_LEN(drivers));
#endif
  handle->usart = usart;
  handle->rx_dma = rx_dma;
  handle->rx_dma_channel = rx_dma_channel;
  handle->rx_dma_channel_id = rx_dma_channel_id;
  handle->tx_dma = tx_dma;
  handle->tx_dma_channel = tx_dma_channel;
  handle->tx_dma_channel_id = tx_dma_channel_id;
  osSemaphoreDef(uart_mutex);
  handle->tx_semaphore = osSemaphoreCreate(osSemaphore(uart_mutex), 1);
  memset(handle->dma_buffer, 0, sizeof(handle->dma_buffer));
  drivers[usart_driver_counter].instance = usart_driver_counter;
  drivers[usart_driver_counter].driver_instance = handle;
  uint8_t ret = usart_driver_counter;
  usart_driver_counter++;
  __disable_irq();
  LL_DMA_SetPeriphAddress(rx_dma, rx_dma_channel_id, (uint32_t)&usart->RDR);
  LL_DMA_SetMemoryAddress(rx_dma, rx_dma_channel_id, (uint32_t)handle->dma_buffer);
  LL_DMA_SetDataLength(rx_dma, rx_dma_channel_id, ARRAY_LEN(handle->dma_buffer));
  LL_DMA_SetMode(rx_dma, rx_dma_channel_id, LL_DMA_MODE_CIRCULAR);
  UD_DMA_DisableIT_HT(rx_dma, rx_dma_channel_id);
  UD_DMA_EnableIT_TC(rx_dma, rx_dma_channel_id);
  LL_USART_EnableDMAReq_RX(usart);
  LL_USART_EnableIT_IDLE(usart);
  LL_DMA_EnableChannel(rx_dma, rx_dma_channel_id);
  LL_USART_Enable(usart);
  __enable_irq();
  return ret;
}


// --------------------------
// ---  private functions ---
// --------------------------

/**
 * @brief Notifies the USART RXDMA task that data was received
 * @param source_type Source type of received data (ESP32 or M95)
 */
static void UsartDma_Insert(uint8_t source_type)
{
  if(source_type == 0)
    osMessagePut(usart_rx_dma_queue_id, source_type, 0);
#if M95_USE_UART_RX_DMA == 1
  else if(source_type == 1)
    osMessagePut(usart_rx_dma_queue_id2, source_type, 0);
#endif
}

// ------------------------
// --- public functions ---
// ------------------------

/**
 * @brief Initialize USART DXDMA module
 * @details more_detailed_description
 * @param input
 * @return return_values
 */
void UsartDriver_Init(void)
{
  osMessageQDef(usart_rx_dma, 10, sizeof(void *));
  usart_rx_dma_queue_id = osMessageCreate(osMessageQ(usart_rx_dma), NULL);
#if M95_USE_UART_RX_DMA == 1
  osMessageQDef(usart_rx_dma2, 10, sizeof(void *));
  usart_rx_dma_queue_id2 = osMessageCreate(osMessageQ(usart_rx_dma2), NULL);
#endif
}

/**
 * @brief Transmit data via UART
 * @details more_detailed_description
 * @param source_type Usart Driver source type
 * @param data Data to transmit
 * @param len Length of data
 * @return return_values
 */
void UsartDriver_Transmit(uint8_t source_type, char* data, uint16_t len)
{
#ifdef DEBUG
  assert(source_type < ARRAY_LEN(drivers));
#endif
  USART_TypeDef* usart = drivers[source_type].driver_instance->usart;
  DMA_TypeDef* tx_dma = drivers[source_type].driver_instance->tx_dma;
  DMA_Channel_TypeDef* tx_dma_channel = drivers[source_type].driver_instance->tx_dma_channel;
  uint8_t tx_dma_channel_id = drivers[source_type].driver_instance->tx_dma_channel_id;
  if(osSemaphoreWait(drivers[source_type].driver_instance->tx_semaphore, UART_SEND_TIMEOUT) == osOK)
  {
    __disable_irq();
    tx_dma_channel->CCR &= ~DMA_CCR_EN;
    tx_dma_channel->CCR = 0;
    LL_DMA_ClearFlag_GI2(tx_dma);
    LL_DMA_SetChannelPriorityLevel(tx_dma,  tx_dma_channel_id, LL_DMA_PRIORITY_HIGH);
    tx_dma_channel->CMAR =  (uint32_t)data;  /* Memory address */
    tx_dma_channel->CPAR =  (uint32_t)&usart->TDR;  /* Peripherial Address */
    tx_dma_channel->CNDTR =  (uint32_t)len;  /* Memory inc., Memory To Perip., TCIE, T. Error IE */
    tx_dma_channel->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE;
    tx_dma_channel->CCR &= ~DMA_CCR_HTIE;  /* Disable HT interrupt */
    tx_dma_channel->CCR |= DMA_CCR_EN;  /* Enable transmission */
    usart->ICR = USART_ICR_TCCF;  /* Transmission Complete Clear Flag */
    usart->CR3 |= USART_CR3_DMAT; /* DMA Enable Transmitter */
    __enable_irq();
  }
}

static void usart_process_data(UsartDriverHandle* handle, uint8_t* data, uint16_t len)
{
  if(handle == &esp32_usart)
  {
    Wifi_DataReceived(data, len);
  }
}

/**
 * @brief Check for new data received with DMA
 * @param handle Esp32 handle
 */
void UsartDriver_UsartRxCheck(UsartDriverHandle* handle)
{
  uint16_t pos = ARRAY_LEN(handle->dma_buffer) - LL_DMA_GetDataLength(handle->rx_dma, handle->rx_dma_channel_id);
  if(pos != handle->old_dma_pos) /* Check change in received data */
  {
    if(pos > handle->old_dma_pos)
    { /* Current position is over previous one */
      /* We are in "linear" mode */
      /* Process data directly by subtracting "pointers" */
      if(handle->old_dma_pos == 0 && pos == ARRAY_LEN(handle->dma_buffer))
        return;  /* CHECKIT */
      usart_process_data(handle, &handle->dma_buffer[handle->old_dma_pos], pos - handle->old_dma_pos);
    }
    else
    {
      /* We are in "overflow" mode - First process data to the end of buffer*/
      usart_process_data(handle, &handle->dma_buffer[handle->old_dma_pos], ARRAY_LEN(handle->dma_buffer) - handle->old_dma_pos);
      if(pos)  /* Continue from beginning of buffer */
        usart_process_data(handle, &handle->dma_buffer[0], pos);
    }
  }
  handle->old_dma_pos = pos;  /* Save current position as old */
  if(handle->old_dma_pos == ARRAY_LEN(handle->dma_buffer))
  {
    handle->old_dma_pos = 0;  /* Check and manually update if we reached end of buffer */
  }
}

/**
 * @brief ESP32 Uart Idle detected callback
 * @details I know It's a silly idea to give parameter "source_type", but it's still faster than using lookup it within usart
 * @param source_type UART handle
 */
void UsartDriver_UartIrqHandler(uint8_t source_type)
{
#ifdef DEBUG
  assert(source_type < ARRAY_LEN(drivers));
#endif
  USART_TypeDef* usart = drivers[source_type].driver_instance->usart;
  uint32_t isrflags   = usart->ISR;
  uint32_t cr1its     = usart->CR1;
  uint32_t cr3its     = usart->CR3;
  uint8_t err = 0;
  uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
  if(LL_USART_IsEnabledIT_IDLE(usart) && LL_USART_IsActiveFlag_IDLE(usart))
  {
    LL_USART_ClearFlag_IDLE(usart); /* Clear IDLE line flag */
    UsartDma_Insert(source_type); /* Write data to queue. Do not use wait function! */
  }

  if(LL_USART_IsActiveFlag_PE(usart))
	  LL_USART_ClearFlag_PE(usart);
  if(LL_USART_IsActiveFlag_FE(usart))
	  LL_USART_ClearFlag_FE(usart);

  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if (((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
  {
    usart->CR1 &= ~USART_CR1_TCIE;
    return;
  }

  /* If some errors occur */
#if defined(USART_CR1_FIFOEN)
  if ((errorflags != RESET)
      && ((((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != RESET)
           || ((cr1its & (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE)) != RESET))))
#else
  if ((errorflags != RESET)
      && (((cr3its & USART_CR3_EIE) != RESET)
          || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
#endif
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if (((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
    {
      usart->ICR |= UART_CLEAR_PEF;
      err = 1;
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      usart->ICR |= UART_CLEAR_FEF;
      err = 1;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      usart->ICR |= UART_CLEAR_NEF;
      err = 1;
    }
    if (((isrflags & USART_ISR_ORE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      usart->ICR |= UART_CLEAR_OREF;
      err = 1;
    }
    if(err)  /* If any error happend, release semaphore */
    {
      osSemaphoreRelease(drivers[source_type].driver_instance->tx_semaphore);
    }
  }
}

/**
 * @brief DMA IRQ handler for USART driver
 * @details more_detailed_description
 * @param source_type Which ID is assigned to driver
 * @param is_tx Is TX DMA IRQ call? (0 = rx, 1 = tx)
 */
void UsartDriver_DmaIrqHandler(uint8_t source_type, uint8_t is_tx)
{
#ifdef DEBUG
  assert(source_type < ARRAY_LEN(drivers));
#endif
  if(!is_tx)  /* RX */
  {
    DMA_TypeDef* rx_dma = drivers[source_type].driver_instance->rx_dma;
    uint8_t dma_channel_id = drivers[source_type].driver_instance->rx_dma_channel_id;
    UsartDma_Insert(source_type);
    if(UD_DMA_IsEnabledIT_TE(rx_dma, dma_channel_id) &&
        (rx_dma->ISR & (1 << (DMA_ISR_TEIF1_Pos + ((dma_channel_id-1) * 4)))))
    {
      rx_dma->IFCR |= 1 << (DMA_IFCR_CGIF1_Pos + ((dma_channel_id-1) * 4));  /* Clear global interrupt flag */
    }

    if(UD_DMA_IsEnabledIT_TC(rx_dma, dma_channel_id) &&  /* Check transfer-complete interrupt */
        (rx_dma->ISR & (1 << (DMA_ISR_TCIF1_Pos + ((dma_channel_id-1) * 4)))))
    {
      rx_dma->IFCR |= 1 << (DMA_IFCR_CTCIF1_Pos + ((dma_channel_id-1) * 4));  /* Clear transfer complete flag */
      rx_dma->IFCR |= 1 << (DMA_IFCR_CHTIF1_Pos + ((dma_channel_id-1) * 4));  /* Clear half transfer complete flag */
    }
  }
  else
  {
    USART_TypeDef* usart = drivers[source_type].driver_instance->usart;
    DMA_TypeDef* tx_dma = drivers[source_type].driver_instance->tx_dma;
    uint8_t dma_channel_id = drivers[source_type].driver_instance->tx_dma_channel_id;
    if(LL_DMA_IsEnabledIT_TE(tx_dma, dma_channel_id) &&
        (tx_dma->ISR & (1 << (DMA_ISR_TEIF1_Pos + ((dma_channel_id) * 4)))))  /* Is transmit error set? */
    {
      tx_dma->IFCR |= 1 << (DMA_IFCR_CGIF1_Pos + ((dma_channel_id) * 4));  /* Clear global interrupt flag */
    }
    if(LL_DMA_IsEnabledIT_TC(tx_dma, dma_channel_id) &&
        (tx_dma->ISR & (1 << (DMA_ISR_TCIF1_Pos + ((dma_channel_id) * 4)))))
    {  /* (tx_dma->ISR & (1 << (DMA_ISR_GIF1_Pos + ((dma_channel_id-1) * 4)))) */
      tx_dma->IFCR |= 1 << (DMA_IFCR_CTCIF1_Pos + ((dma_channel_id) * 4));
      tx_dma->IFCR |= 1 << (DMA_IFCR_CHTIF1_Pos + ((dma_channel_id) * 4));
      tx_dma->IFCR |= 1 << (DMA_IFCR_CTEIF1_Pos + ((dma_channel_id) * 4));
      __disable_irq();
      LL_DMA_DisableIT_TC(tx_dma, dma_channel_id);
      LL_DMA_DisableIT_HT(tx_dma, dma_channel_id);
      usart->CR3 &= ~USART_CR3_DMAT;  /* Disable the DMA transfer for the receiver request */
      __enable_irq();
      osSemaphoreRelease(drivers[source_type].driver_instance->tx_semaphore);
    }
  }
}

/**
 * @brief USART DMA check thread
 * @param[in] arg: Thread argument
 */
void UsartDriver_Task(void const* arg)
{
  osEvent evt;
  while(1)
  {
    evt = osMessageGet(usart_rx_dma_queue_id, osWaitForever);  /* Block thread and wait for event to process USART data */
    uint32_t prim;
    prim = __get_PRIMASK();
    __disable_irq();
    UsartDriver_UsartRxCheck(drivers[0].driver_instance);
    if (!prim)
      __enable_irq();
    (void)evt;
  }
}

#if M95_USE_UART_RX_DMA == 1
/**
 * @brief USART DMA check thread
 * @param[in] arg: Thread argument
 */
void UsartDriver_Task2(void const* arg)
{
  osEvent evt;
  while(1)
  {
    evt = osMessageGet(usart_rx_dma_queue_id2, osWaitForever);  /* Block thread and wait for event to process USART data */
    uint32_t prim;
    prim = __get_PRIMASK();
    __disable_irq();
    UsartDriver_UsartRxCheck(drivers[1].driver_instance);
    if (!prim)
      __enable_irq();
    (void)evt;
  }
}
#endif

UsartDriverHandle* UsartDriver_GetDriver(uint8_t source_type)
{
#ifdef DEBUG
  assert(source_type < ARRAY_LEN(drivers));
#endif

  if(source_type == USART_DRIVER_SOURCE_ESP32)
    return &esp32_usart;
#if M95_USE_UART_RX_DMA == 1
  else if(source_type == USART_DRIVER_SOURCE_M95)
    return &m95_usart;
#endif
  return NULL; //drivers[source_type].driver_instance;
}
