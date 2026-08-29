#include "honeywell_sensor.h"

#include "FreeRTOS.h"
#include "task.h"
#include "hpma/hpma_parser.h"
#include "measurement_store.h"

extern UART_HandleTypeDef huart4;

static HpmaParser parser;
static uint8_t uart_byte;

HAL_StatusTypeDef Honeywell_Init(void)
{
    static const uint8_t enable_auto_send[] = {0x68U, 0x01U, 0x40U, 0x57U};
    static const uint8_t start_measurement[] = {0x68U, 0x01U, 0x01U, 0x96U};

    HpmaParser_Init(&parser);
    if(HAL_UART_Transmit(&huart4, (uint8_t*)enable_auto_send,
                         sizeof(enable_auto_send), 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if(HAL_UART_Transmit(&huart4, (uint8_t*)start_measurement,
                         sizeof(start_measurement), 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_IT(&huart4, &uart_byte, 1U);
}

void Honeywell_UartRecv(UART_HandleTypeDef* huart)
{
    if(huart == NULL || huart->Instance != huart4.Instance)
    {
        return;
    }

    uint16_t pm25 = 0U;
    uint16_t pm10 = 0U;
    if(HpmaParser_Push(&parser, uart_byte, &pm25, &pm10) != 0U)
    {
        Measurements_UpdateParticleFromIsr(pm25, pm10, xTaskGetTickCountFromISR());
    }
    (void)HAL_UART_Receive_IT(&huart4, &uart_byte, 1U);
}
