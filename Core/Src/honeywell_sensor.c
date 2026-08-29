#include "honeywell_sensor.h"

#include "FreeRTOS.h"
#include "hpma/hpma_parser.h"
#include "measurement_store.h"
#include "task.h"

extern UART_HandleTypeDef huart4;

static HpmaParser parser;
static uint8_t uart_byte;
/** Set while interrupt-driven reception is armed; cleared on any failure. */
static volatile uint8_t receive_armed;
/** Tick of the last successfully decoded frame. */
static volatile uint32_t last_frame_ms;

static HAL_StatusTypeDef arm_receive(void)
{
    const HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart4, &uart_byte, 1U);
    receive_armed = (status == HAL_OK) ? 1U : 0U;
    return status;
}

HAL_StatusTypeDef Honeywell_Init(void)
{
    static const uint8_t enable_auto_send[] = {0x68U, 0x01U, 0x40U, 0x57U};
    static const uint8_t start_measurement[] = {0x68U, 0x01U, 0x01U, 0x96U};

    /* Any early return below leaves this cleared, which is what makes
     * Honeywell_Poll() pick the sensor up again later. */
    receive_armed = 0U;
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
    return arm_receive();
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
        const uint32_t now = (uint32_t)xTaskGetTickCountFromISR();
        last_frame_ms = now;
        Measurements_UpdateParticleFromIsr(pm25, pm10, now);
    }

    /* A failure here clears receive_armed and Honeywell_Poll() rearms. */
    (void)arm_receive();
}

void Honeywell_UartError(UART_HandleTypeDef* huart)
{
    if(huart == NULL || huart->Instance != huart4.Instance)
    {
        return;
    }
    receive_armed = 0U;
}

void Honeywell_RequestRestart(void)
{
    receive_armed = 0U;
    last_frame_ms = 0U;
}

void Honeywell_Poll(uint32_t now_ms)
{
    if(receive_armed == 0U)
    {
        HpmaParser_Init(&parser);
        if(arm_receive() == HAL_OK)
        {
            /* Give the sensor a fresh silence window before restarting it. */
            last_frame_ms = now_ms;
        }
        return;
    }

    if((now_ms - last_frame_ms) < HONEYWELL_SILENCE_TIMEOUT_MS)
    {
        return;
    }

    /* Armed but silent: the sensor may have been unplugged or stopped
     * auto-sending. Drop the stale reading and reissue the start commands. */
    Measurements_Invalidate(MEASUREMENT_SOURCE_PARTICLE);
    (void)HAL_UART_AbortReceive(&huart4);
    receive_armed = 0U;
    (void)Honeywell_Init();
    last_frame_ms = now_ms;
}
