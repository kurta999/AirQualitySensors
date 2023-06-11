#include "device_specific.h"

extern UART_HandleTypeDef huart4;

uint16_t pm25 = 0;
uint16_t pm10 = 0;
uint32_t last_particle_meas_recv = 0;

uint8_t rx_circular_buffer[512];
uint8_t uart_byte = 0;
uint16_t uart_recv_cnt = 0;

void Honeywell_Init()
{
    uint8_t data1[] = { 0x68, 0x01, 0x40, 0x57 }; // Enable Auto Send
    HAL_UART_Transmit(&huart4, data1, sizeof(data1), 100);

    uint8_t data2[] = { 0x68, 0x01, 0x01, 0x96 }; // Start Particle Measurement
    HAL_UART_Transmit(&huart4, data2, sizeof(data2), 100);


    HAL_UART_Receive_IT(&huart4, &uart_byte, 1);
}

void Honeywell_UartRecv(UART_HandleTypeDef* huart)
{
    if(huart->Instance == huart4.Instance)
    {
        if(++uart_recv_cnt >= sizeof(rx_circular_buffer) || osKernelSysTick() - last_particle_meas_recv > 1000)
        {
            uart_recv_cnt = 0;
            memset(rx_circular_buffer, 0, sizeof(rx_circular_buffer));
            last_particle_meas_recv = osKernelSysTick();
        }
        rx_circular_buffer[uart_recv_cnt] = uart_byte;

        if(uart_recv_cnt >= 12)
        {
            for(int i = 0; i != sizeof(rx_circular_buffer); i++)
            {
                if(rx_circular_buffer[i] == 'B' && uart_recv_cnt > i + 8)
                {
                    pm25 = rx_circular_buffer[i + 6] * 256 + rx_circular_buffer[i + 7];
                    pm10 = rx_circular_buffer[i + 8] * 256 + rx_circular_buffer[i + 9];
                    uart_recv_cnt = 0;
                    memset(rx_circular_buffer, 0, sizeof(rx_circular_buffer));
                    break;
                }
            }
        }
        HAL_UART_Receive_IT(&huart4, &uart_byte, 1);
    }
}