#include "wifi.h"

#include <stddef.h>
#include <string.h>

#include "cmsis_os.h"
#include "main.h"
#include "measurement_format.h"
#include "measurement_store.h"

static Esp32Handle esp32;
static volatile uint8_t wifi_ready;
static uint8_t failed_send_count;
static volatile uint8_t force_send;
static uint32_t last_data_sent;

static uint8_t at_argument_is_safe(const char* value, size_t capacity)
{
    if(value == NULL || capacity < 2U)
    {
        return 0U;
    }

    for(size_t index = 0U; index < capacity; ++index)
    {
        const unsigned char character = (unsigned char)value[index];
        if(character == '\0')
        {
            return index > 0U ? 1U : 0U;
        }
        if(character < 0x20U || character > 0x7EU ||
           character == '"' || character == '\\')
        {
            return 0U;
        }
    }
    return 0U;
}

static uint8_t payload_equals(const uint8_t* data, uint16_t length,
                              const char* command)
{
    const size_t command_length = strlen(command);
    while(length > 0U && (data[length - 1U] == '\r' || data[length - 1U] == '\n'))
    {
        --length;
    }
    return length == command_length && memcmp(data, command, command_length) == 0;
}

HAL_StatusTypeDef Wifi_Init(void)
{
    return ESP32_Init(
        &esp32,
        USART3,
        DMA1,
        DMA1_Channel3,
        LL_DMA_CHANNEL_3,
        DMA1,
        DMA1_Channel2,
        LL_DMA_CHANNEL_2,
        WIFI_EN_GPIO_Port,
        WIFI_EN_Pin,
        NULL,
        0U,
        115200U);
}

static HAL_StatusTypeDef wifi_setup(void)
{
    if(at_argument_is_safe(APP_WIFI_SSID, ESP32_MAX_AP_NAME_LEN) == 0U ||
       at_argument_is_safe(APP_WIFI_PASSWORD, ESP32_MAX_AP_PW_LEN) == 0U ||
       at_argument_is_safe(APP_SERVER_ADDRESS, 254U) == 0U)
    {
        return HAL_ERROR;
    }

    char access_point_ip[18] = {0};
    char access_point_mac[18] = {0};
    char station_ip[18] = {0};
    char station_mac[18] = {0};

    HAL_GPIO_WritePin(WIFI_EN_GPIO_Port, WIFI_EN_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = ESP32_Reset(&esp32, 0U);
    if(status == HAL_OK)
    {
        (void)ESP32_ToggleAutoConn(&esp32, 0U);
        status = ESP32_ToggleMultiConnection(&esp32, 1U);
    }
    if(status == HAL_OK)
    {
        status = ESP32_SetWifiMode(&esp32, kWiFiSoftApStation);
    }
    if(status == HAL_OK)
    {
        status = ESP32_JoinAccesssPoint(
            &esp32, APP_WIFI_SSID, APP_WIFI_PASSWORD, NULL, 1U);
    }
    if(status == HAL_OK)
    {
        (void)ESP32_SetServerMaxConnection(&esp32, 3U);
        status = ESP32_CreateDeleteServer(&esp32, 1U, 80U, 0U, 0U);
    }
    if(status == HAL_OK)
    {
        status = ESP32_GetLocalIpAddress(
            &esp32,
            access_point_ip,
            access_point_mac,
            station_ip,
            station_mac);
    }
    wifi_ready = status == HAL_OK ? 1U : 0U;
    return status;
}

void ESP32_RxCpltCallback(Esp32Handle* handle, uint8_t link_id,
                          uint8_t* data, uint16_t length)
{
    (void)handle;
    if(data == NULL)
    {
        return;
    }

    if(payload_equals(data, length, "SEND_SENSOR_DATA") != 0U)
    {
        force_send = 1U;
    }
#if APP_ALLOW_REMOTE_RESET
    else if(payload_equals(data, length, "RESET") != 0U)
    {
        HAL_NVIC_SystemReset();
    }
#endif
    (void)ESP32_DisconnectFromServer(&esp32, link_id);
}

void WifiTask(void const* argument)
{
    (void)argument;

    while(wifi_setup() != HAL_OK)
    {
        osDelay(5000U);
    }

    for(;;)
    {
        ESP32_Task(&esp32);
        const uint32_t now = osKernelSysTick();
        if(wifi_ready != 0U &&
           ((now - last_data_sent) >= APP_MEASUREMENT_SEND_INTERVAL_MS ||
            force_send != 0U))
        {
            force_send = 0U;
            HAL_StatusTypeDef status = ESP32_ConnectToServer(
                &esp32,
                4U,
                "TCP",
                APP_SERVER_ADDRESS,
                APP_SERVER_PORT,
                0U,
                0xFFU);
            if(status == HAL_OK)
            {
                MeasurementSnapshot measurement;
                char output[256];
                Measurements_GetSnapshot(&measurement);
                const int length =
                    Measurements_Format(output, sizeof(output), &measurement);
                if(length > 0)
                {
                    status = ESP32_SendTcp(
                        &esp32, 4U, (uint8_t*)output, (uint16_t)length);
                }
                else
                {
                    status = HAL_ERROR;
                }
                (void)ESP32_DisconnectFromServer(&esp32, 4U);
            }

            if(status == HAL_OK)
            {
                failed_send_count = 0U;
                HAL_GPIO_TogglePin(ULED_3_GPIO_Port, ULED_3_Pin);
            }
            else if(failed_send_count < UINT8_MAX)
            {
                ++failed_send_count;
            }
            last_data_sent = now;
        }
        osDelay(1U);
    }
}

void Wifi_DataReceived(const uint8_t* data, uint16_t length)
{
    if(data != NULL && length != 0U)
    {
        ESP32_InsertDataToBuffer(&esp32, data, length);
    }
}

void Wifi_SysTick(void)
{
    ESP32_SysTick(&esp32);
}

void Wifi_10MsIsr(void)
{
    ESP32_10Ms_Isr(&esp32);
}

uint8_t Wifi_IsReady(void)
{
    return wifi_ready;
}

void ESP32_OnWifiConnectioStateChange(Esp32Handle* handle, uint8_t is_lost)
{
    (void)handle;
    wifi_ready = is_lost == 0U ? 1U : 0U;
}

void ESP32_OnReconnectFailed(Esp32Handle* handle)
{
    (void)handle;
    wifi_ready = 0U;
}

void ESP32_OnUnexpectedRestart(Esp32Handle* handle)
{
    (void)handle;
    wifi_ready = 0U;
}

void ESP32_OnBaudrateError(Esp32Handle* handle)
{
    (void)handle;
    wifi_ready = 0U;
}
