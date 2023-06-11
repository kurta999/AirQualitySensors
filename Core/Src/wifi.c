/*
 * wifi.c
 *
 *  Created on: Jun 7, 2021
 *      Author: Ati
 */

#include "wifi.h"
#include "device_specific.h"
#include "main.h"
#include <ESP32/esp32.h>
#include "user_interface.h"
#include "newlib_fixes/printf.h"

#include <stdio.h>

Esp32Handle esp32;
static uint8_t wifi_tx_buffer[512];  /**< WiFi TX buffer */
static char wifi_version_info[255]; /**< Store ESP AT firmware information */

extern MeasData_t m;

int is_wifi_inited = 0;
void DoSend(uint8_t link_id, uint32_t length)
{
  //DEBUG_VOLATILE uint32_t ret = 0xFFFFF;
  Esp32_SendTcp(&esp32, link_id, wifi_tx_buffer, length);
  Esp32_DisconnectFromServer(&esp32, link_id);
#if 0
  if(link_id == 0)  /* link_id can be 0 in case of no TCP connection */
    esp32.link[0] = kLinkTypeLocal; /* restore link type to local to avoid bugs */
#endif
}

/**
 * @brief Setup WiFi connection
 */
static void Wifi_Setup(void)
{
  char ap_ip[32], ap_mac[32], station_ip[32], station_mac[32];
  HAL_GPIO_WritePin(WIFI_EN_GPIO_Port, WIFI_EN_Pin, 0);
  volatile HAL_StatusTypeDef HAL_Status = HAL_ERROR;
  HAL_Status = Esp32_Reset(&esp32, 0);
  switch(HAL_Status)
  {
    case HAL_OK:
      HAL_Status = Esp32_ToggleAutoConn(&esp32, 0);  /* ESP8266 always return "ERROR" */
      HAL_Status = Esp32_GetVersionInfo(&esp32, wifi_version_info, sizeof(wifi_version_info));
      break;
    default:
      osDelay(1);
      break;
  }
  if(HAL_Status == HAL_OK)
    HAL_Status = Esp32_ToggleMultiConnection(&esp32, 1);  /* This function returns error so skip OK cheking afterwards */
  HAL_Status = Esp32_SetWifiMode(&esp32, kWiFiSoftApStation);
  if(HAL_Status == HAL_OK)
    HAL_Status = Esp32_JoinAccesssPoint(&esp32, "TP-Link_1B87", "58397633", NULL, 1);
  if(HAL_Status == HAL_OK)
  {

  }
  else
  {
    //wifi_conn_failed = 1;
  }

	HAL_Status = Esp32_SetServerMaxConnection(&esp32, 3);  /* This function returns error so skip OK cheking afterwards */
  HAL_Status = Esp32_CreateDeleteServer(&esp32, 1, 80, 0, 0);
  if(HAL_Status == HAL_OK)
    HAL_Status = Esp32_GetLocalIpAddress(&esp32, ap_ip, ap_mac, station_ip, station_mac);
  if(HAL_Status == HAL_OK)
  {
    for(int i = 0; i != 5; i++)
    {
      osDelay(30);
      HAL_GPIO_WritePin(ULED_1_GPIO_Port, ULED_1_Pin, 1);
      osDelay(30);
      HAL_GPIO_WritePin(ULED_1_GPIO_Port, ULED_1_Pin, 0);
    }
    is_wifi_inited = 1;
  }
  else
    Error_Handler();
}

/**
 * @brief Initialize WiFi module
 */
void Wifi_Init(void)
{
  Esp32_Init(&esp32, USART3, DMA1, DMA1_Channel3, LL_DMA_CHANNEL_3,
		  DMA1, DMA1_Channel2, LL_DMA_CHANNEL_2, WIFI_EN_GPIO_Port, WIFI_EN_Pin, NULL, 0, 115200);
}

void Esp32_RxCpltCallback(Esp32Handle* handle, uint8_t link_id, uint8_t *data, uint16_t length)
{
  HAL_GPIO_WritePin(ULED_1_GPIO_Port, ULED_1_Pin, 1);
  osDelay(10);
  HAL_GPIO_WritePin(ULED_1_GPIO_Port, ULED_1_Pin, 0);
  float temp, hum;
  int send_interval, co2, voc, pm25, pm10, lux, cct;

  int ret = sscanf((char*)data, "%d|%f,%f,%d,%d,%d,%d,%d,%d,%*d,%*d,%*d", &send_interval, &temp, &hum, &co2, &voc, &pm25, &pm10, &lux, &cct);
  if(ret == 9)
  {
    m.temp = temp;
    m.hum = hum;
    m.co2 = co2;
    m.voc = voc;
    m.pm25 = pm25;
    m.pm10 = pm10;
    m.lux = lux;
    m.cct = cct;
  }
  Esp32_DisconnectFromServer(&esp32, link_id);
}

uint32_t last_data_sent = 0;

extern float co2_ppm, temperature, relative_humidity;
extern float bme680_temp, bme680_hum, bme680_pressure;
extern uint16_t pm25, pm10;
extern uint16_t uv_val;
extern float red, green, blue;
extern uint16_t colorTemp, lux;
extern uint32_t bme680_gas_resistance, bme680_timestamp;

char send_buf[192];

void WifiTask(void const * argument)
{
  /* USER CODE BEGIN WifiTask */
  Wifi_Setup();
  /* Infinite loop */
  for(;;)
  {
    Esp32_Task(&esp32);

    if(is_wifi_inited)
    {
      if(osKernelSysTick() - last_data_sent > 1500)
      {
        HAL_StatusTypeDef HAL_Status = Esp32_ConnectToServer(&esp32, 4, "TCP", "192.168.0.120", 2005, 0, 0xFF, 0);
        if(HAL_Status == HAL_OK)
        {
            int co = 0;
          uint8_t len = snprintf(send_buf, sizeof(send_buf), "MEAS_DATA SCD30: %.4f,%.4f,%.4f CO: %d BME680: %.4f,%.4f,%.4f,%d,%d HONEYWELL: %d,%d VEML6070: %d TCS: %.4f,%.4f,%.4f,%d,%d\n",
              temperature, relative_humidity, co2_ppm, co, bme680_temp, bme680_hum, bme680_pressure, bme680_gas_resistance, bme680_timestamp, pm25, pm10, uv_val, red, green, blue, colorTemp, lux);

          HAL_Status = Esp32_SendTcp(&esp32, 4, (uint8_t*)send_buf, len);
        }
        if(HAL_Status == HAL_OK)
          HAL_Status = Esp32_DisconnectFromServer(&esp32, 4);
        if(HAL_Status != HAL_OK)
        {
          for(int i = 0; i != 2; i++)
          {
            HAL_GPIO_WritePin(ULED_3_GPIO_Port, ULED_3_Pin, 1);
            osDelay(100);
            HAL_GPIO_WritePin(ULED_3_GPIO_Port, ULED_3_Pin, 0);
            osDelay(100);
          }
        }
        HAL_Status = Esp32_DisconnectFromServer(&esp32, 4);

        last_data_sent = osKernelSysTick();
      }
    }
    osDelay(1);
  }
  /* USER CODE END WifiTask */
}


void inline Wifi_DataReceived(uint8_t* data, uint16_t len)
{
  Esp32_InsertDataToBuffer(&esp32, data, len);
}


void Wifi_SysTick(void)
{
  Esp32_SysTick(&esp32);
}

void Wifi_10MsIsr(void)
{
  Esp32_10Ms_Isr(&esp32);
}

