/*
 * user_interface.c
 *
 *  Created on: Jun 7, 2021
 *      Author: Ati
 */

#include "user_interface.h"

#include "device_specific.h"
#include "main.h"
#include "wifi.h"
#include <Consolas_regular_8pt/Consolas_regular_8pt.h>
#include <string.h>
#include <stdio.h>
#include "usb_host.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim3;
extern osMutexId lcd_mutexHandle;
extern IWDG_HandleTypeDef hiwdg;
extern int is_wifi_inited;
extern ApplicationTypeDef Appli_state;
MeasData_t m;

ST7565Handle hlcd;

/**
 * @brief Initialize the UI
 */
void UI_Init(void)
{
  ST7565_Init(&hlcd, &hspi1, &lcd_mutexHandle,
      LCD_CS_GPIO_Port, LCD_CS_Pin,
      LCD_RES_GPIO_Port, LCD_RES_Pin,
      LCD_A0_GPIO_Port, LCD_A0_Pin,
      35, 0);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 255);
  memset(&m, 0, sizeof(m));
}

/**
 * @brief User interface FreeRTOS task
 * @param argument Pointer to task argument
 */
void UserInterfaceTask(void const * argument)
{
  for(;;)
  {
    char str_temphum[32], str_co2voc[32], str_particle[32], str_light[32];
    snprintf(str_temphum, sizeof(str_temphum), "%.1f C, %.1f %%", m.temp, m.hum);
    snprintf(str_co2voc, sizeof(str_co2voc), "%d ppm, %d ppb", m.co2, m.voc);
    snprintf(str_particle, sizeof(str_particle), "%d ugm3, %d ugm3", m.pm25, m.pm10);
    snprintf(str_light, sizeof(str_light), "%d l, %d cct", m.lux, m.cct);

    ST7565_ClearBuffer(&hlcd);
    ST7565_DrawString(&hlcd, 0, 0, str_temphum, consolas_regular_8pt());
    ST7565_DrawString(&hlcd, 0, 10, str_co2voc, consolas_regular_8pt());
    ST7565_DrawString(&hlcd, 0, 20, str_particle, consolas_regular_8pt());
    ST7565_DrawString(&hlcd, 0, 30, str_light, consolas_regular_8pt());

    if(!is_wifi_inited)
      ST7565_DrawString(&hlcd, 0, 50, "WiFi ...", consolas_regular_8pt());
    else
      ST7565_DrawString(&hlcd, 0, 50, "WiFi OK", consolas_regular_8pt());

    char str[16];
    switch(Appli_state)
    {
      case APPLICATION_IDLE:
        strncpy(str, "idle", 5);
        break;
      case APPLICATION_START:
        strncpy(str, "start", 6);
        break;
      case APPLICATION_READY:
        strncpy(str, "ready", 6);
        break;
      case APPLICATION_DISCONNECT:
        strncpy(str, "dis", 4);
        break;
    }

    char kbd_str[32];
    snprintf(kbd_str, sizeof(kbd_str), "KBD %s", str);
    ST7565_DrawString(&hlcd, 60, 50, kbd_str, consolas_regular_8pt());

    if(ST7565_Task(&hlcd) != HAL_OK)
      osDelay(1);
    osDelay(500);
  }
}
