#include "user_interface.h"

#include <math.h>
#include <stdio.h>

#include "cmsis_os.h"
#include "main.h"
#include "measurement_store.h"
#include "st7565/ST7565.h"
#include "usb_host.h"
#include "wifi.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim3;
extern osMutexId lcd_mutexHandle;
extern ApplicationTypeDef Appli_state;

ST7565Handle hlcd;

void UI_Init(void)
{
    ST7565_Init(
        &hlcd,
        &hspi1,
        &lcd_mutexHandle,
        LCD_CS_GPIO_Port,
        LCD_CS_Pin,
        LCD_RES_GPIO_Port,
        LCD_RES_Pin,
        LCD_A0_GPIO_Port,
        LCD_A0_Pin,
        35U,
        0U);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 255U);
}

static const char* keyboard_state_text(ApplicationTypeDef state)
{
    switch(state)
    {
        case APPLICATION_IDLE:
            return "idle";
        case APPLICATION_START:
            return "start";
        case APPLICATION_READY:
            return "ready";
        case APPLICATION_DISCONNECT:
            return "dis";
        default:
            return "unknown";
    }
}

void UserInterfaceTask(void const* argument)
{
    (void)argument;

    for(;;)
    {
        MeasurementSnapshot measurement;
        Measurements_GetSnapshot(&measurement);

        char temperature_humidity[32];
        char co2_uv[32];
        char particles[32];
        char light[32];
        char keyboard[24];
        (void)snprintf(
            temperature_humidity,
            sizeof(temperature_humidity),
            "%.1f C %.1f%%",
            (double)measurement.bme680_temperature_c,
            (double)measurement.bme680_relative_humidity_percent);
        (void)snprintf(
            co2_uv,
            sizeof(co2_uv),
            "%ld ppm UV %u",
            (long)lroundf(measurement.scd30_co2_ppm),
            (unsigned int)measurement.uv_raw);
        (void)snprintf(
            particles,
            sizeof(particles),
            "PM %u/%u ugm3",
            (unsigned int)measurement.pm25_ug_m3,
            (unsigned int)measurement.pm10_ug_m3);
        (void)snprintf(
            light,
            sizeof(light),
            "%u lux %u K",
            (unsigned int)measurement.lux,
            (unsigned int)measurement.color_temperature_k);
        (void)snprintf(
            keyboard,
            sizeof(keyboard),
            "KBD %s",
            keyboard_state_text(Appli_state));

        ST7565_ClearBuffer(&hlcd);
        ST7565_DrawString(&hlcd, 0U, 0U, temperature_humidity);
        ST7565_DrawString(&hlcd, 0U, 1U, co2_uv);
        ST7565_DrawString(&hlcd, 0U, 2U, particles);
        ST7565_DrawString(&hlcd, 0U, 3U, light);
        ST7565_DrawString(
            &hlcd, 0U, 6U, Wifi_IsReady() != 0U ? "WiFi OK" : "WiFi off");
        ST7565_DrawString(&hlcd, 72U, 6U, keyboard);

        (void)ST7565_Task(&hlcd);
        osDelay(500U);
    }
}
