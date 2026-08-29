#include "user_interface.h"

#include <math.h>
#include <stdio.h>

#include "cmsis_os.h"
#include "main.h"
#include "measurement_store.h"
#include "st7565/ST7565.h"
#include "task_health.h"
#include "usb_host.h"
#include "wifi.h"

/* Routes snprintf() to the bundled float-capable implementation; newlib-nano
 * would print nothing for %f. See measurement_format.c. */
#include "newlib_fixes/printf.h"

/** Readings older than this are shown as "--" rather than as live values. */
#define UI_STALE_AFTER_MS 10000U

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
        TaskHealth_ReportAlive(TASK_HEALTH_USER_INTERFACE, osKernelSysTick());

        MeasurementSnapshot measurement;
        Measurements_GetSnapshot(&measurement);

        char temperature_humidity[32];
        char co2_uv[32];
        char particles[32];
        char light[32];
        char keyboard[24];

        /* A sensor that has stopped reporting shows "--" instead of its last
         * value, so a stale reading is never mistaken for a live one. */
        if(Measurements_IsFresh(&measurement, MEASUREMENT_SOURCE_BME680,
                                UI_STALE_AFTER_MS) != 0U)
        {
            (void)snprintf(
                temperature_humidity,
                sizeof(temperature_humidity),
                "%.1f C %.1f%%",
                (double)measurement.bme680_temperature_c,
                (double)measurement.bme680_relative_humidity_percent);
        }
        else
        {
            (void)snprintf(temperature_humidity, sizeof(temperature_humidity),
                           "-- C --%%");
        }

        char co2_text[12];
        char uv_text[8];
        if(Measurements_IsFresh(&measurement, MEASUREMENT_SOURCE_SCD30,
                                UI_STALE_AFTER_MS) != 0U)
        {
            (void)snprintf(co2_text, sizeof(co2_text), "%ld",
                           (long)lroundf(measurement.scd30_co2_ppm));
        }
        else
        {
            (void)snprintf(co2_text, sizeof(co2_text), "--");
        }
        if(Measurements_IsFresh(&measurement, MEASUREMENT_SOURCE_UV,
                                UI_STALE_AFTER_MS) != 0U)
        {
            (void)snprintf(uv_text, sizeof(uv_text), "%u",
                           (unsigned int)measurement.uv_raw);
        }
        else
        {
            (void)snprintf(uv_text, sizeof(uv_text), "--");
        }
        (void)snprintf(
            co2_uv, sizeof(co2_uv), "%s ppm UV %s", co2_text, uv_text);

        if(Measurements_IsFresh(&measurement, MEASUREMENT_SOURCE_PARTICLE,
                                UI_STALE_AFTER_MS) != 0U)
        {
            (void)snprintf(
                particles,
                sizeof(particles),
                "PM %u/%u ugm3",
                (unsigned int)measurement.pm25_ug_m3,
                (unsigned int)measurement.pm10_ug_m3);
        }
        else
        {
            (void)snprintf(particles, sizeof(particles), "PM --/-- ugm3");
        }

        if(Measurements_IsFresh(&measurement, MEASUREMENT_SOURCE_LIGHT,
                                UI_STALE_AFTER_MS) != 0U)
        {
            (void)snprintf(
                light,
                sizeof(light),
                "%u lux %u K",
                (unsigned int)measurement.lux,
                (unsigned int)measurement.color_temperature_k);
        }
        else
        {
            (void)snprintf(light, sizeof(light), "-- lux -- K");
        }
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
