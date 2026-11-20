#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * Local deployments may override these safe defaults in app_config.local.h.
 * Defining APP_CONFIG_IGNORE_LOCAL skips that file so a build (notably the CI
 * Wi-Fi configuration) gets exactly the settings passed on the command line,
 * whatever happens to be in the working tree.
 */
#if defined(__has_include) && !defined(APP_CONFIG_IGNORE_LOCAL)
#  if __has_include("app_config.local.h")
#    include "app_config.local.h"
#  endif
#endif

/* Master default: set to 0, then explicitly enable only attached devices.
 * These are build-time variables; rebuild after changing them. */
#ifndef APP_HARDWARE_ENABLED
#define APP_HARDWARE_ENABLED 1
#endif
#ifndef APP_VEML6070_ENABLED
#define APP_VEML6070_ENABLED APP_HARDWARE_ENABLED
#endif
#ifndef APP_TCS34725_ENABLED
#define APP_TCS34725_ENABLED APP_HARDWARE_ENABLED
#endif
#ifndef APP_SCD30_ENABLED
#define APP_SCD30_ENABLED APP_HARDWARE_ENABLED
#endif
#ifndef APP_BME680_ENABLED
#define APP_BME680_ENABLED APP_HARDWARE_ENABLED
#endif
#ifndef APP_BSEC_ENABLED
#define APP_BSEC_ENABLED 0
#endif
#ifndef APP_BSEC_HEAT_SOURCE_C
#define APP_BSEC_HEAT_SOURCE_C 0.0F
#endif
#ifndef APP_BSEC_STATE_SAVE_INTERVAL_MS
#define APP_BSEC_STATE_SAVE_INTERVAL_MS 21600000U
#endif
#if APP_BSEC_ENABLED && !APP_BME680_ENABLED
#error "BSEC requires BME680 to be enabled"
#endif
#ifndef APP_HPMA_ENABLED
#define APP_HPMA_ENABLED APP_HARDWARE_ENABLED
#endif
#ifndef APP_LCD_ENABLED
#define APP_LCD_ENABLED APP_HARDWARE_ENABLED
#endif
#ifndef APP_USB_HOST_ENABLED
#define APP_USB_HOST_ENABLED APP_HARDWARE_ENABLED
#endif
#ifndef APP_HOST_UART_ENABLED
#define APP_HOST_UART_ENABLED APP_HARDWARE_ENABLED
#endif
#ifndef APP_ADC_ENABLED
#define APP_ADC_ENABLED APP_HARDWARE_ENABLED
#endif
#ifndef APP_I2C3_ENABLED
#define APP_I2C3_ENABLED APP_HARDWARE_ENABLED
#endif
#ifndef APP_I2C1_INTERNAL_PULLUPS
#define APP_I2C1_INTERNAL_PULLUPS 0
#endif
#ifndef APP_I2C1_TIMING
#define APP_I2C1_TIMING 0x00909BEBU
#endif
#ifndef APP_WATCHDOG_ENABLED
#define APP_WATCHDOG_ENABLED APP_HARDWARE_ENABLED
#endif

#define APP_I2C1_ENABLED (APP_VEML6070_ENABLED || APP_TCS34725_ENABLED || APP_SCD30_ENABLED)
#define APP_SENSORS_TASK_ENABLED (APP_VEML6070_ENABLED || APP_SCD30_ENABLED || APP_HPMA_ENABLED)
#define APP_BME_LIGHT_TASK_ENABLED (APP_BME680_ENABLED || APP_TCS34725_ENABLED)

#ifndef APP_WIFI_ENABLED
#define APP_WIFI_ENABLED 0
#endif
#ifndef APP_WIFI_SSID
#define APP_WIFI_SSID ""
#endif
#ifndef APP_WIFI_PASSWORD
#define APP_WIFI_PASSWORD ""
#endif
#ifndef APP_SERVER_ADDRESS
#define APP_SERVER_ADDRESS "192.0.2.1"
#endif
#ifndef APP_SERVER_PORT
#define APP_SERVER_PORT 2005U
#endif
#ifndef APP_ALLOW_REMOTE_RESET
#define APP_ALLOW_REMOTE_RESET 0
#endif
#ifndef APP_MEASUREMENT_SEND_INTERVAL_MS
#define APP_MEASUREMENT_SEND_INTERVAL_MS 1500U
#endif
#ifndef APP_SCD30_AMBIENT_PRESSURE_MBAR
#define APP_SCD30_AMBIENT_PRESSURE_MBAR 0U
#endif
#ifndef APP_SCD30_RECOVERY_ENABLED
#define APP_SCD30_RECOVERY_ENABLED 0
#endif

#endif /* APP_CONFIG_H */
