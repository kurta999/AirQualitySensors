#ifndef WIFI_H
#define WIFI_H

#include <stdint.h>

#include "ESP32/esp32.h"
#include "app_config.h"

#define MAX_WIFI_RETRIES 10U

HAL_StatusTypeDef Wifi_Init(void);
void WifiTask(void const* argument);
void Wifi_DataReceived(const uint8_t* data, uint16_t length);
void Wifi_10MsIsr(void);
void Wifi_SysTick(void);
uint8_t Wifi_IsReady(void);

void ESP32_RxCpltCallback(Esp32Handle* handle, uint8_t link_id,
                          uint8_t* data, uint16_t length);
void ESP32_OnWifiConnectioStateChange(Esp32Handle* handle, uint8_t is_lost);
void ESP32_OnReconnectFailed(Esp32Handle* handle);
void ESP32_OnUnexpectedRestart(Esp32Handle* handle);
void ESP32_OnBaudrateError(Esp32Handle* handle);

#endif /* WIFI_H */
