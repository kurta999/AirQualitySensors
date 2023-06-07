 /*
 * Copyright (C) Effman s.r.o. - All rights reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

/**
 * @file wifi.h
 * @author Attila Kiss <attila@effman.eu>
 * @date 2018. nov. 13.
 * @brief This file is responsible for handling wifi communication
 */

#ifndef WIFI_H_
#define WIFI_H_

#include <inttypes.h>

typedef enum
{
  kWifiConnecting,
  kWifiConnected,
  kWifiConfigIsEmpty,
  kWifiCannotConnect,
  kWifiNoServerConnection,
  kWifiOtherError
} WifiConnectionStates;

void Wifi_Init(void);
void Wifi_Shutdown(void);
uint16_t Wifi_GetTimeout(void);
void Wifi_SetAccessPoint(char* ap_ssid, char* ap_pw);
void Wifi_GetAccessPoint(char* ap_ssid, char* ap_pw);
void Wifi_SetLocalApPass(char* ap_pw);
void Wifi_GetLocalApPass(char* ap_pw);
WifiConnectionStates Wifi_GetConnectionState(void);
void Wifi_SendData(uint8_t* data, uint16_t length);
void Wifi_DataReceived(uint8_t* data, uint16_t len);
void Wifi_UartIrqHandler(void);
void Wifi_DataProcessed(void);
void Wifi_UartError(void);
void Wifi_UartTxDmaIrqHandler(void);
void Wifi_10MsIsr(void);
void Wifi_SysTick(void);

#endif /* WIFI_H_ */
