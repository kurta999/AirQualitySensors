#ifndef ESP32_CONFIG_H
#define ESP32_CONFIG_H

#define ESP32_CFG_FILE

#define CHIP_TYPE_ESP32             0
#define CHIP_TYPE_ESP8266           1

#define ESP32_TYPE                  CHIP_TYPE_ESP32
#define ESP32_USE_UART_RX_DMA       1
#define ESP32_RX_CIRCULAT_BUF_SIZE  4096
#define ESP32_LINE_SIZE             1024
#define ESP32_CONEXT_SWITCH_DELAY   4
#define ESP32_UART_TIMEOUT          500
#define ESP32_UART_BYTE_TIMEOUT     15
#define ESP32_AP_INFO_UPDATE_TICK   45000
#define ESP32_RECONNECT_INTERVAL    12000
#define ESP32_SAFETY_WATCHOD        1

#define ESP32_USE_AP                1
#define ESP32_USE_STATION           1
#define ESP32_UPDATE_AP_INFO        1
#define ESP32_USE_BLE_FUNCTIONS     0
#define ESP32_USE_CLASSIC_BT_FUNCS  0
#define ESP32_USE_SNTP              1
#define ESP32_USE_SNTP_DAYNAME      1
#define ESP32_SNTP_QUERY_INTERVAL   60000
#define ESP32_USE_DNS               1
#define ESP32_USE_QUIT_AP           0
#define ESP32_USE_SLEEP             0
#define ESP32_USE_SMART_CONFIG      0
#define ESP32_USE_GET_CONN_STATUS   0
#define ESP32_USE_PING              1
#define ESP32_MAX_STATION_CONN      2
#define ESP32_MAX_AP_NAME_LEN       33
#define ESP32_MAX_AP_PW_LEN         65

#define ESP32_BLE_MAX_SERVICES          5
#define ESP32_BLE_MAX_CHARS_IN_SERVICES 7
#define ESP32_SOFTAP_MAX_CONNECTIONS    10
#define ESP32_MAX_TCP_CONNECTIONS       5

#endif /* ESP32_CONFIG_H */
