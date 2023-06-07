/*
 * Copyright (C) Effman s.r.o. - All rights reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

/* Copy this file to project path and rename to ESP32_config.h */

/**
 * @file ESP32_config.h
 * @author Attila Kiss <attila@effman.eu>
 * @date 9. 3. 2018
 * @brief Configuration file for ESP32
 */

#ifndef ESP32_CONFIG_H_
#define ESP32_CONFIG_H_

#define ESP32_CFG_FILE

#define CHIP_TYPE_ESP32             0
#define CHIP_TYPE_ESP8266           1

// --------------------------
// ---  driver configurations ---
// --------------------------

#define ESP32_TYPE                  CHIP_TYPE_ESP32 /**< Specifis that older ESP8266 is used or newer ESP32 */
#define ESP32_USE_UART_RX_DMA       1  /**< Use USART DMA mode? Otherwise interrupt mode is used */
#define ESP32_RX_CIRCULAT_BUF_SIZE  4096 /**< RX circular buffer size in bytes */
#define ESP32_LINE_SIZE             1024  /**< Maximum length of one data packet */
#define ESP32_CONEXT_SWITCH_DELAY   4  /**< How many milliseconds pass to osDelay function while waiting for response*/
#define ESP32_UART_TIMEOUT     	    500  /**< UART transmit timeout threshold for semaphore [ms] */
#define ESP32_UART_BYTE_TIMEOUT     15 /**< UART byte timeout (how many ms need to elapse since last received byte */
#define ESP32_AP_INFO_UPDATE_TICK   45000  /**< Access point update tick [ms] */
#define ESP32_RECONNECT_INTERVAL    12000  /**< Reconnect internal in case of lost connection [ms] */
#define ESP32_SAFETY_WATCHOD        1  /**< If ESP32 restarts for unknown reason, reset MCU with ESP to avoid errors? */

#define ESP32_USE_AP                1  /**< Use functions related to Access Point? */
#define ESP32_USE_STATION           1  /**< Use functions related to Station? */
#define ESP32_UPDATE_AP_INFO        1  /**< When enabled access point information will be updated and stored in defined interval (mainly for RSSI) */

#define ESP32_USE_BLE_FUNCTIONS     0  /**< Use BLE (Bluetooth Low Energy) functions? ISN'T FINISHED YET*/
#define ESP32_USE_CLASSIC_BT_FUNCS  0  /**< Use Classic Bluetooth functions? Only works with ESP32-WROVER board*/
#define ESP32_USE_SNTP              1  /**< Use SNTP related functions? */
#define ESP32_USE_SNTP_DAYNAME      1  /**< Store dayname string in time structure?*/
#define ESP32_SNTP_QUERY_INTERVAL   60000  /**< SNTP query internal [ms], use 0 to disable periodically SNTP querying */
#define ESP32_USE_DNS               1  /**< Use SetDnsServer function? */
#define ESP32_USE_QUIT_AP           0  /**< Use QuitAccessPoint function? */
#define ESP32_USE_SLEEP             0  /**< Use Sleep function? */
#define ESP32_USE_SMART_CONFIG      0  /**< Use SmartConfig functions? */
#define ESP32_USE_GET_CONN_STATUS   0  /**< Use GetConnectionStatus function? */
#define ESP32_USE_PING              1  /**< Use ping? */
#define ESP32_MAX_STATION_CONN      2  /**< Maximum number of parallel station connections at same time
                                           (set it to 1 when you want to really save memory - for more info why, look at code & ESP32 datasheet */
#define ESP32_MAX_AP_NAME_LEN       32  /**< Maximum length of access point name */
#define ESP32_MAX_AP_PW_LEN         32  /**< Maximum length of access point password */

#define ESP32_BLE_MAX_SERVICES          5
#define ESP32_BLE_MAX_CHARS_IN_SERVICES 7

// --------------------------
// ---  hard coded configurations ---
// --------------------------

#define ESP32_SOFTAP_MAX_CONNECTIONS  10  /**< Maximum number of connections which Access Point can accept.
                                               This config is hardcoded into AT firmware, can be changed only
                                               if AT firmware gets recompiled. */

#define ESP32_MAX_TCP_CONNECTIONS     5  /**< Maximum number of connections which TCP Server can accept
                                               This config is hardcoded into AT firmware, can be changed only
                                               if AT firmware gets recompiled. */

#endif /* ESP32_CONFIG_H_ */
