#ifndef ESP32_H_
#define ESP32_H_

#include <esp32_config.h>
#include <inttypes.h>
#include "cmsis_os.h"
#include "circbuff/circbuff.h"
#include "device_specific.h"

#define DMA_BUF_SIZE                        1024 /**< DMA circular buffer size in bytes */

#define ESP32_STATUS_IS_CONNECTED_TO_AP     2
#define ESP32_STATUS_TCP_UDP_ALIVE          3
#define ESP32_STATUS_TCP_UDP_DISCONNECTED   4
#define ESP32_STATUS_IS_NOT_CONNECTED_TO_AP 5

#define ESP32_MAX_IP_MAC_LEN                18 /**< Maximum IP & MAC length - used in GetLocalIP address */

#if ESP32_USE_UART_RX_DMA == 1
typedef struct
{
  volatile uint8_t flag; /**< Timeout event flag */
  uint16_t timer; /**< Timeout duration in msec */
  uint16_t prevCNDTR; /**< Holds previous value of DMA_CNDTR */
} DMA_Event_t;
#endif

typedef enum
{
  kEsp32Ok,
  kEsp32Error,
  kEsp32Timeout,
  kEsp32Other
} Esp32AtResponse;  /**< Transmit response codes */

typedef enum
{
  kWiFiDisabled,
  kWiFiStation,
  kWiFiSoftAp,
  kWiFiSoftApStation
} Esp32WiFiMode;  /**< Esp32 WiFi modes */

typedef enum
{
  kEncryptionOpen = 0,
  kEncryptionWpaPsk = 2,
  kEncryptionWpa2Psk,
  kEncryptionWpaWpa2Psk,
} Esp32EncryptionMethod;  /**< Esp32 Encryption methods */

typedef struct
{
  USART_TypeDef* uart_handle;  /**< UART handle */
  uint32_t baudrate;  /**< Desired baudrate for ESP - default is 115200 */
  uint32_t found_baudrate; /**< Baudrate found by auto baudrate detection */
  uint8_t uart_timeout_counter;  /**< Timeout downcounter for uart byte receiving */
  GPIO_TypeDef* en_gpio_port;  /**< Enable GPIO Port */
  uint16_t en_pin;  /**< Enable GPIO Pin */
  GPIO_TypeDef* rst_gpio_port;  /**< Reset GPIO Port */
  uint16_t rst_pin;  /**< Reset GPIO Pin */
  uint8_t rx_circular_buffer[ESP32_RX_CIRCULAT_BUF_SIZE];  /**< Circular buffer for data which received via DMA */
  circbuff_t circbuff_def;  /**< Definition for circular buffer itself */
  uint8_t line_rx[ESP32_LINE_SIZE];  /**< Data received from the device */
  uint16_t line_rx_index;  /**< Length of received data */
  uint16_t timeout_timer_count;  /**< Timeout counter [10ms]*/
  uint8_t connect_in_progress;
#if ESP32_USE_STATION == 1
  char ap_name[ESP32_MAX_AP_NAME_LEN];  /**< Acess Point's name where station is connected */
  char password[ESP32_MAX_AP_PW_LEN];  /**< Acess Point's password where station is connected */
  char SSID[ESP32_MAX_AP_NAME_LEN];  /**< Acess Point's SSID where station is connected */
  char MAC[18];  /**< Acess Point's BSSID (MAC Address) where station is connected */
  uint8_t channel;  /**< WiFI channel */
#endif
  uint8_t wifi_mode;  /**< Wifi mode set with ESP32_SetWifiMode */
  uint8_t is_wifi_connected : 1;  /**< Is WiFi connected to an Access Point? */
  uint8_t is_wifi_connection_lost : 1;  /**< Is WiFi connection lost? */
  uint32_t last_reconnection_tick; /**< Tick when last re-connect attempt happend */
  uint8_t is_inited : 1; /**< Is ESP inited? */
#if ESP32_USE_BLE_FUNCTIONS == 1
  ESP32_PrimaryServices services[ESP32_BLE_MAX_SERVICES];
#endif
  struct
  {
    uint8_t wifi_connected: 1;
    uint8_t wifi_got_ip: 1;
    uint8_t wifi_disconnected: 1;
    uint8_t tcp_connect;
    uint8_t tcp_closed;
    uint8_t sta_connected : 1;
    uint8_t sta_dist_ip : 1;
    uint8_t sta_disconnected : 1;
  } msgs;
} Esp32Handle;  /**< ESP32 Handle */

HAL_StatusTypeDef ESP32_Init(Esp32Handle* handle, USART_TypeDef* usart_handle,
#if ESP32_USE_UART_RX_DMA == 1
    DMA_TypeDef* rx_dma, DMA_Channel_TypeDef* rx_dma_channel, uint8_t rx_dma_channel_id,
    DMA_TypeDef* tx_dma, DMA_Channel_TypeDef* tx_dma_channel, uint8_t tx_dma_channel_id,
#endif
    GPIO_TypeDef* en_gpio_port, uint16_t en_pin, GPIO_TypeDef* rst_gpio_port, uint16_t rst_pin, uint32_t baudrate);
HAL_StatusTypeDef ESP32_Reset(Esp32Handle* handle, uint8_t restore_factory_defaults);
HAL_StatusTypeDef ESP32_GetVersionInfo(Esp32Handle* handle, char *output_buffer, uint8_t len);
HAL_StatusTypeDef ESP32_GetBaudRate(Esp32Handle* handle, uint32_t* baudrate, uint8_t *databits, uint8_t *stopbits, uint8_t *parity,
    uint8_t *flow_control);
HAL_StatusTypeDef ESP32_SetBaudRate(Esp32Handle* handle, uint32_t baudrate, uint8_t databits, uint8_t stopbits, uint8_t parity,
    uint8_t flow_control);
HAL_StatusTypeDef ESP32_ToggleAutoConn(Esp32Handle* handle, uint8_t toggle);
HAL_StatusTypeDef ESP32_SetWifiMode(Esp32Handle* handle, Esp32WiFiMode wifi_mode);
HAL_StatusTypeDef ESP32_ToggleMultiConnection(Esp32Handle* handle, uint8_t multi_connect_mode);
HAL_StatusTypeDef ESP32_SetServerMaxConnection(Esp32Handle* handle, uint8_t num_connections);
HAL_StatusTypeDef ESP32_CreateDeleteServer(Esp32Handle* handle, uint8_t mode, uint16_t port, uint8_t is_ssl, uint8_t enable_ssl_ca);
HAL_StatusTypeDef ESP32_JoinAccesssPoint(Esp32Handle* handle, char* ap_name, char* password, char* bssid, uint8_t try_to_reconnect);
HAL_StatusTypeDef ESP32_ConnectToServer(Esp32Handle* handle, uint8_t link_id, char* connection_mode, char* ip_addr, uint16_t port,
    uint16_t udp_local_port, uint8_t udp_mode);
HAL_StatusTypeDef ESP32_DisconnectFromServer(Esp32Handle* handle, uint8_t link_id);
HAL_StatusTypeDef ESP32_GetLocalIpAddress(Esp32Handle* handle, char* soft_ap_ip, char* soft_ap_mac, char* station_ip, char* station_mac);

HAL_StatusTypeDef ESP32_SendTcp(Esp32Handle* handle, uint8_t link_id, uint8_t* data, uint32_t length);


void ESP32_Task(Esp32Handle* handle);
void ESP32_10Ms_Isr(Esp32Handle* handle);
void ESP32_SysTick(Esp32Handle* handle);

void ESP32_UartRxDmaIrqHandler(Esp32Handle* handle);
void ESP32_UartTxDmaIrqHandler(Esp32Handle* handle);
void ESP32_InsertDataToBuffer(Esp32Handle* handle, const uint8_t* data, uint16_t len);
void ESP32_UartError(Esp32Handle* handle);


/*
__weak void ESP32_RxCpltCallback(Esp32Handle* handle, uint8_t link_id, uint8_t* data, uint16_t length);
__weak void ESP32_TxCpltCallback(Esp32Handle* handle, uint8_t link_id, uint8_t *data, uint16_t length);
__weak void ESP32_OnWifiConnectioStateChange(Esp32Handle* handle, uint8_t is_lost);
__weak void ESP32_OnBaudrateError(Esp32Handle* handle);
__weak void ESP32_OnUnexpectedRestart(Esp32Handle* handle);
__weak void ESP32_OnReconnectFailed(Esp32Handle* handle);
*/
#endif /* ESP32_H_ */
