 /* Includes ------------------------------------------------------------------*/
#include <esp32_config.h>
#include <esp32/esp32.h>
#include <stm32_atomic/stm32_atomic.h>
#include "device_specific.h"
#include <string.h>
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include "cmsis_os.h"
#include "stm32l4xx_ll_usart.h"
#include "usart_driver.h"
#include "wifi.h"

/* Private macro -------------------------------------------------------------*/
#ifndef ESP32_CFG_FILE
#error "Include the esp32_config.h configuration file"
#endif

_Static_assert(ESP32_MAX_STATION_CONN > 0 &&
               ESP32_MAX_STATION_CONN <= ESP32_MAX_TCP_CONNECTIONS,
               "ESP32_MAX_STATION_CONN must be in the range 1..5");
/* Private variables ---------------------------------------------------------*/
static char local_at_format_buffer[2100];  /**< Buffer used by SendData for formatting AT commands */
static char buffer[2100];  /**< Buffer used by ESP32_TcpSendf for formatting TCP messages */

static const uint32_t baudrates[] = { 110, 150, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1843200 };
extern UsartDriverHandle esp32_usart;

static void CopyString(char* destination, size_t destination_size, const char* source)
{
    if(destination == NULL || destination_size == 0U)
        return;
    if(source == NULL)
    {
        destination[0] = '\0';
        return;
    }

    size_t length = 0U;
    while(length + 1U < destination_size && source[length] != '\0')
    {
        destination[length] = source[length];
        ++length;
    }
    destination[length] = '\0';
}

// -----------------------------------------
// --- private functions - timeout timer ---
// -----------------------------------------

/**
 * !\brief Corrtect link id if it's out of range
 */
static void CorrectLinkIdIfInvalid(uint8_t* link_id)
{
    if(*link_id >= ESP32_MAX_TCP_CONNECTIONS)
        *link_id = ESP32_MAX_TCP_CONNECTIONS - 1;
}

/**
 * !\brief Set timeout
 * !\param handle Esp32 Handle
 * !\param timeout_10ms Timeout time [10ms]
 */
static inline void SetTimeout(Esp32Handle* handle, uint16_t timeout_10ms)
{
    handle->timeout_timer_count = timeout_10ms;
}

/**
 * !\brief Returns true if timeout expired, otherwise false
 * !\param handle Esp32 Handle
 * !\return True if timeout expired, otherwise false
 */
static inline uint8_t TimeoutHasExpired(Esp32Handle* handle)
{
    return handle->timeout_timer_count == 0;
}

/**
 * !\brief Add a byte to "line" buffer
 * !\param handle Esp32 Handle
 */
static uint8_t AddByteToLineBuffer(Esp32Handle* handle, const uint8_t* data)
{
    if(handle == NULL || data == NULL ||
       handle->line_rx_index >= (sizeof(handle->line_rx) - 1U))
        return 0U;
    handle->line_rx[handle->line_rx_index++] = *data;
    handle->line_rx[handle->line_rx_index] = '\0';
    return 1U;
}

/**
 * !\brief Return size of current "line" buffer and reset buffer
 * !\param handle Esp32 Handle
 * !\return Size of current "line" buffer
 */
static uint8_t GetAndResetLineBuffer(Esp32Handle* handle)
{
    uint8_t ret = handle->line_rx_index > 1;  /* got a line, minimum 1 character + \0 at the end */
    handle->line_rx_index = 0;
    return ret;
}

static void FlushBuffers(Esp32Handle* handle);


/**
 * @brief Check and handle unsolicited messages
 * @param handle Esp32 Handle
 */
static void CheckForUnsolicitedMessages(Esp32Handle* handle)
{
    if(!memcmp(handle->line_rx, "ready", 5))  /* If ESP reseted itself for unknown reason while FW in STM32 are running */
    {
        if(handle->is_inited)
        {
            handle->is_inited = 0;
            ESP32_OnUnexpectedRestart(handle);
        }
        return;
    }

    if(!memcmp((char*)(handle->line_rx), "WIFI CONNECTED", 14))
        handle->msgs.wifi_connected = 1;
    else if(!memcmp((char*)(handle->line_rx), "WIFI GOT IP", 11))
        handle->msgs.wifi_got_ip = 1;
    else if(!memcmp(handle->line_rx, "WIFI DISCONNECT", 15))
    {
        handle->msgs.wifi_disconnected = 1;
        handle->is_wifi_connection_lost = 1;
        ESP32_OnWifiConnectioStateChange(handle, 1);
    }
    else if(!memcmp(&handle->line_rx[1], ",CONNECT", 8))
    {
        uint8_t link_id = atoi((char*)(&handle->line_rx[0]));
        if(link_id < ESP32_MAX_TCP_CONNECTIONS)
        {
            handle->msgs.tcp_connect |= (uint8_t)(1U << link_id);
            osDelay(100);
        }
    }
    else if(!memcmp(&handle->line_rx[1], ",CLOSED", 7))
    {
        uint8_t link_id = atoi((char*)(&handle->line_rx[0]));
        if(link_id < ESP32_MAX_TCP_CONNECTIONS)
            handle->msgs.tcp_closed |= (uint8_t)(1U << link_id);
    }
}

/**
 * !\brief Process a line from awaiting circular buffer
 * !\param handle Esp32 Handle
 * !\details Read UART circular buffer, process one message if any and return true.
 * !\retval 0 Buffer is empty, nothing was processed
 * !\retval 1 A message was successfully parsed
 */
static uint8_t ReceiveMessage(Esp32Handle* handle)
{
    uint8_t to_exit = 0;
    while(!to_exit)
    {
        uint8_t data;
        to_exit = !circbuff_get_byte(&handle->circbuff_def, &data);
        if(to_exit)
            break;
        if(data == '\r')
            continue;
        else if(data == '\n')
            data = 0;  /* line end -> string end */
        else if(data == '>')  /* prompt char */
        {
            if(AddByteToLineBuffer(handle, &data) == 0U)
            {
                FlushBuffers(handle);
                return 0U;
            }
            GetAndResetLineBuffer(handle);
            return 1;
        }
        else if(data == '+')
        {
            if(circbuff_isnext(&handle->circbuff_def, (uint8_t*)"IPD,", 4, 1) && !handle->connect_in_progress)  /* Received message format is: +IPD,1,423:HTTP/ASD */
            {
                volatile uint16_t data_from_ipd = 0xFFFF;
                uint8_t data_in[17] = {0U};
                uint8_t count = 0U;
                while(circbuff_get_byte(&handle->circbuff_def, &data) && count < 16U)
                {
                    data_in[count++] = data;
                    if(data == ':')
                    {
                        data_in[count] = '\0';
                        count = 0xFF;
                        break;
                    }
                }
                if(count == 0xFF)
                {
                    uint8_t conn_linkid = atoi((char*)data_in);
                    data_from_ipd = atoi((char*)&data_in[2]);
                    if(conn_linkid >= ESP32_MAX_TCP_CONNECTIONS ||
                       data_from_ipd == 0xFFFFU ||
                       data_from_ipd >= sizeof(handle->line_rx))
                    {
                        FlushBuffers(handle);
                        return 0U;
                    }
                    if(data_from_ipd != 0U)
                    {
                        uint16_t i = 0;
                        const uint32_t receive_started = HAL_GetTick();
                        while(i < data_from_ipd)
                        {
                            volatile uint16_t data_remaining = 0;
                            do
                            {
                                data_remaining = circbuff_get_data_len(&handle->circbuff_def);
                                if(!data_remaining)
                                    osDelay(10);
                                if((HAL_GetTick() - receive_started) > 2000U)
                                {
                                    FlushBuffers(handle);
                                    return 0U;
                                }
                            } while(!data_remaining);
                            circbuff_get_byte(&handle->circbuff_def, &data);
                            if(AddByteToLineBuffer(handle, &data) == 0U)
                            {
                                FlushBuffers(handle);
                                return 0U;
                            }
                            if(++i == data_from_ipd)
                            {
                                ESP32_RxCpltCallback(
                                    handle,
                                    conn_linkid,
                                    (uint8_t*)handle->line_rx,
                                    handle->line_rx_index);
                                GetAndResetLineBuffer(handle);
                                return 1;
                            }
                        }
                    }
                }
            }
        }
        if(AddByteToLineBuffer(handle, &data) == 0U)
        {
            FlushBuffers(handle);
            return 0U;
        }
        if(!data)
        {
            CheckForUnsolicitedMessages(handle);
            return GetAndResetLineBuffer(handle);
        }
    }
    return 0;
}

/**
 * !\brief Block until response arrives and return response code based on received string
 * !\param handle Esp32 Handle
 * !\return Esp32AtResponse
 */
static Esp32AtResponse WaitForResponse(Esp32Handle* handle)
{
    while(1)
    {
        if(ReceiveMessage(handle))
        {
            if(!memcmp(handle->line_rx, "OK", 2))
                return kEsp32Ok;
#if ESP32_TYPE == CHIP_TYPE_ESP32
            if(!memcmp(handle->line_rx, "busy p...", 9) || !memcmp(handle->line_rx, "ERR CODE", 8))
#else
            if(!memcmp(handle->line_rx, "busy p...", 9) || !memcmp(handle->line_rx, "busy s...", 9) || !memcmp(handle->line_rx, "ERR CODE", 8))
#endif
            {
                osDelay(300);  /* Give some time for WiFi chip to finish it's pending AT command */
                return kEsp32Error;
            }
            for(char* p = (char*)handle->line_rx; *p != '\0'; p++)
                if(!memcmp("ERROR", p, 5))
                    return kEsp32Error;
            return kEsp32Other;
        }
        if(TimeoutHasExpired(handle))
            return kEsp32Timeout;
        osDelay(ESP32_CONEXT_SWITCH_DELAY);  /* Small delay to allow conext switch happen */
    }
}

/**
 * !\brief Flush UART lines (both RX & TX)
 * !\param handle Esp32 Handle
 */
static void FlushBuffers(Esp32Handle* handle)
{
    handle->line_rx_index = 0;
    memset(handle->line_rx, 0, sizeof(handle->line_rx));
    circbuff_flush(&handle->circbuff_def);
}

/**
 * !\brief Transmit string via UART
 * !\param handle Esp32 Handle
 * !\param timeout_10ms Timeout time [10ms]
 * !\param message String to transfer with format parameters
 * !\return HAL_Status
 */
static HAL_StatusTypeDef ExecuteAtCommand(Esp32Handle* handle, uint16_t timeout_10ms, char* message, ...)
{
    FlushBuffers(handle);
    HAL_StatusTypeDef HAL_Status = HAL_OK;

    va_list args;
    va_start(args, message);
    const int formatted_length = vsnprintf(
        local_at_format_buffer,
        sizeof(local_at_format_buffer) - 2U,
        message,
        args);
    va_end(args);
    if(formatted_length < 0 ||
       (size_t)formatted_length >= (sizeof(local_at_format_buffer) - 2U))
        return HAL_ERROR;

    const uint16_t len = (uint16_t)formatted_length;
    local_at_format_buffer[len] = '\r';
    local_at_format_buffer[len + 1U] = '\n';
    local_at_format_buffer[len + 2U] = '\0';
    HAL_Status = UsartDriver_Transmit(
        USART_DRIVER_SOURCE_ESP32,
        local_at_format_buffer,
        (uint16_t)(len + 2U));

    SetTimeout(handle, timeout_10ms);

    return HAL_Status;
}

/**
 * !\brief Transmit bytes via UART
 * !\param handle Esp32 Handle
 * !\param bytes Bytes to transfer
 * !\param len Length
 */
static inline HAL_StatusTypeDef TransmitBytes(Esp32Handle* handle, uint8_t* bytes, uint16_t len)
{
    FlushBuffers(handle);
    return UsartDriver_Transmit(USART_DRIVER_SOURCE_ESP32, (char*)bytes, len);
}

// -----------------------------------------
// --- private functions - WiFi ---
// -----------------------------------------

/**
 * !\brief Get AT firmware version information
 * !\details Response is:
 *  <AT version info>: information about the AT version.
 *  <SDK version info>: information about the SDK version.
 *  compile time>: the duration of time for compiling the BIN.
 * !\param handle Esp32 handle
 * !\param output_buffer Pointer for output buffer where to store version information
 * !\param len Length of output_buffer in bytes
 * !\return HAL_Status
 */
static HAL_StatusTypeDef GetVersionInfo(Esp32Handle* handle, char* output_buffer, uint8_t len)
{
    uint8_t to_exit_loop = 0;
    HAL_StatusTypeDef HAL_Status = ExecuteAtCommand(handle, 50, "AT+GMR");
    if(HAL_Status == HAL_OK)
    {
        memset(output_buffer, 0, len);
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Other:
                {
                    const size_t used = strlen(output_buffer);
                    if(used < (size_t)len - 1U)
                    {
                        strncat(
                            output_buffer,
                            (char*)handle->line_rx,
                            (size_t)len - used - 1U);
                    }
                    break;
                }
                case kEsp32Ok:
                    HAL_Status = HAL_OK;
                    to_exit_loop = 1;
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Read the UART configuration from flash
 * !\param handle Esp32 handle
 * !\return HAL_Status
 */
static HAL_StatusTypeDef GetBaudRate(Esp32Handle* handle, uint32_t* baudrate, uint8_t* databits, uint8_t* stopbits, uint8_t* parity,
    uint8_t* flow_control)
{
    uint8_t to_exit_loop = 0;
    HAL_StatusTypeDef HAL_Status = ExecuteAtCommand(handle, 50, "AT+UART_DEF?");
    if(HAL_Status == HAL_OK)
    {
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Other:
                    if(!memcmp((char*)(handle->line_rx), "+UART_DEF:", 10))
                    {
                        char* baud_separator = strstr((char*)handle->line_rx, ":");
                        char* baud_str = baud_separator != NULL ? baud_separator + 1 : NULL;
                        char* dbits_separator = baud_str != NULL ? strstr(baud_str, ",") : NULL;
                        char* dbits_str = dbits_separator != NULL ? dbits_separator + 1 : NULL;
                        char* stopbits_separator = dbits_str != NULL ? strstr(dbits_str, ",") : NULL;
                        char* stopbits_str = stopbits_separator != NULL ? stopbits_separator + 1 : NULL;
                        char* parity_separator = stopbits_str != NULL ? strstr(stopbits_str, ",") : NULL;
                        char* parity_str = parity_separator != NULL ? parity_separator + 1 : NULL;
                        char* flow_separator = parity_str != NULL ? strstr(parity_str, ",") : NULL;
                        char* flowctl_str = flow_separator != NULL ? flow_separator + 1 : NULL;
                        if(flowctl_str == NULL)
                        {
                            HAL_Status = HAL_ERROR;
                            to_exit_loop = 1U;
                            break;
                        }
                        uint32_t temp_baud = (uint32_t)atoi(baud_str);
                        uint8_t temp_databits = (uint8_t)atoi(dbits_str);
                        uint8_t temp_stopbits = (uint8_t)atoi(stopbits_str);
                        if(temp_baud == 0 || temp_databits < 7)  /* Sanity checks - I already met scenario when ESP8266 returned 0 for every parameter */
                        {
                            to_exit_loop = 1;
                            HAL_Status = HAL_OK;
                        }
                        else
                        {
                            *baudrate = temp_baud;
                            *databits = temp_databits;
                            *stopbits = temp_stopbits;
                            *parity = (uint8_t)atoi(parity_str);
                            *flow_control = (uint8_t)atoi(flowctl_str);
                            HAL_Status = HAL_OK;
                        }
                    }
                    break;
                case kEsp32Ok:
                    HAL_Status = HAL_OK;
                    to_exit_loop = 1;
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Set UART configuration and saves it to flash
 * !\param handle Esp32 handle
 * !\param baudrate UART Baud rate (default is 115200)
 * !\param databits UART Databits (8 bit data by default)
 * !\param stopbits UART Stopbits (1 bit by default)
 * - 1: 1-bit stop bit
 * - 2: 1.5-bit stop bit
 * - 3: 2-bit stop bit
 * !\param parity UART Parity bit (none by default)
 * - 0: None
 * - 1: Odd
 * - 2: Even
 * !\param flow_control (disabled by default)
 * - 0: flow control is not enabled
 * - 1: enable RTS
 * - 2: enable CTS
 * - 3: enable both RTS and CTS
 * !\return HAL_Status
 */
static HAL_StatusTypeDef SetBaudRate(Esp32Handle* handle, uint32_t baudrate, uint8_t databits, uint8_t stopbits, uint8_t parity,
    uint8_t flow_control)
{
    uint8_t to_exit_loop = 0;
    HAL_StatusTypeDef HAL_Status = ExecuteAtCommand(handle, 30, "AT+UART_DEF=%d,%d,%d,%d,%d", baudrate, databits, stopbits, parity, flow_control);
    if(HAL_Status == HAL_OK)
    {
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Ok:
                    HAL_Status = HAL_OK;
                    to_exit_loop = 1;
                    break;
                case kEsp32Timeout:
                case kEsp32Error:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
                default:
                    break;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Disable AT commands echo mode
 * !\param handle Esp32 handle
 * !\return HAL_Status
 */
static HAL_StatusTypeDef DisableEcho(Esp32Handle* handle)
{
    uint8_t to_exit_loop = 0, recv_echo = 0, recv_ok = 0;
    HAL_StatusTypeDef HAL_Status = ExecuteAtCommand(handle, 10, "ATE0");
    if(HAL_Status == HAL_OK)
    {
        HAL_Status = HAL_ERROR;
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Ok:
                    recv_ok = 1;
                    HAL_Status = HAL_OK;
                    break;
                case kEsp32Timeout:
                case kEsp32Error:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
                case kEsp32Other:
                    if(!memcmp(handle->line_rx, "ATE0", 4))
                    {
                        recv_echo = 1;
                    }
                    break;
            }
            if(recv_echo && recv_ok && HAL_Status == HAL_OK)
            {
                to_exit_loop = 1;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Disable automatic connection to access point when device power up
 * !\details This configuration is saved to internal NVM and will be loaded upon startup
 * !\param handle Esp32 handle
 * !\param toggle Toggle auto connection?
 * !\return HAL_Status
 */
static HAL_StatusTypeDef ToggleAutoConn(Esp32Handle* handle, uint8_t toggle)
{
    uint8_t to_exit_loop = 0;
    HAL_StatusTypeDef HAL_Status = ExecuteAtCommand(handle, 10, "AT+CWAUTOCONN=%d", toggle);
    if(HAL_Status == HAL_OK)
    {
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Ok:
                    HAL_Status = HAL_OK;
                    to_exit_loop = 1;
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Try to find automatically baudrate based on predefined baudrate list
 * !\param handle Esp32 handle
 * !\return HAL_Status
 */
HAL_StatusTypeDef FindBaudRate(Esp32Handle* handle)
{
    HAL_StatusTypeDef HAL_Status = HAL_ERROR;
    uint8_t baudrate_found = 0;
    for(uint8_t i = 0; i < ARRAY_LEN(baudrates); ++i)
    {
        uint8_t to_exit_loop = 0, ok_recv = 0, at_recv = 0;
        LL_USART_SetBaudRate(handle->uart_handle, SystemCoreClock, LL_USART_OVERSAMPLING_16, baudrates[i]);
        HAL_Status = ExecuteAtCommand(handle, 10, "AT");
        while(!to_exit_loop)
        {
            volatile const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Ok:
                    ok_recv = 1;
                    break;
                case kEsp32Other:
                    if(!memcmp((char*)(handle->line_rx), "AT", 2))
                        at_recv = 1;
                    break;
                case kEsp32Timeout:
                    to_exit_loop = 1;
                    break;
                default:
                    break;
            }
            if(ok_recv && at_recv)
            {
                HAL_Status = HAL_OK;
                baudrate_found = i;
                to_exit_loop = 1;
            }
        }
        if(baudrate_found)
        {
            handle->found_baudrate = baudrates[baudrate_found];
            break;
        }
    }
    return HAL_Status;
}

/**
 * !\brief Reset ESP-WROOM-32 device
 * !\param handle Esp32 Handle
 * !\param restore_factory_defaults Restore factory default settings?
 */
static HAL_StatusTypeDef Reset(Esp32Handle* handle, uint8_t restore_factory_defaults)
{
    uint8_t retry_count = 0;
    HAL_StatusTypeDef HAL_Status = HAL_ERROR;
    handle->is_inited = 0;
    handle->timeout_timer_count = handle->uart_timeout_counter = handle->wifi_mode;
    handle->connect_in_progress = 0;
    memset(handle->rx_circular_buffer, 0, sizeof(handle->rx_circular_buffer));
    memset(local_at_format_buffer, 0, sizeof(local_at_format_buffer));
    memset(buffer, 0, sizeof(buffer));
#if ESP32_USE_UART_RX_DMA == 0
    handle->uart_data_byte = 0;
#endif

#if ESP32_USE_STATION == 1
    memset(handle->ap_name, 0, sizeof(handle->ap_name));
    memset(handle->password, 0, sizeof(handle->password));
    memset(handle->SSID, 0, sizeof(handle->SSID));
    memset(handle->MAC, 0, sizeof(handle->MAC));
    handle->channel = 0xFF;
#endif
    FlushBuffers(handle);
    if(handle->en_gpio_port != NULL && !restore_factory_defaults)
    {
    Retry:
        if(++retry_count > 3)
            return HAL_TIMEOUT;
        osDelay(1000);
        HAL_GPIO_WritePin(handle->en_gpio_port, handle->en_pin, 0);
        if(handle->en_gpio_port && handle->rst_pin)  /* Reset pin only used on ESP2866 */
            HAL_GPIO_WritePin(handle->rst_gpio_port, handle->rst_pin, 0);
        osDelay(500);
        SetTimeout(handle, retry_count * 1200);
        HAL_GPIO_WritePin(handle->en_gpio_port, handle->en_pin, 1);
        if(handle->en_gpio_port && handle->rst_pin)  /* Reset pin only used on ESP2866 */
        {
            HAL_GPIO_WritePin(handle->rst_gpio_port, handle->rst_pin, 1);
        }
    }
    else
    {
        osDelay(2000);
        if(!restore_factory_defaults)
            HAL_Status = ExecuteAtCommand(handle, 800, "AT+RST");
        else
            HAL_Status = ExecuteAtCommand(handle, 800, "AT+RESTORE");
    }
    uint8_t to_exit_loop = 0;
    while(!to_exit_loop)
    {
        volatile const Esp32AtResponse resp = WaitForResponse(handle);
        switch(resp)
        {
            case kEsp32Other:
                if(!memcmp((char*)(handle->line_rx), "ready", 5))
                {
                    handle->is_inited = 1;
                    to_exit_loop = 1;
                    HAL_Status = HAL_OK;
                    FlushBuffers(handle);
                }
                break;
            case kEsp32Ok:
                break;
            case kEsp32Timeout:
            case kEsp32Error:
                HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                to_exit_loop = 1;
                break;
            default:
                Error_Handler();
                break;
        }
    }
    if(HAL_Status != HAL_OK)  /* In case of not receiving "ready" intime, try to find automatically the baudrate as last hope */
    {
        HAL_Status = FindBaudRate(handle);
        if(HAL_Status != HAL_OK)
            ESP32_OnBaudrateError(handle);
    }
    if(HAL_Status == HAL_OK)  /* Disable echo by default - THIS STEP IS MUST!!! */
        HAL_Status = DisableEcho(handle);
    uint8_t databits = 0, stopbits = 0, parity = 0, flow_control = 0;
    uint32_t baudrate = 0;
    if(HAL_Status == HAL_OK)
    {
        handle->is_inited = 1;  /* If "ready" wasn't received because of wrongly configured baudrate,
          but afterward chip correct baudrate found, mark chip as inited */
        HAL_Status = ESP32_GetBaudRate(handle, &baudrate, &databits, &stopbits, &parity, &flow_control);
        if(HAL_Status != HAL_OK || baudrate != handle->baudrate)  /* For first time when baudrate wasn't set before, it returns NULL */
        {
#if ESP32_TYPE == CHIP_TYPE_ESP8266
            HAL_Status = ESP32_GetCurrentBaudrate(handle, &baudrate, &databits, &stopbits, &parity, &flow_control);
#endif
        }
    }
    if(handle->baudrate != baudrate && baudrate != 0 && HAL_Status == HAL_OK)
    {
        HAL_Status = ESP32_SetBaudRate(handle, handle->baudrate, 8, 1, 0, 0);
        if(HAL_Status == HAL_OK)
        {
            osDelay(10);
            goto Retry;  /* After successfull baudrate change, restart ESP */
        }
    }
    return HAL_Status;
}

/**
 * !\brief Set WiFi mode
 * !\details This configuration is saved to internal NVM and will be loaded upon startup
 * !\param handle Esp32 handle
 * !\param wifi_mode WiFi mode
 * !\return HAL_Status
 */
static HAL_StatusTypeDef SetWifiMode(Esp32Handle* handle, Esp32WiFiMode wifi_mode)
{
    uint8_t to_exit_loop = 0;
    HAL_StatusTypeDef HAL_Status = ExecuteAtCommand(handle, 30, "AT+CWMODE=%d", (uint8_t)wifi_mode);
    if(HAL_Status == HAL_OK)
    {
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Ok:
                    HAL_Status = HAL_OK;
                    handle->wifi_mode = wifi_mode;
                    to_exit_loop = 1;
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Configures the multiple connections mode
 * !\details This configuration is saved to internal NVM and will be loaded upon startup
 * !\param handle Esp32 handle
 * !\param multi_connect_mode Multi connection mode (0/1)
 * !\return HAL_Status
 */
static HAL_StatusTypeDef ToggleMultiConnection(Esp32Handle* handle, uint8_t multi_connect_mode)
{
    uint8_t to_exit_loop = 0;
    HAL_StatusTypeDef HAL_Status = ExecuteAtCommand(handle, 10, "AT+CIPMUX=%d", (uint8_t)multi_connect_mode);
    if(HAL_Status == HAL_OK)
    {
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Ok:
                    HAL_Status = HAL_OK;
                    to_exit_loop = 1;
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Set maximum allowed conenction on the server
 * !\warning To set this configuration, you should call the command AT+CIPSERVERMAXCONN=<num> before creating a server.
 * !\param handle Esp32 handle
 * !\param num_connections Maximum allowed connection (maximum is 5)
 * !\return HAL_Status
 */
static HAL_StatusTypeDef SetServerMaxConnection(Esp32Handle* handle, uint8_t num_connections)
{
    uint8_t to_exit_loop = 0;
    HAL_StatusTypeDef HAL_Status = ExecuteAtCommand(handle, 10, "AT+CIPSERVERMAXCONN=%d", (uint8_t)num_connections);
    if(HAL_Status == HAL_OK)
    {
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Ok:
                    HAL_Status = HAL_OK;
                    to_exit_loop = 1;
                    break;
                case kEsp32Other:
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Deletes/Creates TCP or SSL Server
 * !\details This configuration is saved to internal NVM and will be loaded upon startup
 * - A server can only be created when multiple connections are activated
 * - Only one server is allowed to be created
 * !\param handle Esp32 handle
 * !\param mode Mode (0: delete server, 1: create server)
 * !\param port Server port (0 - 66535)
 * !\param is_ssl Is SSL connection?
 * !\param enable_ssl_ca Enable SSL CA? (this param is only used when using SSL)
 * !\return HAL_Status
 */
static HAL_StatusTypeDef CreateDeleteServer(Esp32Handle* handle, uint8_t mode, uint16_t port, uint8_t is_ssl, uint8_t enable_ssl_ca)
{
    uint8_t to_exit_loop = 0;
    HAL_StatusTypeDef HAL_Status = HAL_ERROR;
    if(!is_ssl)
        HAL_Status = ExecuteAtCommand(handle, 10, "AT+CIPSERVER=%d,%d", mode, port);
    else
        HAL_Status = ExecuteAtCommand(handle, 10, "AT+CIPSERVER=%d,%d,\"SSL\",%d", mode, port, enable_ssl_ca);
    if(HAL_Status == HAL_OK)
    {
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Ok:
                    HAL_Status = HAL_OK;
                    to_exit_loop = 1;
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Join to access point
 * !\note Possible error codes in case of unsuccessfull connection
 * - 1: connection timeout
 * - 2: wrong password.
 * - 3: cannot find the target AP.
 * - 4: connection failed.
 * - others: unknown error occurred.
 * !\details This configuration is saved to internal NVM and will be loaded upon startup
 * !\param handle Esp32 handle
 * !\param ap_name Access point name
 * !\param password Password for access point (if any - in case of no pw, leave it blank)
 * !\param bssid The target AP's MAC address, used when multiple APs have the same SSID - if unused, leave it blank
 * !\param try_to_reconnect If true, ESP trying to join access point until connection is estabilished
 * !\return HAL_Status
 */
static HAL_StatusTypeDef JoinAccesssPoint(Esp32Handle* handle, char* ap_name, char* password, char* bssid, uint8_t try_to_reconnect)
{
    uint8_t to_exit_loop = 0;
    uint8_t ok_received = 0;
    HAL_StatusTypeDef HAL_Status = HAL_ERROR;
#if ESP32_TYPE == CHIP_TYPE_ESP32
    if(!bssid)
        HAL_Status = ExecuteAtCommand(handle, 2000, "AT+CWJAP=\"%s\",\"%s\"", ap_name, password);
    else
        HAL_Status = ExecuteAtCommand(handle, 2000, "AT+CWJAP=\"%s\",\"%s\",\"%s\"", ap_name, password, bssid);
#else
    if(!bssid)
        HAL_Status = ExecuteAtCommand(handle, 2000, "AT+CWJAP_CUR=\"%s\",\"%s\"", ap_name, password);
    else
        HAL_Status = ExecuteAtCommand(handle, 2000, "AT+CWJAP_CUR=\"%s\",\"%s\",\"%s\"", ap_name, password, bssid);
#endif
    if(HAL_Status == HAL_OK)
    {
        HAL_Status = HAL_ERROR;
        handle->msgs.wifi_connected = handle->msgs.wifi_got_ip = 0;
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Other:  /* WARNING: These messages are randomly get sent out of order!!! */
                    if(!memcmp((char*)(handle->line_rx), "+CWJAP:", 7))  /* Error happend */
                    {
                        HAL_Status = HAL_BUSY;
                        break;
                    }
                    /* Drop other messages and wait */
                    break;
                case kEsp32Ok:
                    ok_received = 1;
                    break;
                default:
                    if(HAL_Status != HAL_BUSY)
                        HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
            if(handle->msgs.wifi_connected && handle->msgs.wifi_got_ip && ok_received)
            {
                HAL_Status = HAL_OK;
                to_exit_loop = 1;
                CopyString(handle->ap_name, sizeof(handle->ap_name), ap_name);
                CopyString(handle->password, sizeof(handle->password), password);
                CopyString(handle->MAC, sizeof(handle->MAC), bssid);
                handle->is_wifi_connected = 1;
            }
        }
    }
    if(!handle->is_wifi_connected && try_to_reconnect)  /* If try_to_reconnect is set and connection was unsuccessfull */
    {
        CopyString(handle->ap_name, sizeof(handle->ap_name), ap_name);
        CopyString(handle->password, sizeof(handle->password), password);
        CopyString(handle->MAC, sizeof(handle->MAC), bssid);
        handle->is_wifi_connected = 0U;
        handle->is_wifi_connection_lost = 1;  /* Mark connection as lost to try everytime estabilish it */
    }
    if(handle->msgs.wifi_connected && handle->msgs.wifi_got_ip && ok_received)
    {
        handle->msgs.wifi_connected = handle->msgs.wifi_got_ip = 0;
        ESP32_OnWifiConnectioStateChange(handle, 0);  /* Notify the user about successfull wifi connection */
    }
    handle->msgs.wifi_connected = handle->msgs.wifi_got_ip = 0;

    return HAL_Status;
}

/**
 * !\brief Connect to server
 * !\param handle Esp32 handle
 * !\param link_id Network connection Link ID (0 - 4)
 * !\param connection_mode Connection mode string (TCP/UDP/SSL)
 * !\param ip_addr IP Address (TCP) or Remote IP Address (UDP)
 * !\param port Port (TCP) or Remote Port (UDP)
 * !\param keep_alive Detection time interval when TCP is kept alive (0: disable TCP keep-alive) or UDP Local Port
 * !\param udp_mode UDP Mode (THIS PARAMETER ISN'T USED IN TCP CONNECTION)
 *  - the entity of UDP transmission. For UDP transparent transmission, the value of this parameter has to be 0
 *  - 0: the destination peer entity of UDP will not change; this is the default setting.
 *  - 1: the destination peer entity of UDP will change once
 *  - 2: the destination peer entity of UDP is allowed to change
 * !\return HAL_Status
 */
static HAL_StatusTypeDef ConnectToServer(Esp32Handle* handle, uint8_t link_id, char* connection_mode, char* ip_addr, uint16_t port,
    uint16_t keep_alive, uint8_t udp_mode)
{
    uint8_t to_exit_loop = 0, rec_connect = 0, rec_ok = 0;
    HAL_StatusTypeDef HAL_Status = HAL_OK;
    if(udp_mode == 0xFF)
        HAL_Status = ExecuteAtCommand(handle, 1300, "AT+CIPSTART=%d,\"%s\",\"%s\",%d,%d", link_id, connection_mode, ip_addr, port, keep_alive);
    else
        HAL_Status = ExecuteAtCommand(handle, 1300, "AT+CIPSTART=%d,\"%s\",\"%s\",%d,%d,%d", link_id, connection_mode, ip_addr, port, keep_alive, udp_mode);
    if(HAL_Status == HAL_OK)
    {
        HAL_Status = HAL_ERROR;
        handle->connect_in_progress = 1;
        handle->msgs.tcp_connect &=
            (uint8_t)~((uint8_t)(1U << link_id));
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Other:
                {
                    if(handle->msgs.tcp_connect & (uint8_t)(1U << link_id))
                    {
                        rec_connect = 1;
                        HAL_Status = HAL_OK;
                    }
                    break;
                }
                case kEsp32Ok:
                    rec_ok = 1;
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
            if((HAL_Status == HAL_OK && rec_connect && rec_ok))
            {
                to_exit_loop = 1;
            }
        }
    }
    handle->msgs.tcp_connect &=
        (uint8_t)~((uint8_t)(1U << link_id));
    handle->connect_in_progress = 0;
    return HAL_Status;
}

/**
 * !\brief Disconnect from server
 * !\param handle Esp32 handle
 * !\param link_id Network connection Link ID (0 - 4)
 * !\return HAL_Status
 */
static HAL_StatusTypeDef DisconnectFromServer(Esp32Handle* handle, uint8_t link_id)
{
    uint8_t to_exit_loop = 0;
    HAL_StatusTypeDef HAL_Status = ExecuteAtCommand(handle, 100, "AT+CIPCLOSE=%d", link_id);
    if(HAL_Status == HAL_OK)
    {
        HAL_Status = HAL_ERROR;
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Other:
                {
                    uint8_t disconnect_link_id = atoi((char*)(&handle->line_rx[0]));
                    if(!memcmp(&handle->line_rx[1], ",CLOSED", 7))
                    {
                        if(disconnect_link_id == link_id)
                            HAL_Status = HAL_OK;
                    }
                    break;
                }
                case kEsp32Ok:
                    if(HAL_Status == HAL_OK)
                    {
                        HAL_Status = HAL_OK;
                    }
                    to_exit_loop = 1;
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Get local ip address of station and soft AP
 * !\note All buffers size must be ESP32_MAX_IP_MAC_LEN!!!
 * !\warning Only works when the ESP32 Station is connected to an AP can the Station IP be queried.
 * !\param handle Esp32 handle
 * !\param soft_ap_ip IP address of the ESP32 SoftAP
 * !\param soft_ap_mac MAC address of the ESP32 SoftAP
 * !\param station_ip IP address of the ESP32 Station.
 * !\param station_mac MAC address of the ESP32 Station
 * !\return HAL_Status
 */
static HAL_StatusTypeDef GetLocalIpAddress(Esp32Handle* handle, char* soft_ap_ip, char* soft_ap_mac, char* station_ip, char* station_mac)
{
    uint8_t to_exit_loop = 0;
    uint8_t count = 0;
    uint8_t other_response, ok_response = 0;
    HAL_StatusTypeDef HAL_Status = ExecuteAtCommand(handle, 500, "AT+CIFSR");
    if(HAL_Status == HAL_OK)
    {
        memset(soft_ap_ip, 0, ESP32_MAX_IP_MAC_LEN);
        memset(soft_ap_mac, 0, ESP32_MAX_IP_MAC_LEN);
        memset(station_ip, 0, ESP32_MAX_IP_MAC_LEN);
        memset(station_mac, 0, ESP32_MAX_IP_MAC_LEN);
        uint8_t count_needed = 2;
        if(handle->wifi_mode == kWiFiSoftApStation)
            count_needed = 4;
        while(!to_exit_loop)
        {
            const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Other:
                {
                    char address_buffer[ESP32_MAX_IP_MAC_LEN] = {0};
                    if(sscanf((char*)handle->line_rx, "+CIFSR:APIP,\"%16[^\"]", address_buffer) == 1)
                        CopyString(soft_ap_ip, ESP32_MAX_IP_MAC_LEN, address_buffer);
                    if(sscanf((char*)handle->line_rx, "+CIFSR:APMAC,\"%17[^\"]", address_buffer) == 1)
                        CopyString(soft_ap_mac, ESP32_MAX_IP_MAC_LEN, address_buffer);
                    if(sscanf((char*)handle->line_rx, "+CIFSR:STAIP,\"%16[^\"]", address_buffer) == 1)
                        CopyString(station_ip, ESP32_MAX_IP_MAC_LEN, address_buffer);
                    if(sscanf((char*)handle->line_rx, "+CIFSR:STAMAC,\"%17[^\"]", address_buffer) == 1)
                        CopyString(station_mac, ESP32_MAX_IP_MAC_LEN, address_buffer);
                    other_response = 1;
                    if(++count == count_needed && ok_response)
                    {
                        HAL_Status = HAL_OK;
                        to_exit_loop = 1;
                    }
                    break;
                }
                case kEsp32Ok:
                    HAL_Status = HAL_OK;
                    ok_response = 1;
                    if(count == count_needed && other_response)
                        to_exit_loop = 1;
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
        }
    }
    return HAL_Status;
}

/**
 * !\brief Send data to server
 * !\param handle Esp32 handle
 * !\param link_id Network connection Link ID (0 - 4)
 * !\param data Data to send
 * - When the requirement of data length, determined by <length>, is met, or when \0 appears, the transmission of data starts.
 *   Go back to the normal command mode and wait for the next AT command.
 * - When sending \0, please send it as \\0.
 * !\param length Data length, MAX: 2048 bytes
 * !\param remote_ip Remote IP (ONLY WITH UDP!)
 * !\param remote_port Remote Port (ONLY WITH UDP!)
 * !\return HAL_Status
 */
static HAL_StatusTypeDef Send(Esp32Handle* handle, uint8_t link_id, uint8_t* data, uint16_t length, char* remote_ip, uint16_t remote_port)
{
    uint8_t to_exit_loop = 0, ok_recv = 0, sendok_recv = 0, recv_recv = 0;
    volatile HAL_StatusTypeDef HAL_Status = HAL_ERROR;
    if(remote_port == 0xFFFF)
        HAL_Status = ExecuteAtCommand(handle, 400, "AT+CIPSEND=%d,%d", link_id, length);
    else
        HAL_Status = ExecuteAtCommand(handle, 400, "AT+CIPSEND=%d,%d,\"%s\",%d", link_id, length, remote_ip, remote_port);
    if(HAL_Status == HAL_OK)
    {
        HAL_Status = HAL_ERROR;
        while(!to_exit_loop)
        {
            volatile const Esp32AtResponse resp = WaitForResponse(handle);
            switch(resp)
            {
                case kEsp32Other:
                {
                    if(!memcmp((char*)(handle->line_rx), "SEND OK", 7))
                    {
                        FlushBuffers(handle);
                        //ESP32_TxCpltCallback(handle, link_id, data, length);
                        HAL_Status = HAL_OK;
                        sendok_recv = 1;
                        break;
                    }
                    if(!memcmp((char*)(handle->line_rx), "SEND FAIL", 9))
                    {
                        FlushBuffers(handle);
                        HAL_Status = HAL_ERROR;
                        to_exit_loop = 1;
                        break;
                    }
                    if(!memcmp((char*)(handle->line_rx), "Recv ", 5))
                    {
                        FlushBuffers(handle);
                        HAL_Status = HAL_OK;
                        recv_recv = 1;
                        break;
                    }
                    if(handle->line_rx[0] == '>')
                    {
                        HAL_Status = TransmitBytes(handle, data, length);
                        break;
                    }
                    break;
                }
                case kEsp32Ok:
                    HAL_Status = HAL_OK;
                    ok_recv = 1;
                    break;
                default:
                    HAL_Status = resp == kEsp32Timeout ? HAL_TIMEOUT : HAL_ERROR;
                    to_exit_loop = 1;
                    break;
            }
            if((recv_recv || (ok_recv && sendok_recv && recv_recv)) && HAL_Status == HAL_OK)
            {
                if(!sendok_recv)  /* Idiot ESP - sometimes SEND OK message is missing */
                    osDelay(100);
                to_exit_loop = 1;
            }
        }
    }
    return HAL_Status;
}


/**
 * !\brief Initializes Esp32
 * !\param handle Esp32 Handle
 * !\param usart_handle USART handle
 * !\param usart_dma_handle USART RX DMA handle (only required if ESP32_USE_UART_RX_DMA is 1)
 * !\param en_gpio_port EN GPIO Port (if NULL, reset is performed by AT+RST)
 * !\param en_pin EN GPIO Pin (if NULL, reset is performed by AT+RST)
 * !\param rst_gpio_port Reset GPIO Port (only for ESP8266, otherwise set it to NULL)
 * !\param rst_pin Reset GPIO Pin (only for ESP8266, otherwise set it to NULL)
 * !\param baudarate Desired baudrate for ESP (default is 115200)
 * !\return HAL_Status
 */
HAL_StatusTypeDef ESP32_Init(Esp32Handle* handle, USART_TypeDef* usart_handle,
#if ESP32_USE_UART_RX_DMA == 1
    DMA_TypeDef* rx_dma, DMA_Channel_TypeDef* rx_dma_channel, uint8_t rx_dma_channel_id,
    DMA_TypeDef* tx_dma, DMA_Channel_TypeDef* tx_dma_channel, uint8_t tx_dma_channel_id,
#endif
    GPIO_TypeDef* en_gpio_port, uint16_t en_pin, GPIO_TypeDef* rst_gpio_port, uint16_t rst_pin, uint32_t baudrate)
{
    if(handle == NULL || usart_handle == NULL || en_gpio_port == NULL || en_pin == 0U)
        return HAL_ERROR;
#ifdef DEBUG
    assert_param(handle != NULL);
    assert_param(usart_handle != NULL);
    assert_param(en_gpio_port != NULL);
    assert_param(en_pin != 0);
#endif
    HAL_StatusTypeDef HAL_Status = HAL_OK;
    memset(handle, 0, sizeof(*handle));
    handle->uart_handle = usart_handle;
#if ESP32_USE_UART_RX_DMA == 1
    if(UsartDriver_Install(&esp32_usart, usart_handle, rx_dma, rx_dma_channel,
                           rx_dma_channel_id, tx_dma, tx_dma_channel,
                           tx_dma_channel_id) != USART_DRIVER_SOURCE_ESP32)
        return HAL_ERROR;
#endif
    handle->en_gpio_port = en_gpio_port;
    handle->en_pin = en_pin;
    handle->rst_gpio_port = rst_gpio_port;
    handle->rst_pin = rst_pin;
    handle->baudrate = baudrate;
    circbuff_init(&handle->circbuff_def, handle->rx_circular_buffer, ESP32_RX_CIRCULAT_BUF_SIZE);
#if ESP32_USE_UART_RX_DMA == 1
#else
    HAL_Status = HAL_UART_Receive_IT(handle->uart_handle, &handle->uart_data_byte, 1);
#endif
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_Reset(Esp32Handle* handle, uint8_t restore_factory_defaults)
{
    HAL_StatusTypeDef HAL_Status = Reset(handle, restore_factory_defaults);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_GetVersionInfo(Esp32Handle* handle, char* output_buffer, uint8_t len)
{
    HAL_StatusTypeDef HAL_Status = GetVersionInfo(handle, output_buffer, len);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_GetBaudRate(Esp32Handle* handle, uint32_t* baudrate, uint8_t* databits, uint8_t* stopbits, uint8_t* parity,
    uint8_t* flow_control)
{
    HAL_StatusTypeDef HAL_Status = GetBaudRate(handle, baudrate, databits, stopbits, parity, flow_control);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_SetBaudRate(Esp32Handle* handle, uint32_t baudrate, uint8_t databits, uint8_t stopbits, uint8_t parity,
    uint8_t flow_control)
{
    HAL_StatusTypeDef HAL_Status = SetBaudRate(handle, baudrate, databits, stopbits, parity, flow_control);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_DisableEcho(Esp32Handle* handle)
{
    HAL_StatusTypeDef HAL_Status = DisableEcho(handle);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_ToggleAutoConn(Esp32Handle* handle, uint8_t toggle)
{
    HAL_StatusTypeDef HAL_Status = ToggleAutoConn(handle, toggle);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_SetWifiMode(Esp32Handle* handle, Esp32WiFiMode wifi_mode)
{
    HAL_StatusTypeDef HAL_Status = SetWifiMode(handle, wifi_mode);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_ToggleMultiConnection(Esp32Handle* handle, uint8_t multi_connect_mode)
{
    HAL_StatusTypeDef HAL_Status = ToggleMultiConnection(handle, multi_connect_mode);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_SetServerMaxConnection(Esp32Handle* handle, uint8_t num_connections)
{
    HAL_StatusTypeDef HAL_Status = SetServerMaxConnection(handle, num_connections);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_CreateDeleteServer(Esp32Handle* handle, uint8_t mode, uint16_t port, uint8_t is_ssl, uint8_t enable_ssl_ca)
{
    HAL_StatusTypeDef HAL_Status = CreateDeleteServer(handle, mode, port, is_ssl, enable_ssl_ca);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_JoinAccesssPoint(Esp32Handle* handle, char* ap_name, char* password, char* bssid, uint8_t try_to_reconnect)
{
    HAL_StatusTypeDef HAL_Status = JoinAccesssPoint(handle, ap_name, password, bssid, try_to_reconnect);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_ConnectToServer(Esp32Handle* handle, uint8_t link_id, char* connection_mode, char* ip_addr, uint16_t port,
    uint16_t udp_local_port, uint8_t udp_mode)
{
    CorrectLinkIdIfInvalid(&link_id);
    HAL_StatusTypeDef HAL_Status = ConnectToServer(handle, link_id, connection_mode, ip_addr, port, udp_local_port, udp_mode);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_DisconnectFromServer(Esp32Handle* handle, uint8_t link_id)
{
    CorrectLinkIdIfInvalid(&link_id);
    HAL_StatusTypeDef HAL_Status = DisconnectFromServer(handle, link_id);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_GetLocalIpAddress(Esp32Handle* handle, char* soft_ap_ip, char* soft_ap_mac, char* station_ip, char* station_mac)
{
    HAL_StatusTypeDef HAL_Status = GetLocalIpAddress(handle, soft_ap_ip, soft_ap_mac, station_ip, station_mac);
    return HAL_Status;
}

HAL_StatusTypeDef ESP32_SendTcp(Esp32Handle* handle, uint8_t link_id, uint8_t* data, uint32_t length)
{
    HAL_StatusTypeDef HAL_Status = Send(handle, link_id, data, length, NULL, 0xFFFF);
    return HAL_Status;
}

/**
 * @brief Attempt to reconnect lost WiFi connection
 * @param handle Esp32 handle
 */
static void DoWifiReconnect(Esp32Handle* handle)
{
    HAL_StatusTypeDef HAL_Status = JoinAccesssPoint(handle, handle->ap_name, handle->password, NULL, 0);
    if(HAL_Status == HAL_OK)
    {
        handle->is_wifi_connection_lost = 0;
        ESP32_OnWifiConnectioStateChange(handle, 0);
    }
    else
    {
        ESP32_OnReconnectFailed(handle);
    }
}

/**
 * !\brief Task function
 * !\param handle Esp32 handle
 */
void ESP32_Task(Esp32Handle* handle)
{
    if(!handle->uart_timeout_counter)
    {
        while(1)  /* If data is waiting to be processed */
        {
            const uint8_t recv = ReceiveMessage(handle);
            const circbuffsize_t datalen = circbuff_get_data_len(&handle->circbuff_def);
            if((!datalen && !recv) || !handle->line_rx[0])
            {
                break;
            }
        }
    }

    if(handle->is_wifi_connection_lost)
    {
        if((HAL_GetTick() - handle->last_reconnection_tick) > ESP32_RECONNECT_INTERVAL)
        {
            if(handle->is_wifi_connection_lost)  /* Always try to first reconnect to WiFi if both WiFi & TCP connection are lost */
            {
                DoWifiReconnect(handle);
                handle->last_reconnection_tick = HAL_GetTick();
            }
        }
    }

}

/**
 * !\brief Lower ESP32 timeout counter
 * !\param handle Esp32 handle
 * !\details This function MUST be called from an interrupt in every 10ms
 */
void ESP32_10Ms_Isr(Esp32Handle* handle)
{
    if(handle->timeout_timer_count)
        handle->timeout_timer_count--;
}

/**
 * !\brief Lower UART byte timeout counter
 * !\param handle Esp32 handle
 * !\details This function MUST be called from an interrupt in every 1ms
 */
void ESP32_SysTick(Esp32Handle* handle)
{
    if(handle->uart_timeout_counter)
        handle->uart_timeout_counter--;
}

/**
 * !\brief Process data received via UART
 * !\param handle Esp32 handle
 * !\param data Data received via UART
 * !\param len Legth of received data
 */
void ESP32_InsertDataToBuffer(Esp32Handle* handle, const uint8_t* data, uint16_t len)
{
    if(handle == NULL || data == NULL || len == 0U)
        return;
    handle->uart_timeout_counter = ESP32_UART_BYTE_TIMEOUT;
    if(circbuff_insert_bytes(&handle->circbuff_def, data, len) == 0U)
        circbuff_flush(&handle->circbuff_def);
}

#if 0
__weak void ESP32_RxCpltCallback(Esp32Handle* handle, uint8_t link_id, uint8_t* data, uint16_t length)
{

}

__weak void ESP32_TxCpltCallback(Esp32Handle* handle, uint8_t link_id, uint8_t* data, uint16_t length)
{

}

__weak void ESP32_OnWifiConnectioStateChange(Esp32Handle* handle, uint8_t is_lost)
{

}

__weak void ESP32_OnBaudrateError(Esp32Handle* handle)
{

}

__weak void ESP32_OnUnexpectedRestart(Esp32Handle* handle)
{

}

__weak void ESP32_OnReconnectFailed(Esp32Handle* handle)
{

}
#endif
