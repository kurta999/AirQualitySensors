#include "app_config.h"
#if APP_USB_RNDIS_ENABLED

#include "usb_net.h"
#include "usb_dhcp.h"
#include "cmsis_os.h"
#include "main.h"
#include "measurement_store.h"
#include "measurement_format.h"
#include "task_health.h"
#include "tusb.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "lwip/timeouts.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "netif/ethernet.h"
#include "newlib_fixes/printf.h"

#include <math.h>
#include <string.h>

volatile UsbNetDiagnostics usb_net_diagnostics;
uint8_t tud_network_mac_address[6];
static struct netif usb_interface;
static const uint8_t server_ip[4] = {192,168,7,1};
static const uint8_t client_ip[4] = {192,168,7,2};
static const uint8_t subnet_mask[4] = {255,255,255,252};
static struct tcp_pcb* receiver;
static uint32_t reconnect_at;
static uint32_t last_sent;

typedef struct { uint16_t length; uint8_t data[1514]; } EthernetFrame;
static EthernetFrame transmit_queue[8];
static unsigned int transmit_head;
static unsigned int transmit_count;

typedef struct
{
    struct tcp_pcb* pcb;
    uint32_t started;
    uint16_t request_length;
    uint16_t reply_length;
    uint16_t acknowledged;
    uint8_t submitted;
    char request[768];
    char reply[2304];
} HttpConnection;
static HttpConnection http_connections[4];

uint32_t sys_now(void) { return HAL_GetTick(); }
uint32_t tusb_time_millis_api(void) { return HAL_GetTick(); }
uint32_t UsbNet_Random(void)
{
    static uint32_t value;
    if(value == 0U) { value = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetTick() ^ 0x19660721U; }
    value ^= value << 13U; value ^= value >> 17U; value ^= value << 5U;
    return value;
}
void UsbNet_Assert(const char* message)
{
    usb_net_diagnostics.assertion = message;
    Error_Handler();
}

void UsbNet_IrqHandler(void) { tud_int_handler(0); }

static void usb_hardware_init(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    oscillator.HSI48State = RCC_HSI48_ON;
    if(HAL_RCC_OscConfig(&oscillator) != HAL_OK) { Error_Handler(); }
    RCC_PeriphCLKInitTypeDef clock = {0};
    clock.PeriphClockSelection = RCC_PERIPHCLK_USB;
    clock.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if(HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) { Error_Handler(); }
    __HAL_RCC_CRS_CLK_ENABLE();
    RCC_CRSInitTypeDef sync = {0};
    sync.Prescaler = RCC_CRS_SYNC_DIV1;
    sync.Source = RCC_CRS_SYNC_SOURCE_USB;
    sync.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    sync.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U);
    sync.ErrorLimitValue = 34;
    sync.HSI48CalibrationValue = 32;
    HAL_RCCEx_CRSConfig(&sync);
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef pins = {0};
    pins.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    pins.Mode = GPIO_MODE_AF_PP;
    pins.Pull = GPIO_NOPULL;
    pins.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pins.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &pins);
    pins.Pin = GPIO_PIN_9;
    pins.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &pins);
    HAL_PWREx_EnableVddUSB();
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0);
    tud_configure_dwc2_t config = CFG_TUD_CONFIGURE_DWC2_DEFAULT;
    config.vbus_sensing = true;
    if(!tud_configure(0, TUD_CFGID_DWC2, &config)) { Error_Handler(); }
}

static err_t link_output(struct netif* interface, struct pbuf* packet)
{
    (void)interface;
    if(!tud_ready()) { return ERR_IF; }
    if(packet->tot_len > sizeof(transmit_queue[0].data) || transmit_count == 8U)
    {
        ++usb_net_diagnostics.dropped_frames;
        return ERR_MEM;
    }
    EthernetFrame* frame = &transmit_queue[(transmit_head + transmit_count) % 8U];
    frame->length = pbuf_copy_partial(packet, frame->data, packet->tot_len, 0U);
    ++transmit_count;
    return ERR_OK;
}

static err_t interface_init(struct netif* interface)
{
    interface->name[0] = 'u'; interface->name[1] = 's';
    interface->hostname = "airquality";
    interface->mtu = 1500;
    interface->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
    interface->output = etharp_output;
    interface->linkoutput = link_output;
    interface->hwaddr_len = 6;
    memcpy(interface->hwaddr, tud_network_mac_address, 6);
    interface->hwaddr[5] ^= 1U;
    return ERR_OK;
}

bool tud_network_recv_cb(const uint8_t* source, uint16_t length)
{
    ++usb_net_diagnostics.rx_frames;
    if(length >= 14U && length <= 1514U)
    {
        struct pbuf* packet = pbuf_alloc(PBUF_RAW, length, PBUF_POOL);
        if(packet != NULL)
        {
            if(pbuf_take(packet, source, length) != ERR_OK ||
               usb_interface.input(packet, &usb_interface) != ERR_OK) { pbuf_free(packet); }
        }
        else { ++usb_net_diagnostics.dropped_frames; }
    }
    else { ++usb_net_diagnostics.dropped_frames; }
    tud_network_recv_renew();
    return true;
}

uint16_t tud_network_xmit_cb(uint8_t* destination, void* reference, uint16_t argument)
{
    (void)argument;
    const EthernetFrame* frame = reference;
    memcpy(destination, frame->data, frame->length);
    return frame->length;
}

void tud_network_init_cb(void)
{
    transmit_head = 0U;
    transmit_count = 0U;
}
void tud_mount_cb(void)
{
    usb_net_diagnostics.mounted = 1;
    netif_set_link_up(&usb_interface);
}
void tud_umount_cb(void)
{
    usb_net_diagnostics.mounted = 0;
    netif_set_link_down(&usb_interface);
    transmit_count = 0;
    if(receiver != NULL) { tcp_abort(receiver); receiver = NULL; }
    usb_net_diagnostics.client_connected = 0;
}

static void dhcp_receive(void* argument, struct udp_pcb* pcb, struct pbuf* packet,
                         const ip_addr_t* source, u16_t port)
{
    (void)argument; (void)source;
    uint8_t request[576];
    uint8_t reply[300];
    ++usb_net_diagnostics.dhcp_requests;
    if(port == 68U && packet->tot_len <= sizeof(request))
    {
        const uint16_t length = pbuf_copy_partial(packet, request, packet->tot_len, 0);
        const size_t reply_length = UsbDhcp_Reply(request, length, reply, sizeof(reply), server_ip, client_ip, subnet_mask);
        if(reply_length != 0U)
        {
            struct pbuf* response = pbuf_alloc(PBUF_TRANSPORT, (uint16_t)reply_length, PBUF_RAM);
            if(response != NULL)
            {
                if(pbuf_take(response, reply, (uint16_t)reply_length) == ERR_OK &&
                   udp_sendto_if(pcb, response, IP_ADDR_BROADCAST, 68U, &usb_interface) == ERR_OK)
                {
                    ++usb_net_diagnostics.dhcp_replies;
                }
                pbuf_free(response);
            }
        }
    }
    pbuf_free(packet);
}

static double finite_value(float value) { return isfinite(value) ? (double)value : 0.0; }

static int format_json(char* output, size_t capacity)
{
    MeasurementSnapshot m;
    Measurements_GetSnapshot(&m);
    return snprintf(output, capacity,
        "{\"uptime_ms\":%lu,\"valid_mask\":%lu,"
        "\"scd30\":{\"co2_ppm\":%.2f,\"temperature_c\":%.2f,\"humidity_percent\":%.2f,\"timestamp_ms\":%lu},"
        "\"bme680\":{\"temperature_c\":%.2f,\"humidity_percent\":%.2f,\"pressure_hpa\":%.2f,\"gas_ohm\":%lu,\"timestamp_ms\":%lu},"
        "\"uv_raw\":%u,\"light\":{\"lux\":%u,\"color_temperature_k\":%u},"
        "\"iaq\":{\"index\":%.2f,\"accuracy\":%u,\"stabilized\":%u,\"run_in\":%u,\"timestamp_ms\":%lu},"
        "\"usb\":{\"dhcp_replies\":%lu,\"tcp_connected\":%lu,\"records_sent\":%lu,\"acknowledged_bytes\":%lu}}\n",
        (unsigned long)m.snapshot_timestamp_ms, (unsigned long)m.valid_mask,
        finite_value(m.scd30_co2_ppm), finite_value(m.scd30_temperature_c), finite_value(m.scd30_relative_humidity_percent),
        (unsigned long)m.timestamp_ms[MEASUREMENT_SOURCE_SCD30],
        finite_value(m.bme680_temperature_c), finite_value(m.bme680_relative_humidity_percent), finite_value(m.bme680_pressure_hpa),
        (unsigned long)m.bme680_gas_resistance_ohm, (unsigned long)m.timestamp_ms[MEASUREMENT_SOURCE_BME680],
        (unsigned int)m.uv_raw, (unsigned int)m.lux, (unsigned int)m.color_temperature_k,
        finite_value(m.air_quality.index), (unsigned int)m.air_quality.accuracy,
        (unsigned int)m.air_quality.stabilized, (unsigned int)m.air_quality.run_in,
        (unsigned long)m.timestamp_ms[MEASUREMENT_SOURCE_IAQ],
        (unsigned long)usb_net_diagnostics.dhcp_replies, (unsigned long)usb_net_diagnostics.client_connected,
        (unsigned long)usb_net_diagnostics.records_sent, (unsigned long)usb_net_diagnostics.acknowledged_bytes);
}

static void http_release(HttpConnection* connection)
{
    connection->pcb = NULL;
}
static void http_error(void* argument, err_t error)
{
    (void)error;
    http_release(argument);
}
static err_t http_close(HttpConnection* connection)
{
    struct tcp_pcb* pcb = connection->pcb;
    tcp_arg(pcb, NULL); tcp_err(pcb, NULL); tcp_recv(pcb, NULL); tcp_sent(pcb, NULL); tcp_poll(pcb, NULL, 0);
    http_release(connection);
    if(tcp_close(pcb) != ERR_OK) { tcp_abort(pcb); return ERR_ABRT; }
    return ERR_OK;
}
static err_t http_sent(void* argument, struct tcp_pcb* pcb, u16_t length)
{
    (void)pcb;
    HttpConnection* connection = argument;
    connection->acknowledged = (uint16_t)(connection->acknowledged + length);
    if(connection->acknowledged >= connection->reply_length) { return http_close(connection); }
    return ERR_OK;
}
static err_t http_poll(void* argument, struct tcp_pcb* pcb)
{
    HttpConnection* connection = argument;
    if(HAL_GetTick() - connection->started > 10000U) { return http_close(connection); }
    if(connection->reply_length != 0U && !connection->submitted)
    {
        if(tcp_write(pcb, connection->reply, connection->reply_length, TCP_WRITE_FLAG_COPY) == ERR_OK)
        {
            connection->submitted = 1;
            (void)tcp_output(pcb);
        }
    }
    return ERR_OK;
}
static err_t http_reply(HttpConnection* connection)
{
    static char body[1536];
    const char* type = "application/json";
    const char* status = "200 OK";
    int length;
    ++usb_net_diagnostics.http_requests;
    if(strncmp(connection->request, "GET /api/measurements ", 22) == 0)
    {
        length = format_json(body, sizeof(body));
    }
    else if(strncmp(connection->request, "GET /measurements ", 18) == 0)
    {
        MeasurementSnapshot snapshot;
        Measurements_GetSnapshot(&snapshot);
        length = Measurements_Format(body, sizeof(body), &snapshot);
        type = "text/plain";
    }
    else if(strncmp(connection->request, "GET / ", 6) == 0)
    {
        static const char page[] = "<!doctype html><meta charset=utf-8><title>Air quality sensors</title>"
            "<h1>Air quality sensors</h1><p>Connected over USB.</p><pre id=v>Loading measurements...</pre>"
            "<script>async function update(){try{const r=await fetch('/api/measurements',{cache:'no-store'});"
            "v.textContent=JSON.stringify(await r.json(),null,2)}catch(e){v.textContent='Connection lost'}}"
            "update();setInterval(update,2000)</script>";
        memcpy(body, page, sizeof(page));
        length = sizeof(page) - 1U;
        type = "text/html; charset=utf-8";
    }
    else
    {
        status = "404 Not Found"; type = "text/plain";
        memcpy(body, "Not found\n", 11U); length = 10;
    }
    if(length < 0 || (size_t)length >= sizeof(body)) { status = "500 Internal Server Error"; length = 0; }
    const int header = snprintf(connection->reply, sizeof(connection->reply),
        "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
        status, type, length);
    if(header < 0 || (size_t)header + (size_t)length > sizeof(connection->reply)) { return http_close(connection); }
    memcpy(connection->reply + header, body, (size_t)length);
    connection->reply_length = (uint16_t)(header + length);
    return http_poll(connection, connection->pcb);
}
static err_t http_receive(void* argument, struct tcp_pcb* pcb, struct pbuf* packet, err_t error)
{
    HttpConnection* connection = argument;
    if(packet == NULL) { return http_close(connection); }
    if(error != ERR_OK) { pbuf_free(packet); return ERR_OK; }
    const u16_t length = packet->tot_len;
    tcp_recved(pcb, length);
    if(connection->reply_length == 0U)
    {
        if(length >= sizeof(connection->request) - connection->request_length)
        {
            pbuf_free(packet);
            return http_close(connection);
        }
        pbuf_copy_partial(packet, connection->request + connection->request_length, length, 0);
        connection->request_length = (uint16_t)(connection->request_length + length);
        connection->request[connection->request_length] = 0;
        /* Preserve the companion application's existing force-send command. */
        static const char force_send[] = "SEND_SENSOR_DATA";
        size_t command_length = connection->request_length;
        while(command_length != 0U && (connection->request[command_length - 1U] == '\r' ||
              connection->request[command_length - 1U] == '\n')) { --command_length; }
        if(command_length == sizeof(force_send) - 1U &&
           memcmp(connection->request, force_send, sizeof(force_send) - 1U) == 0)
        {
            last_sent = HAL_GetTick() - APP_MEASUREMENT_SEND_INTERVAL_MS;
            reconnect_at = HAL_GetTick();
            pbuf_free(packet);
            return http_close(connection);
        }
        if(strstr(connection->request, "\r\n\r\n") != NULL)
        {
            const err_t result = http_reply(connection);
            pbuf_free(packet);
            return result;
        }
    }
    pbuf_free(packet);
    return ERR_OK;
}
static err_t http_accept(void* argument, struct tcp_pcb* pcb, err_t error)
{
    (void)argument;
    if(error != ERR_OK) { return error; }
    for(unsigned int i = 0; i < 4U; ++i)
    {
        HttpConnection* connection = &http_connections[i];
        if(connection->pcb == NULL)
        {
            memset(connection, 0, sizeof(*connection));
            connection->pcb = pcb;
            connection->started = HAL_GetTick();
            tcp_arg(pcb, connection); tcp_err(pcb, http_error); tcp_recv(pcb, http_receive);
            tcp_sent(pcb, http_sent); tcp_poll(pcb, http_poll, 2);
            return ERR_OK;
        }
    }
    tcp_abort(pcb);
    return ERR_ABRT;
}

static void receiver_error(void* argument, err_t error)
{
    (void)argument;
    receiver = NULL;
    usb_net_diagnostics.client_connected = 0;
    usb_net_diagnostics.last_tcp_error = error;
    reconnect_at = HAL_GetTick() + 5000U;
}
static err_t receiver_received(void* argument, struct tcp_pcb* pcb, struct pbuf* packet, err_t error)
{
    (void)argument;
    if(packet == NULL)
    {
        receiver = NULL;
        usb_net_diagnostics.client_connected = 0;
        tcp_err(pcb, NULL);
        if(tcp_close(pcb) != ERR_OK) { tcp_abort(pcb); return ERR_ABRT; }
        /* The existing PC receiver closes after each measurement record. */
        reconnect_at = last_sent + APP_MEASUREMENT_SEND_INTERVAL_MS;
        return ERR_OK;
    }
    tcp_recved(pcb, packet->tot_len);
    pbuf_free(packet);
    return error;
}
static err_t receiver_sent(void* argument, struct tcp_pcb* pcb, u16_t length)
{
    (void)argument; (void)pcb;
    usb_net_diagnostics.acknowledged_bytes += length;
    return ERR_OK;
}
static err_t receiver_connected(void* argument, struct tcp_pcb* pcb, err_t error)
{
    (void)argument;
    if(error == ERR_OK)
    {
        usb_net_diagnostics.client_connected = 1;
        usb_net_diagnostics.last_tcp_error = 0;
        tcp_recv(pcb, receiver_received); tcp_sent(pcb, receiver_sent);
    }
    return error;
}
static void send_measurements(void)
{
    const uint32_t now = HAL_GetTick();
    if(!tud_ready() || !netif_is_link_up(&usb_interface)) { return; }
    if(receiver == NULL && (int32_t)(now - reconnect_at) >= 0)
    {
        receiver = tcp_new();
        if(receiver != NULL)
        {
            ip_addr_t address; IP_ADDR4(&address, client_ip[0], client_ip[1], client_ip[2], client_ip[3]);
            tcp_err(receiver, receiver_error);
            const err_t result = tcp_connect(receiver, &address, APP_SERVER_PORT, receiver_connected);
            if(result != ERR_OK) { tcp_abort(receiver); }
        }
        reconnect_at = now + 5000U;
    }
    if(usb_net_diagnostics.client_connected && receiver != NULL && now - last_sent >= APP_MEASUREMENT_SEND_INTERVAL_MS)
    {
        MeasurementSnapshot snapshot;
        char record[MEASUREMENT_FORMAT_MAX_LEN];
        Measurements_GetSnapshot(&snapshot);
        const int length = Measurements_Format(record, sizeof(record), &snapshot);
        if(length > 0 && tcp_sndbuf(receiver) >= (uint16_t)length &&
           tcp_write(receiver, record, (uint16_t)length, TCP_WRITE_FLAG_COPY) == ERR_OK)
        {
            ++usb_net_diagnostics.records_sent;
            last_sent = now;
            (void)tcp_output(receiver);
        }
    }
}

void UsbNet_Task(void const* argument)
{
    (void)argument;
    const uint32_t unique = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
    tud_network_mac_address[0] = 0x02;
    tud_network_mac_address[1] = 0x53;
    for(unsigned int i = 0; i < 4U; ++i) { tud_network_mac_address[i + 2U] = (uint8_t)(unique >> (8U * i)); }
    tud_network_mac_address[5] &= 0xFEU;
    lwip_init();
    ip4_addr_t address, mask, gateway;
    IP4_ADDR(&address, server_ip[0], server_ip[1], server_ip[2], server_ip[3]);
    IP4_ADDR(&mask, subnet_mask[0], subnet_mask[1], subnet_mask[2], subnet_mask[3]);
    IP4_ADDR(&gateway, 0,0,0,0);
    if(netif_add(&usb_interface, &address, &mask, &gateway, NULL, interface_init, ethernet_input) == NULL) { Error_Handler(); }
    netif_set_default(&usb_interface);
    netif_set_up(&usb_interface);
    struct udp_pcb* dhcp = udp_new();
    if(dhcp == NULL || udp_bind(dhcp, IP_ADDR_ANY, 67) != ERR_OK) { Error_Handler(); }
    udp_bind_netif(dhcp, &usb_interface);
    udp_recv(dhcp, dhcp_receive, NULL);
    struct tcp_pcb* http = tcp_new();
    if(http == NULL || tcp_bind(http, IP_ADDR_ANY, 80) != ERR_OK) { Error_Handler(); }
    http = tcp_listen(http);
    if(http == NULL) { Error_Handler(); }
    tcp_accept(http, http_accept);
    usb_hardware_init();
    const tusb_rhport_init_t init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    if(!tusb_init(0, &init)) { Error_Handler(); }
    usb_net_diagnostics.initialized = 1;
    for(;;)
    {
        TaskHealth_ReportAlive(TASK_HEALTH_USB_NET, osKernelSysTick());
        tud_task_ext(0, false);
        sys_check_timeouts();
        send_measurements();
        if(transmit_count != 0U && tud_ready() && tud_network_can_xmit(transmit_queue[transmit_head].length))
        {
            tud_network_xmit(&transmit_queue[transmit_head], 0);
            transmit_head = (transmit_head + 1U) % 8U;
            --transmit_count;
            ++usb_net_diagnostics.tx_frames;
        }
        osDelay(1U);
    }
}
#endif
