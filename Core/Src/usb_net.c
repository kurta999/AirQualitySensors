#include "app_config.h"
#if APP_USB_RNDIS_ENABLED

#include "usb_net.h"
#include "device_config.h"
#include "device_web.h"
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
    interface->hostname = "air";
    interface->mtu = 1500;
    interface->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;
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
        DeviceConfig config;
        DeviceConfig_Get(&config);
        reconnect_at = last_sent + config.send_interval_ms;
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
    static DeviceConfig previous;
    DeviceConfig config;
    DeviceConfig_Get(&config);
    if(config.receiver_enabled != previous.receiver_enabled || config.receiver_port != previous.receiver_port ||
       strcmp(config.receiver_ip, previous.receiver_ip) != 0)
    {
        if(receiver != NULL) { tcp_abort(receiver); receiver = NULL; }
        usb_net_diagnostics.client_connected = 0;
        reconnect_at = now;
    }
    previous = config;
    if(!config.receiver_enabled) { return; }
    if(!tud_ready() || !netif_is_link_up(&usb_interface)) { return; }
    if(receiver == NULL && (int32_t)(now - reconnect_at) >= 0)
    {
        receiver = tcp_new();
        if(receiver != NULL)
        {
            ip_addr_t address;
            if(!ipaddr_aton(config.receiver_ip, &address)) { tcp_abort(receiver); return; }
            tcp_err(receiver, receiver_error);
            const err_t result = tcp_connect(receiver, &address, (uint16_t)config.receiver_port, receiver_connected);
            if(result != ERR_OK) { tcp_abort(receiver); }
        }
        reconnect_at = now + 5000U;
    }
    if(usb_net_diagnostics.client_connected && receiver != NULL && now - last_sent >= config.send_interval_ms)
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

void UsbNet_RequestSend(void)
{
    DeviceConfig config;
    DeviceConfig_Get(&config);
    last_sent = HAL_GetTick() - config.send_interval_ms;
    reconnect_at = HAL_GetTick();
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
    DeviceWeb_Start(&usb_interface);
    usb_hardware_init();
    const tusb_rhport_init_t init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    if(!tusb_init(0, &init)) { Error_Handler(); }
    usb_net_diagnostics.initialized = 1;
    for(;;)
    {
        TaskHealth_ReportAlive(TASK_HEALTH_USB_NET, osKernelSysTick());
        tud_task_ext(0, false);
        sys_check_timeouts();
        DeviceWeb_Service();
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
