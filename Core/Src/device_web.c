#include "app_config.h"
#if APP_USB_RNDIS_ENABLED
#include "device_web.h"
#include "device_config.h"
#include "measurement_history.h"
#include "measurement_format.h"
#include "http_request.h"
#include "usb_net.h"
#include "web_assets.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/tcp.h"
#include "lwip/apps/mdns.h"
#include "newlib_fixes/printf.h"
#include <math.h>
#include <string.h>

typedef struct
{
    struct tcp_pcb* pcb;
    uint32_t activity, started, acknowledged, body_length, queued;
    uint16_t request_length, header_length;
    uint8_t ready, header_queued;
    const uint8_t* body;
    char request[3072];
    char header[512];
    uint8_t buffer[4096];
} HttpConnection;
static HttpConnection connections[4];
static struct netif* network;
static DeviceConfig config;
static uint32_t revision, last_history_check;
extern uint8_t _sdata, _edata, _ebss, _sidata;

static uint32_t fresh_mask(const MeasurementSnapshot* m)
{
    if(!config.master_enabled) { return 0; }
    uint32_t mask = m->valid_mask & (config.sensor_mask | ((config.sensor_mask & 2U) ? 32U : 0U));
    for(unsigned int source = 0; source < MEASUREMENT_SOURCE_COUNT; ++source)
    {
        const uint32_t maximum = source == MEASUREMENT_SOURCE_SCD30 ? config.scd_interval_s * 2000U + 15000U : 15000U;
        if((uint32_t)(m->snapshot_timestamp_ms - m->timestamp_ms[source]) > maximum) { mask &= ~(1UL << source); }
    }
    return mask;
}
static double finite_value(float f) { return isfinite(f) ? (double)f : 0.0; }

static int measurements_json(char* output, size_t capacity)
{
    MeasurementSnapshot m;
    Measurements_GetSnapshot(&m);
    int n = snprintf(output, capacity,
        "{\"uptime_ms\":%lu,\"valid_mask\":%lu,\"fresh_mask\":%lu,\"config_revision\":%lu,"
        "\"scd30\":{\"co2_ppm\":%.2f,\"temperature_c\":%.2f,\"humidity_percent\":%.2f,\"timestamp_ms\":%lu},"
        "\"bme680\":{\"temperature_c\":%.2f,\"humidity_percent\":%.2f,\"pressure_hpa\":%.2f,\"gas_ohm\":%lu,\"timestamp_ms\":%lu},"
        "\"uv_raw\":%u,\"light\":{\"lux\":%u,\"color_temperature_k\":%u},"
        "\"iaq\":{\"index\":%.2f,\"accuracy\":%u,\"stabilized\":%u,\"run_in\":%u,\"timestamp_ms\":%lu},"
        "\"usb\":{\"dhcp_replies\":%lu,\"tcp_connected\":%lu,\"records_sent\":%lu,\"acknowledged_bytes\":%lu},\"values\":[",
        (unsigned long)m.snapshot_timestamp_ms, (unsigned long)m.valid_mask, (unsigned long)fresh_mask(&m),
        (unsigned long)DeviceConfig_Revision(), finite_value(m.scd30_co2_ppm), finite_value(m.scd30_temperature_c),
        finite_value(m.scd30_relative_humidity_percent), (unsigned long)m.timestamp_ms[0],
        finite_value(m.bme680_temperature_c), finite_value(m.bme680_relative_humidity_percent), finite_value(m.bme680_pressure_hpa),
        (unsigned long)m.bme680_gas_resistance_ohm, (unsigned long)m.timestamp_ms[1],
        m.uv_raw, m.lux, m.color_temperature_k, finite_value(m.air_quality.index),
        m.air_quality.accuracy, m.air_quality.stabilized, m.air_quality.run_in, (unsigned long)m.timestamp_ms[5],
        (unsigned long)usb_net_diagnostics.dhcp_replies, (unsigned long)usb_net_diagnostics.client_connected,
        (unsigned long)usb_net_diagnostics.records_sent, (unsigned long)usb_net_diagnostics.acknowledged_bytes);
    float values[HISTORY_COLUMNS];
    History_Values(&m, values);
    for(unsigned int i = 0; i < HISTORY_COLUMNS; ++i)
    {
        if(n < 0 || (size_t)n >= capacity) { return -1; }
        if(isfinite(values[i]))
        { n += snprintf(output + n, capacity - (size_t)n, "%s%.4f", i ? "," : "", (double)values[i]); }
        else { n += snprintf(output + n, capacity - (size_t)n, "%snull", i ? "," : ""); }
    }
    if(n < 0 || (size_t)n >= capacity) { return -1; }
    n += snprintf(output + n, capacity - (size_t)n, "],\"history_count\":%lu}\n", (unsigned long)History_Count(HAL_GetTick()));
    return n;
}

static int config_json(char* output, size_t capacity)
{
    const uint32_t sampling = History_IntervalSeconds(config.history_interval_s);
    uint32_t retention = sampling * (HISTORY_CAPACITY - 1U);
    if(retention > 86400U) { retention = 86400U; }
    return snprintf(output, capacity,
        "{\"config\":{\"master_enabled\":%lu,\"sensor_mask\":%lu,\"sensor_poll_ms\":%lu,\"light_poll_ms\":%lu,"
        "\"send_interval_ms\":%lu,\"receiver_enabled\":%lu,\"receiver_port\":%lu,\"history_interval_s\":%lu,"
        "\"scd_interval_s\":%lu,\"scd_pressure_mbar\":%lu,\"scd_temperature_offset\":%lu,\"scd_altitude_m\":%lu,"
        "\"scd_asc\":%lu,\"scd_recovery\":%lu,\"bsec_heat_offset\":%ld,\"bsec_save_interval_s\":%lu,"
        "\"light_gain\":%lu,\"light_cycles\":%lu,\"uv_integration\":%lu,"
        "\"receiver_ip\":\"%s\",\"hostname\":\"%s\",\"device_name\":\"%s\"},"
        "\"capabilities\":{\"sensor_mask\":%u,\"bsec\":%u},"
        "\"storage\":{\"sequence\":%lu,\"history_capacity\":%u,\"history_bytes\":%u,\"interval_s\":%lu,"
        "\"retention_s\":%lu,\"history_count\":%lu,\"ram1_static_bytes\":%lu,\"ram1_total_bytes\":262144,"
        "\"ram2_total_bytes\":65536,\"rtos_heap_free_bytes\":%lu,\"firmware_bytes\":%lu},"
        "\"network\":{\"ip\":\"192.168.7.1\",\"client_ip\":\"192.168.7.2\",\"transport\":\"USB RNDIS\"}}\n",
        (unsigned long)config.master_enabled, (unsigned long)config.sensor_mask,
        (unsigned long)config.sensor_poll_ms, (unsigned long)config.light_poll_ms,
        (unsigned long)config.send_interval_ms, (unsigned long)config.receiver_enabled, (unsigned long)config.receiver_port,
        (unsigned long)config.history_interval_s, (unsigned long)config.scd_interval_s, (unsigned long)config.scd_pressure_mbar,
        (unsigned long)config.scd_temperature_offset, (unsigned long)config.scd_altitude_m,
        (unsigned long)config.scd_asc, (unsigned long)config.scd_recovery, (long)config.bsec_heat_offset,
        (unsigned long)config.bsec_save_interval_s, (unsigned long)config.light_gain,
        (unsigned long)config.light_cycles, (unsigned long)config.uv_integration,
        config.receiver_ip, config.hostname, config.device_name,
        (APP_SCD30_ENABLED ? 1U : 0U) | (APP_BME680_ENABLED ? 2U : 0U) | (APP_VEML6070_ENABLED ? 8U : 0U) | (APP_TCS34725_ENABLED ? 16U : 0U),
        APP_BSEC_ENABLED, (unsigned long)DeviceConfig_Sequence(), HISTORY_CAPACITY, (unsigned int)(sizeof(HistoryPoint) * HISTORY_CAPACITY),
        (unsigned long)sampling, (unsigned long)retention, (unsigned long)History_Count(HAL_GetTick()),
        (unsigned long)((uintptr_t)&_ebss - UINT32_C(0x20000000)),
        (unsigned long)xPortGetFreeHeapSize(),
        (unsigned long)((uintptr_t)&_sidata + ((uintptr_t)&_edata - (uintptr_t)&_sdata) - FLASH_BASE));
}

static void release(HttpConnection* c) { c->pcb = NULL; }
static void on_error(void* argument, err_t error) { (void)error; release(argument); }
static err_t close_connection(HttpConnection* c)
{
    struct tcp_pcb* pcb = c->pcb;
    const int unacknowledged = c->ready && c->acknowledged < c->header_length + c->body_length;
    tcp_arg(pcb, NULL); tcp_err(pcb, NULL); tcp_recv(pcb, NULL); tcp_sent(pcb, NULL); tcp_poll(pcb, NULL, 0);
    release(c);
    if(unacknowledged) { tcp_abort(pcb); return ERR_ABRT; }
    if(tcp_close(pcb) != ERR_OK) { tcp_abort(pcb); return ERR_ABRT; }
    return ERR_OK;
}
static err_t poll_connection(void* argument, struct tcp_pcb* pcb)
{
    HttpConnection* c = argument;
    const uint32_t now = HAL_GetTick();
    if(now - c->activity > 10000U || now - c->started > 60000U) { return close_connection(c); }
    if(!c->ready) { return ERR_OK; }
    if(!c->header_queued)
    {
        if(tcp_write(pcb, c->header, c->header_length, TCP_WRITE_FLAG_COPY) != ERR_OK) { return ERR_OK; }
        c->header_queued = 1;
    }
    while(c->queued < c->body_length)
    {
        uint32_t length = c->body_length - c->queued;
        if(length > TCP_MSS) { length = TCP_MSS; }
        if(length > tcp_sndbuf(pcb)) { length = tcp_sndbuf(pcb); }
        if(length == 0U) { break; }
        /* Flash assets and per-connection API buffers remain valid until their
         * final ACK. Zero-copy pbufs keep the complete gzip file out of RAM. */
        if(tcp_write(pcb, c->body + c->queued, (uint16_t)length, 0) != ERR_OK) { break; }
        c->queued += length;
    }
    (void)tcp_output(pcb);
    return ERR_OK;
}
static err_t on_sent(void* argument, struct tcp_pcb* pcb, u16_t length)
{
    HttpConnection* c = argument;
    c->acknowledged += length;
    c->activity = HAL_GetTick();
    if(c->ready && c->acknowledged >= c->header_length + c->body_length) { return close_connection(c); }
    return poll_connection(c, pcb);
}
static err_t respond(HttpConnection* c, const char* status, const char* type, uint32_t length, int gzip)
{
    c->body_length = length;
    const int n = snprintf(c->header, sizeof(c->header),
        "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %lu\r\n%s"
        "Cache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\n"
        "Content-Security-Policy: default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'\r\n"
        "Connection: close\r\n\r\n", status, type, (unsigned long)length, gzip ? "Content-Encoding: gzip\r\nVary: Accept-Encoding\r\n" : "");
    if(n < 0 || (size_t)n >= sizeof(c->header)) { return close_connection(c); }
    c->header_length = (uint16_t)n;
    c->ready = 1;
    return poll_connection(c, c->pcb);
}
static err_t message(HttpConnection* c, const char* status, const char* body)
{
    c->body = (const uint8_t*)body;
    return respond(c, status, "application/json", (uint32_t)strlen(body), 0);
}

static err_t reply(HttpConnection* c, const HttpRequest* r)
{
    ++usb_net_diagnostics.http_requests;
    if(!HttpRequest_AllowedHost(r, config.hostname)) { return message(c, "403 Forbidden", "{\"error\":\"Same-origin device request required\"}"); }
    c->body = c->buffer;
    int n = 0;
    const char* type = "application/json";
    if(r->post)
    {
        if(strcmp(r->path, "/api/config") != 0) { return message(c, "404 Not Found", "{\"error\":\"Not found\"}"); }
        DeviceConfig candidate;
        if(!DeviceConfig_Parse(&config, r->body, r->body_length, &candidate))
        { return message(c, "400 Bad Request", "{\"error\":\"Invalid setting, unknown field, or out-of-range value\"}"); }
        if(DeviceConfig_Save(&candidate) != 0)
        { return message(c, "500 Internal Server Error", "{\"error\":\"Flash save failed; previous settings retained\"}"); }
        /* Renaming mDNS is deferred until the response is prepared. */
        config = candidate;
        n = config_json((char*)c->buffer, sizeof(c->buffer));
    }
    else if(strcmp(r->path, "/api/config") == 0) { n = config_json((char*)c->buffer, sizeof(c->buffer)); }
    else if(strcmp(r->path, "/api/measurements") == 0) { n = measurements_json((char*)c->buffer, sizeof(c->buffer)); }
    else if(strcmp(r->path, "/measurements") == 0)
    {
        MeasurementSnapshot m; Measurements_GetSnapshot(&m);
        n = Measurements_Format((char*)c->buffer, sizeof(c->buffer), &m); type = "text/plain";
    }
    else if(strcmp(r->path, "/api/history") == 0 || strncmp(r->path, "/api/history?after=", 19) == 0)
    {
        uint32_t after = 0;
        if(r->path[12] == '?')
        {
            const char* p = r->path + 19;
            if(*p == 0) { return message(c, "400 Bad Request", "{\"error\":\"Invalid history cursor\"}"); }
            for(; *p; ++p)
            {
                if(*p < '0' || *p > '9' || after > (UINT32_MAX - (uint32_t)(*p - '0')) / 10U)
                { return message(c, "400 Bad Request", "{\"error\":\"Invalid history cursor\"}"); }
                after = after * 10U + (uint32_t)(*p - '0');
            }
        }
        n = (int)History_Export(c->buffer, sizeof(c->buffer), after, HAL_GetTick(), config.history_interval_s);
        type = "application/octet-stream";
    }
    else
    {
        for(size_t i = 0; i < web_asset_count; ++i)
        {
            if(strcmp(r->path, web_assets[i].path) == 0)
            {
                c->body = web_assets[i].data;
                return respond(c, "200 OK", web_assets[i].type, web_assets[i].length, 1);
            }
        }
        return message(c, "404 Not Found", "{\"error\":\"Not found\"}");
    }
    if(n < 0 || (size_t)n >= sizeof(c->buffer)) { return message(c, "500 Internal Server Error", "{\"error\":\"Response too large\"}"); }
    return respond(c, "200 OK", type, (uint32_t)n, 0);
}
static err_t on_receive(void* argument, struct tcp_pcb* pcb, struct pbuf* packet, err_t error)
{
    HttpConnection* c = argument;
    if(packet == NULL) { return c->ready ? ERR_OK : close_connection(c); }
    if(error != ERR_OK) { pbuf_free(packet); return ERR_OK; }
    tcp_recved(pcb, packet->tot_len);
    if(c->ready) { pbuf_free(packet); return ERR_OK; }
    if(packet->tot_len >= sizeof(c->request) - c->request_length)
    { pbuf_free(packet); return message(c, "413 Content Too Large", "{\"error\":\"Request too large\"}"); }
    c->request_length += pbuf_copy_partial(packet, c->request + c->request_length, packet->tot_len, 0);
    c->request[c->request_length] = 0;
    c->activity = HAL_GetTick();
    pbuf_free(packet);
    size_t n = c->request_length;
    while(n && (c->request[n - 1U] == '\r' || c->request[n - 1U] == '\n')) { --n; }
    if(n == 16U && memcmp(c->request, "SEND_SENSOR_DATA", 16) == 0)
    { UsbNet_RequestSend(); return close_connection(c); }
    HttpRequest request;
    const int parsed = HttpRequest_Parse(c->request, c->request_length, &request);
    if(parsed < 0) { return message(c, "400 Bad Request", "{\"error\":\"Malformed HTTP request\"}"); }
    if(parsed == 0) { return ERR_OK; }
    return reply(c, &request);
}
static err_t on_accept(void* argument, struct tcp_pcb* pcb, err_t error)
{
    (void)argument;
    if(error != ERR_OK) { return error; }
    for(size_t i = 0; i < sizeof(connections) / sizeof(connections[0]); ++i)
    {
        HttpConnection* c = &connections[i];
        if(c->pcb == NULL)
        {
            memset(c, 0, sizeof(*c)); c->pcb = pcb; c->activity = c->started = HAL_GetTick();
            tcp_arg(pcb, c); tcp_err(pcb, on_error); tcp_recv(pcb, on_receive);
            tcp_sent(pcb, on_sent); tcp_poll(pcb, poll_connection, 2);
            return ERR_OK;
        }
    }
    tcp_abort(pcb); return ERR_ABRT;
}
void DeviceWeb_Start(struct netif* interface)
{
    network = interface;
    DeviceConfig_Get(&config);
    revision = DeviceConfig_Revision();
    History_Init();
    struct tcp_pcb* http = tcp_new();
    if(http == NULL || tcp_bind(http, IP_ADDR_ANY, 80) != ERR_OK) { Error_Handler(); }
    http = tcp_listen(http);
    if(http == NULL) { Error_Handler(); }
    tcp_accept(http, on_accept);
    mdns_resp_init();
    if(mdns_resp_add_netif(network, config.hostname, 120U) != ERR_OK) { Error_Handler(); }
    if(mdns_resp_add_service(network, config.device_name, "_http", DNSSD_PROTO_TCP, 80, 120, NULL, NULL) < 0) { Error_Handler(); }
}
void DeviceWeb_Service(void)
{
    if(revision != DeviceConfig_Revision())
    {
        DeviceConfig_Get(&config);
        revision = DeviceConfig_Revision();
        (void)mdns_resp_rename_netif(network, config.hostname);
        (void)mdns_resp_rename_service(network, 0, config.device_name);
    }
    const uint32_t now = HAL_GetTick();
    if(now - last_history_check >= 250U)
    {
        last_history_check = now;
        MeasurementSnapshot m; Measurements_GetSnapshot(&m);
        m.valid_mask = fresh_mask(&m);
        History_Service(&m, config.history_interval_s);
    }
}
#endif
