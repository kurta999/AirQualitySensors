#include "device_config.h"
#include "app_config.h"
#include <string.h>

void DeviceConfig_Defaults(DeviceConfig* c)
{
    memset(c, 0, sizeof(*c));
    c->version = 1;
    c->master_enabled = 1;
    c->sensor_mask = (APP_SCD30_ENABLED ? 1U : 0U) | (APP_BME680_ENABLED ? 2U : 0U) |
        (APP_VEML6070_ENABLED ? 8U : 0U) | (APP_TCS34725_ENABLED ? 16U : 0U);
    c->sensor_poll_ms = 1000;
    c->light_poll_ms = 1000;
    c->send_interval_ms = APP_MEASUREMENT_SEND_INTERVAL_MS;
    c->receiver_enabled = 1;
    c->receiver_port = APP_SERVER_PORT;
    c->scd_interval_s = 2;
    c->scd_pressure_mbar = APP_SCD30_AMBIENT_PRESSURE_MBAR;
    c->scd_asc = 1;
    c->scd_recovery = APP_SCD30_RECOVERY_ENABLED;
    c->bsec_heat_offset = (int32_t)(APP_BSEC_HEAT_SOURCE_C * 100.0F);
    c->bsec_save_interval_s = APP_BSEC_STATE_SAVE_INTERVAL_MS / 1000U;
    c->light_gain = 1;
    c->light_cycles = 21;
    c->uv_integration = 3;
#if APP_USB_RNDIS_ENABLED
    memcpy(c->receiver_ip, "192.168.7.2", 12);
#else
    const size_t address_length = sizeof(APP_SERVER_ADDRESS) - 1U;
    memcpy(c->receiver_ip, APP_SERVER_ADDRESS, address_length < sizeof(c->receiver_ip) ? address_length : sizeof(c->receiver_ip) - 1U);
#endif
    memcpy(c->hostname, "air", 4);
    memcpy(c->device_name, "Air station", 12);
}

static int ipv4_valid(const char* s)
{
    unsigned int first = 0;
    for(unsigned int octet = 0; octet < 4; ++octet)
    {
        unsigned int value = 0, count = 0;
        while(*s >= '0' && *s <= '9')
        {
            value = value * 10U + (unsigned int)(*s++ - '0');
            if(++count > 3U || value > 255U) { return 0; }
        }
        if(count == 0U) { return 0; }
        if(octet == 0) { first = value; }
        if(octet < 3U) { if(*s++ != '.') { return 0; } }
        else if(*s != 0) { return 0; }
    }
    return first > 0U && first < 224U && first != 127U;
}

int DeviceConfig_Validate(const DeviceConfig* c)
{
    if(c == NULL || c->version != 1U || c->master_enabled > 1U || (c->sensor_mask & ~27U) != 0U ||
       c->sensor_poll_ms < 250U || c->sensor_poll_ms > 5000U ||
       c->light_poll_ms < 100U || c->light_poll_ms > 5000U ||
       c->send_interval_ms < 250U || c->send_interval_ms > 3600000U ||
       c->receiver_enabled > 1U || c->receiver_port == 0U || c->receiver_port > 65535U ||
       (c->history_interval_s != 0U && (c->history_interval_s < 5U || c->history_interval_s > 3600U)) ||
       c->scd_interval_s < 2U || c->scd_interval_s > 1800U ||
       (c->scd_pressure_mbar != 0U && (c->scd_pressure_mbar < 700U || c->scd_pressure_mbar > 1400U)) ||
       c->scd_temperature_offset > 2000U || c->scd_altitude_m > 3000U ||
       c->scd_asc > 1U || c->scd_recovery > 1U ||
       c->bsec_heat_offset < -2000 || c->bsec_heat_offset > 2000 ||
       c->bsec_save_interval_s < 3600U || c->bsec_save_interval_s > 86400U ||
       c->light_gain > 3U || c->light_cycles < 1U || c->light_cycles > 256U || c->uv_integration > 3U)
    { return 0; }
    if(memchr(c->receiver_ip, 0, sizeof(c->receiver_ip)) == NULL || !ipv4_valid(c->receiver_ip) ||
       memchr(c->hostname, 0, sizeof(c->hostname)) == NULL || c->hostname[0] == 0 ||
       memchr(c->device_name, 0, sizeof(c->device_name)) == NULL || c->device_name[0] == 0)
    { return 0; }
    size_t i;
    for(i = 0; c->hostname[i]; ++i)
    {
        const char ch = c->hostname[i];
        if(!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || (ch == '-' && i != 0U))) { return 0; }
    }
    if(c->hostname[i - 1U] == '-') { return 0; }
    for(i = 0; c->device_name[i]; ++i)
    {
        const char ch = c->device_name[i];
        if(!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
             (ch >= '0' && ch <= '9') || ch == ' ' || ch == '-' || ch == '_')) { return 0; }
    }
    return 1;
}

typedef struct { const char* key; size_t offset; size_t capacity; } ConfigField;
#define NUMBER(name) {#name, offsetof(DeviceConfig, name), 0}
#define STRING(name) {#name, offsetof(DeviceConfig, name), sizeof(((DeviceConfig*)0)->name)}
static const ConfigField fields[] = {
    NUMBER(master_enabled), NUMBER(sensor_mask), NUMBER(sensor_poll_ms), NUMBER(light_poll_ms),
    NUMBER(send_interval_ms), NUMBER(receiver_enabled), NUMBER(receiver_port), NUMBER(history_interval_s),
    NUMBER(scd_interval_s), NUMBER(scd_pressure_mbar), NUMBER(scd_temperature_offset), NUMBER(scd_altitude_m),
    NUMBER(scd_asc), NUMBER(scd_recovery), NUMBER(bsec_heat_offset), NUMBER(bsec_save_interval_s),
    NUMBER(light_gain), NUMBER(light_cycles), NUMBER(uv_integration),
    STRING(receiver_ip), STRING(hostname), STRING(device_name)
};

static int hex(char c)
{
    if(c >= '0' && c <= '9') { return c - '0'; }
    if(c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    if(c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    return -1;
}
static int decode(const char* source, size_t length, char* dest, size_t capacity)
{
    size_t n = 0;
    for(size_t i = 0; i < length; ++i)
    {
        unsigned char c = (unsigned char)source[i];
        if(c == '%')
        {
            if(i + 2U >= length || hex(source[i + 1U]) < 0 || hex(source[i + 2U]) < 0) { return 0; }
            c = (unsigned char)(hex(source[i + 1U]) * 16 + hex(source[i + 2U]));
            i += 2U;
        }
        else if(c == '+') { c = ' '; }
        if(c < 32U || c > 126U || n + 1U >= capacity) { return 0; }
        dest[n++] = (char)c;
    }
    dest[n] = 0;
    return n != 0U;
}

int DeviceConfig_Parse(const DeviceConfig* current, const char* form, size_t length, DeviceConfig* output)
{
    if(current == NULL || form == NULL || output == NULL || length == 0U || length > 1536U) { return 0; }
    DeviceConfig candidate = *current;
    uint32_t seen = 0;
    for(size_t pos = 0; pos < length;)
    {
        size_t end = pos;
        while(end < length && form[end] != '&') { ++end; }
        size_t equal = pos;
        while(equal < end && form[equal] != '=') { ++equal; }
        char key[40], value[64];
        if(equal == end || !decode(form + pos, equal - pos, key, sizeof(key)) ||
           !decode(form + equal + 1U, end - equal - 1U, value, sizeof(value))) { return 0; }
        size_t index;
        for(index = 0; index < sizeof(fields) / sizeof(fields[0]); ++index)
        { if(strcmp(key, fields[index].key) == 0) { break; } }
        if(index == sizeof(fields) / sizeof(fields[0]) || (seen & (1UL << index)) != 0U) { return 0; }
        seen |= 1UL << index;
        void* dest = (uint8_t*)&candidate + fields[index].offset;
        if(fields[index].capacity != 0U)
        {
            if(strlen(value) >= fields[index].capacity) { return 0; }
            memset(dest, 0, fields[index].capacity);
            memcpy(dest, value, strlen(value));
        }
        else
        {
            const char* s = value;
            const int negative = *s == '-';
            if(negative) { ++s; }
            if(*s == 0 || (negative && strcmp(key, "bsec_heat_offset") != 0)) { return 0; }
            uint32_t number = 0;
            for(; *s; ++s)
            {
                if(*s < '0' || *s > '9' || number > 100000000U) { return 0; }
                number = number * 10U + (uint32_t)(*s - '0');
            }
            if(number > 1000000000U) { return 0; }
            if(negative) { const int32_t signed_number = -(int32_t)number; memcpy(dest, &signed_number, 4); }
            else { memcpy(dest, &number, 4); }
        }
        pos = end + 1U;
        if(pos == length) { return 0; } /* trailing '&' */
    }
    if(!DeviceConfig_Validate(&candidate)) { return 0; }
    *output = candidate;
    return 1;
}
