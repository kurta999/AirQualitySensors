#include "device_config.h"
#include "http_request.h"
#include "measurement_history.h"
#include <stdio.h>
#include <string.h>

static unsigned int errors;
#define EXPECT(condition) do { if(!(condition)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); ++errors; } } while(0)

static void test_config(void)
{
    DeviceConfig original, parsed;
    DeviceConfig_Defaults(&original);
    EXPECT(DeviceConfig_Validate(&original));
    const char valid[] = "device_name=My+air&hostname=my-air&bsec_heat_offset=-125&history_interval_s=5&master_enabled=0";
    EXPECT(DeviceConfig_Parse(&original, valid, sizeof(valid) - 1U, &parsed));
    EXPECT(strcmp(parsed.device_name, "My air") == 0 && strcmp(parsed.hostname, "my-air") == 0);
    EXPECT(parsed.bsec_heat_offset == -125 && parsed.master_enabled == 0 && original.master_enabled == 1);
    static const char* invalid[] = {
        "master_enabled=2", "hostname=-air", "hostname=air-", "hostname=air.local", "hostname=air%00x",
        "device_name=%22evil", "scd_interval_s=1", "scd_pressure_mbar=699", "sensor_mask=64",
        "receiver_ip=192.168.7.999", "receiver_ip=127.0.0.1", "receiver_ip=224.0.0.251", "receiver_port=65536",
        "bsec_heat_offset=-2001", "receiver_port=-1", "receiver_port=1x", "receiver_port=999999999999999",
        "light_gain=4", "light_cycles=0", "history_interval_s=1", "history_interval_s=3601",
        "unknown=1", "hostname=air&hostname=other", "device_name=%", "device_name=%GG", "hostname=air&",
        "master_enabled=0&hostname=%00"
    };
    for(size_t i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i)
    {
        parsed = original;
        EXPECT(!DeviceConfig_Parse(&original, invalid[i], strlen(invalid[i]), &parsed));
        EXPECT(memcmp(&parsed, &original, sizeof(parsed)) == 0);
    }
    memset(parsed.hostname, 'a', sizeof(parsed.hostname));
    EXPECT(!DeviceConfig_Validate(&parsed));
}
static void test_http(void)
{
    const char valid[] = "POST /api/config HTTP/1.1\r\nHost: air.local\r\nOrigin: http://air.local\r\nContent-Type: application/x-www-form-urlencoded\r\nX-Air-Config: 1\r\nContent-Length: 16\r\n\r\nmaster_enabled=0";
    HttpRequest r;
    for(size_t n = 0; n < sizeof(valid) - 1U; ++n) { EXPECT(HttpRequest_Parse(valid, n, &r) == 0); }
    EXPECT(HttpRequest_Parse(valid, sizeof(valid) - 1U, &r) == 1);
    EXPECT(r.body_length == 16 && memcmp(r.body,"master_enabled=0",16) == 0);
    EXPECT(HttpRequest_AllowedHost(&r,"air"));
    r.write_token = 0; EXPECT(!HttpRequest_AllowedHost(&r,"air")); r.write_token = 1;
    memcpy(r.origin,"http://evil.example",20); EXPECT(!HttpRequest_AllowedHost(&r,"air"));
    r.origin[0] = 0; memcpy(r.host,"evil.example",13); EXPECT(!HttpRequest_AllowedHost(&r,"air"));
    memcpy(r.host,"192.168.7.1:80",15); EXPECT(HttpRequest_AllowedHost(&r,"air"));
    static const char* invalid[] = {
        "GET / HTTP/1.1\r\n\r\n",
        "GET / HTTP/1.1\r\nHost: air.local\r\nHost: evil\r\n\r\n",
        "POST /api/config HTTP/1.1\r\nHost: air.local\r\nTransfer-Encoding: chunked\r\n\r\n",
        "POST /api/config HTTP/1.1\r\nHost: air.local\r\nContent-Length: 1\r\nContent-Length: 2\r\n\r\nx",
        "POST /api/config HTTP/1.1\r\nHost: air.local\r\nContent-Length: -1\r\n\r\n",
        "POST /api/config HTTP/1.1\r\nHost: air.local\r\nContent-Length: 1537\r\n\r\n",
        "GET / HTTP/1.1\r\nHost: air.local\r\n\r\nGET / HTTP/1.1\r\nHost: air.local\r\n\r\n",
        "GET / HTTP/1.1\r\nHost: air.local\r\nContent-Length: 1\r\n\r\nx"
    };
    for(size_t i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i) { EXPECT(HttpRequest_Parse(invalid[i],strlen(invalid[i]),&r) == -1); }
    const char get[] = "GET /api/history?after=32 HTTP/1.1\r\nHost: AIR.LOCAL\r\n\r\n";
    EXPECT(HttpRequest_Parse(get, sizeof(get)-1U, &r) == 1 && HttpRequest_AllowedHost(&r,"air"));
}
static void test_history(void)
{
    MeasurementSnapshot m = {0};
    uint8_t data[4096]; uint32_t header[8]; HistoryPoint point;
    History_Init();
    EXPECT(History_IntervalSeconds(0) == 150U);
    EXPECT(History_Export(data,31,0,0,0) == 0);
    EXPECT(History_Export(data,sizeof(data),0,0,0) == 32U);
    m.valid_mask=59; m.scd30_co2_ppm=421.5F; m.uv_raw=42; m.air_quality.accuracy=2;
    for(uint32_t i=0;i<601;++i) { m.snapshot_timestamp_ms=i*5000; History_Append(&m); }
    EXPECT(History_Count(3000000)==600);
    EXPECT(History_Export(data,sizeof(data),0,3000000,5)==32+32*sizeof(HistoryPoint));
    memcpy(header,data,32); memcpy(&point,data+32,sizeof(point));
    EXPECT(header[0]==UINT32_C(0x31524941) && header[4]==32 && header[7]==2);
    EXPECT(point.timestamp_ms==5000 && point.values[0]==421.5F && point.values[7]==42 && point.accuracy==2);
    (void)History_Export(data,sizeof(data),590,3000000,5); memcpy(header,data,32);
    EXPECT(header[4]==11 && header[7]==591);
    EXPECT(History_Count(3000000+HISTORY_MAX_AGE_MS+1U)==0);
    History_Init(); m.snapshot_timestamp_ms=UINT32_MAX-1000U; History_Append(&m);
    EXPECT(History_Count(1000)==1); EXPECT(History_Count(HISTORY_MAX_AGE_MS)==0);
    History_Init(); m.snapshot_timestamp_ms=1000; History_Service(&m,0);
    m.snapshot_timestamp_ms=2000; History_Service(&m,0); EXPECT(History_Count(2000)==1);
    m.snapshot_timestamp_ms=151000; History_Service(&m,0); EXPECT(History_Count(151000)==2);
    m.snapshot_timestamp_ms=152000; History_Service(&m,5); EXPECT(History_Count(152000)==1);
    History_Init();
    for(uint32_t t=0;t<=HISTORY_MAX_AGE_MS;t+=150000U) {m.snapshot_timestamp_ms=t;History_Service(&m,0);}
    EXPECT(History_Count(HISTORY_MAX_AGE_MS)==577);
}
unsigned int test_device_web(void)
{
    errors=0; test_config(); test_http(); test_history(); return errors;
}
