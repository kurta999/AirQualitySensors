#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "circbuff/circbuff.h"
#include "bsec_state_record.h"
#include "usb_dhcp.h"
#include "hpma/hpma_parser.h"
#include "measurement_format.h"
#include "measurement_store.h"
#include "protocol.h"
#include "sensor_retry.h"
#include "task_health.h"

/* Required by the tiny printf implementation the firmware and these tests
   share; nothing here uses its console output. */
void _putchar(char character)
{
    (void)character;
}

static unsigned int failures;

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if(!(condition))                                                        \
        {                                                                       \
            fprintf(stderr, "%s:%d: check failed: %s\n",                     \
                    __FILE__, __LINE__, #condition);                            \
            ++failures;                                                         \
        }                                                                       \
    } while(0)

static void test_crc16(void)
{
    static const uint8_t input[] = "123456789";
    CHECK(Protocol_Crc16(input, sizeof(input) - 1U) == 0x4B37U);
    CHECK(Protocol_Crc16(NULL, 10U) == 0xFFFFU);
}

static void test_circular_buffer(void)
{
    uint8_t storage[4] = {0};
    circbuff_t buffer;
    circbuff_init(&buffer, storage, sizeof(storage));

    static const uint8_t first[] = {1U, 2U, 3U};
    static const uint8_t expected[] = {2U, 3U};
    CHECK(circbuff_insert_bytes(&buffer, first, sizeof(first)) == 1U);
    CHECK(circbuff_get_data_len(&buffer) == 3U);
    CHECK(circbuff_isnext(&buffer, expected, sizeof(expected), 0U) == 0U);

    uint8_t value = 0U;
    CHECK(circbuff_get_byte(&buffer, &value) == 1U && value == 1U);
    CHECK(circbuff_isnext(&buffer, expected, sizeof(expected), 1U) == 1U);
    CHECK(circbuff_get_data_len(&buffer) == 0U);

    static const uint8_t full[] = {4U, 5U, 6U, 7U};
    CHECK(circbuff_insert_bytes(&buffer, full, sizeof(full)) == 1U);
    CHECK(circbuff_add_byte(&buffer, 8U) == 0U);
    CHECK(circbuff_get_byte(&buffer, &value) == 1U && value == 4U);
    CHECK(circbuff_add_byte(&buffer, 8U) == 1U);
    static const uint8_t wrapped[] = {5U, 6U, 7U, 8U};
    CHECK(circbuff_isnext(&buffer, wrapped, sizeof(wrapped), 1U) == 1U);
    CHECK(circbuff_get_byte(&buffer, &value) == 0U);

    circbuff_flush(&buffer);
    CHECK(circbuff_get_data_len(&buffer) == 0U);
}

static void make_hpma_frame(uint8_t frame[HPMA_FRAME_SIZE])
{
    memset(frame, 0, HPMA_FRAME_SIZE);
    frame[0] = 0x42U;
    frame[1] = 0x4DU;
    frame[2] = 0x00U;
    frame[3] = 0x1CU;
    frame[6] = 0x00U;
    frame[7] = 0x0CU;
    frame[8] = 0x01U;
    frame[9] = 0x59U;

    uint16_t checksum = 0U;
    for(size_t index = 0U; index < HPMA_FRAME_SIZE - 2U; ++index)
    {
        checksum = (uint16_t)(checksum + frame[index]);
    }
    frame[30] = (uint8_t)(checksum >> 8U);
    frame[31] = (uint8_t)(checksum & 0xFFU);
}

static void test_hpma_parser(void)
{
    uint8_t frame[HPMA_FRAME_SIZE];
    make_hpma_frame(frame);

    HpmaParser parser;
    HpmaParser_Init(&parser);
    uint16_t pm25 = 0U;
    uint16_t pm10 = 0U;
    for(size_t index = 0U; index < HPMA_FRAME_SIZE - 1U; ++index)
    {
        CHECK(HpmaParser_Push(&parser, frame[index], &pm25, &pm10) == 0U);
    }
    CHECK(HpmaParser_Push(&parser, frame[31], &pm25, &pm10) == 1U);
    CHECK(pm25 == 12U);
    CHECK(pm10 == 345U);

    frame[31] ^= 1U;
    HpmaParser_Init(&parser);
    uint8_t accepted = 0U;
    for(size_t index = 0U; index < HPMA_FRAME_SIZE; ++index)
    {
        accepted = HpmaParser_Push(&parser, frame[index], &pm25, &pm10);
    }
    CHECK(accepted == 0U);
}

static MeasurementSnapshot make_snapshot(void)
{
    MeasurementSnapshot measurement = {
        .scd30_co2_ppm = 420.5F,
        .scd30_temperature_c = 22.25F,
        .scd30_relative_humidity_percent = 45.0F,
        .bme680_temperature_c = 23.0F,
        .bme680_relative_humidity_percent = 44.5F,
        .bme680_pressure_hpa = 1001.25F,
        .bme680_gas_resistance_ohm = 12345U,
        .pm25_ug_m3 = 12U,
        .pm10_ug_m3 = 34U,
        .uv_raw = 56U,
        .red = 1.0F,
        .green = 2.0F,
        .blue = 3.0F,
        .color_temperature_k = 4000U,
        .lux = 250U,
        .snapshot_timestamp_ms = 10000U,
    };

    for(int source = 0; source < MEASUREMENT_SOURCE_COUNT; ++source)
    {
        measurement.timestamp_ms[source] = 9000U;
        measurement.valid_mask |= 1U << source;
    }
    measurement.timestamp_ms[MEASUREMENT_SOURCE_BME680] = 6789U;
    return measurement;
}

static void test_measurement_format(void)
{
    const MeasurementSnapshot measurement = make_snapshot();

    char output[MEASUREMENT_FORMAT_MAX_LEN];
    const int length = Measurements_Format(output, sizeof(output), &measurement);
    CHECK(length > 0);
    CHECK((size_t)length == strlen(output));
    CHECK(strstr(output, "SCD30: 22.2500,45.0000,420.5000") != NULL);
    CHECK(strstr(output, "HONEYWELL: 12,34") != NULL);

    /*
     * The record is consumed by the backend
     * (https://github.com/kurta999/WindowsAddon), which reads the values
     * positionally. Pin the whole line so any change to field order, count or
     * spacing fails here instead of silently breaking the receiver.
     */
    CHECK(strcmp(output,
                 "MEAS_DATA SCD30: 22.2500,45.0000,420.5000 "
                 "BME680: 23.0000,44.5000,1001.2500,12345,6789 "
                 "HONEYWELL: 12,34 VEML6070: 56 "
                 "TCS: 1.0000,2.0000,3.0000,4000,250\n") == 0);

    char too_small[8];
    CHECK(Measurements_Format(too_small, sizeof(too_small), &measurement) == -1);
    CHECK(too_small[sizeof(too_small) - 1U] == '\0');
    CHECK(Measurements_Format(NULL, 0U, &measurement) == -1);
}

/*
 * Guards the P1 defect directly: newlib-nano drops %f unless _printf_float is
 * linked, which the firmware does not do. These tests link the same tiny
 * printf the firmware uses, so an exact-output check here is evidence about
 * the target, not just about the host.
 */
static void test_float_formatting(void)
{
    MeasurementSnapshot measurement = make_snapshot();
    measurement.scd30_temperature_c = -7.5F;
    measurement.bme680_pressure_hpa = 1013.0625F;

    char output[MEASUREMENT_FORMAT_MAX_LEN];
    CHECK(Measurements_Format(output, sizeof(output), &measurement) > 0);
    CHECK(strstr(output, "SCD30: -7.5000,") != NULL);
    CHECK(strstr(output, ",1013.0625,") != NULL);
    /* A dropped conversion would leave the literal format specifier behind. */
    CHECK(strstr(output, "%f") == NULL);
    CHECK(strstr(output, "SCD30: ,") == NULL);
}

/** The declared buffer size must hold the widest line the formatter can emit. */
static void test_measurement_format_worst_case(void)
{
    MeasurementSnapshot measurement;
    memset(&measurement, 0, sizeof(measurement));
    measurement.scd30_co2_ppm = -99999.9375F;
    measurement.scd30_temperature_c = -99999.9375F;
    measurement.scd30_relative_humidity_percent = -99999.9375F;
    measurement.bme680_temperature_c = -99999.9375F;
    measurement.bme680_relative_humidity_percent = -99999.9375F;
    measurement.bme680_pressure_hpa = -99999.9375F;
    measurement.bme680_gas_resistance_ohm = UINT32_MAX;
    measurement.pm25_ug_m3 = UINT16_MAX;
    measurement.pm10_ug_m3 = UINT16_MAX;
    measurement.uv_raw = UINT16_MAX;
    measurement.red = -99999.9375F;
    measurement.green = -99999.9375F;
    measurement.blue = -99999.9375F;
    measurement.color_temperature_k = UINT16_MAX;
    measurement.lux = UINT16_MAX;
    measurement.valid_mask = 0x1FU;
    measurement.snapshot_timestamp_ms = 0x7FFFFFFFU;
    measurement.timestamp_ms[MEASUREMENT_SOURCE_BME680] = UINT32_MAX;

    char output[MEASUREMENT_FORMAT_MAX_LEN];
    const int length = Measurements_Format(output, sizeof(output), &measurement);
    CHECK(length > 0);
    CHECK((size_t)length < MEASUREMENT_FORMAT_MAX_LEN);
}

static void test_measurement_validity(void)
{
    MeasurementSnapshot measurement;
    memset(&measurement, 0, sizeof(measurement));
    measurement.snapshot_timestamp_ms = 5000U;

    /* A zeroed store must not look like a fresh set of readings. */
    for(int source = 0; source < MEASUREMENT_SOURCE_COUNT; ++source)
    {
        CHECK(Measurements_IsValid(&measurement, (MeasurementSource)source) == 0U);
        CHECK(Measurements_AgeMs(&measurement, (MeasurementSource)source)
              == UINT32_MAX);
        CHECK(Measurements_IsFresh(&measurement, (MeasurementSource)source,
                                   MEASUREMENT_DEFAULT_MAX_AGE_MS) == 0U);
    }

    measurement.valid_mask = 1U << MEASUREMENT_SOURCE_UV;
    measurement.timestamp_ms[MEASUREMENT_SOURCE_UV] = 4000U;
    CHECK(Measurements_IsValid(&measurement, MEASUREMENT_SOURCE_UV) == 1U);
    CHECK(Measurements_AgeMs(&measurement, MEASUREMENT_SOURCE_UV) == 1000U);
    CHECK(Measurements_IsFresh(&measurement, MEASUREMENT_SOURCE_UV, 1000U) == 1U);
    CHECK(Measurements_IsFresh(&measurement, MEASUREMENT_SOURCE_UV, 999U) == 0U);

    /* Ages must stay correct across the 32-bit tick wraparound. */
    measurement.timestamp_ms[MEASUREMENT_SOURCE_UV] = UINT32_MAX - 99U;
    measurement.snapshot_timestamp_ms = 100U;
    CHECK(Measurements_AgeMs(&measurement, MEASUREMENT_SOURCE_UV) == 200U);

    CHECK(Measurements_IsValid(NULL, MEASUREMENT_SOURCE_UV) == 0U);
    CHECK(Measurements_IsValid(&measurement, MEASUREMENT_SOURCE_COUNT) == 0U);
}

static void test_sensor_retry(void)
{
    SensorRetryState state;
    SensorRetry_Reset(&state);
    CHECK(SensorRetry_IsInitialized(&state) == 0U);

    SensorRetry_ReportInitialized(&state);
    CHECK(SensorRetry_IsInitialized(&state) == 1U);

    /* Transient failures must not drop a working sensor. */
    for(unsigned int attempt = 1U; attempt < SENSOR_MAX_READ_FAILURES; ++attempt)
    {
        CHECK(SensorRetry_ReportFailure(&state) == 0U);
        CHECK(SensorRetry_IsInitialized(&state) == 1U);
    }
    SensorRetry_ReportSuccess(&state);

    /* A sustained run of failures drops it for reinitialisation. */
    uint8_t dropped = 0U;
    for(unsigned int attempt = 0U; attempt < SENSOR_MAX_READ_FAILURES; ++attempt)
    {
        dropped = SensorRetry_ReportFailure(&state);
    }
    CHECK(dropped == 1U);
    CHECK(SensorRetry_IsInitialized(&state) == 0U);
}

static void test_task_health(void)
{
    TaskHealth_Reset();

    /* With nothing registered the watchdog must not be blocked. */
    CHECK(TaskHealth_AllAlive(0U) == 1U);

    TaskHealth_Register(TASK_HEALTH_SENSORS, 1000U, 0U);
    TaskHealth_Register(TASK_HEALTH_USER_INTERFACE, 500U, 0U);
    CHECK(TaskHealth_AllAlive(400U) == 1U);

    /* A task that never reports is caught even if it never ran. */
    CHECK(TaskHealth_AllAlive(600U) == 0U);
    CHECK(TaskHealth_LateMask(600U) == (1U << TASK_HEALTH_USER_INTERFACE));

    TaskHealth_ReportAlive(TASK_HEALTH_USER_INTERFACE, 600U);
    CHECK(TaskHealth_AllAlive(900U) == 1U);

    /* One wedged task is enough to stop the watchdog refresh. */
    TaskHealth_ReportAlive(TASK_HEALTH_USER_INTERFACE, 2000U);
    CHECK(TaskHealth_AllAlive(2000U) == 0U);
    CHECK(TaskHealth_LateMask(2000U) == (1U << TASK_HEALTH_SENSORS));

    /* Deadlines must survive the tick wraparound. */
    TaskHealth_Reset();
    TaskHealth_Register(TASK_HEALTH_SENSORS, 1000U, UINT32_MAX - 100U);
    CHECK(TaskHealth_AllAlive(500U) == 1U);
    CHECK(TaskHealth_AllAlive(1500U) == 0U);

    TaskHealth_Reset();
}

static void test_bsec_state_records(void)
{
    static const uint8_t payload[] = {1U, 4U, 9U, 16U, 25U};
    BsecStateRecord first;
    BsecStateRecord second;
    CHECK(BsecState_Crc32("123456789", 9U) == UINT32_C(0xCBF43926));
    CHECK(BsecStateRecord_Create(&first, 17U, UINT32_MAX, payload, sizeof(payload)) == 0);
    CHECK(BsecStateRecord_Create(&second, 17U, 0U, payload, sizeof(payload)) == 0);
    CHECK(BsecStateRecord_Valid(&first, 17U));
    CHECK(!BsecStateRecord_Valid(&first, 18U));
    CHECK(BsecStateRecord_Latest(&first, &second, 17U) == 1);
    second.payload[2] ^= 1U;
    CHECK(BsecStateRecord_Latest(&first, &second, 17U) == 0);
    CHECK(BsecStateRecord_Create(&second, 17U, 1U, payload, sizeof(payload)) == 0);
    second.length = UINT32_MAX;
    CHECK(!BsecStateRecord_Valid(&second, 17U));
    CHECK(BsecStateRecord_Create(&second, 17U, 1U, payload, sizeof(payload)) == 0);
    second.sequence ^= 1U;
    CHECK(!BsecStateRecord_Valid(&second, 17U));
    CHECK(BsecStateRecord_Create(&second, 17U, 1U, payload, sizeof(payload)) == 0);
    /* Every interrupted doubleword write must leave the old record usable. */
    BsecStateRecord interrupted;
    for(size_t written = 0U; written < sizeof(second); written += 8U)
    {
        memset(&interrupted, 0xFF, sizeof(interrupted));
        memcpy(&interrupted, &second, written);
        CHECK(!BsecStateRecord_Valid(&interrupted, 17U));
        CHECK(BsecStateRecord_Latest(&first, &interrupted, 17U) == 0);
    }
    CHECK(BsecStateRecord_Create(&second, 17U, 2U, payload, 0U) != 0);
    CHECK(BsecStateRecord_Create(&second, 17U, 2U, payload, BSEC_STATE_PAYLOAD_CAPACITY + 1U) != 0);
}

static void test_usb_dhcp(void)
{
    uint8_t request[300] = {0};
    uint8_t reply[300];
    static const uint8_t server[] = {192,168,7,1};
    static const uint8_t client[] = {192,168,7,2};
    static const uint8_t mask[] = {255,255,255,252};
    request[0] = 1; request[1] = 1; request[2] = 6; request[4] = 73;
    request[28] = 2; request[29] = 17;
    request[236] = 99; request[237] = 130; request[238] = 83; request[239] = 99;
    request[240] = 0; /* PAD must not be interpreted as a length/value pair. */
    request[241] = 53; request[242] = 1; request[243] = 1; request[244] = 255;
    for(size_t length = 0; length < 240; ++length)
    {
        CHECK(UsbDhcp_Reply(request, length, reply, sizeof(reply), server, client, mask) == 0);
    }
    CHECK(UsbDhcp_Reply(request, sizeof(request), reply, 299, server, client, mask) == 0);
    CHECK(UsbDhcp_Reply(request, sizeof(request), reply, sizeof(reply), server, client, mask) == 300);
    CHECK(reply[0] == 2 && reply[4] == 73 && reply[242] == 2);
    CHECK(memcmp(reply + 16, client, 4) == 0 && memcmp(reply + 28, request + 28, 16) == 0);
    for(size_t pos = 240; pos < 300 && reply[pos] != 255; pos += 2U + reply[pos + 1U])
    {
        CHECK(reply[pos] != 3 && reply[pos] != 6); /* Do not alter PC gateway/DNS. */
    }
    request[243] = 3; /* Selecting the offered lease. */
    request[244] = 50; request[245] = 4; memcpy(request + 246, client, 4); request[250] = 255;
    CHECK(UsbDhcp_Reply(request, sizeof(request), reply, sizeof(reply), server, client, mask) == 300);
    CHECK(reply[242] == 5);
    request[249] = 3;
    CHECK(UsbDhcp_Reply(request, sizeof(request), reply, sizeof(reply), server, client, mask) == 300);
    CHECK(reply[242] == 6 && reply[16] == 0);
    request[249] = 2; request[250] = 54; request[251] = 4;
    memcpy(request + 252, server, 4); request[255] = 99; request[256] = 255;
    CHECK(UsbDhcp_Reply(request, sizeof(request), reply, sizeof(reply), server, client, mask) == 0);
    request[255] = 1;
    CHECK(UsbDhcp_Reply(request, 254, reply, sizeof(reply), server, client, mask) == 0);
    CHECK(UsbDhcp_Reply(request, sizeof(request), reply, sizeof(reply), server, client, mask) == 300);
}

unsigned int test_device_web(void);
int main(void)
{
    failures += test_device_web();
    test_crc16();
    test_circular_buffer();
    test_hpma_parser();
    test_measurement_format();
    test_float_formatting();
    test_measurement_format_worst_case();
    test_measurement_validity();
    test_sensor_retry();
    test_task_health();
    test_bsec_state_records();
    test_usb_dhcp();

    if(failures != 0U)
    {
        fprintf(stderr, "%u test check(s) failed\n", failures);
        return 1;
    }
    puts("All tests passed");
    return 0;
}
