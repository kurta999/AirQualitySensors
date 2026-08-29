#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "circbuff/circbuff.h"
#include "hpma/hpma_parser.h"
#include "measurement_format.h"
#include "protocol.h"

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

static void test_measurement_format(void)
{
    const MeasurementSnapshot measurement = {
        .scd30_co2_ppm = 420.5F,
        .scd30_temperature_c = 22.25F,
        .scd30_relative_humidity_percent = 45.0F,
        .bme680_temperature_c = 23.0F,
        .bme680_relative_humidity_percent = 44.5F,
        .bme680_pressure_hpa = 1001.25F,
        .bme680_gas_resistance_ohm = 12345U,
        .bme680_timestamp_ms = 6789U,
        .pm25_ug_m3 = 12U,
        .pm10_ug_m3 = 34U,
        .uv_raw = 56U,
        .red = 1.0F,
        .green = 2.0F,
        .blue = 3.0F,
        .color_temperature_k = 4000U,
        .lux = 250U,
    };
    char output[256];
    const int length = Measurements_Format(output, sizeof(output), &measurement);
    CHECK(length > 0);
    CHECK((size_t)length == strlen(output));
    CHECK(strstr(output, "SCD30: 22.2500,45.0000,420.5000") != NULL);
    CHECK(strstr(output, "HONEYWELL: 12,34") != NULL);

    char too_small[8];
    CHECK(Measurements_Format(too_small, sizeof(too_small), &measurement) == -1);
    CHECK(too_small[sizeof(too_small) - 1U] == '\0');
    CHECK(Measurements_Format(NULL, 0U, &measurement) == -1);
}

int main(void)
{
    test_crc16();
    test_circular_buffer();
    test_hpma_parser();
    test_measurement_format();

    if(failures != 0U)
    {
        fprintf(stderr, "%u test check(s) failed\n", failures);
        return 1;
    }
    puts("All tests passed");
    return 0;
}
