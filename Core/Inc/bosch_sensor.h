#ifndef BOSCH_SENSOR_H
#define BOSCH_SENSOR_H

#include <stdint.h>

typedef struct
{
    int8_t init_status;
    int8_t read_status;
    int8_t trigger_status;
    uint8_t chip_id;
    uint8_t field_status;
    uint8_t last_read_register;
    uint8_t last_read_value;
    uint32_t spi_error;
    uint32_t init_attempts;
    uint32_t sample_count;
    uint32_t last_sample_ms;
} Bme680Diagnostics;

extern volatile Bme680Diagnostics bme680_diagnostics;

void SensorsBme680Task(void const* argument);

#endif /* BOSCH_SENSOR_H */
