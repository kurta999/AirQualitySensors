#include "device_specific.h"
#include "bme680/bme680.h"
#include "bme680/bme680_defs.h"
#include <math.h>
#include <string.h>
#include "light_sensor.h"
#include "newlib_fixes/printf.h"
#include "cmsis_os.h"

#include "bsec/bsec_interface.h"

#include "main.h"

struct bme680_dev gas_sensor;
struct bme680_field_data data;

void user_delay_ms(uint32_t period);
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len);
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len);

float bme680_temp, bme680_hum, bme680_pressure;
uint32_t bme680_gas_resistance, bme680_timestamp;

#define NUM_USED_OUTPUTS 9

static bsec_library_return_t bme68x_bsec_update_subscription(float sample_rate)
{
    bsec_sensor_configuration_t requested_virtual_sensors[NUM_USED_OUTPUTS];
    uint8_t n_requested_virtual_sensors = NUM_USED_OUTPUTS;

    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

    bsec_library_return_t status = BSEC_OK;

    /* note: Virtual sensors as desired to be added here */
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_1;
    requested_virtual_sensors[0].sample_rate = sample_rate;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_2;
    requested_virtual_sensors[1].sample_rate = sample_rate;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_3;
    requested_virtual_sensors[2].sample_rate = sample_rate;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_4;
    requested_virtual_sensors[3].sample_rate = sample_rate;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[4].sample_rate = sample_rate;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    requested_virtual_sensors[5].sample_rate = sample_rate;
    requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    requested_virtual_sensors[6].sample_rate = sample_rate;
    requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[7].sample_rate = sample_rate;
    requested_virtual_sensors[8].sensor_id = BSEC_OUTPUT_RAW_GAS_INDEX;
    requested_virtual_sensors[8].sample_rate = sample_rate;

    /* Call bsec_update_subscription() to enable/disable the requested virtual sensors */
    status = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings,
        &n_required_sensor_settings);

    return status;
}

void __ARM_scalbnf()
{

}

int __hardfp_sqrt(int f) { return sqrt(f); }
int __hardfp_sqrtf(int f) { return sqrtf(f); }
int __hardfp_floorf(int f) { return floor(f); }
int __hardfp_sinf(int f) { return sin(f); }
int __hardfp_fminf(int f, int f2) { return fmin(f, f2); }
int __hardfp_log10f(int f) { return log(f); }
int __hardfp_fabs(int f) { return fabs(f); }
int __hardfp_ceil(int f) { return ceilf(f); }
int __hardfp_floor(int f) { return floorf(f); }
int __hardfp_roundf(int f) { return roundf(f); }

void SensorsBme680Task(void const* argument)
{
    TCS_Init();

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);

    /* You may assign a chip select identifier to be handled later */
    gas_sensor.dev_id = 0;
    gas_sensor.intf = BME680_SPI_INTF;
    gas_sensor.read = user_spi_read;
    gas_sensor.write = user_spi_write;
    gas_sensor.delay_ms = user_delay_ms;
    /* amb_temp can be set to 25 prior to configuring the gas sensor
     * or by performing a few temperature readings without operating the gas sensor.
     */
    gas_sensor.amb_temp = 25;

    /* Set the temperature, pressure and humidity settings */
    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;
    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    gas_sensor.power_mode = BME680_FORCED_MODE;

    /* Set the required sensor settings needed */
    uint16_t set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
        | BME680_GAS_SENSOR_SEL;

    int8_t rslt = bme680_init(&gas_sensor);

    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings, &gas_sensor);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&gas_sensor);

    /* Get the total measurement duration so as to sleep or wait till the
     * measurement is complete */
    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &gas_sensor);

    while(1)
    {
        user_delay_ms(meas_period * 10); /* Delay till the measurement is ready */
        rslt = bme680_get_sensor_data(&data, &gas_sensor);

        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
        {
            printf(", G: %d ohms", data.gas_resistance);
        }

        bme680_temp = data.temperature / 100.0f;
        bme680_hum = data.humidity / 1000.0f;
        bme680_pressure = data.pressure / 100.0f;
        bme680_gas_resistance = data.gas_resistance;
        bme680_timestamp = osKernelSysTick();

        /* Trigger the next measurement if you would like to read data out continuously */
        if(gas_sensor.power_mode == BME680_FORCED_MODE)
        {
            rslt = bme680_set_sensor_mode(&gas_sensor);
        }

        TCS_Read();
    }
}

extern  SPI_HandleTypeDef hspi2;

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
    osDelay(period);
}

int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
    uint8_t data_to_recv[128];
    //rslt = HAL_SPI_TransmitReceive(&hspi1, &reg_addr, data_to_send, len + 1, 250);

    rslt = HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 350);
    rslt = HAL_SPI_Receive(&hspi2, data_to_recv, len, 350);

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
    memcpy(reg_data, data_to_recv, len);

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

     /*
      * Data on the bus should be like
      * |----------------+---------------------+-------------|
      * | MOSI           | MISO                | Chip Select |
      * |----------------+---------------------|-------------|
      * | (don't care)   | (don't care)        | HIGH        |
      * | (reg_addr)     | (don't care)        | LOW         |
      * | (don't care)   | (reg_data[0])       | LOW         |
      * | (....)         | (....)              | LOW         |
      * | (don't care)   | (reg_data[len - 1]) | LOW         |
      * | (don't care)   | (don't care)        | HIGH        |
      * |----------------+---------------------|-------------|
      */

    return rslt;
}

int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
    uint8_t data_to_send[128];
    data_to_send[0] = reg_addr;
    memcpy(&data_to_send[1], reg_data, len);
    rslt = HAL_SPI_Transmit(&hspi2, data_to_send, len + 1, 250);

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

     /*
      * Data on the bus should be like
      * |---------------------+--------------+-------------|
      * | MOSI                | MISO         | Chip Select |
      * |---------------------+--------------|-------------|
      * | (don't care)        | (don't care) | HIGH        |
      * | (reg_addr)          | (don't care) | LOW         |
      * | (reg_data[0])       | (don't care) | LOW         |
      * | (....)              | (....)       | LOW         |
      * | (reg_data[len - 1]) | (don't care) | LOW         |
      * | (don't care)        | (don't care) | HIGH        |
      * |---------------------+--------------|-------------|
      */

    return rslt;
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

     /*
      * Data on the bus should be like
      * |------------+---------------------|
      * | I2C action | Data                |
      * |------------+---------------------|
      * | Start      | -                   |
      * | Write      | (reg_addr)          |
      * | Stop       | -                   |
      * | Start      | -                   |
      * | Read       | (reg_data[0])       |
      * | Read       | (....)              |
      * | Read       | (reg_data[len - 1]) |
      * | Stop       | -                   |
      * |------------+---------------------|
      */

    return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

     /*
      * Data on the bus should be like
      * |------------+---------------------|
      * | I2C action | Data                |
      * |------------+---------------------|
      * | Start      | -                   |
      * | Write      | (reg_addr)          |
      * | Write      | (reg_data[0])       |
      * | Write      | (....)              |
      * | Write      | (reg_data[len - 1]) |
      * | Stop       | -                   |
      * |------------+---------------------|
      */

    return rslt;
}
