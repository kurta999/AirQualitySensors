#include "bosch_sensor.h"

#include <stdint.h>

#include "bme680/bme680.h"
#include "cmsis_os.h"
#include "light_sensor.h"
#include "main.h"
#include "measurement_store.h"

extern SPI_HandleTypeDef hspi2;

static struct bme680_dev gas_sensor;

static void sensor_delay_ms(uint32_t period)
{
    osDelay(period);
}

static int8_t sensor_spi_read(uint8_t device_id, uint8_t register_address,
                              uint8_t* register_data, uint16_t length)
{
    (void)device_id;
    if(register_data == NULL || length == 0U)
    {
        return BME680_E_INVALID_LENGTH;
    }

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi2, &register_address, 1U, 350U);
    if(status == HAL_OK)
    {
        status = HAL_SPI_Receive(&hspi2, register_data, length, 350U);
    }
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    return status == HAL_OK ? BME680_OK : BME680_E_COM_FAIL;
}

static int8_t sensor_spi_write(uint8_t device_id, uint8_t register_address,
                               uint8_t* register_data, uint16_t length)
{
    (void)device_id;
    if(register_data == NULL || length == 0U)
    {
        return BME680_E_INVALID_LENGTH;
    }

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi2, &register_address, 1U, 350U);
    if(status == HAL_OK)
    {
        status = HAL_SPI_Transmit(&hspi2, register_data, length, 350U);
    }
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    return status == HAL_OK ? BME680_OK : BME680_E_COM_FAIL;
}

static int8_t initialize_bme680(uint16_t* measurement_period_ms)
{
    gas_sensor.dev_id = 0U;
    gas_sensor.intf = BME680_SPI_INTF;
    gas_sensor.read = sensor_spi_read;
    gas_sensor.write = sensor_spi_write;
    gas_sensor.delay_ms = sensor_delay_ms;
    gas_sensor.amb_temp = 25;

    int8_t status = bme680_init(&gas_sensor);
    if(status != BME680_OK)
    {
        return status;
    }

    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    gas_sensor.gas_sett.heatr_temp = 320U;
    gas_sensor.gas_sett.heatr_dur = 150U;
    gas_sensor.power_mode = BME680_FORCED_MODE;

    const uint16_t settings =
        BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL |
        BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;
    status = bme680_set_sensor_settings(settings, &gas_sensor);
    if(status == BME680_OK)
    {
        status = bme680_set_sensor_mode(&gas_sensor);
    }
    if(status == BME680_OK)
    {
        bme680_get_profile_dur(measurement_period_ms, &gas_sensor);
    }
    return status;
}

void SensorsBme680Task(void const* argument)
{
    (void)argument;
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

    while(Tcs34725_Init() != HAL_OK)
    {
        osDelay(1000U);
    }

    uint16_t measurement_period_ms = 0U;
    while(initialize_bme680(&measurement_period_ms) != BME680_OK)
    {
        osDelay(1000U);
    }

    for(;;)
    {
        osDelay(measurement_period_ms);

        struct bme680_field_data measurement = {0};
        const int8_t status = bme680_get_sensor_data(&measurement, &gas_sensor);
        if(status == BME680_OK)
        {
            Measurements_UpdateBme680(
                measurement.temperature / 100.0F,
                measurement.humidity / 1000.0F,
                measurement.pressure / 100.0F,
                measurement.gas_resistance,
                osKernelSysTick());
        }

        if(gas_sensor.power_mode == BME680_FORCED_MODE)
        {
            (void)bme680_set_sensor_mode(&gas_sensor);
        }
        (void)Tcs34725_Read();
    }
}
