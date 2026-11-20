#include "bosch_sensor.h"

#include <stdint.h>

#include "app_config.h"
#include "device_config.h"
#include "bme680/bme680.h"
#if APP_BSEC_ENABLED
#include "bsec_processing.h"
#endif
#include "cmsis_os.h"
#include "light_sensor.h"
#include "main.h"
#include "measurement_store.h"
#include "sensor_retry.h"
#include "task_health.h"

/** Loop period used while the BME680 has not reported a profile duration. */
#define BME680_RETRY_PERIOD_MS 1000U
/** Floor on the loop period so a bogus profile duration cannot spin the task. */
#define BME680_MIN_PERIOD_MS 50U

extern SPI_HandleTypeDef hspi2;

#if APP_BME680_ENABLED
static struct bme680_dev gas_sensor;
volatile Bme680Diagnostics bme680_diagnostics;

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
    bme680_diagnostics.spi_error = HAL_SPI_GetError(&hspi2);
    bme680_diagnostics.last_read_register = register_address;
    bme680_diagnostics.last_read_value = register_data[0];
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

#if APP_BSEC_ENABLED
    *measurement_period_ms = 50U;
    return BsecProcessing_Init();
#else
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
#endif
}

/*
 * The BME680 (SPI2) and the TCS34725 (I2C1) are initialised and retried
 * independently: neither sensor being absent may stop the other from running.
 */
static void service_bme680(SensorRetryState* state, uint16_t* measurement_period_ms)
{
    if(SensorRetry_IsInitialized(state) == 0U)
    {
        ++bme680_diagnostics.init_attempts;
        const int8_t init_status = initialize_bme680(measurement_period_ms);
        bme680_diagnostics.init_status = init_status;
        bme680_diagnostics.chip_id = gas_sensor.chip_id;
        if(init_status != BME680_OK)
        {
            return;
        }
        SensorRetry_ReportInitialized(state);
        /* Raw mode arms a conversion here; BSEC schedules it on a later poll. */
        return;
    }

#if APP_BSEC_ENABLED
    const int8_t status = BsecProcessing_Service(&gas_sensor);
    if(status == BME680_OK)
    {
        SensorRetry_ReportSuccess(state);
    }
    else if(status != BSEC_SERVICE_IDLE && SensorRetry_ReportFailure(state) != 0U)
    {
        Measurements_Invalidate(MEASUREMENT_SOURCE_BME680);
        Measurements_Invalidate(MEASUREMENT_SOURCE_IAQ);
    }
#else
    struct bme680_field_data measurement = {0};
    const int8_t status = bme680_get_sensor_data(&measurement, &gas_sensor);
    bme680_diagnostics.read_status = status;
    bme680_diagnostics.field_status = measurement.status;
    if(status == BME680_OK)
    {
        ++bme680_diagnostics.sample_count;
        bme680_diagnostics.last_sample_ms = osKernelSysTick();
        SensorRetry_ReportSuccess(state);
        Measurements_UpdateBme680(
            measurement.temperature / 100.0F,
            measurement.humidity / 1000.0F,
            measurement.pressure / 100.0F,
            measurement.gas_resistance,
            osKernelSysTick());
    }
    else if(SensorRetry_ReportFailure(state) != 0U)
    {
        Measurements_Invalidate(MEASUREMENT_SOURCE_BME680);
        return;
    }

    if(gas_sensor.power_mode == BME680_FORCED_MODE)
    {
        bme680_diagnostics.trigger_status = bme680_set_sensor_mode(&gas_sensor);
    }
#endif
}

#endif

#if APP_TCS34725_ENABLED
static void service_tcs34725(SensorRetryState* state)
{
    if(SensorRetry_IsInitialized(state) == 0U)
    {
        if(Tcs34725_Init() != HAL_OK)
        {
            return;
        }
        SensorRetry_ReportInitialized(state);
    }

    if(Tcs34725_Read() == HAL_OK)
    {
        SensorRetry_ReportSuccess(state);
    }
    else if(SensorRetry_ReportFailure(state) != 0U)
    {
        Measurements_Invalidate(MEASUREMENT_SOURCE_LIGHT);
    }
}
#endif

void SensorsBme680Task(void const* argument)
{
    (void)argument;
#if APP_BME680_ENABLED
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    SensorRetryState bme680_state;
    SensorRetry_Reset(&bme680_state);
    uint16_t measurement_period_ms = 0U;
#endif
#if APP_TCS34725_ENABLED
    SensorRetryState tcs34725_state;
    SensorRetry_Reset(&tcs34725_state);
    uint32_t next_light_poll_ms = 0U;
#endif

    DeviceConfig previous;
    DeviceConfig_Get(&previous);
    for(;;)
    {
        TaskHealth_ReportAlive(TASK_HEALTH_BME680, osKernelSysTick());

        uint16_t period_ms = BME680_RETRY_PERIOD_MS;
        DeviceConfig config;
        DeviceConfig_Get(&config);
#if APP_BME680_ENABLED
        if(config.master_enabled && (config.sensor_mask & 2U))
        {
            service_bme680(&bme680_state, &measurement_period_ms);
            period_ms = SensorRetry_IsInitialized(&bme680_state) != 0U ? measurement_period_ms : BME680_RETRY_PERIOD_MS;
        }
        else
        {
            if(SensorRetry_IsInitialized(&bme680_state))
            { gas_sensor.power_mode = BME680_SLEEP_MODE; (void)bme680_set_sensor_mode(&gas_sensor); }
            SensorRetry_Reset(&bme680_state);
            Measurements_Invalidate(MEASUREMENT_SOURCE_BME680);
            Measurements_Invalidate(MEASUREMENT_SOURCE_IAQ);
        }
        if(period_ms < BME680_MIN_PERIOD_MS)
        {
            period_ms = BME680_MIN_PERIOD_MS;
        }
#endif
#if APP_TCS34725_ENABLED
        const uint32_t now = osKernelSysTick();
        if(config.master_enabled && (config.sensor_mask & 16U))
        {
            if(config.light_gain != previous.light_gain || config.light_cycles != previous.light_cycles)
            { SensorRetry_Reset(&tcs34725_state); }
            if((int32_t)(now - next_light_poll_ms) >= 0)
            { service_tcs34725(&tcs34725_state); next_light_poll_ms = now + config.light_poll_ms; }
        }
        else
        {
            if(SensorRetry_IsInitialized(&tcs34725_state)) { (void)Tcs34725_Stop(); }
            SensorRetry_Reset(&tcs34725_state);
            Measurements_Invalidate(MEASUREMENT_SOURCE_LIGHT);
        }
        if(period_ms > config.light_poll_ms) { period_ms = (uint16_t)config.light_poll_ms; }
#endif
        previous = config;
        (void)previous;
        osDelay(period_ms);
    }
}
