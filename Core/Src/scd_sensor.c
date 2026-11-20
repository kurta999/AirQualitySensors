#include "scd_sensor.h"

#include "app_config.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "i2c_bus.h"
#include "measurement_store.h"
#include "scd30/scd30.h"
#include "sensirion_common.h"

#define SCD30_NO_ERROR 0
#define SCD30_MEASUREMENT_INTERVAL_SECONDS 2U
/* Sensirion specifies up to 150 ms clock stretching during calibration. */
#define SCD30_I2C_TIMEOUT_MS 250U
#define SCD30_NO_DATA_TIMEOUT_MS 30000U
#define SCD30_CMD_SOFT_RESET 0xD304U
#define SCD30_CMD_FIRMWARE_VERSION 0xD100U
#define SCD30_CMD_MEASUREMENT_INTERVAL 0x4600U

extern I2C_HandleTypeDef hi2c1;
volatile Scd30Diagnostics scd30_diagnostics;
static uint32_t last_progress_ms;

static int16_t scd30_init_locked(void)
{
    int16_t status = scd30_probe();
    if(status != SCD30_NO_ERROR)
    {
        return status;
    }

#if APP_SCD30_RECOVERY_ENABLED
    status = sensirion_i2c_write_cmd(scd30_get_configured_address(), SCD30_CMD_SOFT_RESET);
    if(status != SCD30_NO_ERROR)
    {
        return status;
    }
    ++scd30_diagnostics.reset_count;
    sensirion_sleep_usec(2000000U);
#endif

    uint16_t firmware_version = 0U;
    status = sensirion_i2c_delayed_read_cmd(scd30_get_configured_address(),
        SCD30_CMD_FIRMWARE_VERSION, 3000U, &firmware_version, 1U);
    if(status != SCD30_NO_ERROR)
    {
        return status;
    }
    scd30_diagnostics.firmware_version = firmware_version;

    status = scd30_stop_periodic_measurement();
    if(status != SCD30_NO_ERROR)
    {
        return status;
    }
    sensirion_sleep_usec(20000U);
    status = scd30_set_measurement_interval(SCD30_MEASUREMENT_INTERVAL_SECONDS);
    if(status != SCD30_NO_ERROR)
    {
        return status;
    }
    sensirion_sleep_usec(20000U);
    uint16_t interval_s = 0U;
    status = sensirion_i2c_delayed_read_cmd(scd30_get_configured_address(),
        SCD30_CMD_MEASUREMENT_INTERVAL, 3000U, &interval_s, 1U);
    if(status != SCD30_NO_ERROR || interval_s != SCD30_MEASUREMENT_INTERVAL_SECONDS)
    {
        return status != SCD30_NO_ERROR ? status : -1;
    }
    scd30_diagnostics.measurement_interval_s = interval_s;
    status = scd30_start_periodic_measurement(APP_SCD30_AMBIENT_PRESSURE_MBAR);
    if(status == SCD30_NO_ERROR)
    {
        /* Let the start command settle before the task's first status poll. */
        sensirion_sleep_usec(20000U);
        last_progress_ms = osKernelSysTick();
    }
    return status;
}

int16_t Scd30_Init(void)
{
    if(I2cBus_Lock() != HAL_OK)
    {
        return -1;
    }
    const int16_t status = scd30_init_locked();
    scd30_diagnostics.init_status = status;
    I2cBus_Unlock();
    return status;
}

static int16_t scd30_poll_locked(void)
{
    uint16_t data_ready = 0U;
    int16_t status = scd30_get_data_ready(&data_ready);
    if(status != SCD30_NO_ERROR)
    {
        return status;
    }
    scd30_diagnostics.data_ready = data_ready;
    if(data_ready == 0U)
    {
        ++scd30_diagnostics.not_ready_count;
#if APP_SCD30_RECOVERY_ENABLED
        if((osKernelSysTick() - last_progress_ms) >= SCD30_NO_DATA_TIMEOUT_MS)
        {
            return -2;
        }
#endif
        return SCD30_NO_ERROR;
    }

    float co2_ppm = 0.0F;
    float temperature = 0.0F;
    float relative_humidity = 0.0F;
    status = scd30_read_measurement(&co2_ppm, &temperature, &relative_humidity);
    if(status == SCD30_NO_ERROR)
    {
        last_progress_ms = osKernelSysTick();
        scd30_diagnostics.last_sample_ms = last_progress_ms;
        ++scd30_diagnostics.sample_count;
        Measurements_UpdateScd30(
            co2_ppm, temperature, relative_humidity, osKernelSysTick());
    }
    return status;
}

int16_t Scd30_Poll(void)
{
    if(I2cBus_Lock() != HAL_OK)
    {
        return -1;
    }
    const int16_t status = scd30_poll_locked();
    scd30_diagnostics.poll_status = status;
    I2cBus_Unlock();
    return status;
}

void sensirion_i2c_init(void)
{
}

void sensirion_i2c_release(void)
{
}

/*
 * The Sensirion transport callbacks intentionally do not take the I2C lock:
 * a single scd30_* call is several transfers that must not be interleaved, so
 * the lock is held by Scd30_Init()/Scd30_Poll() around the whole operation.
 */
int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count)
{
    return (int8_t)HAL_I2C_Master_Receive(
        &hi2c1, (uint16_t)(address << 1U), data, count, SCD30_I2C_TIMEOUT_MS);
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data, uint16_t count)
{
    return (int8_t)HAL_I2C_Master_Transmit(
        &hi2c1, (uint16_t)(address << 1U), (uint8_t*)data, count, SCD30_I2C_TIMEOUT_MS);
}

void sensirion_sleep_usec(uint32_t useconds)
{
    uint32_t milliseconds = useconds / 1000U;
    if((useconds % 1000U) != 0U)
    {
        ++milliseconds;
    }
    osDelay(milliseconds);
}
