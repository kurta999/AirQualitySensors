#include "scd_sensor.h"

#include "app_config.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "i2c_bus.h"
#include "measurement_store.h"
#include "scd30/scd30.h"

#define SCD30_NO_ERROR 0
#define SCD30_MEASUREMENT_INTERVAL_SECONDS 2U

extern I2C_HandleTypeDef hi2c1;

static int16_t scd30_init_locked(void)
{
    int16_t status = scd30_probe();
    if(status != SCD30_NO_ERROR)
    {
        return status;
    }

    status = scd30_set_measurement_interval(SCD30_MEASUREMENT_INTERVAL_SECONDS);
    if(status != SCD30_NO_ERROR)
    {
        return status;
    }
    sensirion_sleep_usec(20000U);
    return scd30_start_periodic_measurement(APP_SCD30_AMBIENT_PRESSURE_MBAR);
}

int16_t Scd30_Init(void)
{
    if(I2cBus_Lock() != HAL_OK)
    {
        return -1;
    }
    const int16_t status = scd30_init_locked();
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
    if(data_ready == 0U)
    {
        return SCD30_NO_ERROR;
    }

    float co2_ppm = 0.0F;
    float temperature = 0.0F;
    float relative_humidity = 0.0F;
    status = scd30_read_measurement(&co2_ppm, &temperature, &relative_humidity);
    if(status == SCD30_NO_ERROR)
    {
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
        &hi2c1, (uint16_t)(address << 1U), data, count, 100U);
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data, uint16_t count)
{
    return (int8_t)HAL_I2C_Master_Transmit(
        &hi2c1, (uint16_t)(address << 1U), (uint8_t*)data, count, 100U);
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
