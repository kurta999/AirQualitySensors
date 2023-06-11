#include "device_specific.h"
#include "scd30/scd30.h"
#include "newlib_fixes/printf.h"

#define NO_ERROR 0
int interval_in_seconds = 2;
float co2_ppm, temperature, relative_humidity;

extern I2C_HandleTypeDef hi2c1;

void scd_init()
{
    while(scd30_probe() != NO_ERROR)
    {
        printf("SCD30 sensor probing failed\n");
        osDelay(1000);
    }

    scd30_set_measurement_interval(interval_in_seconds);
    sensirion_sleep_usec(20000u);
    scd30_start_periodic_measurement(999);

}

void scd_read_loop()
{
    uint16_t data_ready = 0;
    uint16_t timeout = 0;
    int err;

    /* Poll data_ready flag until data is available. Allow 20% more than
 * the measurement interval to account for clock imprecision of the
 * sensor.
 */
    for(timeout = 0; (100000 * timeout) < (interval_in_seconds * 1200000); ++timeout) 
    {
        err = scd30_get_data_ready(&data_ready);
        if(err != NO_ERROR) {
            printf("Error reading data_ready flag: %i\n", err);
        }
        if(data_ready) {
            break;
        }
        sensirion_sleep_usec(100000);
    }
    if(!data_ready) {
        printf("Timeout waiting for data_ready flag\n");
        //continue;
    }

    /* Measure co2, temperature and relative humidity and store into
     * variables.
     */
    err = scd30_read_measurement(&co2_ppm, &temperature, &relative_humidity);
    if(err != NO_ERROR)
    {
        printf("error reading measurement\n");

    }
    else
    {
        printf("measured co2 concentration: %0.2f ppm, "
            "measured temperature: %0.2f degreeCelsius, "
            "measured humidity: %0.2f %%RH\n",
            co2_ppm, temperature, relative_humidity);
    }

}

void sensirion_i2c_init(void)
{

}

void sensirion_i2c_release(void)
{

}

int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count)
{
    return (int8_t)HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(address << 1), data, count, 100);
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data, uint16_t count)
{
    return (int8_t)HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(address << 1), (uint8_t*)data, count, 100);
}

void sensirion_sleep_usec(uint32_t useconds)
{
    uint32_t msec = useconds / 1000;
    if(useconds % 1000 > 0) {
        msec++;
    }

    /*
     * Increment by 1 if STM32F1 driver version less than 1.1.1
     * Old firmwares of STM32F1 sleep 1ms shorter than specified in HAL_Delay.
     * This was fixed with firmware 1.6 (driver version 1.1.1), so we have to
     * fix it ourselves for older firmwares
     */
    if(HAL_GetHalVersion() < 0x01010100) {
        msec++;
    }

    //HAL_Delay(msec);
    osDelay(msec);
}
