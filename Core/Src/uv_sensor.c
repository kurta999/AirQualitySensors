#include "uv_sensor.h"

#include <stddef.h>

#include "i2c_bus.h"

extern I2C_HandleTypeDef hi2c1;

#define VEML6070_ADDRESS_LOW        0x38U
#define VEML6070_ADDRESS_HIGH       0x39U
#define VEML6070_ADDRESS_ARA        0x0CU
#define VEML6070_COMMAND_RESERVED   0x02U
#define VEML6070_COMMAND_IT_4T      0x0CU

/* Each public entry point holds the shared I2C1 lock for one whole
 * operation; the helpers below must therefore never take it themselves. */

static HAL_StatusTypeDef veml6070_init_locked(void)
{
    uint8_t ara_value = 0U;
    (void)HAL_I2C_Master_Receive(&hi2c1, VEML6070_ADDRESS_ARA << 1U,
                                 &ara_value, 1U, 100U);

    uint8_t command = VEML6070_COMMAND_RESERVED | VEML6070_COMMAND_IT_4T;
    return HAL_I2C_Master_Transmit(&hi2c1, VEML6070_ADDRESS_LOW << 1U,
                                   &command, 1U, 100U);
}

static HAL_StatusTypeDef veml6070_read_locked(uint16_t* uv_raw)
{
    uint8_t low = 0U;
    uint8_t high = 0U;
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
        &hi2c1, VEML6070_ADDRESS_LOW << 1U, &low, 1U, 100U);
    if(status == HAL_OK)
    {
        status = HAL_I2C_Master_Receive(
            &hi2c1, VEML6070_ADDRESS_HIGH << 1U, &high, 1U, 100U);
    }
    if(status == HAL_OK)
    {
        *uv_raw = (uint16_t)(((uint16_t)high << 8U) | low);
    }
    return status;
}

HAL_StatusTypeDef VEML6070_Init(void)
{
    HAL_StatusTypeDef status = I2cBus_Lock();
    if(status != HAL_OK)
    {
        return status;
    }
    status = veml6070_init_locked();
    I2cBus_Unlock();
    return status;
}

HAL_StatusTypeDef VEML6070_ReadUv(uint16_t* uv_raw)
{
    if(uv_raw == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = I2cBus_Lock();
    if(status != HAL_OK)
    {
        return status;
    }
    status = veml6070_read_locked(uv_raw);
    I2cBus_Unlock();
    return status;
}
