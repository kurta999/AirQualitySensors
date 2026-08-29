#include "light_sensor.h"

#include <math.h>
#include <stdint.h>

#include "cmsis_os.h"
#include "measurement_store.h"

extern I2C_HandleTypeDef hi2c1;

#define TCS34725_ADDRESS                0x29U
#define TCS34725_COMMAND_BIT            0x80U
#define TCS34725_AUTO_INCREMENT         0x20U
#define TCS34725_REGISTER_ENABLE        0x00U
#define TCS34725_REGISTER_ATIME         0x01U
#define TCS34725_REGISTER_CONTROL       0x0FU
#define TCS34725_REGISTER_ID            0x12U
#define TCS34725_REGISTER_CLEAR_LOW     0x14U
#define TCS34725_ENABLE_POWER_ON        0x01U
#define TCS34725_ENABLE_ADC             0x02U
#define TCS34725_INTEGRATION_TIME_50MS  0xEBU
#define TCS34725_GAIN_4X                0x01U

static HAL_StatusTypeDef write_register(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(
        &hi2c1,
        TCS34725_ADDRESS << 1U,
        TCS34725_COMMAND_BIT | reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        100U);
}

static HAL_StatusTypeDef read_register(uint8_t reg, uint8_t* value)
{
    return HAL_I2C_Mem_Read(
        &hi2c1,
        TCS34725_ADDRESS << 1U,
        TCS34725_COMMAND_BIT | reg,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1U,
        100U);
}

static HAL_StatusTypeDef read_raw(uint16_t* red, uint16_t* green,
                                  uint16_t* blue, uint16_t* clear)
{
    uint8_t input[8] = {0U};
    const HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1,
        TCS34725_ADDRESS << 1U,
        TCS34725_COMMAND_BIT | TCS34725_AUTO_INCREMENT | TCS34725_REGISTER_CLEAR_LOW,
        I2C_MEMADD_SIZE_8BIT,
        input,
        sizeof(input),
        100U);
    if(status != HAL_OK)
    {
        return status;
    }

    *clear = (uint16_t)(((uint16_t)input[1] << 8U) | input[0]);
    *red = (uint16_t)(((uint16_t)input[3] << 8U) | input[2]);
    *green = (uint16_t)(((uint16_t)input[5] << 8U) | input[4]);
    *blue = (uint16_t)(((uint16_t)input[7] << 8U) | input[6]);
    return HAL_OK;
}

static uint16_t calculate_color_temperature(uint16_t red, uint16_t green,
                                             uint16_t blue, uint16_t clear)
{
    if(clear == 0U)
    {
        return 0U;
    }

    const uint32_t total = (uint32_t)red + green + blue;
    const uint16_t infrared = total > clear
        ? (uint16_t)((total - clear) / 2U)
        : 0U;
    const uint16_t red_without_ir = red > infrared ? (uint16_t)(red - infrared) : 0U;
    const uint16_t blue_without_ir = blue > infrared ? (uint16_t)(blue - infrared) : 0U;
    if(red_without_ir == 0U)
    {
        return 0U;
    }

    const uint32_t temperature =
        ((3810U * blue_without_ir) / red_without_ir) + 1391U;
    return temperature > UINT16_MAX ? UINT16_MAX : (uint16_t)temperature;
}

static uint16_t calculate_lux(uint16_t red, uint16_t green, uint16_t blue)
{
    const float illuminance =
        (-0.32466F * red) + (1.57837F * green) + (-0.73191F * blue);
    if(illuminance <= 0.0F)
    {
        return 0U;
    }
    if(illuminance >= (float)UINT16_MAX)
    {
        return UINT16_MAX;
    }
    return (uint16_t)lroundf(illuminance);
}

HAL_StatusTypeDef Tcs34725_Init(void)
{
    uint8_t id = 0U;
    HAL_StatusTypeDef status = read_register(TCS34725_REGISTER_ID, &id);
    if(status != HAL_OK || (id != 0x4DU && id != 0x44U && id != 0x10U))
    {
        return HAL_ERROR;
    }

    status = write_register(TCS34725_REGISTER_ATIME, TCS34725_INTEGRATION_TIME_50MS);
    if(status == HAL_OK)
    {
        status = write_register(TCS34725_REGISTER_CONTROL, TCS34725_GAIN_4X);
    }
    if(status == HAL_OK)
    {
        status = write_register(TCS34725_REGISTER_ENABLE, TCS34725_ENABLE_POWER_ON);
    }
    if(status == HAL_OK)
    {
        osDelay(3U);
        status = write_register(
            TCS34725_REGISTER_ENABLE,
            TCS34725_ENABLE_POWER_ON | TCS34725_ENABLE_ADC);
    }
    if(status == HAL_OK)
    {
        osDelay(55U);
    }
    return status;
}

HAL_StatusTypeDef Tcs34725_Read(void)
{
    uint16_t red_raw = 0U;
    uint16_t green_raw = 0U;
    uint16_t blue_raw = 0U;
    uint16_t clear_raw = 0U;
    const HAL_StatusTypeDef status =
        read_raw(&red_raw, &green_raw, &blue_raw, &clear_raw);
    if(status != HAL_OK)
    {
        return status;
    }

    float red = 0.0F;
    float green = 0.0F;
    float blue = 0.0F;
    if(clear_raw != 0U)
    {
        red = ((float)red_raw / (float)clear_raw) * 255.0F;
        green = ((float)green_raw / (float)clear_raw) * 255.0F;
        blue = ((float)blue_raw / (float)clear_raw) * 255.0F;
    }

    Measurements_UpdateLight(
        red,
        green,
        blue,
        calculate_color_temperature(red_raw, green_raw, blue_raw, clear_raw),
        calculate_lux(red_raw, green_raw, blue_raw));
    return HAL_OK;
}
