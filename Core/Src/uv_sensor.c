#include "device_specific.h"

extern I2C_HandleTypeDef hi2c1;

/* VEML 6070 */

#define VEML6070_ADDR_H (0x39) ///< High address
#define VEML6070_ADDR_L (0x38) ///< Low address

typedef union {
    struct {
        uint8_t SD : 1;
        uint8_t : 1;
        uint8_t IT : 2;
        uint8_t ACK_THD : 1;
        uint8_t ACK : 1;
    } bit;
    uint8_t reg;

} commandRegister;

typedef enum veml6070_integrationtime {
    VEML6070_HALF_T,
    VEML6070_1_T,
    VEML6070_2_T,
    VEML6070_4_T,
} veml6070_integrationtime_t;

commandRegister _commandRegister;

void Adafruit_VEML6070_begin(veml6070_integrationtime_t itime) {
    _commandRegister.bit.IT = itime;

    //clearAck();
    //writeCommand();
}


void Adafruit_VEML6070__waitForNext() {
    // Map the integration time code to the correct multiple (datasheet p. 8)
    // {0 -> 1, 1 -> 2; 2 -> 4; 3 -> 8}
    uint8_t itCount = 1;
    for(uint8_t i = _commandRegister.bit.IT; i > 0; i--) {
        itCount *= 2;
    }

    for(uint8_t i = 0; i < itCount; i++) {
        osDelay(63); // Depends on RSET = 270K, note actual time is shorter
        // than 62.5ms for RSET = 300K in datasheet table
    }
}

//#define VEML6070_ADDR_ARA (0x18 >> 1)
#define VEML6070_ADDR_CMD (0x70 >> 1)
#define VEML6070_ADDR_DATA_LSB (0x71 >> 1)
#define VEML6070_ADDR_DATA_MSB (0x73 >> 1)
// VEML6070 command register bits
#define VEML6070_CMD_SD 0x01
#define VEML6070_CMD_IT_0_5T 0x00
#define VEML6070_CMD_IT_1T 0x04
#define VEML6070_CMD_IT_2T 0x08
#define VEML6070_CMD_IT_4T 0x0C
#define VEML6070_CMD_DEFAULT (VEML6070_CMD_IT_4T)

uint8_t cmd = VEML6070_CMD_DEFAULT;

#define VEML6070_ADDR_H (0x39) ///< High address
#define VEML6070_ADDR_L (0x38) ///< Low address
#define VEML6070_ADDR_ARA  (0x0C) ///< Alert Resp Address (read to clear condition)

#define VEML6070_CMD_SD           0x01 /* Shutdown command */
#define VEML6070_CMD_RSV          0x02


uint16_t VEML6070_Init()
{
    volatile HAL_StatusTypeDef HAL_Status = HAL_ERROR;
    uint8_t data_out = VEML6070_ADDR_ARA;  // Read ARA to clear interrupt

    volatile uint8_t addr_l = 0;
    //HAL_Status = HAL_I2C_Master_Transmit(&hi2c1, VEML6070_ADDR_ARA << 1, &data_out, 1, 100);
    HAL_Status = HAL_I2C_Master_Receive(&hi2c1, VEML6070_ADDR_ARA << 1, &addr_l, 1, 100);
}

uint16_t VEML6070_ReadUV()
{
    //Adafruit_VEML6070__waitForNext();

    uint8_t input_[8];
    volatile uint8_t uvi;
    volatile HAL_StatusTypeDef HAL_Status = HAL_ERROR;

    volatile uint8_t addr_l = 0;
    volatile uint8_t addr_h = 0;

    cmd = VEML6070_CMD_RSV & ~VEML6070_CMD_SD;  // enable_sensor
    HAL_Status = HAL_I2C_Master_Transmit(&hi2c1, VEML6070_ADDR_L << 1, &cmd, 1, 100);

    osDelay(200);

    uint8_t data_out = 0x73;
    //HAL_Status = HAL_I2C_Master_Transmit(&hi2c1, VEML6070_ADDR_L << 1, &data_out, 1, 100);
    HAL_Status = HAL_I2C_Master_Receive(&hi2c1, VEML6070_ADDR_H << 1, &addr_l, 1, 100);

    data_out = 0x71;
    //HAL_Status = HAL_I2C_Master_Transmit(&hi2c1, VEML6070_ADDR_H << 1, &data_out, 1, 100);
    HAL_Status = HAL_I2C_Master_Receive(&hi2c1, VEML6070_ADDR_L << 1, &addr_h, 1, 100);

    uint16_t data = ((uint16_t)addr_l << 8) | (uint16_t)addr_h;
    return data;
}

