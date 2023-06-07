#include "cmsis_os.h"
#include "device_specific.h"
#include "scd30.h"
#include "bme680.h"
#include "bme680_defs.h"
#include "main.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

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
    if (useconds % 1000 > 0) {
        msec++;
    }

    /*
     * Increment by 1 if STM32F1 driver version less than 1.1.1
     * Old firmwares of STM32F1 sleep 1ms shorter than specified in HAL_Delay.
     * This was fixed with firmware 1.6 (driver version 1.1.1), so we have to
     * fix it ourselves for older firmwares
     */
    if (HAL_GetHalVersion() < 0x01010100) {
        msec++;
    }

    //HAL_Delay(msec);
    osDelay(msec);
}

#define NO_ERROR 0
int interval_in_seconds = 2;
float co2_ppm, temperature, relative_humidity;
float bme680_temp, bme680_hum, bme680_pressure;
uint16_t uv;

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
  for (uint8_t i = _commandRegister.bit.IT; i > 0; i--) {
    itCount *= 2;
  }

  for (uint8_t i = 0; i < itCount; i++) {
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

struct bme680_dev gas_sensor;
struct bme680_field_data data;

void user_delay_ms(uint32_t period);
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);


#define TCS34725_ADDRESS (0x29)     /**< I2C address **/
#define TCS34725_COMMAND_BIT (0x80) /**< Command bit **/

#define TCS34725_ENABLE (0x00)      /**< Interrupt Enable register */
#define TCS34725_ENABLE_AIEN (0x10) /**< RGBC Interrupt Enable */
#define TCS34725_ENABLE_WEN                                                    \
  (0x08) /**< Wait Enable - Writing 1 activates the wait timer */
#define TCS34725_ENABLE_AEN                                                    \
  (0x02) /**< RGBC Enable - Writing 1 actives the ADC, 0 disables it */
#define TCS34725_ENABLE_PON                                                    \
  (0x01) /**< Power on - Writing 1 activates the internal oscillator, 0        \
            disables it */
#define TCS34725_ATIME (0x01) /**< Integration time */
#define TCS34725_WTIME                                                         \
  (0x03) /**< Wait time (if TCS34725_ENABLE_WEN is asserted) */
#define TCS34725_WTIME_2_4MS (0xFF) /**< WLONG0 = 2.4ms   WLONG1 = 0.029s */
#define TCS34725_WTIME_204MS (0xAB) /**< WLONG0 = 204ms   WLONG1 = 2.45s  */
#define TCS34725_WTIME_614MS (0x00) /**< WLONG0 = 614ms   WLONG1 = 7.4s   */
#define TCS34725_AILTL                                                         \
  (0x04) /**< Clear channel lower interrupt threshold (lower byte) */
#define TCS34725_AILTH                                                         \
  (0x05) /**< Clear channel lower interrupt threshold (higher byte) */
#define TCS34725_AIHTL                                                         \
  (0x06) /**< Clear channel upper interrupt threshold (lower byte) */
#define TCS34725_AIHTH                                                         \
  (0x07) /**< Clear channel upper interrupt threshold (higher byte) */
#define TCS34725_PERS                                                          \
  (0x0C) /**< Persistence register - basic SW filtering mechanism for          \
            interrupts */
#define TCS34725_PERS_NONE                                                     \
  (0b0000) /**< Every RGBC cycle generates an interrupt */
#define TCS34725_PERS_1_CYCLE                                                  \
  (0b0001) /**< 1 clean channel value outside threshold range generates an     \
              interrupt */
#define TCS34725_PERS_2_CYCLE                                                  \
  (0b0010) /**< 2 clean channel values outside threshold range generates an    \
              interrupt */
#define TCS34725_PERS_3_CYCLE                                                  \
  (0b0011) /**< 3 clean channel values outside threshold range generates an    \
              interrupt */
#define TCS34725_PERS_5_CYCLE                                                  \
  (0b0100) /**< 5 clean channel values outside threshold range generates an    \
              interrupt */
#define TCS34725_PERS_10_CYCLE                                                 \
  (0b0101) /**< 10 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_PERS_15_CYCLE                                                 \
  (0b0110) /**< 15 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_PERS_20_CYCLE                                                 \
  (0b0111) /**< 20 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_PERS_25_CYCLE                                                 \
  (0b1000) /**< 25 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_PERS_30_CYCLE                                                 \
  (0b1001) /**< 30 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_PERS_35_CYCLE                                                 \
  (0b1010) /**< 35 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_PERS_40_CYCLE                                                 \
  (0b1011) /**< 40 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_PERS_45_CYCLE                                                 \
  (0b1100) /**< 45 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_PERS_50_CYCLE                                                 \
  (0b1101) /**< 50 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_PERS_55_CYCLE                                                 \
  (0b1110) /**< 55 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_PERS_60_CYCLE                                                 \
  (0b1111) /**< 60 clean channel values outside threshold range generates an   \
              interrupt*/
#define TCS34725_CONFIG (0x0D) /**< Configuration **/
#define TCS34725_CONFIG_WLONG                                                  \
  (0x02) /**< Choose between short and long (12x) wait times via               \
            TCS34725_WTIME */
#define TCS34725_CONTROL (0x0F) /**< Set the gain level for the sensor */
#define TCS34725_ID                                                            \
  (0x12) /**< 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_STATUS (0x13)      /**< Device status **/
#define TCS34725_STATUS_AINT (0x10) /**< RGBC Clean channel interrupt */
#define TCS34725_STATUS_AVALID                                                 \
  (0x01) /**< Indicates that the RGBC channels have completed an integration   \
            cycle */
#define TCS34725_CDATAL (0x14) /**< Clear channel data low byte */
#define TCS34725_CDATAH (0x15) /**< Clear channel data high byte */
#define TCS34725_RDATAL (0x16) /**< Red channel data low byte */
#define TCS34725_RDATAH (0x17) /**< Red channel data high byte */
#define TCS34725_GDATAL (0x18) /**< Green channel data low byte */
#define TCS34725_GDATAH (0x19) /**< Green channel data high byte */
#define TCS34725_BDATAL (0x1A) /**< Blue channel data low byte */
#define TCS34725_BDATAH (0x1B) /**< Blue channel data high byte */

/** Integration time settings for TCS34725 */
/*
 * 60-Hz period: 16.67ms, 50-Hz period: 20ms
 * 100ms is evenly divisible by 50Hz periods and by 60Hz periods
 */
#define TCS34725_INTEGRATIONTIME_2_4MS                                         \
  (0xFF) /**< 2.4ms - 1 cycle - Max Count: 1024 */
#define TCS34725_INTEGRATIONTIME_24MS                                          \
  (0xF6) /**< 24.0ms - 10 cycles - Max Count: 10240 */
#define TCS34725_INTEGRATIONTIME_50MS                                          \
  (0xEB) /**< 50.4ms - 21 cycles - Max Count: 21504 */
#define TCS34725_INTEGRATIONTIME_60MS                                          \
  (0xE7) /**< 60.0ms - 25 cycles - Max Count: 25700 */
#define TCS34725_INTEGRATIONTIME_101MS                                         \
  (0xD6) /**< 100.8ms - 42 cycles - Max Count: 43008 */
#define TCS34725_INTEGRATIONTIME_120MS                                         \
  (0xCE) /**< 120.0ms - 50 cycles - Max Count: 51200 */
#define TCS34725_INTEGRATIONTIME_154MS                                         \
  (0xC0) /**< 153.6ms - 64 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_180MS                                         \
  (0xB5) /**< 180.0ms - 75 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_199MS                                         \
  (0xAD) /**< 199.2ms - 83 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_240MS                                         \
  (0x9C) /**< 240.0ms - 100 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_300MS                                         \
  (0x83) /**< 300.0ms - 125 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_360MS                                         \
  (0x6A) /**< 360.0ms - 150 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_401MS                                         \
  (0x59) /**< 400.8ms - 167 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_420MS                                         \
  (0x51) /**< 420.0ms - 175 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_480MS                                         \
  (0x38) /**< 480.0ms - 200 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_499MS                                         \
  (0x30) /**< 499.2ms - 208 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_540MS                                         \
  (0x1F) /**< 540.0ms - 225 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_600MS                                         \
  (0x06) /**< 600.0ms - 250 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_614MS                                         \
  (0x00) /**< 614.4ms - 256 cycles - Max Count: 65535 */

/** Gain settings for TCS34725  */
typedef enum {
  TCS34725_GAIN_1X = 0x00,  /**<  No gain  */
  TCS34725_GAIN_4X = 0x01,  /**<  4x gain  */
  TCS34725_GAIN_16X = 0x02, /**<  16x gain */
  TCS34725_GAIN_60X = 0x03  /**<  60x gain */
} tcs34725Gain_t;

tcs34725Gain_t _tcs34725Gain;
uint8_t _tcs34725IntegrationTime;


void TCS_write8(uint8_t reg, uint8_t value)
{
  uint8_t buffer[2] = {(uint8_t)(TCS34725_COMMAND_BIT | reg), value};
  HAL_StatusTypeDef HAL_Status = HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS << 1, buffer, 2, 100);
  printf("%d", HAL_Status);
}

uint8_t TCS_read8(uint8_t reg)
{

  uint8_t buffer[1] = {(uint8_t)(TCS34725_COMMAND_BIT | reg)};
  uint8_t input;
  HAL_StatusTypeDef HAL_Status = HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS << 1, buffer, 1, 100);
  printf("%d", HAL_Status);

  HAL_Status = HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS << 1, &input, 1, 100);
  printf("%d", HAL_Status);
  return input;
}

uint16_t TCS_read16(uint8_t reg)
{

  uint8_t buffer[1] = {(uint8_t)(TCS34725_COMMAND_BIT | reg)};
  uint8_t input[2];
  HAL_StatusTypeDef HAL_Status = HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS << 1, buffer, 1, 100);
  printf("%d", HAL_Status);

  HAL_Status = HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS << 1, input, 2, 100);
  printf("%d", HAL_Status);
  return (uint16_t)(input[1]) << 8 | (uint16_t)(input[0]) & 0xFF;
}

void TCS_setIntegrationTime(uint8_t it)
{

  /* Update the timing register */
  TCS_write8(TCS34725_ATIME, it);

  /* Update value placeholders */
  _tcs34725IntegrationTime = it;
}

void TCS_setGain(tcs34725Gain_t gain)
{

  /* Update the timing register */
  TCS_write8(TCS34725_CONTROL, gain);

  /* Update value placeholders */
  _tcs34725Gain = gain;
}

void TCS_enable()
{
  TCS_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  osDelay(3);
  TCS_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
  /* Set a delay for the integration time.
    This is only necessary in the case where enabling and then
    immediately trying to read values back. This is because setting
    AEN triggers an automatic integration, so if a read RGBC is
    performed too quickly, the data is not yet valid and all 0's are
    returned */
  /* 12/5 = 2.4, add 1 to account for integer truncation */
  osDelay((256 - _tcs34725IntegrationTime) * 12 / 5 + 1);
}

void TCS_getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = TCS_read16(TCS34725_CDATAL);
  *r = TCS_read16(TCS34725_RDATAL);
  *g = TCS_read16(TCS34725_GDATAL);
  *b = TCS_read16(TCS34725_BDATAL);

  /* Set a delay for the integration time */
  /* 12/5 = 2.4, add 1 to account for integer truncation */
  osDelay((256 - _tcs34725IntegrationTime) * 12 / 5 + 1);
}

void TCS_getRGB(float *r, float *g, float *b) {
  uint16_t red, green, blue, clear;
  TCS_getRawData(&red, &green, &blue, &clear);
  uint32_t sum = clear;

  // Avoid divide by zero errors ... if clear = 0 return black
  if (clear == 0) {
    *r = *g = *b = 0;
    return;
  }

  *r = (float)red / sum * 255.0;
  *g = (float)green / sum * 255.0;
  *b = (float)blue / sum * 255.0;
}

void TCS_setInterrupt(uint8_t i) {
  uint8_t r = TCS_read8(TCS34725_ENABLE);
  if (i) {
    r |= TCS34725_ENABLE_AIEN;
  } else {
    r &= ~TCS34725_ENABLE_AIEN;
  }
  TCS_write8(TCS34725_ENABLE, r);
}

uint16_t TCS_calculateColorTemperature_dn40(uint16_t r,
                                                           uint16_t g,
                                                           uint16_t b,
                                                           uint16_t c) {
  uint16_t r2, b2; /* RGB values minus IR component */
  uint16_t sat;    /* Digital saturation level */
  uint16_t ir;     /* Inferred IR content */

  if (c == 0) {
    return 0;
  }

  /* Analog/Digital saturation:
   *
   * (a) As light becomes brighter, the clear channel will tend to
   *     saturate first since R+G+B is approximately equal to C.
   * (b) The TCS34725 accumulates 1024 counts per 2.4ms of integration
   *     time, up to a maximum values of 65535. This means analog
   *     saturation can occur up to an integration time of 153.6ms
   *     (64*2.4ms=153.6ms).
   * (c) If the integration time is > 153.6ms, digital saturation will
   *     occur before analog saturation. Digital saturation occurs when
   *     the count reaches 65535.
   */
  if ((256 - _tcs34725IntegrationTime) > 63) {
    /* Track digital saturation */
    sat = 65535;
  } else {
    /* Track analog saturation */
    sat = 1024 * (256 - _tcs34725IntegrationTime);
  }

  /* Ripple rejection:
   *
   * (a) An integration time of 50ms or multiples of 50ms are required to
   *     reject both 50Hz and 60Hz ripple.
   * (b) If an integration time faster than 50ms is required, you may need
   *     to average a number of samples over a 50ms period to reject ripple
   *     from fluorescent and incandescent light sources.
   *
   * Ripple saturation notes:
   *
   * (a) If there is ripple in the received signal, the value read from C
   *     will be less than the max, but still have some effects of being
   *     saturated. This means that you can be below the 'sat' value, but
   *     still be saturating. At integration times >150ms this can be
   *     ignored, but <= 150ms you should calculate the 75% saturation
   *     level to avoid this problem.
   */
  if ((256 - _tcs34725IntegrationTime) <= 63) {
    /* Adjust sat to 75% to avoid analog saturation if atime < 153.6ms */
    sat -= sat / 4;
  }

  /* Check for saturation and mark the sample as invalid if true */
  if (c >= sat) {
    return 0;
  }

  /* AMS RGB sensors have no IR channel, so the IR content must be */
  /* calculated indirectly. */
  ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;

  /* Remove the IR component from the raw RGB values */
  r2 = r - ir;
  b2 = b - ir;

  if (r2 == 0) {
    return 0;
  }

  /* A simple method of measuring color temp is to use the ratio of blue */
  /* to red light, taking IR cancellation into account. */
  uint16_t cct = (3810 * (uint32_t)b2) / /** Color temp coefficient. */
                     (uint32_t)r2 +
                 1391; /** Color temp offset. */

  return cct;
}

uint16_t TCS_calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}

float red, green, blue;
uint16_t colorTemp, lux;

uint16_t TCS_Init()
{
  uint8_t x = TCS_read8(TCS34725_ID);
  if ((x != 0x4d) && (x != 0x44) && (x != 0x10))
  {

  }

  TCS_setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
  TCS_setGain(TCS34725_GAIN_4X);
  TCS_enable();

  osDelay(1000);
}

void TCS_Read()
{
  TCS_setInterrupt(0); // turn on LED
  osDelay(60);

  TCS_getRGB(&red, &green, &blue);

  uint16_t raw_red, raw_green, raw_blue, raw_clear;
  TCS_getRawData(&raw_red, &raw_green, &raw_blue, &raw_clear);

  TCS_setInterrupt(1); // turn off LED

  colorTemp = TCS_calculateColorTemperature_dn40(raw_red, raw_green, raw_blue, raw_clear);
  lux = TCS_calculateLux(raw_red, raw_green, raw_blue);
}

void SensorsBme680Task(void const * argument)
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

      /* Trigger the next measurement if you would like to read data out continuously */
      if (gas_sensor.power_mode == BME680_FORCED_MODE)
      {
          rslt = bme680_set_sensor_mode(&gas_sensor);
      }

      TCS_Read();
  }
}

/*
  osDelay(3000);
  Adafruit_VEML6070_begin(VEML6070_1_T);
  uint16_t uv_index = Adafruit_VEML6070__readUV();
*/



void SensorsTask(void const * argument)
{
  /* USER CODE BEGIN SensorsTask */
  int16_t err;
  uint16_t interval_in_seconds = 2;

  VEML6070_Init();

  while (scd30_probe() != NO_ERROR)
  {
      printf("SCD30 sensor probing failed\n");
      osDelay(1000);
  }

  scd30_set_measurement_interval(interval_in_seconds);
  sensirion_sleep_usec(20000u);
  scd30_start_periodic_measurement(999);

  /* Infinite loop */
  for(;;)
  {
    uv = VEML6070_ReadUV();
    uint16_t data_ready = 0;
    uint16_t timeout = 0;

      /* Poll data_ready flag until data is available. Allow 20% more than
       * the measurement interval to account for clock imprecision of the
       * sensor.
       */
      for (timeout = 0; (100000 * timeout) < (interval_in_seconds * 1200000); ++timeout) {
          err = scd30_get_data_ready(&data_ready);
          if (err != NO_ERROR) {
              printf("Error reading data_ready flag: %i\n", err);
          }
          if (data_ready) {
              break;
          }
          sensirion_sleep_usec(100000);
      }
      if (!data_ready) {
          printf("Timeout waiting for data_ready flag\n");
          continue;
      }

      /* Measure co2, temperature and relative humidity and store into
       * variables.
       */
      err = scd30_read_measurement(&co2_ppm, &temperature, &relative_humidity);
      if (err != NO_ERROR)
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
      osDelay(1000);
  }
  /* USER CODE END SensorsTask */
}



void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
  osDelay(period);
}

extern  SPI_HandleTypeDef hspi2;

int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
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

int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
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

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
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

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
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
