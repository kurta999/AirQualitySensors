
#ifndef ST7565_ST7565_H_
#define ST7565_ST7565_H_

#include <stdint.h>
#include "cmsis_os.h"
#include "device_specific.h"
#include "ST7565_config.h"
#if USE_OWN_FONTS
#include "st7565_fonts/st7565_fonts.h"
#endif

typedef struct
{
  SPI_HandleTypeDef* spi_handle;  /**< SPI Handle */
  osMutexId* spi_mutex;  /**< SPI mutex for LCD */
  uint8_t graphic_buffer[1024];  /**< Graphics buffer for LCD */
  GPIO_TypeDef* CS_GPIO_Port;  /**< CS GPIO Port */
  uint16_t CS_GPIO_Pin;  /**< CS GPIO Pin */
  GPIO_TypeDef* RESET_GPIO_Port;  /**< RESET GPIO Port */
  uint16_t RESET_GPIO_Pin;  /**< RESET GPIO Pin */
  GPIO_TypeDef* A_GPIO_Port;  /**< A GPIO GPIO Port */
  uint16_t A_GPIO_Pin;  /**< A GPIO GPIO Pin */
#if ENABLE_PARTIAL_UPDATE == 1
  uint8_t xUpdateMin;  /**< Min X pixel in buffer where updating display starts from */
  uint8_t xUpdateMax;  /**< Max Y pixel in buffer where updating display starts from */
  uint8_t yUpdateMin;  /**< Min X pixel in buffer where updating display starts from */
  uint8_t yUpdateMax;  /**< Max Y pixel in buffer where updating display starts from */
#endif
  uint8_t to_init;  /**< Init sequence required? */
  uint8_t contrast;  /**< Contrast for LCD */
  uint8_t direction;  /**< Direction of LDC: 0 = normal, 1 = upside down */
} ST7565Handle;  /**< ST7565 LCD Handle */

void ST7565_Init(ST7565Handle* handle, SPI_HandleTypeDef* spi_handle, osMutexId* spi_mutex,
                 GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_GPIO_Pin, GPIO_TypeDef* RESET_GPIO_Port,
                 uint16_t RESET_GPIO_Pin, GPIO_TypeDef* A_GPIO_Port, uint16_t A_GPIO_Pin,
                 uint8_t contrast, uint8_t direction);
HAL_StatusTypeDef ST7565_Task(ST7565Handle* handle);
HAL_StatusTypeDef ST7565_ClearDisplay(ST7565Handle* handle);
void ST7565_DrawString(ST7565Handle* handle, uint8_t x, uint8_t line, const char *str
#if USE_OWN_FONTS
,FONT_INFO* font
#endif
);
void ST7565_DrawChar(ST7565Handle* handle, uint8_t pos_x, uint8_t pos_y, char character
#if USE_OWN_FONTS
,FONT_INFO* font
#endif
);
HAL_StatusTypeDef ST7565_DeInit(ST7565Handle* handle);
HAL_StatusTypeDef ST7565_SetBrightness(ST7565Handle* handle, uint8_t contrast);
void ST7565_ClearBuffer(ST7565Handle* handle);
HAL_StatusTypeDef ST7565_Display(ST7565Handle* handle);
void ST7565_SetPixel(ST7565Handle* handle, uint8_t x, uint8_t y, uint8_t color);
uint8_t ST7565_GetPixel(ST7565Handle* handle, uint8_t x, uint8_t y);
void ST7565_FillCircle(ST7565Handle* handle, uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
void ST7565_DrawCircle(ST7565Handle* handle, uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
void ST7565_DrawRectangle(ST7565Handle* handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void ST7565_FillRectangle(ST7565Handle* handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void ST7565_DrawLine(ST7565Handle* handle, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void ST7565_DrawBitmap(ST7565Handle* handle, uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color);
#endif /* ST7565_ST7565_H_ */
