

/* Includes ------------------------------------------------------------------*/
#include "ST7565.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "cmsis_os.h"
#include "device_specific.h"
//#include "ST7565_font_config.h"
#include "ST7565_config.h"
#if USE_OWN_FONTS
#include "st7565_fonts/st7565_fonts.h"
#else
#include "ST7565_font_example.h"
#endif
/*
#ifndef ST7565_FONT_CONFIG_H_
#error "ST7565_font_config.h not found!"
#endif
*/
/* Private macro -------------------------------------------------------------*/
#define ST7565_LCDWIDTH         128
#define ST7565_LCDHEIGHT        64
#define CMD_DISPLAY_OFF         0xAE
#define CMD_DISPLAY_ON          0xAF
#define CMD_SET_DISP_START_LINE 0x40
#define CMD_SET_PAGE            0xB0
#define CMD_SET_COLUMN_UPPER    0x10
#define CMD_SET_COLUMN_LOWER    0x00
#define CMD_SET_ADC_NORMAL      0xA0
#define CMD_SET_ADC_REVERSE     0xA1
#define CMD_SET_DISP_NORMAL     0xA6
#define CMD_SET_DISP_REVERSE    0xA7
#define CMD_SET_ALLPTS_NORMAL   0xA4
#define CMD_SET_ALLPTS_ON       0xA5
#define CMD_SET_BIAS_9          0xA2
#define CMD_SET_BIAS_7          0xA3
#define CMD_RMW                 0xE0
#define CMD_RMW_CLEAR           0xEE
#define CMD_INTERNAL_RESET      0xE2
#define CMD_SET_COM_NORMAL      0xC0
#define CMD_SET_COM_REVERSE     0xC8
#define CMD_SET_POWER_CONTROL   0x28
#define CMD_SET_RESISTOR_RATIO  0x20
#define CMD_SET_VOLUME_FIRST    0x81
#define CMD_SET_VOLUME_SECOND   0
#define CMD_SET_STATIC_OFF      0xAC
#define CMD_SET_STATIC_ON       0xAD
#define CMD_SET_STATIC_REG      0x0
#define CMD_SET_BOOSTER_FIRST   0xF8
#define CMD_SET_BOOSTER_234     0
#define CMD_SET_BOOSTER_5       1
#define CMD_SET_BOOSTER_6       3
#define CMD_NOP                 0xE3
#define CMD_TEST                0xF0
#define swap(a, b) { uint8_t t = a; a = b; b = t; }
/* Private variables ---------------------------------------------------------*/
static const uint8_t pagemap[] = { 7, 6, 5, 4, 3, 2, 1, 0 }; /**< A handy reference to where the pages are on the screen */

// --------------------------
// ---  private functions ---
// --------------------------

/**
 * @brief Sends data/command to the display via SPI
 * @param handle The handle for display
 * @param c Command bits
 */
static HAL_StatusTypeDef ST7565_SPIWrite(ST7565Handle* handle, uint8_t c)
{
  HAL_StatusTypeDef HAL_Status = HAL_ERROR;
  if(handle == NULL || handle->spi_handle == NULL || handle->spi_mutex == NULL ||
     *handle->spi_mutex == NULL)
    return HAL_ERROR;
  if(osMutexWait(*handle->spi_mutex, 1000) == osOK)
  {
    HAL_GPIO_WritePin(handle->CS_GPIO_Port, handle->CS_GPIO_Pin, 0);
    HAL_Status = HAL_SPI_Transmit(handle->spi_handle, &c, 1, 100);
    HAL_GPIO_WritePin(handle->CS_GPIO_Port, handle->CS_GPIO_Pin, 1);
    (void)osMutexRelease(*handle->spi_mutex);
  }
  return HAL_Status;
}

/**
 * @brief Sends command to the display via SPI
 * @param handle The handle for display
 * @param c Command bits
 */
static HAL_StatusTypeDef ST7565_Command(ST7565Handle* handle, uint8_t c)
{
  HAL_GPIO_WritePin(handle->A_GPIO_Port, handle->A_GPIO_Pin, 0);
  return ST7565_SPIWrite(handle, c);
}

/**
 * @brief Sends data to the display via SPI
 * @param handle The handle for display
 * @param c Command bits
 */
static HAL_StatusTypeDef ST7565_Data(ST7565Handle* handle, uint8_t c)
{
  HAL_GPIO_WritePin(handle->A_GPIO_Port, handle->A_GPIO_Pin, 1);
  return ST7565_SPIWrite(handle, c);
}

/**
 * @brief Turns on/off given pixel
 * @param handle The handle for display
 * @param x X pixel
 * @param y Y pixel
 * @param color Toggle the pixel: 0/1
 */
static void ST7565_SetPixelInternal(ST7565Handle* handle, uint8_t x, uint8_t y, uint8_t color)
{
  if((x >= ST7565_LCDWIDTH) || (y >= ST7565_LCDHEIGHT))
    return;
  if(color)
    handle->graphic_buffer[x + (y / 8) * 128] |= (1 << (7 - (y % 8)));
  else
    handle->graphic_buffer[x + (y / 8) * 128] &= ~(1 << (7 - (y % 8)));
}

/**
 * @brief Update bounding box
 * @param handle The handle for display
 * @param xmin Min X pixel
 * @param ymin Min Y pixel
 * @param xmax Max X pixel
 * @param ymax Max Y pixel
 */
static void ST7565_UpdateBoundingBox(ST7565Handle* handle, uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax)
{
#if ENABLE_PARTIAL_UPDATE == 1
  if(xmin < handle->xUpdateMin)
    handle->xUpdateMin = xmin;
  if(xmax > handle->xUpdateMax)
    handle->xUpdateMax = xmax;
  if(ymin < handle->yUpdateMin)
    handle->yUpdateMin = ymin;
  if(ymax > handle->yUpdateMax)
    handle->yUpdateMax = ymax;
#else
  (void)handle;
  (void)xmin;
  (void)ymin;
  (void)xmax;
  (void)ymax;
#endif
}

/**
 * @brief Init sequence for ST7565
 * @param handle The handle for display
 */
static HAL_StatusTypeDef ST7565_InitSequence(ST7565Handle* handle)
{
  HAL_StatusTypeDef HAL_Status = HAL_OK;
  HAL_GPIO_WritePin(handle->CS_GPIO_Port, handle->CS_GPIO_Pin, 0);
  HAL_GPIO_WritePin(handle->RESET_GPIO_Port, handle->RESET_GPIO_Pin, 0);
  osDelay(500);
  HAL_GPIO_WritePin(handle->RESET_GPIO_Port, handle->RESET_GPIO_Pin, 1);
  HAL_Status = ST7565_Command(handle, CMD_SET_BIAS_9);  /* LCD bias select */
  if(HAL_Status == HAL_OK)
  {
    if(!handle->direction)
      HAL_Status = ST7565_Command(handle, CMD_SET_ADC_NORMAL);  /* ADC select */
    else
      HAL_Status = ST7565_Command(handle, CMD_SET_ADC_REVERSE);  /* ADC select */
  }
  if(HAL_Status == HAL_OK)
  {
    if(!handle->direction)
      HAL_Status = ST7565_Command(handle, CMD_SET_COM_NORMAL);  /* SHL select */
    else
      HAL_Status = ST7565_Command(handle, CMD_SET_COM_REVERSE);  /* SHL select */
  }
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Command(handle, CMD_SET_DISP_START_LINE);  /* Initial display line */
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Command(handle, CMD_SET_POWER_CONTROL | 0x4);  /* turn on voltage converter (VC=1, VR=0, VF=0) */
  osDelay(50);  /* wait for 50% rising */
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Command(handle, CMD_SET_POWER_CONTROL | 0x6);  /* turn on voltage regulator (VC=1, VR=1, VF=0) */
  osDelay(50);
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Command(handle, CMD_SET_POWER_CONTROL | 0x7);  /* turn on voltage follower (VC=1, VR=1, VF=1) */
  osDelay(10);
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Command(handle, CMD_SET_RESISTOR_RATIO | 0x6);  /* set lcd operating voltage
                                                                         * (regulator resistor, ref voltage resistor)
                                                                         */
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Command(handle, CMD_SET_RESISTOR_RATIO | 0x5);  /* set lcd operating voltage
                                                                         * (regulator resistor, ref voltage resistor)
                                                                         */
  ST7565_UpdateBoundingBox(handle, 0, 0, ST7565_LCDWIDTH - 1, ST7565_LCDHEIGHT - 1);
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Command(handle, CMD_SET_ALLPTS_NORMAL);
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Command(handle, CMD_DISPLAY_ON);
  if(HAL_Status == HAL_OK)
      HAL_Status = ST7565_SetBrightness(handle, handle->contrast);
  return HAL_Status;
}

// -------------------------
// ---  public functions ---
// -------------------------

/**
 * @brief Initializes display
 * @param handle The handle for display
 * @param spi_handle SPI handle for display
 * @param spi_mutex SPI mutex pointer
 * @param CS_GPIO_Port Chip Select GPIO Port
 * @param CS_GPIO_Pin Chip Select GPIO Pin
 * @param RESET_GPIO_Port Reset GPIO Port
 * @param RESET_GPIO_Pin Reset GPIO Pin
 * @param A_GPIO_Port A GPIO Port
 * @param A_GPIO_Pin A GPIO Pin
 * @param contrast Contrast for display (refer to datasheet)
 * @param direction Direction of display (0 = normal, 1 = upside down)
 */
void ST7565_Init(ST7565Handle* handle, SPI_HandleTypeDef* spi_handle, osMutexId* spi_mutex,
                 GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_GPIO_Pin, GPIO_TypeDef* RESET_GPIO_Port,
                 uint16_t RESET_GPIO_Pin, GPIO_TypeDef* A_GPIO_Port, uint16_t A_GPIO_Pin,
                 uint8_t contrast, uint8_t direction)
{
  handle->A_GPIO_Pin = A_GPIO_Pin;
  handle->A_GPIO_Port = A_GPIO_Port;
  handle->CS_GPIO_Pin = CS_GPIO_Pin;
  handle->CS_GPIO_Port = CS_GPIO_Port;
  handle->RESET_GPIO_Pin = RESET_GPIO_Pin;
  handle->RESET_GPIO_Port = RESET_GPIO_Port;
  handle->spi_handle = spi_handle;
  handle->spi_mutex = spi_mutex;
  handle->to_init = 1;
  handle->contrast = contrast;
  handle->direction = direction;
}

/**
 * @brief ST7565 LCD display task
 * @details Displays buffered data
 * @param handle Display handle
 * @return HAL Status
 */
HAL_StatusTypeDef ST7565_Task(ST7565Handle* handle)
{
  if(handle == NULL)
    return HAL_ERROR;
  HAL_StatusTypeDef HAL_Status = HAL_OK;
  if(handle->to_init)
  {
    HAL_Status = ST7565_InitSequence(handle);
    handle->to_init = 0;
  }
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Display(handle);
  return HAL_Status;
}

/**
 * @brief Clears LCD screen
 * @note This doesnt touch the buffer, just clears the display RAM - might be handy
 * @param handle The handle for display
 * @return HAL Status
 */
HAL_StatusTypeDef ST7565_ClearDisplay(ST7565Handle* handle)
{
  HAL_StatusTypeDef HAL_Status = HAL_OK;
  uint8_t p, c;
  for(p = 0; p < 8; p++)
  {
    if(HAL_Status == HAL_OK)
      HAL_Status = ST7565_Command(handle, CMD_SET_PAGE | p);
    for(c = 0; c < 128; c++)
    {
      if(HAL_Status == HAL_OK)
        HAL_Status = ST7565_Command(handle, CMD_SET_COLUMN_LOWER | (c & 0xf));
      if(HAL_Status == HAL_OK)
        HAL_Status = ST7565_Command(handle, CMD_SET_COLUMN_UPPER | ((c >> 4) & 0xf));
      if(HAL_Status == HAL_OK)
        HAL_Status = ST7565_Data(handle, 0x0);
    }
    if(HAL_Status == HAL_ERROR)
      break;
  }
  return HAL_Status;
}

#if USE_OWN_FONTS == 1
/**
 * @brief Return space length in pixels
 * @param font Pointer for font info
 * @return Space lenght in pixels
 */
static uint8_t inline ST7565_GetSpaceLength(FONT_INFO* font)
{
  return font->space_width;
}

/**
 * @brief Return character length
 * @param font Pointer for font info
 * @return Character length in pixels
 */
static uint8_t ST7565_GetCharLength(char character, FONT_INFO* font)
{
  if(character == ' ')
    return ST7565_GetSpaceLength(font);
  const uint8_t char_loc_start_in_array = character - font->start_char;
  return font->pDescriptors[char_loc_start_in_array].lenght;
}

/**
 * @brief Return character width
 * @param font Pointer for font info
 * @return Character width in pixels
 */
static uint8_t inline ST7565_GetCharHeigth(FONT_INFO* font)
{
  return font->char_height;
}
#endif

/**
 * @brief Draw string to internal buffer
 * @param handle Display handle
 * @param x Starting coordinate X
 * @param line Starting line
 * @param str String to be written
 */
void ST7565_DrawString(ST7565Handle* handle, uint8_t x, uint8_t line, const char *str
#if USE_OWN_FONTS
,FONT_INFO* font
#endif
)
{
  if(handle == NULL || str == NULL || line >= (ST7565_LCDHEIGHT / 8U))
    return;
#if USE_OWN_FONTS==1
  char c;
  while (1)
  {
    c = *str++;
    if (!c)
      return;
    if(c != ' ')  /* Skip space character and just increase the text position */
    {
      ST7565_DrawChar(handle, x, line, c, font);
      x += ST7565_GetCharLength(c,font);
    }
    else
    {
      x += ST7565_GetSpaceLength(font);
    }
    if((x + ST7565_GetCharLength('_', font)) >= ST7565_LCDWIDTH)
    {
      x = 0;  /* ran out of this line */
      line += ST7565_GetCharHeigth(font);
    }
    if(line > (ST7565_LCDHEIGHT - ST7565_GetCharHeigth(font)))
      return;  /* ran out of space */
  }
#else
  char c;
  while(1)
  {
    c = *str++;
    if(!c)
      return;
    ST7565_DrawChar(handle, x, line, c);
    x += 6;  /* 6 pixels wide */
    if(x + 6 >= ST7565_LCDWIDTH)
    {
      x = 0;  /* ran out of this line */
      line++;
    }
    if(line >= (ST7565_LCDHEIGHT / 8))
      return;  /* ran out of space */
  }
#endif
}

/**
 * @brief Draw a character to the LCD screen
 * @param handle The handle for display
 * @param pos_x X pixel
 * @param pos_y Y pixel
 * @param character Character to draw
 * @param font Font to use
 */
void ST7565_DrawChar(ST7565Handle* handle, uint8_t pos_x, uint8_t pos_y, char character
#if USE_OWN_FONTS
,FONT_INFO* font
#endif
)
{
#if USE_OWN_FONTS
  const uint8_t char_loc_start_in_array = character - font->start_char;
  if(char_loc_start_in_array >= (font->end_char - font->start_char))
    return; /* Avoid array index out of bounds */
  const uint8_t char_width = font->pDescriptors[char_loc_start_in_array].lenght;
  const uint8_t character_bit_size = ((char_width - 1) / 8) + 1;
  const uint32_t char_pos_in_array = font->pDescriptors[char_loc_start_in_array].offset;
  if(char_pos_in_array >= font->bitmap_size - 1) /* TODO (Attila) currently last character from map can't be used, sometime fix it */
    return; /* Avoid array index out of bounds */
  const uint16_t char_pos_end_in_array = font->pDescriptors[char_loc_start_in_array + 1].offset -
                                   font->pDescriptors[char_loc_start_in_array].offset;
  uint8_t i = 0;
  while(i < char_pos_end_in_array)
  {
    uint32_t line = 0;
    for (uint8_t x = 0; x < character_bit_size; ++x)
    {
      line |= font->pBitmaps[char_pos_in_array + i + x] << (8 * x);
    }
    i += character_bit_size;

    for (uint8_t j = 0; j != char_width; ++j)
    {
      if(line & (1 << j))
        ST7565_SetPixel(handle, pos_x + j, pos_y, 1);
    }
    pos_y++;
  }
#else
  if(handle == NULL || character > 127U ||
     pos_x > (ST7565_LCDWIDTH - 5U) ||
     pos_y >= (ST7565_LCDHEIGHT / 8U))
    return;
  for(uint8_t i = 0; i < 5; i++)
  {
    handle->graphic_buffer[pos_x + (pos_y * 128)] = font_table[(character * 5) + i];
    pos_x++;
  }
  ST7565_UpdateBoundingBox(handle, pos_x, pos_y * 8, pos_x + 5, pos_y * 8 + 8);
#endif
}

/**
 * @brief Draw bitmap
 * @param handle Display handle
 * @param x Starting coordinate X
 * @param y Starting coordinate Y
 * @param bitmap Pointer to the bitmap array
 * @param w Bitmap width
 * @param h Bitmap height
 * @param color Color (0/1)
 */
void ST7565_DrawBitmap(ST7565Handle* handle, uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color)
{
  uint8_t i, j;
  for(j = 0; j < h; j++)
  {
    for(i = 0; i < w; i++)
    {
      if(bitmap[i + (j / 8) * w] & (1 << (j % 8)))
      {
        ST7565_SetPixelInternal(handle, x + i, y + j, color);
      }
    }
  }
  ST7565_UpdateBoundingBox(handle, x, y, x + w, y + h);
}

/**
 * @brief Draw line
 * @details Bresenham's algorithm is used
 * @param handle Display handle
 * @param x0 Starting X coordinate
 * @param y0 Starting Y coordinate
 * @param x1 Ending X coordinate
 * @param y1 Ending Y coordinate
 */
void ST7565_DrawLine(ST7565Handle* handle, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
  uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
  if(steep)
  {
    swap(x0, y0);
    swap(x1, y1);
  }
  if(x0 > x1)
  {
    swap(x0, x1);
    swap(y0, y1);
  }
  ST7565_UpdateBoundingBox(handle, x0, y0, x1, y1);
  uint8_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);
  int8_t err = dx / 2;
  int8_t ystep;
  if(y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }
  for(; x0 <= x1; x0++)
  {
    if(steep)
    {
      ST7565_SetPixelInternal(handle, y0, x0, color);
    }
    else
    {
      ST7565_SetPixelInternal(handle, x0, y0, color);
    }
    err -= dy;
    if(err < 0)
    {
      y0 += ystep;
      err += dx;
    }
  }
}

/**
 * @brief Fill a rectangle in LCD buffer
 * @param x X Position
 * @param y Y Position
 * @param w Width in pixels
 * @param h Height in pixels
 * @param color Color (0/1)
 */
void ST7565_FillRectangle(ST7565Handle* handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
  uint8_t i, j;
  for(i = x; i < x + w; i++)
  {
    for(j = y; j < y + h; j++)
    {
      ST7565_SetPixelInternal(handle, i, j, color);
    }
  }
  ST7565_UpdateBoundingBox(handle, x, y, x + w, y + h);
}

/**
 * @brief Draw a rectangle in LCD buffer
 * @param x X Position
 * @param y Y Position
 * @param w Width in pixels
 * @param h Height in pixels
 * @param color Color (0/1)
 */
void ST7565_DrawRectangle(ST7565Handle* handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
  uint8_t i;
  for(i = x; i < x + w; i++)
  {
    ST7565_SetPixelInternal(handle, i, y, color);
    ST7565_SetPixelInternal(handle, i, y + h - 1, color);
  }
  for(i = y; i < y + h; i++)
  {
    ST7565_SetPixelInternal(handle, x, i, color);
    ST7565_SetPixelInternal(handle, x + w - 1, i, color);
  }
  ST7565_UpdateBoundingBox(handle, x, y, x + w, y + h);
}

/**
 * @brief Draw a circle in LCD buffer
 * @param x0 X Position
 * @param y0 Y Position
 * @param r Radius in pixels
 * @param color Color (0/1)
 */
void ST7565_DrawCircle(ST7565Handle* handle, uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
  ST7565_UpdateBoundingBox(handle, x0 - r, y0 - r, x0 + r, y0 + r);

  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;
  ST7565_SetPixelInternal(handle, x0, y0 + r, color);
  ST7565_SetPixelInternal(handle, x0, y0 - r, color);
  ST7565_SetPixelInternal(handle, x0 + r, y0, color);
  ST7565_SetPixelInternal(handle, x0 - r, y0, color);
  while(x < y)
  {
    if(f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    ST7565_SetPixelInternal(handle, x0 + x, y0 + y, color);
    ST7565_SetPixelInternal(handle, x0 - x, y0 + y, color);
    ST7565_SetPixelInternal(handle, x0 + x, y0 - y, color);
    ST7565_SetPixelInternal(handle, x0 - x, y0 - y, color);
    ST7565_SetPixelInternal(handle, x0 + y, y0 + x, color);
    ST7565_SetPixelInternal(handle, x0 - y, y0 + x, color);
    ST7565_SetPixelInternal(handle, x0 + y, y0 - x, color);
    ST7565_SetPixelInternal(handle, x0 - y, y0 - x, color);
  }
}

/**
 * @brief Fills a circle in LCD buffer
 * @details To draw these changes to the screen, ST7565_Display function needs to be called
 * @param handle The handle for display
 * @param x0 X position
 * @param y0 Y position
 * @param color Color (0/1)
 */
void ST7565_FillCircle(ST7565Handle* handle, uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
  ST7565_UpdateBoundingBox(handle, x0 - r, y0 - r, x0 + r, y0 + r);

  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;
  uint8_t i;
  for(i = y0 - r; i <= y0 + r; i++)
  {
    ST7565_SetPixelInternal(handle, x0, i, color);
  }
  while(x < y)
  {
    if(f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    for(i = y0 - y; i <= y0 + y; i++)
    {
      ST7565_SetPixelInternal(handle, x0 + x, i, color);
      ST7565_SetPixelInternal(handle, x0 - x, i, color);
    }
    for(i = y0 - x; i <= y0 + x; i++)
    {
      ST7565_SetPixelInternal(handle, x0 + y, i, color);
      ST7565_SetPixelInternal(handle, x0 - y, i, color);
    }
  }
}

/**
 * @brief Set a pixel in LCD buffer
 * @details To draw these changes to the screen, ST7565_Display function needs to be called
 * @param handle The handle for display
 * @param x X pixel
 * @param y Y pixel
 * @param color Color (0/1)
 */
void ST7565_SetPixel(ST7565Handle* handle, uint8_t x, uint8_t y, uint8_t color)
{
  if((x >= ST7565_LCDWIDTH) || (y >= ST7565_LCDHEIGHT))
    return;
  if(color)
    handle->graphic_buffer[x + (y / 8) * 128] |= (1 << (7 - (y % 8)));
  else
    handle->graphic_buffer[x + (y / 8) * 128] &= ~(1 << (7 - (y % 8)));
  ST7565_UpdateBoundingBox(handle, x, y, x, y);
}

/**
 * @brief Returns the current state of the pixel
 * @param handle The handle for display
 * @param x X pixel
 * @param y Y pixel
 * @return Pixel state
 */
uint8_t ST7565_GetPixel(ST7565Handle* handle, uint8_t x, uint8_t y)
{
  if((x >= ST7565_LCDWIDTH) || (y >= ST7565_LCDHEIGHT))
    return 0;
  return (handle->graphic_buffer[x + (y / 8) * 128] >> (7 - (y % 8))) & 0x1;
}

/**
 * @brief DeInitializes display
 * @param handle The handle for display
 * @return HAL Status
 */
HAL_StatusTypeDef ST7565_DeInit(ST7565Handle* handle)
{
  HAL_StatusTypeDef HAL_Status;
  HAL_Status = ST7565_Command(handle, CMD_SET_ALLPTS_NORMAL);
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Command(handle, CMD_DISPLAY_OFF);
  return HAL_Status;
}

/**
 * @brief Change brightness
 * @param handle The handle for display
 * @param contrast Contrast value
 * @return HAL Status
 */
HAL_StatusTypeDef ST7565_SetBrightness(ST7565Handle* handle, uint8_t contrast)
{
  HAL_StatusTypeDef HAL_Status;
  HAL_Status = ST7565_Command(handle, CMD_SET_VOLUME_FIRST);
  if(HAL_Status == HAL_OK)
    HAL_Status = ST7565_Command(handle, CMD_SET_VOLUME_SECOND | (contrast & 0x3f));
  return HAL_Status;
}

/**
 * @brief Displays internal buffer on the LCD screen
 * @param handle The handle for display
 * @return HAL Status
 */
HAL_StatusTypeDef ST7565_Display(ST7565Handle* handle)
{
  HAL_StatusTypeDef HAL_Status = HAL_OK;
  uint8_t col, maxcol, p;
  for(p = 0; p < 8; p++)
  {
#if ENABLE_PARTIAL_UPDATE == 1
    if(handle->yUpdateMin >= ((p + 1) * 8))  /* check if this page is part of update */
    {
      continue;  /* nope, skip it! */
    }
    if(handle->yUpdateMax < p * 8)
    {
      break;
    }
#endif
    osDelay(1);
    if(HAL_Status == HAL_OK)
      HAL_Status = ST7565_Command(handle, CMD_SET_PAGE | pagemap[p]);
    osDelay(1);
#if ENABLE_PARTIAL_UPDATE == 1
    col = handle->xUpdateMin;
    maxcol = handle->xUpdateMax;
#else
    col = 0;
    maxcol = ST7565_LCDWIDTH;
#endif
    if(HAL_Status == HAL_OK)
      HAL_Status = ST7565_Command(handle, CMD_SET_COLUMN_LOWER | ((col + ST7565_STARTBYTES) & 0xf));
    osDelay(1);
    if(HAL_Status == HAL_OK)
      HAL_Status = ST7565_Command(handle, CMD_SET_COLUMN_UPPER | (((col + ST7565_STARTBYTES) >> 4) & 0x0F));
    osDelay(1);
    if(HAL_Status == HAL_OK)
      HAL_Status = ST7565_Command(handle, CMD_RMW);
    osDelay(1);
    for(; col < maxcol; col++)
    {
      if(HAL_Status == HAL_OK)
        HAL_Status = ST7565_Data(handle, handle->graphic_buffer[(128 * p) + col]);
      else
        break;
    }
    osDelay(1);
  }
#if ENABLE_PARTIAL_UPDATE == 1
  handle->xUpdateMin = ST7565_LCDWIDTH;
  handle->xUpdateMax = 0;
  handle->yUpdateMin = ST7565_LCDHEIGHT;
  handle->yUpdateMax = 0;
#endif
  return HAL_Status;
}

/**
 * @brief Clears LCD internal buffer
 * @note Display needs to be called to really clear the display
 * @param handle The handle for display
 */
void ST7565_ClearBuffer(ST7565Handle* handle)
{
  memset(handle->graphic_buffer, 0, 1024);
  ST7565_UpdateBoundingBox(handle, 0, 0, ST7565_LCDWIDTH - 1, ST7565_LCDHEIGHT - 1);
}
