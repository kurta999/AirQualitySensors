#include "app_hardware.h"

#include "app_config.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

static void output_pins(GPIO_TypeDef* port, uint16_t pins, GPIO_PinState level)
{
    if(pins == 0U)
    {
        return;
    }
    GPIO_InitTypeDef gpio = {0};
    HAL_GPIO_WritePin(port, pins, level);
    gpio.Pin = pins;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gpio);
}

void App_HardwareInit(void)
{
    /* Only configure pins for enabled devices. PB7/PB14 are Nucleo LEDs
     * used by startup and Error_Handler, including the all-disabled build. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    output_pins(GPIOB, ULED_2_Pin | ULED_3_Pin, GPIO_PIN_RESET);
#if APP_HARDWARE_ENABLED || APP_WIFI_ENABLED
    __HAL_RCC_GPIOC_CLK_ENABLE();
    output_pins(GPIOC,
        (APP_HARDWARE_ENABLED ? ULED_1_Pin : 0U) |
        (APP_WIFI_ENABLED ? WIFI_EN_Pin : 0U), GPIO_PIN_RESET);
#endif
#if APP_HARDWARE_ENABLED
    __HAL_RCC_GPIOD_CLK_ENABLE();
    output_pins(GPIOD, LED_1_Pin | LED_2_Pin | LED_3_Pin | LED_4_Pin, GPIO_PIN_RESET);
#endif
#if APP_LCD_ENABLED
    output_pins(GPIOB, LCD_A0_Pin | LCD_RES_Pin | LCD_CS_Pin, GPIO_PIN_RESET);
    MX_SPI1_Init();
    MX_TIM3_Init();
#endif
#if APP_BME680_ENABLED
    __HAL_RCC_GPIOF_CLK_ENABLE();
    output_pins(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    MX_SPI2_Init();
#endif
#if APP_USB_HOST_ENABLED
    __HAL_RCC_GPIOG_CLK_ENABLE();
    HAL_PWREx_EnableVddIO2();
    output_pins(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_RESET);
#endif
#if APP_ADC_ENABLED || APP_WIFI_ENABLED
    MX_DMA_Init();
#endif
#if APP_HOST_UART_ENABLED
    MX_USART2_UART_Init();
#endif
#if APP_WATCHDOG_ENABLED
    MX_IWDG_Init();
#endif
#if APP_WIFI_ENABLED
    MX_USART3_UART_Init();
    MX_TIM16_Init();
#endif
#if APP_ADC_ENABLED
    MX_ADC1_Init();
#endif
#if APP_I2C1_ENABLED
    MX_I2C1_Init();
#endif
#if APP_I2C3_ENABLED
    MX_I2C3_Init();
#endif
#if APP_HPMA_ENABLED
    MX_UART4_Init();
#endif
}
