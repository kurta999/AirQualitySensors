#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "app_config.h"
#include "bosch_sensor.h"
#include "cmsis_os.h"
#include "i2c_bus.h"
#include "iwdg.h"
#include "main.h"
#include "measurement_format.h"
#include "measurement_store.h"
#include "protocol.h"
#include "sensors.h"
#include "task_health.h"
#include "usart.h"
#include "usart_driver.h"
#include "usb_host.h"
#include "usbh_hid_keybd.h"
#include "user_interface.h"
#include "wifi.h"

extern USBH_HandleTypeDef hUsbHostFS;
extern ApplicationTypeDef Appli_state;

osThreadId defaultTaskHandle;
osThreadId UserInterfaceHandle;
osThreadId UsartDriverTHandle;
osThreadId WifiHandle;
osThreadId SensorsHandle;
osThreadId SensorsBme680Handle;
osMutexId lcd_mutexHandle;

static uint32_t last_measurement_sent;

/*
 * Watchdog deadlines. Each is generous relative to its task's loop period so
 * that normal jitter never trips it, while still catching a task that has
 * stopped making progress. The IWDG itself expires about 4 s after the last
 * refresh, so a missed deadline resets the device shortly after it is noticed.
 */
#define TASK_DEADLINE_USER_INTERFACE_MS 5000U   /* loop period 500 ms  */
#define TASK_DEADLINE_SENSORS_MS        10000U  /* loop period 1000 ms */
#define TASK_DEADLINE_BME680_MS         10000U  /* loop period <= 1000 ms */
#define TASK_DEADLINE_WIFI_MS           30000U  /* setup retries take 5 s each */

static void SendMessage(const uint8_t* message, size_t length)
{
    uint8_t output[257];
    if(message == NULL || length > (sizeof(output) - 2U))
    {
        return;
    }

    memcpy(output, message, length);
    const uint16_t crc = Protocol_Crc16(message, length);
    output[length] = (uint8_t)(crc & 0xFFU);
    output[length + 1U] = (uint8_t)(crc >> 8U);
    (void)HAL_UART_Transmit(
        &huart2, output, (uint16_t)(length + 2U), 100U);
}

static void SendPressedKeys(void)
{
    if(Appli_state != APPLICATION_READY)
    {
        return;
    }

    HID_KEYBD_Info_TypeDef keyboard_info;
    uint8_t have_keyboard_info = 0U;
    const uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    const HID_KEYBD_Info_TypeDef* source = USBH_HID_GetKeybdInfo(&hUsbHostFS);
    if(source != NULL)
    {
        keyboard_info = *source;
        have_keyboard_info = 1U;
    }
    if(interrupt_state == 0U)
    {
        __enable_irq();
    }

    if(have_keyboard_info != 0U)
    {
        SendMessage((const uint8_t*)&keyboard_info, sizeof(keyboard_info));
    }
}

static void SendMeasurements(void)
{
    const uint32_t now = osKernelSysTick();
    if((now - last_measurement_sent) < APP_MEASUREMENT_SEND_INTERVAL_MS)
    {
        return;
    }

    MeasurementSnapshot measurement;
    char output[MEASUREMENT_FORMAT_MAX_LEN];
    Measurements_GetSnapshot(&measurement);
    const int length = Measurements_Format(output, sizeof(output), &measurement);
    if(length > 0)
    {
        (void)HAL_UART_Transmit(
            &huart2, (uint8_t*)output, (uint16_t)length, 100U);
    }
    last_measurement_sent = now;
}

static void StartDefaultTask(void const* argument)
{
    (void)argument;
    MX_USB_HOST_Init();
    static const uint8_t reset_message[] = "reset";
    SendMessage(reset_message, sizeof(reset_message) - 1U);

    for(;;)
    {
        SendPressedKeys();
        SendMeasurements();

        /* Refresh the watchdog only while every essential task is still
         * making progress. Refreshing unconditionally from here meant the
         * device stayed alive with all of its real work wedged. */
        if(hiwdg.Instance != NULL && TaskHealth_AllAlive(osKernelSysTick()) != 0U)
        {
            (void)HAL_IWDG_Refresh(&hiwdg);
        }
        osDelay(10U);
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, signed char* task_name)
{
    (void)task;
    (void)task_name;
    Error_Handler();
}

void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}

void MX_FREERTOS_Init(void)
{
    osMutexDef(lcd_mutex);
    lcd_mutexHandle = osMutexCreate(osMutex(lcd_mutex));
    if(lcd_mutexHandle == NULL)
    {
        Error_Handler();
    }

    I2cBus_Init();

    /* Registering before the scheduler starts means a task that never reaches
     * its first loop iteration is caught by the watchdog as well. */
    const uint32_t now = osKernelSysTick();
    TaskHealth_Reset();
    TaskHealth_Register(
        TASK_HEALTH_USER_INTERFACE, TASK_DEADLINE_USER_INTERFACE_MS, now);
    TaskHealth_Register(TASK_HEALTH_SENSORS, TASK_DEADLINE_SENSORS_MS, now);
    TaskHealth_Register(TASK_HEALTH_BME680, TASK_DEADLINE_BME680_MS, now);
#if APP_WIFI_ENABLED
    TaskHealth_Register(TASK_HEALTH_WIFI, TASK_DEADLINE_WIFI_MS, now);
#endif

    osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0U, 384U);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    osThreadDef(UserInterface, UserInterfaceTask, osPriorityNormal, 0U, 384U);
    UserInterfaceHandle = osThreadCreate(osThread(UserInterface), NULL);

#if APP_WIFI_ENABLED
    osThreadDef(UsartDriverT, UsartDriver_Task, osPriorityHigh, 0U, 256U);
    UsartDriverTHandle = osThreadCreate(osThread(UsartDriverT), NULL);

    osThreadDef(Wifi, WifiTask, osPriorityNormal, 0U, 512U);
    WifiHandle = osThreadCreate(osThread(Wifi), NULL);
#endif

    osThreadDef(Sensors, SensorsTask, osPriorityNormal, 0U, 512U);
    SensorsHandle = osThreadCreate(osThread(Sensors), NULL);

    osThreadDef(SensorsBme680, SensorsBme680Task, osPriorityBelowNormal, 0U, 768U);
    SensorsBme680Handle = osThreadCreate(osThread(SensorsBme680), NULL);

    if(defaultTaskHandle == NULL || UserInterfaceHandle == NULL ||
       SensorsHandle == NULL || SensorsBme680Handle == NULL)
    {
        Error_Handler();
    }
#if APP_WIFI_ENABLED
    if(UsartDriverTHandle == NULL || WifiHandle == NULL)
    {
        Error_Handler();
    }
#endif
}
