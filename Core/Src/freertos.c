/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "device_specific.h"
#include "usbh_def.h"
#include "usb_host.h"
#include "usbh_hid_keybd.h"
extern USBH_HandleTypeDef hUsbHostFS;
//extern ApplicationTypeDef Appli_state;
extern UART_HandleTypeDef huart2;
extern ApplicationTypeDef Appli_state;
extern IWDG_HandleTypeDef hiwdg;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId UserInterfaceHandle;
osThreadId UsartDriverTHandle;
osThreadId WifiHandle;
osThreadId SensorsHandle;
osThreadId SensorsBme680Handle;
osSemaphoreId lcd_mutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void UserInterfaceTask(void const * argument);
void UsartDriver_Task(void const * argument);
void WifiTask(void const * argument);
void SensorsTask(void const * argument);
void SensorsBme680Task(void const * argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	Error_Handler();
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	Error_Handler();
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of lcd_mutex */
  osSemaphoreDef(lcd_mutex);
  lcd_mutexHandle = osSemaphoreCreate(osSemaphore(lcd_mutex), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UserInterface */
  osThreadDef(UserInterface, UserInterfaceTask, osPriorityNormal, 0, 384);
  UserInterfaceHandle = osThreadCreate(osThread(UserInterface), NULL);

  /* definition and creation of UsartDriverT */
  osThreadDef(UsartDriverT, UsartDriver_Task, osPriorityHigh, 0, 128);
  UsartDriverTHandle = osThreadCreate(osThread(UsartDriverT), NULL);

  /* definition and creation of Wifi */
  osThreadDef(Wifi, WifiTask, osPriorityNormal, 0, 512);
  WifiHandle = osThreadCreate(osThread(Wifi), NULL);

  /* definition and creation of Sensors */
  osThreadDef(Sensors, SensorsTask, osPriorityNormal, 0, 512);
  SensorsHandle = osThreadCreate(osThread(Sensors), NULL);

  /* definition and creation of SensorsBme680 */
  osThreadDef(SensorsBme680, SensorsBme680Task, osPriorityIdle, 0, 256);
  SensorsBme680Handle = osThreadCreate(osThread(SensorsBme680), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for(i = 0; i < 8; ++i)
  {
    if(crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }
  return crc;
}

/**
 * @brief Calculate CRC-16
 * @param data_in Data to calculate CRC from
 * @param data_length Length of the data
 */
uint16_t crc16_calculate(uint8_t* data_in, uint16_t data_length)
{
  uint16_t crc_calc = 0xFFFF;
  for(uint8_t* p = data_in; p < (data_in + data_length); p++)
  {
    crc_calc = crc16_update(crc_calc, *p);
  }
  return crc_calc;
}

static void SendMessage(uint8_t* msg, uint8_t len)
{
  uint8_t output_buffer[255];
  memcpy(output_buffer, msg, len);
  uint16_t crc16 = crc16_calculate(msg, len);
  memcpy(&output_buffer[len], &crc16, sizeof(crc16));
  HAL_UART_Transmit(&huart2, output_buffer, len + 2, 100);
}

void SendInitMessage(void)
{
  SendMessage((uint8_t*)"reset", 5);
}

void SendPressedKeys(void)
{
  if(Appli_state != APPLICATION_READY)
    return;

  __disable_irq();
  DEBUG_VOLATILE HID_KEYBD_Info_TypeDef* info = USBH_HID_GetKeybdInfo(&hUsbHostFS);
  if(info)
  {
    SendMessage((uint8_t*)info, sizeof(HID_KEYBD_Info_TypeDef));
  }
  __enable_irq();
}

extern uint8_t rx_circular_buffer[512];
extern uint8_t uart_byte;
extern uint16_t uart_recv_cnt;


uint32_t last_particle_meas = 0;

extern UART_HandleTypeDef huart4;
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */

  SendInitMessage();
/*
  uint8_t data[] = {0x68, 0x01, 0x20, 0x77}; // Disable auto send
  HAL_UART_Transmit(&huart4, data, sizeof(data), 100);

  osDelay(500);
  uint8_t data2[] = {0x68, 0x01, 0x01, 0x96}; // Start Particle Measurement
  HAL_UART_Transmit(&huart4, data2, sizeof(data2), 100);
*/
  /* Infinite loop */
  for(;;)
  {
    SendPressedKeys();
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(10);
#if 0
    if(osKernelSysTick() - last_particle_meas > 10000)
    {
      uint8_t data3[] = {0x68, 0x01, 0x04, 0x93}; // Read Particle Measuring Results
      HAL_UART_Transmit(&huart4, data3, sizeof(data3), 100);
      HAL_UART_Receive_IT(&huart4, &uart_byte, 1);
      osDelay(5000);
      last_particle_meas = osKernelSysTick();
    }
    if(rx_circular_buffer[0] == 0x40 && rx_circular_buffer[1] == 0x05 && rx_circular_buffer[2] == 0x04)
    {
      pm25 = rx_circular_buffer[3] * 256 + rx_circular_buffer[4];
      pm10 = rx_circular_buffer[3] * 256 + rx_circular_buffer[4];
      memset(rx_circular_buffer, 0, sizeof(rx_circular_buffer));
      uart_recv_cnt = 0;
    }
#endif
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_UserInterfaceTask */
/**
* @brief Function implementing the UserInterface thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UserInterfaceTask */
__weak void UserInterfaceTask(void const * argument)
{
  /* USER CODE BEGIN UserInterfaceTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UserInterfaceTask */
}

/* USER CODE BEGIN Header_UsartDriver_Task */
/**
* @brief Function implementing the UsartDriverT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UsartDriver_Task */
__weak void UsartDriver_Task(void const * argument)
{
  /* USER CODE BEGIN UsartDriver_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UsartDriver_Task */
}

/* USER CODE BEGIN Header_WifiTask */
/**
* @brief Function implementing the Wifi thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WifiTask */
__weak void WifiTask(void const * argument)
{
  /* USER CODE BEGIN WifiTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END WifiTask */
}

/* USER CODE BEGIN Header_SensorsTask */
/**
* @brief Function implementing the Sensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorsTask */
__weak void SensorsTask(void const * argument)
{
  /* USER CODE BEGIN SensorsTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SensorsTask */
}

/* USER CODE BEGIN Header_SensorsBme680Task */
/**
* @brief Function implementing the SensorsBme680 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorsBme680Task */
__weak void SensorsBme680Task(void const * argument)
{
  /* USER CODE BEGIN SensorsBme680Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SensorsBme680Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
