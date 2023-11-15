/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "usart.h"

#include <stdio.h>
#include <string.h>

#include "WifiController/WifiController.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_HUART (&huart1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void handleButtonAction(const char *args);
void WifiSetupTask(void *pvParameters);
void WifiReceiveTask(void *pvParameters);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void handleButtonAction(const char *args)
{
    // Use the default GUI of the Wi-F controller which provides simple buttons for moving the motors
    // of the robotarm.
    char buffer[7] = {'\0'};
    strcat(buffer, "- ");

    const char *arg_text = "btn=";
    const size_t arg_text_length = 4;

    char *p = strstr(args, arg_text);
    if ((p == NULL) || (strlen(p) < (arg_text_length + 2)))
    {
        strncat(buffer, "xx", 2 + 1);
    }
    else
    {
        strncat(buffer, p + arg_text_length, 2 + 1);
    }

    strcat(buffer, "\r\n");

    HAL_UART_Transmit(USB_HUART, (uint8_t *)buffer, strlen(buffer), 200);
}

WifiController_ActionList_t actionList;

void WifiSetupTask(void *pvParameters)
{
    const char *ap_ssid = "indact_wific";
    const char *ap_password = "pirosalma";

    // 1.) Define actions
    WifiController_ActionList_Init(&actionList);
    WifiController_ActionList_Add(&actionList, "/button", handleButtonAction);

    // 2.) Call WifiController initialization function
    configASSERT(WifiController_WifiController_Init(&actionList) == WifiController_ErrorCode_NONE);

    // 3.) Start the receiver task
    xTaskCreate(WifiReceiveTask, "wifi_receive", configMINIMAL_STACK_SIZE * 4, NULL, configMAX_PRIORITIES / 4 * 3, NULL);

    // 3.) Set parameters for the Wi-Fi module
    configASSERT(WifiController_WifiController_ResetModule() == WifiController_ErrorCode_NONE);
    // Wait for the module to reset
    vTaskDelay(pdMS_TO_TICKS(3 * 1000));
    configASSERT(WifiController_WifiController_SetSsid(ap_ssid) == WifiController_ErrorCode_NONE);
    configASSERT(WifiController_WifiController_SetPassword(ap_password) == WifiController_ErrorCode_NONE);

    // 4.) Start the access point or station mode
    configASSERT(WifiController_WifiController_BeginAccessPoint(10 * 1000) == WifiController_ErrorCode_NONE);

    vTaskDelete(NULL);
}

void WifiReceiveTask(void *pvParameters)
{
    while (1)
    {
        WifiController_ErrorCode_t e = WifiController_WifiController_Receive();
        if (e != WifiController_ErrorCode_NONE)
        {
            // TODO: Handle error.
        }
    }
}

/* USER CODE END Application */
