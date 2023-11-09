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
osThreadId indicator_blinkingHandle;
osThreadId control_viaGPIOHandle;
osThreadId demo_moveHandle;
osThreadId inverseGeometry_demoHandle;
osMessageQId demo_move_positionsHandle;
osMessageQId inverseGeometry_demo_move_positionsHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void indicator_blinking_f(void const * argument);
void control_viaGPIO_f(void const * argument);
void demo_move_f(void const * argument);
void inverseGeometry_demo_f(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
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
}
/* USER CODE END 5 */

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

  /* Create the queue(s) */
  /* definition and creation of demo_move_positions */
  osMessageQDef(demo_move_positions, 15, s_GEO_ToolPosition_Cylinder);
  demo_move_positionsHandle = osMessageCreate(osMessageQ(demo_move_positions), NULL);

  /* definition and creation of inverseGeometry_demo_move_positions */
  osMessageQDef(inverseGeometry_demo_move_positions, 16, s_GEO_ToolPosition_Descartes);
  inverseGeometry_demo_move_positionsHandle = osMessageCreate(osMessageQ(inverseGeometry_demo_move_positions), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of indicator_blinking */
  osThreadDef(indicator_blinking, indicator_blinking_f, osPriorityRealtime, 0, 256);
  indicator_blinkingHandle = osThreadCreate(osThread(indicator_blinking), NULL);

  /* definition and creation of control_viaGPIO */
  osThreadDef(control_viaGPIO, control_viaGPIO_f, osPriorityRealtime, 0, 256);
  control_viaGPIOHandle = osThreadCreate(osThread(control_viaGPIO), NULL);

  /* definition and creation of demo_move */
  osThreadDef(demo_move, demo_move_f, osPriorityRealtime, 0, 256);
  demo_moveHandle = osThreadCreate(osThread(demo_move), NULL);

  /* definition and creation of inverseGeometry_demo */
  osThreadDef(inverseGeometry_demo, inverseGeometry_demo_f, osPriorityNormal, 0, 256);
  inverseGeometry_demoHandle = osThreadCreate(osThread(inverseGeometry_demo), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_indicator_blinking_f */
/**
  * @brief  Function implementing the indicator_blinking thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_indicator_blinking_f */
void indicator_blinking_f(void const * argument)
{
  /* USER CODE BEGIN indicator_blinking_f */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END indicator_blinking_f */
}

/* USER CODE BEGIN Header_control_viaGPIO_f */
/**
* @brief Function implementing the control_viaGPIO thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_control_viaGPIO_f */
void control_viaGPIO_f(void const * argument)
{
  /* USER CODE BEGIN control_viaGPIO_f */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END control_viaGPIO_f */
}

/* USER CODE BEGIN Header_demo_move_f */
/**
* @brief Function implementing the demo_move thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_demo_move_f */
void demo_move_f(void const * argument)
{
  /* USER CODE BEGIN demo_move_f */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END demo_move_f */
}

/* USER CODE BEGIN Header_inverseGeometry_demo_f */
/**
* @brief Function implementing the inverseGeometry_demo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_inverseGeometry_demo_f */
void inverseGeometry_demo_f(void const * argument)
{
  /* USER CODE BEGIN inverseGeometry_demo_f */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END inverseGeometry_demo_f */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
