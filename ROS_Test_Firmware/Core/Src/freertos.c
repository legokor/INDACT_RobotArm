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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "microros_task.h"
#include "motion_task.h"
#include "queue_msg_types.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Queue Definitions */
xQueueHandle CommandQueueHandle;
xQueueHandle StateQueueHandle;


/* Definitions for MicrorosTask */
osThreadId_t MicrorosTaskHandle;
uint32_t MicrorosTaksBuffer[ 3000 ];
osStaticThreadDef_t MicrorosTaksControlBlock;
const osThreadAttr_t MicrorosTask_attributes = {
  .name = "MicrorosTask",
  .cb_mem = &MicrorosTaksControlBlock,
  .cb_size = sizeof(MicrorosTaksControlBlock),
  .stack_mem = &MicrorosTaksBuffer[0],
  .stack_size = sizeof(MicrorosTaksBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for MotionTask */
osThreadId_t MotionControlTaskHandle;
uint32_t MotionTaskBuffer[ 128 ];
osStaticThreadDef_t MotionTaskControlBlock;
const osThreadAttr_t MotionTask_attributes = {
  .name = "MotionTask",
  .cb_mem = &MotionTaskControlBlock,
  .cb_size = sizeof(MotionTaskControlBlock),
  .stack_mem = &MotionTaskBuffer[0],
  .stack_size = sizeof(MotionTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
	CommandQueueHandle = xQueueCreate(5, sizeof(CommandMsg));
	StateQueueHandle = xQueueCreate(5, sizeof(StateMsg));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  MicrorosTaskHandle = osThreadNew(MicrorosTaskFunction, NULL, &MicrorosTask_attributes);

  MotionControlTaskHandle = osThreadNew(MotionTaskFunction, NULL, &MotionTask_attributes);

  vTaskDelete(defaultTaskHandle);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

