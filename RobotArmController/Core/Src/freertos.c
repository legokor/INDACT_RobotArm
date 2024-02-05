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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "event_groups.h"
#include "queue.h"
#include "semphr.h"

#include "logger.h"
#include "rtos_priorities.h"
#include "stepper_motor.h"
#include "tim.h"
#include "usart.h"

#include "MotorControl/KAR_MC_handler.h"
#include "WifiController/WifiController.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

typedef int32_t MC_Step_t;

typedef struct PositionCyl
{
    MC_Step_t r;
    MC_Step_t phi;
    MC_Step_t z;
} PositionCyl_t;

typedef enum AppState
{
    AppState_Error,
    AppState_Idle,
    AppState_DemoMoveControl,
    AppState_GpioControl,
    AppState_WifiControl
} AppState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Task settings
#define INDICATOR_BLINKING_TASK_STACK_SIZE 128
#define INDICATOR_BLINKING_TASK_PRIORITY TASK_PRIORITY_LOWEST
#define WIFI_RECEIVE_TASK_STACK_SIZE 512
#define WIFI_RECEIVE_TASK_PRIORITY TASK_PRIORITY_ABOVE_NORMAL
#define DEMO_MOVE_CONTROL_TASK_STACK_SIZE 512
#define DEMO_MOVE_CONTROL_TASK_PRIORITY TASK_PRIORITY_NORMAL
#define GPIO_CONTROL_TASK_STACK_SIZE 512
#define GPIO_CONTROL_TASK_PRIORITY TASK_PRIORITY_NORMAL
#define WIFI_CONTROL_TASK_STACK_SIZE 512
#define WIFI_CONTROL_TASK_PRIORITY TASK_PRIORITY_NORMAL

// Queue settings
#define NEXT_POSITION_QUEUE_LENGTH 64
#define NEXT_POSITION_QUEUE_ITEM_SIZE sizeof(PositionCyl_t)

// Event group settings
#define STATE_DEMO_MOVE_CONTROL_BIT (1 << 0)
#define STATE_GPIO_CONTROL_BIT (1 << 1)
#define STATE_WIFI_CONTROL_BIT (1 << 2)

#define TASK_STARTED_INDICATOR_BLINKING_TASK_BIT (1 << 0)
#define TASK_STARTED_WIFI_RECEIVE_TASK_BIT (1 << 1)
#define TASK_STARTED_DEMO_MOVE_CONTROL_TASK_BIT (1 << 2)
#define TASK_STARTED_GPIO_CONTROL_TASK_BIT (1 << 3)
#define TASK_STARTED_WIFI_CONTROL_TASK_BIT (1 << 4)

#define USB_HUART (&huart1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ABS(X) (((X) < 0) ? (-(X)) : (X))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// Task variables
static StaticTask_t indicatorBlinkingTaskBuffer;
static StackType_t indicatorBlinkingTaskStack[INDICATOR_BLINKING_TASK_STACK_SIZE];
TaskHandle_t indicatorBlinkingTaskHandle = NULL;
static StaticTask_t wifiReceiveTaskBuffer;
static StackType_t wifiReceiveTaskStack[WIFI_RECEIVE_TASK_STACK_SIZE];
TaskHandle_t wifiReceiveTaskHandle = NULL;
static StaticTask_t demoMoveControlTaskBuffer;
static StackType_t demoMoveControlTaskStack[DEMO_MOVE_CONTROL_TASK_STACK_SIZE];
TaskHandle_t demoMoveControlTaskHandle = NULL;
static StaticTask_t gpioControlTaskBuffer;
static StackType_t gpioControlTaskStack[GPIO_CONTROL_TASK_STACK_SIZE];
TaskHandle_t gpioControlTaskHandle = NULL;
static StaticTask_t wifiControlTaskBuffer;
static StackType_t wifiControlTaskStack[WIFI_CONTROL_TASK_STACK_SIZE];
TaskHandle_t wifiControlTaskHandle = NULL;
TaskHandle_t setupTaskHandle = NULL;

// Semaphores and mutexes
StaticSemaphore_t controlMutexBuffer;
SemaphoreHandle_t controlMutexHandle = NULL;

// Queue variables
static StaticQueue_t nextPositionQueueBuffer;
static uint8_t nextPositionQueueStorageArea[NEXT_POSITION_QUEUE_LENGTH * NEXT_POSITION_QUEUE_ITEM_SIZE];
QueueHandle_t nextPositionQueueHandle = NULL;

// Event group variables
static StaticEventGroup_t stateEventGroupBuffer;
EventGroupHandle_t stateEventGroupHandle = NULL;
static StaticEventGroup_t taskStartedEventGroupBuffer;
EventGroupHandle_t taskStartedEventGroupHandle = NULL;

// Other variables
e_MC_ErrorCode_t mcErrorCode = e_MC_ErrorCode_OK;
bool homingState = false;
volatile AppState_t appState = AppState_Idle;

WifiController_ActionList_t wifiActionList;

/* USER CODE END Variables */
/* Definitions for logState */
osThreadId_t logStateHandle;
uint32_t logStateTaskBuffer[ 512 ];
osStaticThreadDef_t logStateTaskControlBlock;
const osThreadAttr_t logState_attributes = {
  .name = "logState",
  .cb_mem = &logStateTaskControlBlock,
  .cb_size = sizeof(logStateTaskControlBlock),
  .stack_mem = &logStateTaskBuffer[0],
  .stack_size = sizeof(logStateTaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void setupTask(void *pvParameters);
void indicatorBlinkingTask(void *pvParameters);
void wifiReceiveTask(void *pvParameters);
void demoMoveControlTask(void *pvParameters);
void gpioControlTask(void *pvParameters);
void wifiControlTask(void *pvParameters);

static void changeAppStateFromISR(BaseType_t *xHigherPriorityTaskWoken);
static bool debounce(uint32_t *last_tick, uint32_t tick_count);
static void handleButtonAction(const char *args);
static void homingSequence();
static void moveToPosition(const PositionCyl_t *position);
static void sendNewPosition(const char *btn);
static void setupWifi();
/* USER CODE END FunctionPrototypes */

void logStateTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    controlMutexHandle = xSemaphoreCreateMutexStatic(&controlMutexBuffer);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* Create the queue(s) */
    nextPositionQueueHandle = xQueueCreateStatic(
            NEXT_POSITION_QUEUE_LENGTH,
            NEXT_POSITION_QUEUE_ITEM_SIZE,
            nextPositionQueueStorageArea,
            &nextPositionQueueBuffer);
    configASSERT(nextPositionQueueHandle != NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of logState */
  logStateHandle = osThreadNew(logStateTask, NULL, &logState_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* Create the task(s) */
    indicatorBlinkingTaskHandle = xTaskCreateStatic(
            indicatorBlinkingTask,
            "IndicatorBlinking",
            INDICATOR_BLINKING_TASK_STACK_SIZE,
            NULL,
            INDICATOR_BLINKING_TASK_PRIORITY,
            indicatorBlinkingTaskStack,
            &indicatorBlinkingTaskBuffer);
    configASSERT(indicatorBlinkingTaskHandle != NULL);

    wifiReceiveTaskHandle = xTaskCreateStatic(
            wifiReceiveTask,
            "WifiReceive",
            WIFI_RECEIVE_TASK_STACK_SIZE,
            NULL,
            WIFI_RECEIVE_TASK_PRIORITY,
            wifiReceiveTaskStack,
            &wifiReceiveTaskBuffer);
    configASSERT(wifiReceiveTaskHandle != NULL);

    demoMoveControlTaskHandle = xTaskCreateStatic(
            demoMoveControlTask,
            "DemoMoveControl",
            DEMO_MOVE_CONTROL_TASK_STACK_SIZE,
            NULL,
            DEMO_MOVE_CONTROL_TASK_PRIORITY,
            demoMoveControlTaskStack,
            &demoMoveControlTaskBuffer);
    configASSERT(demoMoveControlTaskHandle != NULL);

    gpioControlTaskHandle = xTaskCreateStatic(
            gpioControlTask,
            "GpioControl",
            GPIO_CONTROL_TASK_STACK_SIZE,
            NULL,
            GPIO_CONTROL_TASK_PRIORITY,
            gpioControlTaskStack,
            &gpioControlTaskBuffer);
    configASSERT(gpioControlTaskHandle != NULL);

    wifiControlTaskHandle = xTaskCreateStatic(
            wifiControlTask,
            "WifiControl",
            WIFI_CONTROL_TASK_STACK_SIZE,
            NULL,
            WIFI_CONTROL_TASK_PRIORITY,
            wifiControlTaskStack,
            &wifiControlTaskBuffer);
    configASSERT(wifiControlTaskHandle != NULL);

    configASSERT(xTaskCreate(
            setupTask,
            "Setup",
            configMINIMAL_STACK_SIZE,
            NULL,
            TASK_PRIORITY_NORMAL,
            &setupTaskHandle) == pdPASS);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    stateEventGroupHandle = xEventGroupCreateStatic(&stateEventGroupBuffer);
    configASSERT(stateEventGroupHandle != NULL);
    taskStartedEventGroupHandle = xEventGroupCreateStatic(&taskStartedEventGroupBuffer);
    configASSERT(taskStartedEventGroupHandle != NULL);
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_logStateTask */
/**
  * @brief  Function implementing the logState thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_logStateTask */
void logStateTask(void *argument)
{
    /* USER CODE BEGIN logStateTask */
    /* Infinite loop */
    while (1)
    {
        PositionCyl_t cp = { -1, -1, -1 };
        vPortEnterCritical();
        cp.r = as_stepper_motors[KAR_MC_MOTORID_R].currPos;
        cp.phi = as_stepper_motors[KAR_MC_MOTORID_PHI].currPos;
        cp.z = as_stepper_motors[KAR_MC_MOTORID_Z].currPos;
        vPortExitCritical();
        Logger_LogPrintf(
                LogLevel_Info,
                "position: (%ld, %ld, %ld), state: %d",
                cp.r, cp.phi, cp.z,
                appState);

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    /* USER CODE END logStateTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
 *===================================================================*
 * Function name: homing_sequence
 *-------------------------------------------------------------------
 * Description:
 * This function is responsible the to do the first homing sequence. It drives all the motors towards negative direction
 * to the point where all of the axis are at the zero position. Here the current position values get the zero initial value.
 * After the sequence the timer interrupts can follow the movement of the arm.
 * After power reset homing must run again.
 *
 * INPUT: none
 *
 * OUTPUT: none
 *-------------------------------------------------------------------
 */
static void homingSequence()
{
    mcErrorCode = u8_MC_StartAllMotor_f(as_stepper_motors, KAR_MC_DIR_NEGATIVE);

    if (mcErrorCode == e_MC_ErrorCode_OK)
    {
        bool r = false, phi = false, z = false;

        // Waiting until all of the motors have reached their limit
        while (!(r && phi && z))
        {
            // R axis check
            if ((!r) && (GPIO_PIN_SET == as_limit_switches[KAR_MC_MOTORID_R].null_point))
            {
                v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_R);
                r = true;
            }

            // Phi axis check
            if ((!phi) && (GPIO_PIN_SET == as_limit_switches[KAR_MC_MOTORID_PHI].null_point))
            {
                v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_PHI);
                phi = true;
            }

            // Z axis check
            if ((!z) && (GPIO_PIN_SET == as_limit_switches[KAR_MC_MOTORID_Z].null_point))
            {
                v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_Z);
                z = true;
            }

            vTaskDelay(10);
        }

        homingState = true;
        Logger_LogPrintf(LogLevel_Info, "Homing complete.");
    }
    else
    {
        Logger_LogPrintf(LogLevel_Error, "Homig error!");
        Error_Handler();
    }
}

static void changeAppStateFromISR(BaseType_t *xHigherPriorityTaskWoken)
{
    *xHigherPriorityTaskWoken = pdFALSE;

    // Change the app state and wake up the appropriate task.
    switch (appState)
    {
    case AppState_Idle:
        appState = AppState_DemoMoveControl;
        xEventGroupSetBitsFromISR(
                stateEventGroupHandle,
                STATE_DEMO_MOVE_CONTROL_BIT,
                xHigherPriorityTaskWoken);
        break;
    case AppState_DemoMoveControl:
        appState = AppState_GpioControl;
        xEventGroupSetBitsFromISR(
                stateEventGroupHandle,
                STATE_GPIO_CONTROL_BIT,
                xHigherPriorityTaskWoken);
        break;
    case AppState_GpioControl:
        appState = AppState_WifiControl;
        xEventGroupSetBitsFromISR(
                stateEventGroupHandle,
                STATE_WIFI_CONTROL_BIT,
                xHigherPriorityTaskWoken);
        break;
    case AppState_WifiControl:
        appState = AppState_DemoMoveControl;
        xEventGroupSetBitsFromISR(
                stateEventGroupHandle,
                STATE_DEMO_MOVE_CONTROL_BIT,
                xHigherPriorityTaskWoken);
        break;
    default:
        appState = AppState_Error;
        break;
    }
}

static bool debounce(uint32_t *last_tick, uint32_t tick_count)
{
    uint32_t current_tick = HAL_GetTick();
    if (current_tick - *last_tick > tick_count)
    {
        *last_tick = current_tick;
        return true;
    }
    return false;
}

static inline bool check_move_finished(const s_MC_StepperMotor *sm, const s_GEO_LimitSwitch *ls)
{
    // {next position reached} || {limit max reached} || {limit min reached}
    return ((sm->dir == KAR_MC_DIR_POSITIVE) ? (sm->currPos >= sm->nextPos) : (sm->currPos <= sm->nextPos))
            || (ls->max_point && (sm->dir == KAR_MC_DIR_POSITIVE))
            || (ls->null_point && (sm->dir == KAR_MC_DIR_NEGATIVE));
}

static void moveToPosition(const PositionCyl_t *position)
{
    vPortEnterCritical();
    as_stepper_motors[KAR_MC_MOTORID_R].nextPos = position->r;
    as_stepper_motors[KAR_MC_MOTORID_PHI].nextPos = position->phi;
    as_stepper_motors[KAR_MC_MOTORID_Z].nextPos = position->z;
    vPortExitCritical();

    u8_MC_setAllMotorDir_TowardsDesiredPos_f(as_stepper_motors);

    u8_MC_StartMotor_f(as_stepper_motors, KAR_MC_MOTORID_R, as_stepper_motors[KAR_MC_MOTORID_R].dir);
    u8_MC_StartMotor_f(as_stepper_motors, KAR_MC_MOTORID_PHI, as_stepper_motors[KAR_MC_MOTORID_PHI].dir);
    u8_MC_StartMotor_f(as_stepper_motors, KAR_MC_MOTORID_Z, as_stepper_motors[KAR_MC_MOTORID_Z].dir);

    bool r_finished = false;
    bool phi_finished = false;
    bool z_finished = false;

    /* Wait for tool to reach next position */
    while (!(r_finished && phi_finished && z_finished))
    {
        if ((!r_finished) && check_move_finished(&(as_stepper_motors[KAR_MC_MOTORID_R]), &(as_limit_switches[KAR_MC_MOTORID_R])))
        {
            v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_R);
            r_finished = true;
        }

        if ((!phi_finished) && check_move_finished(&(as_stepper_motors[KAR_MC_MOTORID_PHI]), &(as_limit_switches[KAR_MC_MOTORID_PHI])))
        {
            v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_PHI);
            phi_finished = true;
        }

        if ((!z_finished) && check_move_finished(&(as_stepper_motors[KAR_MC_MOTORID_Z]), &(as_limit_switches[KAR_MC_MOTORID_Z])))
        {
            v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_Z);
            z_finished = true;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void sendNewPosition(const char *btn)
{
    PositionCyl_t position;
    // Disable OS and interrupts while creating local copy of global data.
    vPortEnterCritical();
    position.r = as_stepper_motors[KAR_MC_MOTORID_R].currPos;
    position.phi = as_stepper_motors[KAR_MC_MOTORID_PHI].currPos;
    position.z = as_stepper_motors[KAR_MC_MOTORID_Z].currPos;
    vPortExitCritical();

    const MC_Step_t d = 10;

    if (strncmp(btn, "rp", 2))
    {
        position.r += d;
        if (position.r > U32_KAR_MC_MAXPOS_R)
        {
            position.r = U32_KAR_MC_MAXPOS_R;
        }
    }
    else if (strncmp(btn, "rn", 2))
    {
        position.r -= d;
        if (position.r < 0)
        {
            position.r = 0;
        }
    }
    else if (strncmp(btn, "fp", 2))
    {
        position.phi += d;
        if (position.phi > U32_KAR_MC_MAXPOS_FI)
        {
            position.phi = U32_KAR_MC_MAXPOS_FI;
        }
    }
    else if (strncmp(btn, "fn", 2))
    {
        position.phi -= d;
        if (position.phi < 0)
        {
            position.phi = 0;
        }
    }
    else if (strncmp(btn, "zp", 2))
    {
        position.z += d;
        if (position.phi > U32_KAR_MC_MAXPOS_Z)
        {
            position.phi = U32_KAR_MC_MAXPOS_Z;
        }
    }
    else if (strncmp(btn, "zn", 2))
    {
        position.z -= d;
        if (position.phi < 0)
        {
            position.phi = 0;
        }
    }
    else if (strncmp(btn, "hx", 2))
    {
        position.r = 0;
        position.phi = 0;
        position.z = 0;
    }

    xQueueSend(nextPositionQueueHandle, &position, 10);
}

static void handleButtonAction(const char *args)
{
    // Use the default GUI of the Wi-Fi controller which provides simple buttons for moving the
    // motors of the robotarm.
    char btn[3] = { '\0' };

    const char *arg_text = "btn=";
    const size_t arg_text_length = 4;

    char *p = strstr(args, arg_text);
    if ((p != NULL) && (strlen(p) >= (arg_text_length + 2)))
    {
        strncpy(btn, p + arg_text_length, 2);
        sendNewPosition(btn);
    }
    else
    {
        strcpy(btn, "xx");
    }

    Logger_LogPrintf(LogLevel_Info, "Button action: %s", btn);
}

static void setupWifi()
{
    const char *ap_ssid = "indactrobot";
    const char *ap_password = "pirosalma";

    // Wait for the module to start after power-up
    vTaskDelay(pdMS_TO_TICKS(2 * 1000));

    if (WifiController_WifiController_ResetModule() != WifiController_ErrorCode_NONE)
    {
        Logger_LogPrintf(LogLevel_Error, "Wi-Fi reset error.");
        return;
    }

    // Wait for the module to reset
    vTaskDelay(pdMS_TO_TICKS(2 * 1000));

    if (WifiController_WifiController_SetSsid(ap_ssid) != WifiController_ErrorCode_NONE)
    {
        Logger_LogPrintf(LogLevel_Error, "Wi-Fi set SSID error.");
        return;
    }

    if (WifiController_WifiController_SetPassword(ap_password) != WifiController_ErrorCode_NONE)
    {
        Logger_LogPrintf(LogLevel_Error, "Wi-Fi set password error.");
        return;
    }

    if (WifiController_WifiController_BeginAccessPoint(10 * 1000) != WifiController_ErrorCode_NONE)
    {
        Logger_LogPrintf(LogLevel_Error, "Wi-Fi begin access point error.");
        return;
    }
}

void setupTask(void *pvParameters)
{
    // Wait for tasks to be ready
    xEventGroupWaitBits(
            taskStartedEventGroupHandle,
            TASK_STARTED_WIFI_RECEIVE_TASK_BIT,
            pdFALSE,
            pdTRUE,
            portMAX_DELAY);

    // ================================================================================
    // Wi-Fi settings
    // ================================================================================
    setupWifi();

    // ================================================================================
    // Homing, then release control
    // ================================================================================
    homingSequence();
    xSemaphoreGive(controlMutexHandle);

    // ================================================================================
    // Start application
    // ================================================================================
    appState = AppState_DemoMoveControl;
    xEventGroupSetBits(stateEventGroupHandle, STATE_DEMO_MOVE_CONTROL_BIT);

    vTaskDelete(NULL);
}

/*
 *===================================================================*
 * Function name: indicatorBlinkingTask
 *-------------------------------------------------------------------
 * Description:
 * Using LEDs to indicate any software error on the board.
 *-------------------------------------------------------------------
 */
void indicatorBlinkingTask(void *pvParameters)
{
    // Signal that the task is ready to run
    xEventGroupSetBits(taskStartedEventGroupHandle, TASK_STARTED_INDICATOR_BLINKING_TASK_BIT);
    Logger_LogPrintf(LogLevel_Info, "Ready to run.");

    while (1)
    {
        if (appState == AppState_Error)
        {
            HAL_GPIO_WritePin(RedLed_LD4_GPIO_Port, RedLed_LD4_Pin, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(200));
            HAL_GPIO_WritePin(RedLed_LD4_GPIO_Port, RedLed_LD4_Pin, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        else
        {
            HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(500));
            HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

void wifiReceiveTask(void *pvParameters)
{
    WifiController_ActionList_Init(&wifiActionList);
    WifiController_ActionList_Add(&wifiActionList, "/button", handleButtonAction);

    configASSERT(WifiController_WifiController_Init(&wifiActionList) == WifiController_ErrorCode_NONE);

    // Signal that the task is ready to run
    xEventGroupSetBits(taskStartedEventGroupHandle, TASK_STARTED_WIFI_RECEIVE_TASK_BIT);
    Logger_LogPrintf(LogLevel_Info, "Ready to run.");

    while (1)
    {
        WifiController_WifiController_Receive();
    }
}

/*
 *===================================================================*
 * Function name: demoMoveControlTask
 *-------------------------------------------------------------------
 * Description:
 * Purpose of this task is to show off, to show the kids how cool this huge pile of metal and cable is.
 * It doesn't do much: There is 5 positions declared and stored in a queue. The task takes the positions
 * and drives the arm to them in an infinite loop.
 *-------------------------------------------------------------------
 */
void demoMoveControlTask(void *pvParameters)
{
    // Demo positions
    PositionCyl_t positions[5];
    positions[0].r = 0;
    positions[0].phi = 0;
    positions[0].z = 0;
    positions[1].r = 400;
    positions[1].phi = 4000;
    positions[1].z = 4000;
    positions[2].r = 0;
    positions[2].phi = 9000;
    positions[2].z = 7000;
    positions[3].r = 900;
    positions[3].phi = 13874;
    positions[3].z = 10000;
    positions[4].r = 200;
    positions[4].phi = 6000;
    positions[4].z = 5000;

    // Signal that the task is ready to run
    xEventGroupSetBits(taskStartedEventGroupHandle, TASK_STARTED_DEMO_MOVE_CONTROL_TASK_BIT);
    Logger_LogPrintf(LogLevel_Info, "Ready to run.");

    // Only enter the loop if the control is not taken by any of the other tasks
    xSemaphoreTake(controlMutexHandle, portMAX_DELAY);
    Logger_LogPrintf(LogLevel_Info, "Take control.");

    size_t idx = 0;
    while (1)
    {
        if (appState != AppState_DemoMoveControl)
        {
            // Give up the control
            xSemaphoreGive(controlMutexHandle);
            Logger_LogPrintf(LogLevel_Info, "Give control.");

            // Wait for the task specific wake-up event
            xEventGroupWaitBits(
                    stateEventGroupHandle,
                    STATE_DEMO_MOVE_CONTROL_BIT,
                    pdTRUE,
                    pdTRUE,
                    portMAX_DELAY);
            Logger_LogPrintf(LogLevel_Info, "Wake-up.");

            // Wait for the other tasks to give up control and take it
            xSemaphoreTake(controlMutexHandle, portMAX_DELAY);
            Logger_LogPrintf(LogLevel_Info, "Take control.");

            // Always start from the first position
            idx = 0;
        }

        moveToPosition(&positions[idx]);
        idx = (idx + 1) % 5;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*
 *===================================================================*
 * Function name: gpioControlTask
 *-------------------------------------------------------------------
 * Description:
 * This task lets the user to control the arm via GPIO buttons
 *-------------------------------------------------------------------
 */
void gpioControlTask(void *pvParameters)
{
    s_GEN_GPIO r_pos_button = {
            .GPIO_Port = motor_r_positive_button_GPIO_Port,
            .GPIO_Pin = motor_r_positive_button_Pin
    };
    s_GEN_GPIO r_neg_button = {
            .GPIO_Port = motor_r_negative_button_GPIO_Port,
            .GPIO_Pin = motor_r_negative_button_Pin
    };
    s_GEN_GPIO fi_pos_button = {
            .GPIO_Port = motor_fi_positive_button_GPIO_Port,
            .GPIO_Pin = motor_fi_positive_button_Pin
    };
    s_GEN_GPIO fi_neg_button = {
            .GPIO_Port = motor_fi_negative_button_GPIO_Port,
            .GPIO_Pin = motor_fi_negative_button_Pin
    };
    s_GEN_GPIO z_pos_button = {
            .GPIO_Port = motor_z_positive_button_GPIO_Port,
            .GPIO_Pin = motor_z_positive_button_Pin
    };
    s_GEN_GPIO z_neg_button = {
            .GPIO_Port = motor_z_negative_button_GPIO_Port,
            .GPIO_Pin = motor_z_negative_button_Pin
    };

    // Signal that the task is ready to run
    xEventGroupSetBits(taskStartedEventGroupHandle, TASK_STARTED_GPIO_CONTROL_TASK_BIT);
    Logger_LogPrintf(LogLevel_Info, "Ready to run.");

    // Only enter the loop if the control is not taken by any of the other tasks
    xSemaphoreTake(controlMutexHandle, portMAX_DELAY);
    Logger_LogPrintf(LogLevel_Info, "Take control.");

    while (1)
    {
        if (appState != AppState_GpioControl)
        {
            // Give up the control
            xSemaphoreGive(controlMutexHandle);
            Logger_LogPrintf(LogLevel_Info, "Give control.");

            // Wait for the task specific wake-up event
            xEventGroupWaitBits(
                    stateEventGroupHandle,
                    STATE_GPIO_CONTROL_BIT,
                    pdTRUE,
                    pdTRUE,
                    portMAX_DELAY);
            Logger_LogPrintf(LogLevel_Info, "Wake-up.");

            // Wait for the other tasks to give up control and take it
            xSemaphoreTake(controlMutexHandle, portMAX_DELAY);
            Logger_LogPrintf(LogLevel_Info, "Take control.");
        }

        /* Axis control functions */
        u8_MC_ControlMotor_viaGPIO_f(as_stepper_motors, KAR_MC_MOTORID_R, as_limit_switches, r_pos_button, r_neg_button);
        u8_MC_ControlMotor_viaGPIO_f(as_stepper_motors, KAR_MC_MOTORID_PHI, as_limit_switches, fi_pos_button, fi_neg_button);
        u8_MC_ControlMotor_viaGPIO_f(as_stepper_motors, KAR_MC_MOTORID_Z, as_limit_switches, z_pos_button, z_neg_button);

        vTaskDelay(10);
    }
}

void wifiControlTask(void *pvParameters)
{
    // Signal that the task is ready to run
    xEventGroupSetBits(taskStartedEventGroupHandle, TASK_STARTED_WIFI_CONTROL_TASK_BIT);
    Logger_LogPrintf(LogLevel_Info, "Ready to run.");

    // Only enter the loop if the control is not taken by any of the other tasks
    xSemaphoreTake(controlMutexHandle, portMAX_DELAY);
    Logger_LogPrintf(LogLevel_Info, "Take control.");

    while (1)
    {
        if (appState != AppState_WifiControl)
        {
            // Give up the control
            xSemaphoreGive(controlMutexHandle);
            Logger_LogPrintf(LogLevel_Info, "Give control.");

            // Wait for the task specific wake-up event
            xEventGroupWaitBits(
                    stateEventGroupHandle,
                    STATE_WIFI_CONTROL_BIT,
                    pdTRUE,
                    pdTRUE,
                    portMAX_DELAY);
            Logger_LogPrintf(LogLevel_Info, "Wake-up.");

            // Wait for the other tasks to give up control and take it
            xSemaphoreTake(controlMutexHandle, portMAX_DELAY);
            Logger_LogPrintf(LogLevel_Info, "Take control.");

            // Empty the queue before continuing
            xQueueReset(nextPositionQueueHandle);
        }

        PositionCyl_t position;
        if (xQueueReceive(nextPositionQueueHandle, &position, 200) == pdTRUE)
        {
            Logger_LogPrintf(
                    LogLevel_Info,
                    "Position received: (%ld, %ld, %ld).",
                    position.r, position.phi, position.z);
            moveToPosition(&position);
        }
    }
}

/*
 *===================================================================*
 * Function name: HAL_GPIO_EXTI_Callback
 *-------------------------------------------------------------------
 * Description:
 * When the arm reaches it's limits, the limit switches trigger external interrupt request.
 * To prevent further movement, the motor of the axis stops and starting the motor to that direction gets forbidden temporarily.
 * The mode changing button also uses EXTI. In this case the s_program_status.task_id is updated.
 *
 * INPUT:
 * @param (uint16_t) GPIO_Pin: GPIO Pin number which had triggered the interrupt.
 *
 * OUTPUT: none
 *-------------------------------------------------------------------
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (GPIO_Pin)
    {
    case controller_mode_switch_Pin:
        static uint32_t last_tick = 0;
        if (debounce(&last_tick, 200))
        {
            changeAppStateFromISR(&xHigherPriorityTaskWoken);
        }
        break;
    case limswitch_z_null_Pin:
        as_limit_switches[KAR_MC_MOTORID_Z].null_point = HAL_GPIO_ReadPin(limswitch_z_null_GPIO_Port, limswitch_z_null_Pin);
        if (as_limit_switches[KAR_MC_MOTORID_Z].null_point)
        {
            as_stepper_motors[KAR_MC_MOTORID_Z].currPos = 0u;
            as_stepper_motors[KAR_MC_MOTORID_Z].allowedDir = KAR_MC_ALLOWDIR_POSDIR;
        }
        else
        {
            as_stepper_motors[KAR_MC_MOTORID_Z].allowedDir = KAR_MC_ALLOWDIR_BOTHDIR;
        }
        break;
    case limswitch_z_max_Pin:
        as_limit_switches[KAR_MC_MOTORID_Z].max_point = HAL_GPIO_ReadPin(limswitch_z_max_GPIO_Port, limswitch_z_max_Pin);
        if (as_limit_switches[KAR_MC_MOTORID_Z].max_point)
        {
            as_stepper_motors[KAR_MC_MOTORID_Z].currPos = U32_KAR_MC_MAXPOS_Z;
            as_stepper_motors[KAR_MC_MOTORID_Z].allowedDir = KAR_MC_ALLOWDIR_NEGDIR;
        }
        else
        {
            as_stepper_motors[KAR_MC_MOTORID_Z].allowedDir = KAR_MC_ALLOWDIR_BOTHDIR;
        }
        break;
    case limswitch_r_null_Pin:
        as_limit_switches[KAR_MC_MOTORID_R].null_point = HAL_GPIO_ReadPin(limswitch_r_null_GPIO_Port, limswitch_r_null_Pin);
        if (as_limit_switches[KAR_MC_MOTORID_R].null_point)
        {
            as_stepper_motors[KAR_MC_MOTORID_R].currPos = 0u;
            as_stepper_motors[KAR_MC_MOTORID_R].allowedDir = KAR_MC_ALLOWDIR_POSDIR;
        }
        else
        {
            as_stepper_motors[KAR_MC_MOTORID_R].allowedDir = KAR_MC_ALLOWDIR_BOTHDIR;
        }
        break;
    case limswitch_r_max_Pin:
        as_limit_switches[KAR_MC_MOTORID_R].max_point = HAL_GPIO_ReadPin(limswitch_r_max_GPIO_Port, limswitch_r_max_Pin);
        if (as_limit_switches[KAR_MC_MOTORID_R].max_point)
        {
            as_stepper_motors[KAR_MC_MOTORID_R].currPos = U32_KAR_MC_MAXPOS_R;
            as_stepper_motors[KAR_MC_MOTORID_R].allowedDir = KAR_MC_ALLOWDIR_NEGDIR;
        }
        else
        {
            as_stepper_motors[KAR_MC_MOTORID_R].allowedDir = KAR_MC_ALLOWDIR_BOTHDIR;
        }
        break;
    case limswich_fi_null_Pin:
        as_limit_switches[KAR_MC_MOTORID_PHI].null_point = HAL_GPIO_ReadPin(limswich_fi_null_GPIO_Port, limswich_fi_null_Pin);
        if (as_limit_switches[KAR_MC_MOTORID_PHI].null_point)
        {
            as_stepper_motors[KAR_MC_MOTORID_PHI].currPos = 0u;
            as_stepper_motors[KAR_MC_MOTORID_PHI].allowedDir = KAR_MC_ALLOWDIR_POSDIR;
        }
        else
        {
            as_stepper_motors[KAR_MC_MOTORID_PHI].allowedDir = KAR_MC_ALLOWDIR_BOTHDIR;
        }
        break;
    case limswitch_fi_max_Pin:
        as_limit_switches[KAR_MC_MOTORID_PHI].max_point = HAL_GPIO_ReadPin(limswitch_fi_max_GPIO_Port, limswitch_fi_max_Pin);
        if (as_limit_switches[KAR_MC_MOTORID_PHI].max_point)
        {
            as_stepper_motors[KAR_MC_MOTORID_PHI].currPos = U32_KAR_MC_MAXPOS_FI;
            as_stepper_motors[KAR_MC_MOTORID_PHI].allowedDir = KAR_MC_ALLOWDIR_NEGDIR;
        }
        else
        {
            as_stepper_motors[KAR_MC_MOTORID_PHI].allowedDir = KAR_MC_ALLOWDIR_BOTHDIR;
        }
        break;
    }

    // If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch should be requested.
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 *===================================================================*
 * Function name: HAL_TIM_PWM_PulseFinishedCallback
 *-------------------------------------------------------------------
 * Description:
 * TIM1, TIM2 and TIM3 control the motors with PWM signals. Every rising edge of a PWM signal triggers a timer interrupt.
 * This handler increases or decreases the current position value of the appropriate motor depending on it's moving direction.
 * This way I don't need encoders to know where is the arm in the coordinate system.
 *
 * INPUT:
 * @param (TIM_HandleTypeDef*) htim: Timer handler which had triggered the interrupt.
 *
 * OUTPUT: none
 *-------------------------------------------------------------------
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        if (KAR_MC_DIR_NEGATIVE == as_stepper_motors[KAR_MC_MOTORID_PHI].dir)
        {
            as_stepper_motors[KAR_MC_MOTORID_PHI].currPos--;
        }
        else if (KAR_MC_DIR_POSITIVE == as_stepper_motors[KAR_MC_MOTORID_PHI].dir)
        {
            as_stepper_motors[KAR_MC_MOTORID_PHI].currPos++;
        }
    }
    if (htim->Instance == TIM2)
    {
        if (KAR_MC_DIR_NEGATIVE == as_stepper_motors[KAR_MC_MOTORID_Z].dir)
        {
            as_stepper_motors[KAR_MC_MOTORID_Z].currPos--;
        }
        else if (KAR_MC_DIR_POSITIVE == as_stepper_motors[KAR_MC_MOTORID_Z].dir)
        {
            as_stepper_motors[KAR_MC_MOTORID_Z].currPos++;
        }
    }
    if (htim->Instance == TIM3)
    {
        if (KAR_MC_DIR_NEGATIVE == as_stepper_motors[KAR_MC_MOTORID_R].dir)
        {
            as_stepper_motors[KAR_MC_MOTORID_R].currPos--;
        }
        else if (KAR_MC_DIR_POSITIVE == as_stepper_motors[KAR_MC_MOTORID_R].dir)
        {
            as_stepper_motors[KAR_MC_MOTORID_R].currPos++;
        }
    }
}

/* USER CODE END Application */
