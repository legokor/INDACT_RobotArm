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
#include "tim.h"

#include "queue.h"

#include "KAR_MC_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TASK_PRIORITY_NORMAL ((tskIDLE_PRIORITY + configMAX_PRIORITIES) / 2)

// Task settings
#define INDICATOR_BLINKING_TASK_STACK_SIZE 256
#define INDICATOR_BLINKING_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define CONTROL_VIA_GPIO_TASK_STACK_SIZE 256
#define CONTROL_VIA_GPIO_TASK_PRIORITY TASK_PRIORITY_NORMAL
#define DEMO_MOVE_TASK_STACK_SIZE 256
#define DEMO_MOVE_TASK_PRIORITY TASK_PRIORITY_NORMAL
#define MOVE_TO_POSITION_TASK_STACK_SIZE 256
#define MOVE_TO_POSITION_TASK_PRIORITY TASK_PRIORITY_NORMAL

// Queue settings
#define DEMO_MOVE_POSITIONS_QUEUE_LENGTH 15
#define DEMO_MOVE_POSITIONS_QUEUE_ITEM_SIZE sizeof(s_GEO_ToolPosition_Cylinder)
#define NEXT_POSITION_QUEUE_LENGTH 64
#define NEXT_POSITION_QUEUE_ITEM_SIZE sizeof(s_GEO_ToolPosition_Cylinder)

// Task codes
#define L_GPIO_TASK_CODE                                    (0u)
#define L_NO_TASK_CODE                                      (1u)
#define L_DEMO_TASK_CODE                                    (2u)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// Task variables
static StaticTask_t indicatorBlinkingTaskBuffer;
static StackType_t indicatorBlinkingTaskStack[INDICATOR_BLINKING_TASK_STACK_SIZE];
TaskHandle_t indicatorBlinkingTaskHandle = NULL;

static StaticTask_t controlViaGpioTaskBuffer;
static StackType_t controlViaGpioTaskStack[CONTROL_VIA_GPIO_TASK_STACK_SIZE];
TaskHandle_t controlViaGpioTaskHandle = NULL;

static StaticTask_t demoMoveTaskBuffer;
static StackType_t demoMoveTaskStack[DEMO_MOVE_TASK_STACK_SIZE];
TaskHandle_t demoMoveTaskHandle = NULL;

static StaticTask_t moveToPositionTaskBuffer;
static StackType_t moveToPositionTaskStack[MOVE_TO_POSITION_TASK_STACK_SIZE];
TaskHandle_t moveToPositionTaskHandle = NULL;

// Queue variables
static StaticQueue_t demoMovePositionsQueueBuffer;
static uint8_t demoMovePositionsQueueStorageArea[DEMO_MOVE_POSITIONS_QUEUE_LENGTH * DEMO_MOVE_POSITIONS_QUEUE_ITEM_SIZE];
QueueHandle_t demoMovePositionsQueueHandle = NULL;

static StaticQueue_t nextPositionQueueBuffer;
static uint8_t nextPositionQueueStorageArea[NEXT_POSITION_QUEUE_LENGTH * NEXT_POSITION_QUEUE_ITEM_SIZE];
QueueHandle_t nextPositionQueueHandle = NULL;

// Other variables
static s_MC_StepperMotor as_stepper_motors[KAR_MC_NUMBER_OF_MOTORS];
static s_GEO_LimitSwitch as_limit_switches[KAR_MC_NUMBER_OF_MOTORS];
static s_GEN_ProgramStatus s_program_status;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask", .stack_size = 128 * 4, .priority = (osPriority_t)osPriorityNormal, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void indicator_blinking_f(void *pvParameters);
static void control_viaGPIO_f(void *pvParameters);
static void demo_move_f(void *pvParameters);
static void moveToPositionTask(void *pvParameters);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* Positions for demo_move task (Simonyi conference demo program) */
    const s_GEO_ToolPosition_Cylinder Pos1 = (s_GEO_ToolPosition_Cylinder ) { .fi = 0, .z = 0, .r = 0 };
    const s_GEO_ToolPosition_Cylinder Pos2 = (s_GEO_ToolPosition_Cylinder ) { .fi = 4000, .z = 4000, .r = 400 };
    const s_GEO_ToolPosition_Cylinder Pos3 = (s_GEO_ToolPosition_Cylinder ) { .fi = 9000, .z = 7000, .r = 0 };
    const s_GEO_ToolPosition_Cylinder Pos4 = (s_GEO_ToolPosition_Cylinder ) { .fi = 13874, .z = 10000, .r = 900 };
    const s_GEO_ToolPosition_Cylinder Pos5 = (s_GEO_ToolPosition_Cylinder ) { .fi = 6000, .z = 5000, .r = 200 };

    /* Create the queue(s) */
    demoMovePositionsQueueHandle = xQueueCreateStatic(
            DEMO_MOVE_POSITIONS_QUEUE_LENGTH,
            DEMO_MOVE_POSITIONS_QUEUE_ITEM_SIZE,
            demoMovePositionsQueueStorageArea,
            &demoMovePositionsQueueBuffer);
    configASSERT(demoMovePositionsQueueHandle != NULL);

    xQueueSend(demoMovePositionsQueueHandle, (void* )(&Pos1), 0);
    xQueueSend(demoMovePositionsQueueHandle, (void* )(&Pos2), 0);
    xQueueSend(demoMovePositionsQueueHandle, (void* )(&Pos3), 0);
    xQueueSend(demoMovePositionsQueueHandle, (void* )(&Pos4), 0);
    xQueueSend(demoMovePositionsQueueHandle, (void* )(&Pos5), 0);

    nextPositionQueueHandle = xQueueCreateStatic(
            NEXT_POSITION_QUEUE_LENGTH,
            NEXT_POSITION_QUEUE_ITEM_SIZE,
            nextPositionQueueStorageArea,
            &nextPositionQueueBuffer);
    configASSERT(nextPositionQueueHandle != NULL);

    /* Create the task(s) */
    indicatorBlinkingTaskHandle = xTaskCreateStatic(
            indicator_blinking_f,
            "IndicatorBlinking",
            INDICATOR_BLINKING_TASK_STACK_SIZE,
            NULL,
            INDICATOR_BLINKING_TASK_PRIORITY,
            indicatorBlinkingTaskStack,
            &indicatorBlinkingTaskBuffer);
    configASSERT(indicatorBlinkingTaskHandle != NULL);

    controlViaGpioTaskHandle = xTaskCreateStatic(
            control_viaGPIO_f,
            "ControlViaGpio",
            CONTROL_VIA_GPIO_TASK_STACK_SIZE,
            NULL,
            CONTROL_VIA_GPIO_TASK_PRIORITY,
            controlViaGpioTaskStack,
            &controlViaGpioTaskBuffer);
    configASSERT(controlViaGpioTaskHandle != NULL);

    demoMoveTaskHandle = xTaskCreateStatic(
            demo_move_f,
            "DemoMove",
            DEMO_MOVE_TASK_STACK_SIZE,
            NULL,
            DEMO_MOVE_TASK_PRIORITY,
            demoMoveTaskStack,
            &demoMoveTaskBuffer);
    configASSERT(demoMoveTaskHandle != NULL);

    moveToPositionTaskHandle = xTaskCreateStatic(
            moveToPositionTask,
            "MoveToPosition",
            MOVE_TO_POSITION_TASK_STACK_SIZE,
            NULL,
            MOVE_TO_POSITION_TASK_PRIORITY,
            moveToPositionTaskStack,
            &moveToPositionTaskBuffer);
    configASSERT(moveToPositionTaskHandle != NULL);

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
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
 *===================================================================*
 * Function name: stepperMotor_init
 *-------------------------------------------------------------------
 * Description:
 * This function gives starting values to the as_stepper_motor array elements.
 *
 * INPUT: none
 *
 * OUTPUT: none
 *-------------------------------------------------------------------
 */
void stepperMotor_init()
{
    if (!s_program_status.homing_state)
    {
        /* LIMIT SWITCH INIT */
        as_limit_switches[KAR_MC_MOTORID_FI].max_point = HAL_GPIO_ReadPin(limswitch_fi_max_GPIO_Port, limswitch_fi_max_Pin);
        as_limit_switches[KAR_MC_MOTORID_FI].null_point = HAL_GPIO_ReadPin(limswich_fi_null_GPIO_Port, limswich_fi_null_Pin);
        as_limit_switches[KAR_MC_MOTORID_Z].max_point = HAL_GPIO_ReadPin(limswitch_z_max_GPIO_Port, limswitch_z_max_Pin);
        as_limit_switches[KAR_MC_MOTORID_Z].null_point = HAL_GPIO_ReadPin(limswitch_z_null_GPIO_Port, limswitch_z_null_Pin);
        as_limit_switches[KAR_MC_MOTORID_R].max_point = HAL_GPIO_ReadPin(limswitch_r_max_GPIO_Port, limswitch_r_max_Pin);
        as_limit_switches[KAR_MC_MOTORID_R].null_point = HAL_GPIO_ReadPin(limswitch_r_null_GPIO_Port, limswitch_r_null_Pin);

        /* MOTOR INIT */
        as_stepper_motors[KAR_MC_MOTORID_FI] = (s_MC_StepperMotor ) { .id = KAR_MC_MOTORID_FI, .dir = KAR_MC_DIR_UNDEFINED, .allowedDir = KAR_MC_ALLOWDIR_BOTHDIR, .motorState = KAR_MC_STATE_STOPPED,

                .currPos = U32_KAR_MC_MAXPOS_FI / 2, .nextPos = 0,

                .TIM_CH = TIM_CHANNEL_2,    //PE11
                        .TIM = &htim1,

                        .ENA = (s_GEN_GPIO ) { .GPIO_Port = motor_fi_ENA_GPIO_Port, .GPIO_Pin = motor_fi_ENA_Pin }, .DIR = (s_GEN_GPIO ) { .GPIO_Port = motor_fi_DIR_GPIO_Port, .GPIO_Pin = motor_fi_DIR_Pin } , };
        as_stepper_motors[KAR_MC_MOTORID_Z] = (s_MC_StepperMotor ) { .id = KAR_MC_MOTORID_Z, .dir = KAR_MC_DIR_UNDEFINED, .allowedDir = KAR_MC_ALLOWDIR_BOTHDIR, .motorState = KAR_MC_STATE_STOPPED,

                .currPos = U32_KAR_MC_MAXPOS_Z / 2, .nextPos = 0,

                .TIM_CH = TIM_CHANNEL_1,    //PA0
                        .TIM = &htim2,

                        .ENA = (s_GEN_GPIO ) { .GPIO_Port = motor_z_ENA_GPIO_Port, .GPIO_Pin = motor_z_ENA_Pin }, .DIR = (s_GEN_GPIO ) { .GPIO_Port = motor_z_DIR_GPIO_Port, .GPIO_Pin = motor_z_DIR_Pin } , };
        as_stepper_motors[KAR_MC_MOTORID_R] = (s_MC_StepperMotor ) { .id = KAR_MC_MOTORID_R, .dir = KAR_MC_DIR_UNDEFINED, .allowedDir = KAR_MC_ALLOWDIR_BOTHDIR, .motorState = KAR_MC_STATE_STOPPED,

                .currPos = U32_KAR_MC_MAXPOS_R / 2, .nextPos = 0,

                .TIM_CH = TIM_CHANNEL_1,    //PA6
                        .TIM = &htim3,

                        .ENA = (s_GEN_GPIO ) { .GPIO_Port = motor_r_ENA_GPIO_Port, .GPIO_Pin = motor_r_ENA_Pin }, .DIR = (s_GEN_GPIO ) { .GPIO_Port = motor_r_DIR_GPIO_Port, .GPIO_Pin = motor_r_DIR_Pin } , };

        /* Set the COM pins to GND */
        HAL_GPIO_WritePin(motor_fi_COM_GPIO_Port, motor_fi_COM_Pin, 0);
        HAL_GPIO_WritePin(motor_r_COM_GPIO_Port, motor_r_COM_Pin, 0);
        HAL_GPIO_WritePin(motor_z_COM_GPIO_Port, motor_z_COM_Pin, 0);

        /* Enable all motor */
        HAL_GPIO_WritePin(as_stepper_motors[KAR_MC_MOTORID_R].ENA.GPIO_Port, as_stepper_motors[KAR_MC_MOTORID_R].ENA.GPIO_Pin, 1);  //enable = HIGH
        HAL_GPIO_WritePin(as_stepper_motors[KAR_MC_MOTORID_Z].ENA.GPIO_Port, as_stepper_motors[KAR_MC_MOTORID_Z].ENA.GPIO_Pin, 1); // enable = HIGH
        HAL_GPIO_WritePin(as_stepper_motors[KAR_MC_MOTORID_FI].ENA.GPIO_Port, as_stepper_motors[KAR_MC_MOTORID_FI].ENA.GPIO_Pin, 0);  // enable = GND
    }
}

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
void homing_sequence()
{
    if (!s_program_status.homing_state)
    {
        s_program_status.error_code = u8_MC_StartAllMotor_f(as_stepper_motors, KAR_MC_DIR_NEGATIVE);

        //Only run if the arm is not homed
        if (0u == s_program_status.error_code)
        {
            uint8_t z = 0u, fi = 0u, r = 0u;

            //waiting untill all of the motors has reached their limit
            while (!(r && fi && z))
            {
                //FI axis check
                if ((GPIO_PIN_SET == as_limit_switches[KAR_MC_MOTORID_FI].null_point) && (0u == fi))
                {
                    v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_FI);
                    fi = 1;
                }

                //Z axis check
                if ((GPIO_PIN_SET == as_limit_switches[KAR_MC_MOTORID_Z].null_point) && (0u == z))
                {
                    v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_Z);
                    z = 1;
                }

                //R axis check
                if ((GPIO_PIN_SET == as_limit_switches[KAR_MC_MOTORID_R].null_point) && (0u == r))
                {
                    v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_R);
                    r = 1;
                }
            }

            s_program_status.homing_state = 1;
        }
        else
        {
            Error_Handler();
        }
    }
}

/*
 *===================================================================*
 * Function name: indicator_blinking_f
 *-------------------------------------------------------------------
 * Description:
 * Using LEDs here to indicate any software error on the board.
 *
 * INPUT: none
 *
 * OUTPUT: none
 *-------------------------------------------------------------------
 */
void indicator_blinking_f(void *pvParameters)
{
    /* Infinite loop */
    for (;;)
    {
        if (L_NO_TASK_CODE == s_program_status.task_id)
        {
            HAL_GPIO_WritePin(RedLed_LD4_GPIO_Port, RedLed_LD4_Pin, 1);
            osDelay(200);
            HAL_GPIO_WritePin(RedLed_LD4_GPIO_Port, RedLed_LD4_Pin, 0);
            osDelay(200);
        }
        else
        {
            HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, 1);
            osDelay(1000);
            HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, 0);
            osDelay(1000);
        }
    }
}

/*
 *===================================================================*
 * Function name: control_viaGPIO_f
 *-------------------------------------------------------------------
 * Description:
 * This task lets the user to control the arm via the controller.
 *
 * INPUT: none
 *
 * OUTPUT: none
 *-------------------------------------------------------------------
 */
void control_viaGPIO_f(void *pvParameters)
{
    s_GEN_GPIO r_pos_button = (s_GEN_GPIO ) { .GPIO_Port = motor_r_positive_button_GPIO_Port, .GPIO_Pin = motor_r_positive_button_Pin };
    s_GEN_GPIO r_neg_button = (s_GEN_GPIO ) { .GPIO_Port = motor_r_negative_button_GPIO_Port, .GPIO_Pin = motor_r_negative_button_Pin };
    s_GEN_GPIO fi_pos_button = (s_GEN_GPIO ) { .GPIO_Port = motor_fi_positive_button_GPIO_Port, .GPIO_Pin = motor_fi_positive_button_Pin };
    s_GEN_GPIO fi_neg_button = (s_GEN_GPIO ) { .GPIO_Port = motor_fi_negative_button_GPIO_Port, .GPIO_Pin = motor_fi_negative_button_Pin };
    s_GEN_GPIO z_pos_button = (s_GEN_GPIO ) { .GPIO_Port = motor_z_positive_button_GPIO_Port, .GPIO_Pin = motor_z_positive_button_Pin };
    s_GEN_GPIO z_neg_button = (s_GEN_GPIO ) { .GPIO_Port = motor_z_negative_button_GPIO_Port, .GPIO_Pin = motor_z_negative_button_Pin };

    stepperMotor_init();

    //HOMING ----------------------------------------------------------------------------//
    homing_sequence();

    //START CONTROL TASK -----------------------------------------------------------------------//
    /* Infinite contol loop */
    for (;;)
    {
        //Switching beetwin modes
        if (L_DEMO_TASK_CODE == s_program_status.task_id)
        {
            vTaskResume(demoMoveTaskHandle);
            vTaskSuspend(NULL);
        }

        /* Axis control functions */
        u8_MC_ControlMotor_viaGPIO_f(as_stepper_motors, KAR_MC_MOTORID_R, as_limit_switches, r_pos_button, r_neg_button);
        u8_MC_ControlMotor_viaGPIO_f(as_stepper_motors, KAR_MC_MOTORID_FI, as_limit_switches, fi_pos_button, fi_neg_button);
        u8_MC_ControlMotor_viaGPIO_f(as_stepper_motors, KAR_MC_MOTORID_Z, as_limit_switches, z_pos_button, z_neg_button);
    }
}

/*
 *===================================================================*
 * Function name: demo_move_f
 *-------------------------------------------------------------------
 * Description:
 * Purpose of this task is to show off, to show the kids how cool this huge pile of metal and cable is.
 * It doesn't do much: There is 5 positions declared and stored in a queue. The task takes the positions
 * and drives the arm to them in an infinite loop.
 *
 * INPUT: none
 *
 * OUTPUT: none
 *-------------------------------------------------------------------
 */
void demo_move_f(void *pvParameters)
{
    // Indicating on the board, that this task has the control
    HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, 1);

    // Filling up the motors' struct with parameters
    stepperMotor_init();

    // Moving the arm to the zero position and initialize the coordinate system
    homing_sequence();

    s_GEO_ToolPosition_Cylinder next_pos = (s_GEO_ToolPosition_Cylinder ) { .fi = 0, .z = 0, .r = 0 };
    int32_t z_steps_to_make = 0, r_steps_to_make = 0, fi_steps_to_make = 0;
    int32_t z_tmp = 0, r_tmp = 0, fi_tmp = 0;
    uint8_t z = 0u, fi = 0u, r = 0u;

    /* Infinite control loop */
    for (;;)
    {
        /* Switching beetwin modes */
        if (L_GPIO_TASK_CODE == s_program_status.task_id)
        {
            vTaskResume(controlViaGpioTaskHandle);
            vTaskSuspend(NULL);
        }

        if (xQueueReceive(demoMovePositionsQueueHandle, (void*)&next_pos, 1))
        {
            as_stepper_motors[KAR_MC_MOTORID_FI].nextPos = next_pos.fi;
            as_stepper_motors[KAR_MC_MOTORID_Z].nextPos = next_pos.z;
            as_stepper_motors[KAR_MC_MOTORID_R].nextPos = next_pos.r;

            u8_MC_setAllMotorDir_TowardsDesiredPos_f(as_stepper_motors);

            z_steps_to_make = i32_GEN_AbsoluteValue_f((as_stepper_motors[KAR_MC_MOTORID_Z].nextPos - as_stepper_motors[KAR_MC_MOTORID_Z].currPos));
            r_steps_to_make = i32_GEN_AbsoluteValue_f((as_stepper_motors[KAR_MC_MOTORID_R].nextPos - as_stepper_motors[KAR_MC_MOTORID_R].currPos));
            fi_steps_to_make = i32_GEN_AbsoluteValue_f((as_stepper_motors[KAR_MC_MOTORID_FI].nextPos - as_stepper_motors[KAR_MC_MOTORID_FI].currPos));

            z_tmp = as_stepper_motors[KAR_MC_MOTORID_Z].currPos;
            r_tmp = as_stepper_motors[KAR_MC_MOTORID_R].currPos;
            fi_tmp = as_stepper_motors[KAR_MC_MOTORID_FI].currPos;

            u8_MC_StartMotor_f(as_stepper_motors, KAR_MC_MOTORID_FI, as_stepper_motors[KAR_MC_MOTORID_FI].dir);
            u8_MC_StartMotor_f(as_stepper_motors, KAR_MC_MOTORID_R, as_stepper_motors[KAR_MC_MOTORID_R].dir);
            u8_MC_StartMotor_f(as_stepper_motors, KAR_MC_MOTORID_Z, as_stepper_motors[KAR_MC_MOTORID_Z].dir);

            /* Wait for tool to reach next position */
            while (!(r && fi && z))
            {
                if ((fi_steps_to_make <= i32_GEN_AbsoluteValue_f(fi_tmp - as_stepper_motors[KAR_MC_MOTORID_FI].currPos)) || (as_limit_switches[KAR_MC_MOTORID_FI].max_point && KAR_MC_DIR_POSITIVE == as_stepper_motors[KAR_MC_MOTORID_FI].dir)
                        || (as_limit_switches[KAR_MC_MOTORID_FI].null_point && KAR_MC_DIR_NEGATIVE == as_stepper_motors[KAR_MC_MOTORID_FI].dir))
                {
                    v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_FI);
                    fi = 1;
                }
                if ((z_steps_to_make <= i32_GEN_AbsoluteValue_f(z_tmp - as_stepper_motors[KAR_MC_MOTORID_Z].currPos)) || (as_limit_switches[KAR_MC_MOTORID_Z].max_point && KAR_MC_DIR_POSITIVE == as_stepper_motors[KAR_MC_MOTORID_Z].dir)
                        || (as_limit_switches[KAR_MC_MOTORID_Z].null_point && KAR_MC_DIR_NEGATIVE == as_stepper_motors[KAR_MC_MOTORID_Z].dir))
                {
                    v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_Z);
                    z = 1;
                }
                if ((r_steps_to_make <= i32_GEN_AbsoluteValue_f(r_tmp - as_stepper_motors[KAR_MC_MOTORID_R].currPos)) || (as_limit_switches[KAR_MC_MOTORID_R].max_point && KAR_MC_DIR_POSITIVE == as_stepper_motors[KAR_MC_MOTORID_R].dir)
                        || (as_limit_switches[KAR_MC_MOTORID_R].null_point && KAR_MC_DIR_NEGATIVE == as_stepper_motors[KAR_MC_MOTORID_R].dir))
                {
                    v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_R);
                    r = 1;
                }
            }

            /* Stopping for a moment to make moving a little bit cooler */
            osDelay(1000);
            r = z = fi = 0;
        }

        /* Put the position back to the queue */
        xQueueSend(demoMovePositionsQueueHandle, (void* )&next_pos, 0);
    }
}

void moveToPositionTask(void *pvParameters)
{
    s_GEO_ToolPosition_Cylinder next_pos = (s_GEO_ToolPosition_Cylinder){.fi = 0, .z = 0, .r = 0};
    int32_t z_steps_to_make = 0, r_steps_to_make = 0, fi_steps_to_make = 0;
    int32_t z_tmp = 0, r_tmp = 0, fi_tmp = 0;
    uint8_t z = 0u, fi = 0u, r = 0u;
    while (1)
    {
        if (xQueueReceive(nextPositionQueueHandle, (void*)(&next_pos), portMAX_DELAY) != pdTRUE)
        {
            // TODO: Signal error
            continue;
        }

        as_stepper_motors[KAR_MC_MOTORID_FI].nextPos = next_pos.fi;
        as_stepper_motors[KAR_MC_MOTORID_Z].nextPos = next_pos.z;
        as_stepper_motors[KAR_MC_MOTORID_R].nextPos = next_pos.r;

        u8_MC_setAllMotorDir_TowardsDesiredPos_f(as_stepper_motors);

        z_steps_to_make = i32_GEN_AbsoluteValue_f((as_stepper_motors[KAR_MC_MOTORID_Z].nextPos - as_stepper_motors[KAR_MC_MOTORID_Z].currPos));
        r_steps_to_make = i32_GEN_AbsoluteValue_f((as_stepper_motors[KAR_MC_MOTORID_R].nextPos - as_stepper_motors[KAR_MC_MOTORID_R].currPos));
        fi_steps_to_make = i32_GEN_AbsoluteValue_f((as_stepper_motors[KAR_MC_MOTORID_FI].nextPos - as_stepper_motors[KAR_MC_MOTORID_FI].currPos));

        z_tmp = as_stepper_motors[KAR_MC_MOTORID_Z].currPos;
        r_tmp = as_stepper_motors[KAR_MC_MOTORID_R].currPos;
        fi_tmp = as_stepper_motors[KAR_MC_MOTORID_FI].currPos;

        u8_MC_StartMotor_f(as_stepper_motors, KAR_MC_MOTORID_FI, as_stepper_motors[KAR_MC_MOTORID_FI].dir);
        u8_MC_StartMotor_f(as_stepper_motors, KAR_MC_MOTORID_R, as_stepper_motors[KAR_MC_MOTORID_R].dir);
        u8_MC_StartMotor_f(as_stepper_motors, KAR_MC_MOTORID_Z, as_stepper_motors[KAR_MC_MOTORID_Z].dir);

        /* Wait for tool to reach next position */
        while (!(r && fi && z))
        {
            if ((fi_steps_to_make <= i32_GEN_AbsoluteValue_f(fi_tmp - as_stepper_motors[KAR_MC_MOTORID_FI].currPos)) ||
                    (as_limit_switches[KAR_MC_MOTORID_FI].max_point && KAR_MC_DIR_POSITIVE == as_stepper_motors[KAR_MC_MOTORID_FI].dir) ||
                    (as_limit_switches[KAR_MC_MOTORID_FI].null_point && KAR_MC_DIR_NEGATIVE == as_stepper_motors[KAR_MC_MOTORID_FI].dir))
            {
                v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_FI);
                fi = 1;
            }
            if ((z_steps_to_make <= i32_GEN_AbsoluteValue_f(z_tmp - as_stepper_motors[KAR_MC_MOTORID_Z].currPos)) ||
                    (as_limit_switches[KAR_MC_MOTORID_Z].max_point && KAR_MC_DIR_POSITIVE == as_stepper_motors[KAR_MC_MOTORID_Z].dir) ||
                    (as_limit_switches[KAR_MC_MOTORID_Z].null_point && KAR_MC_DIR_NEGATIVE == as_stepper_motors[KAR_MC_MOTORID_Z].dir))
            {
                v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_Z);
                z = 1;
            }
            if ((r_steps_to_make <= i32_GEN_AbsoluteValue_f(r_tmp - as_stepper_motors[KAR_MC_MOTORID_R].currPos)) ||
                    (as_limit_switches[KAR_MC_MOTORID_R].max_point && KAR_MC_DIR_POSITIVE == as_stepper_motors[KAR_MC_MOTORID_R].dir) ||
                    (as_limit_switches[KAR_MC_MOTORID_R].null_point && KAR_MC_DIR_NEGATIVE == as_stepper_motors[KAR_MC_MOTORID_R].dir))
            {
                v_MC_StopMotor_f(as_stepper_motors, KAR_MC_MOTORID_R);
                r = 1;
            }
        }

        r = z = fi = 0;
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
    switch (GPIO_Pin)
    {
    case controller_mode_switch_Pin:
        if (s_program_status.task_id)
        {
            s_program_status.task_id = L_GPIO_TASK_CODE;
            HAL_GPIO_WritePin(controller_LED_GPIO_Port, controller_LED_Pin, s_program_status.task_id);
        }
        else
        {
            s_program_status.task_id = L_DEMO_TASK_CODE;
            HAL_GPIO_WritePin(controller_LED_GPIO_Port, controller_LED_Pin, s_program_status.task_id);
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
        as_limit_switches[KAR_MC_MOTORID_FI].null_point = HAL_GPIO_ReadPin(limswich_fi_null_GPIO_Port, limswich_fi_null_Pin);
        if (as_limit_switches[KAR_MC_MOTORID_FI].null_point)
        {
            as_stepper_motors[KAR_MC_MOTORID_FI].currPos = 0u;
            as_stepper_motors[KAR_MC_MOTORID_FI].allowedDir = KAR_MC_ALLOWDIR_POSDIR;
        }
        else
        {
            as_stepper_motors[KAR_MC_MOTORID_FI].allowedDir = KAR_MC_ALLOWDIR_BOTHDIR;
        }
        break;
    case limswitch_fi_max_Pin:
        as_limit_switches[KAR_MC_MOTORID_FI].max_point = HAL_GPIO_ReadPin(limswitch_fi_max_GPIO_Port, limswitch_fi_max_Pin);
        if (as_limit_switches[KAR_MC_MOTORID_FI].max_point)
        {
            as_stepper_motors[KAR_MC_MOTORID_FI].currPos = U32_KAR_MC_MAXPOS_FI;
            as_stepper_motors[KAR_MC_MOTORID_FI].allowedDir = KAR_MC_ALLOWDIR_NEGDIR;
        }
        else
        {
            as_stepper_motors[KAR_MC_MOTORID_FI].allowedDir = KAR_MC_ALLOWDIR_BOTHDIR;
        }
        break;
    }
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
        if (KAR_MC_DIR_NEGATIVE == as_stepper_motors[KAR_MC_MOTORID_FI].dir)
        {
            as_stepper_motors[KAR_MC_MOTORID_FI].currPos--;
        }
        else if (KAR_MC_DIR_POSITIVE == as_stepper_motors[KAR_MC_MOTORID_FI].dir)
        {
            as_stepper_motors[KAR_MC_MOTORID_FI].currPos++;
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
