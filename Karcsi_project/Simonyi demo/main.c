/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "StepperMotorControll.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Defines for indexing limit_switches array
#define fi_max 1
#define fi_null 2
#define z_max 3
#define z_null 4
#define r_max 5
#define r_null 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId indicator_blinkingHandle;
osThreadId demoMoveHandle;
osMessageQId demoMove_positionsHandle;
/* USER CODE BEGIN PV */
//Array that contains the motor structs, offers easy acccess to motor variables
static StepperMotor stepperMotors[NUMBER_OF_MOTORS];

/*
 * Pinstates of the limit switches
 * The @limit_switches array must contain 6 element:
 * limit_switches[0] : NOT IN USE
 * limit_switches[1] : FI axis' maximal positive excursion (rotated all the way clockwise)
 * limit_switches[2] : FI axis' zero point (when the arm is rotated all the way counter-clockwise)
 * limit_switches[3] : Z axis' maximal positiv excursion (upper sitch)
 * limit_switches[4] : Z axis' zero point (lower switch)
 * limit_switches[5] : R axis' maximal positive excursion (the arm reaches the furthest)
 * limit_switches[6] : R axis' zero ponit (when the arm is fully retracted)
 */
static GPIO_PinState limit_switches[7];

static ToolPosition current_position;

//Controlling Task output
static uint8_t error_byte = 0;

//It's true, if the homing has already run. Zero if homing is still needed.
static uint8_t homing_state = 0;

//Positions for Simonyi conference demo program
const ToolPosition Pos1 = (ToolPosition){.fi = 0, .z = 0, .r = 0};
const ToolPosition Pos2 = (ToolPosition){.fi = 4000, .z = 1000, .r = 400};
const ToolPosition Pos3 = (ToolPosition){.fi = 9000, .z = 3000, .r = 0};
const ToolPosition Pos4 = (ToolPosition){.fi = 13874, .z = 5000, .r = 900};
const ToolPosition Pos5 = (ToolPosition){.fi = 6000, .z = 2000, .r = 200};
const ToolPosition Pos15 = Pos1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void fc_indicator_blinking(void const * argument);
void fc_demoMove(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  /* definition and creation of demoMove_positions */
  osMessageQDef(demoMove_positions, 15, ToolPosition);
  demoMove_positionsHandle = osMessageCreate(osMessageQ(demoMove_positions), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  xQueueSend(demoMove_positionsHandle, (void*)&Pos1, 0);
  xQueueSend(demoMove_positionsHandle, (void*)&Pos2, 0);
  xQueueSend(demoMove_positionsHandle, (void*)&Pos3, 0);
  xQueueSend(demoMove_positionsHandle, (void*)&Pos4, 0);
  xQueueSend(demoMove_positionsHandle, (void*)&Pos5, 0);
  xQueueSend(demoMove_positionsHandle, (void*)&Pos15, 0);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of indicator_blinking */
  osThreadDef(indicator_blinking, fc_indicator_blinking, osPriorityRealtime, 0, 256);
  indicator_blinkingHandle = osThreadCreate(osThread(indicator_blinking), (void*) (&error_byte));

  /* definition and creation of demoMove */
  osThreadDef(demoMove, fc_demoMove, osPriorityRealtime, 0, 2048);
  demoMoveHandle = osThreadCreate(osThread(demoMove), (void*)stepperMotors);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 168-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart2, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, motor_z_DIR_Pin|motor_z_ENA_Pin|motor_z_COM_Pin|motor_fi_DIR_Pin
                          |motor_fi_ENA_Pin|motor_fi_COM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, motor_r_DIR_Pin|motor_r_COM_Pin|motor_r_ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GreenLed_LD3_Pin|RedLed_LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : limswitch_r_null_Pin limswitch_r_max_Pin limswich_fi_null_Pin limswitch_fi_max_Pin
                           limswitch_z_null_Pin limswitch_z_max_Pin */
  GPIO_InitStruct.Pin = limswitch_r_null_Pin|limswitch_r_max_Pin|limswich_fi_null_Pin|limswitch_fi_max_Pin
                          |limswitch_z_null_Pin|limswitch_z_max_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : motor_z_DIR_Pin motor_z_ENA_Pin motor_z_COM_Pin motor_fi_DIR_Pin
                           motor_fi_ENA_Pin motor_fi_COM_Pin */
  GPIO_InitStruct.Pin = motor_z_DIR_Pin|motor_z_ENA_Pin|motor_z_COM_Pin|motor_fi_DIR_Pin
                          |motor_fi_ENA_Pin|motor_fi_COM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : motor_z_negative_button_Pin motor_z_positive_button_Pin motor_r_negative_button_Pin motor_r_positive_button_Pin
                           motor_fi_negative_button_Pin motor_fi_positive_button_Pin */
  GPIO_InitStruct.Pin = motor_z_negative_button_Pin|motor_z_positive_button_Pin|motor_r_negative_button_Pin|motor_r_positive_button_Pin
                          |motor_fi_negative_button_Pin|motor_fi_positive_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : motor_r_DIR_Pin motor_r_COM_Pin motor_r_ENA_Pin */
  GPIO_InitStruct.Pin = motor_r_DIR_Pin|motor_r_COM_Pin|motor_r_ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : EmergencyButton_Pin */
  GPIO_InitStruct.Pin = EmergencyButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EmergencyButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GreenLed_LD3_Pin RedLed_LD4_Pin */
  GPIO_InitStruct.Pin = GreenLed_LD3_Pin|RedLed_LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*
 * In case of EXTI, one of the limit switches turn on. To prevent further movement, the axis' motor turns off
 * and @variable current_position is updated.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch (GPIO_Pin){
		case limswitch_z_null_Pin:{
			limit_switches[z_null] = HAL_GPIO_ReadPin(limswitch_z_null_GPIO_Port, limswitch_z_null_Pin);
			if(limit_switches[z_null]){
				current_position.z = 0;
				stepperMotors[MOTOR_Z_ID].allowedDir = MOTORALLOW_POSDIR;
			}
			else{
				stepperMotors[MOTOR_Z_ID].allowedDir = MOTORALLOW_BOTHDIR;
			}
			break;
		}
		case limswitch_z_max_Pin:{
			limit_switches[z_max] = HAL_GPIO_ReadPin(limswitch_z_max_GPIO_Port, limswitch_z_max_Pin);
			if(limit_switches[z_max]){
				current_position.z = MOTOR_Z_MAXPOS;
				stepperMotors[MOTOR_Z_ID].allowedDir = MOTORALLOW_NEGDIR;
			}
			else{
				stepperMotors[MOTOR_Z_ID].allowedDir = MOTORALLOW_BOTHDIR;
			}
			break;
		}
		case limswitch_r_null_Pin:{
			limit_switches[r_null] = HAL_GPIO_ReadPin(limswitch_r_null_GPIO_Port, limswitch_r_null_Pin);
			if(limit_switches[r_null]){
				current_position.r = 0;
				stepperMotors[MOTOR_R_ID].allowedDir = MOTORALLOW_POSDIR;
			}
			else{
				stepperMotors[MOTOR_R_ID].allowedDir = MOTORALLOW_BOTHDIR;
			}
			break;
		}
		case limswitch_r_max_Pin:{
			limit_switches[r_max] = HAL_GPIO_ReadPin(limswitch_r_max_GPIO_Port, limswitch_r_max_Pin);
			if(limit_switches[r_max]){
				current_position.r = MOTOR_R_MAXPOS;
				stepperMotors[MOTOR_R_ID].allowedDir = MOTORALLOW_NEGDIR;
			}
			else{
				stepperMotors[MOTOR_R_ID].allowedDir = MOTORALLOW_BOTHDIR;
			}
			break;
		}
		case limswich_fi_null_Pin:{
			limit_switches[fi_null] = HAL_GPIO_ReadPin(limswich_fi_null_GPIO_Port, limswich_fi_null_Pin);
			if(limit_switches[fi_null]){
				current_position.fi = 0;
				stepperMotors[MOTOR_FI_ID].allowedDir = MOTORALLOW_POSDIR;
			}
			else{
				stepperMotors[MOTOR_FI_ID].allowedDir = MOTORALLOW_BOTHDIR;
			}
			break;
		}
		case limswitch_fi_max_Pin:{
			limit_switches[fi_max] = HAL_GPIO_ReadPin(limswitch_fi_max_GPIO_Port, limswitch_fi_max_Pin);
			if(limit_switches[fi_max]){
				current_position.fi = MOTOR_FI_MAXPOS;
				stepperMotors[MOTOR_FI_ID].allowedDir = MOTORALLOW_NEGDIR;
			}
			else{
				stepperMotors[MOTOR_FI_ID].allowedDir = MOTORALLOW_BOTHDIR;
			}
			break;
		}
	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1) {
		if(MOTORDIR_NEGATIVE == stepperMotors[MOTOR_FI_ID].dir){
		  current_position.fi--;
		}
		else if(MOTORDIR_POSITIVE == stepperMotors[MOTOR_FI_ID].dir){
		  current_position.fi++;
		}
	}
	if(htim->Instance == TIM2) {
		if(MOTORDIR_NEGATIVE == stepperMotors[MOTOR_Z_ID].dir){
		  current_position.z--;
		}
		else if(MOTORDIR_POSITIVE == stepperMotors[MOTOR_Z_ID].dir){
		  current_position.z++;
		}
	}
	if(htim->Instance == TIM3) {
		if(MOTORDIR_NEGATIVE == stepperMotors[MOTOR_R_ID].dir){
		  current_position.r--;
		}
		else if(MOTORDIR_POSITIVE == stepperMotors[MOTOR_R_ID].dir){
		  current_position.r++;
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_fc_indicator_blinking */
/**
  * @brief  Function implementing the indicator_blinking thread.
  * @param  argument: Not used
  * @retval None
  *
  * This task is only used for debugging.
  * If any of the functions returns with error, the red LED on the board starts blinking.
  * If everything is OK, then a green LED blinks.
  */
/* USER CODE END Header_fc_indicator_blinking */
void fc_indicator_blinking(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(error_byte){
		  HAL_GPIO_WritePin(RedLed_LD4_GPIO_Port, RedLed_LD4_Pin, 1);
		  osDelay(200);
		  HAL_GPIO_WritePin(RedLed_LD4_GPIO_Port, RedLed_LD4_Pin, 0);
		  osDelay(200);
	  }
	  else{
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, 1);
		  osDelay(1000);
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, 0);
		  osDelay(1000);
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_fc_demoMove */
/**
* @brief Function implementing the demoMove thread.
* @param argument: Pointer to the array of the stepper motors
* @retval None
*
* The task has made for 2023 Simonyi Conference.
* It implements a demo program: the arm moves on a hard-coded route endlessly.
* I tried to desing the rout to stay completely over the table, so people who are passing by won't hit the arm.
* However I strongly recommend to keep an eye on it while it's moving. Dropping Karcsi on the ground wouldn't be so funny :)
*/
/* USER CODE END Header_fc_demoMove */
void fc_demoMove(void const * argument)
{
  /* USER CODE BEGIN fc_demoMove */

	//LIMIT SWITCH INIT --------------------------------------------------------------//
	limit_switches[1] = HAL_GPIO_ReadPin(limswitch_fi_max_GPIO_Port, limswitch_fi_max_Pin);
	limit_switches[2] = HAL_GPIO_ReadPin(limswich_fi_null_GPIO_Port, limswich_fi_null_Pin);
	limit_switches[3] = HAL_GPIO_ReadPin(limswitch_z_max_GPIO_Port, limswitch_z_max_Pin);
	limit_switches[4] = HAL_GPIO_ReadPin(limswitch_z_null_GPIO_Port, limswitch_z_null_Pin);
	limit_switches[5] = HAL_GPIO_ReadPin(limswitch_r_max_GPIO_Port, limswitch_r_max_Pin);
	limit_switches[6] = HAL_GPIO_ReadPin(limswitch_r_null_GPIO_Port, limswitch_r_null_Pin);

	//MOTOR INIT ------------------------------------------------------------------//
	StepperMotor* stepperMotors = (StepperMotor*)argument;
	stepperMotors[MOTOR_FI_ID] = (StepperMotor) {
			.ID = MOTOR_FI_ID,
			.allowedDir = MOTORALLOW_BOTHDIR,
			.motorState = MOTORSTATE_STOPPED,

			.TIM_CH = TIM_CHANNEL_2,	//PE11
			.TIM = &htim1,

			.enablePORT = motor_fi_ENA_GPIO_Port,  //PC11
			.enablePIN = motor_fi_ENA_Pin,

			.dirPORT = motor_fi_DIR_GPIO_Port,	//PC8
			.dirPIN = motor_fi_DIR_Pin,
	};
	stepperMotors[MOTOR_Z_ID] = (StepperMotor) {
			.ID = MOTOR_Z_ID,
			.allowedDir = MOTORALLOW_BOTHDIR,
			.motorState = MOTORSTATE_STOPPED,

			.TIM_CH = TIM_CHANNEL_1,	//PA0
			.TIM = &htim2,

			.enablePORT = motor_z_ENA_GPIO_Port,	//PC14
			.enablePIN = motor_z_ENA_Pin,

			.dirPORT = motor_z_DIR_GPIO_Port,	//PC13
			.dirPIN = motor_z_DIR_Pin,
	};
	stepperMotors[MOTOR_R_ID] = (StepperMotor) {
			.ID = MOTOR_R_ID,
			.allowedDir = MOTORALLOW_BOTHDIR,
			.motorState = MOTORSTATE_STOPPED,

			.TIM_CH = TIM_CHANNEL_1,	//PA6
			.TIM = &htim3,

			.enablePORT = motor_r_ENA_GPIO_Port,  //PE15
			.enablePIN = motor_r_ENA_Pin,

			.dirPORT = motor_r_DIR_GPIO_Port,		//PE12
			.dirPIN = motor_r_DIR_Pin,
	};

	//Set the COM pins to GND
	HAL_GPIO_WritePin(motor_fi_COM_GPIO_Port, motor_fi_COM_Pin, 0);
	HAL_GPIO_WritePin(motor_r_COM_GPIO_Port, motor_r_COM_Pin , 0);
	HAL_GPIO_WritePin(motor_z_COM_GPIO_Port, motor_z_COM_Pin, 0);

	//Enable all motor
	HAL_GPIO_WritePin(stepperMotors[MOTOR_R_ID].enablePORT,stepperMotors[MOTOR_R_ID].enablePIN , 1);	//enable = HIGH
	HAL_GPIO_WritePin(stepperMotors[MOTOR_Z_ID].enablePORT,stepperMotors[MOTOR_Z_ID].enablePIN , 1);	// enable = HIGH
	HAL_GPIO_WritePin(stepperMotors[MOTOR_FI_ID].enablePORT,stepperMotors[MOTOR_FI_ID].enablePIN , 0);  // enable = GND

	HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, 1);

	//HOMING ----------------------------------------------------------------------------//
		if(!homing_state){
			error_byte = startAllMotor(stepperMotors, MOTORDIR_NEGATIVE);
			//Only run if the arm is not homed
			if(!error_byte){
				uint8_t z = 0, fi = 0, r = 0;
				//waiting untill all of the motors has reached their limit
				while(!(r && fi && z)){
					//FI axis check
					if(GPIO_PIN_SET == limit_switches[fi_null]){
						stopMotor(stepperMotors, MOTOR_FI_ID);
						fi = 1;
					}

					//Z axis check
					if(GPIO_PIN_SET == limit_switches[z_null]){
						stopMotor(stepperMotors, MOTOR_Z_ID);
						z = 1;
					}

					//R axis check
					if(GPIO_PIN_SET == limit_switches[r_null]){
						stopMotor(stepperMotors, MOTOR_R_ID);
						r = 1;
					}
				}
				homing_state = 1;
			}
			else {
				//if there is a problem about the motors, dont do nothing stupid
				for(;;){}
			}
		}


	ToolPosition next_pos = (ToolPosition){.fi = 0, .z = 0, .r = 0};
	uint8_t z = 0, fi = 0, r = 0;
	/* Infinite control loop */
	for(;;)
	{
		if(xQueueReceive(demoMove_positionsHandle, (void*)&next_pos, 1)){

			setAllMotorDirTowardsDesiredPos(stepperMotors, current_position, next_pos);
			startMotor(stepperMotors, MOTOR_FI_ID, stepperMotors[MOTOR_FI_ID].dir);
			startMotor(stepperMotors, MOTOR_R_ID, stepperMotors[MOTOR_R_ID].dir);
			startMotor(stepperMotors, MOTOR_Z_ID, stepperMotors[MOTOR_Z_ID].dir);
			while(!(r && fi && z)){
				if((next_pos.fi == current_position.fi) || (limit_switches[fi_max] && MOTORDIR_POSITIVE == stepperMotors[MOTOR_FI_ID].dir) || (limit_switches[fi_null] && MOTORDIR_NEGATIVE == stepperMotors[MOTOR_FI_ID].dir)){
					stopMotor(stepperMotors, MOTOR_FI_ID);
					fi = 1;
				}
				if((next_pos.z == current_position.z) || (limit_switches[z_max] && MOTORDIR_POSITIVE == stepperMotors[MOTOR_Z_ID].dir) || (limit_switches[z_null] && MOTORDIR_NEGATIVE == stepperMotors[MOTOR_Z_ID].dir)){
					stopMotor(stepperMotors, MOTOR_Z_ID);
					z = 1;
				}
				if((next_pos.r == current_position.r) || (limit_switches[r_max] && MOTORDIR_POSITIVE == stepperMotors[MOTOR_R_ID].dir) || (limit_switches[r_null] && MOTORDIR_NEGATIVE == stepperMotors[MOTOR_R_ID].dir)){
					stopMotor(stepperMotors, MOTOR_R_ID);
					r = 1;
				}
			}
			osDelay(1000);
			r = z = fi = 0;
		}

		 xQueueSend(demoMove_positionsHandle, (void*)&next_pos, 0);
		/* USER CODE END fc_demoMove */
	}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
