/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @brief
  *
  * @TODO
  * - van két fv-ed, amik globális változókat basztatnak. Jó az ha nem ad vissza semmit?
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "KAR_MC_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define L_GPIO_TASK_CODE									(0u)
#define L_NO_TASK_CODE										(1u)
#define L_DEMO_TASK_CODE									(2u)

/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId indicator_blinkingHandle;
osThreadId control_viaGPIOHandle;
osThreadId demo_moveHandle;
osThreadId inverseGeometry_demoHandle;
osMessageQId demo_move_positionsHandle;
osMessageQId inverseGeometry_demo_move_positionsHandle;
/* USER CODE BEGIN PV */

static s_MC_StepperMotor as_stepper_motors[KAR_MC_NUMBER_OF_MOTORS];
static s_GEO_LimitSwitch as_limit_switches[KAR_MC_NUMBER_OF_MOTORS];
static s_GEN_ProgramStatus s_program_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void indicator_blinking_f(void const * argument);
void control_viaGPIO_f(void const * argument);
void demo_move_f(void const * argument);
void inverseGeometry_demo_f(void const * argument);

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

	/* Positions for demo_move task (Simonyi conference demo program) */
	const s_GEO_ToolPosition_Cylinder Pos1 = (s_GEO_ToolPosition_Cylinder){.fi = 0, .z = 0, .r = 0};
	const s_GEO_ToolPosition_Cylinder Pos2 = (s_GEO_ToolPosition_Cylinder){.fi = 4000, .z = 4000, .r = 400};
	const s_GEO_ToolPosition_Cylinder Pos3 = (s_GEO_ToolPosition_Cylinder){.fi = 9000, .z = 7000, .r = 0};
	const s_GEO_ToolPosition_Cylinder Pos4 = (s_GEO_ToolPosition_Cylinder){.fi = 13874, .z = 10000, .r = 900};
	const s_GEO_ToolPosition_Cylinder Pos5 = (s_GEO_ToolPosition_Cylinder){.fi = 6000, .z = 5000, .r = 200};

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
  /* definition and creation of demo_move_positions */
  osMessageQDef(demo_move_positions, 15, s_GEO_ToolPosition_Cylinder);
  demo_move_positionsHandle = osMessageCreate(osMessageQ(demo_move_positions), NULL);

  /* definition and creation of inverseGeometry_demo_move_positions */
  osMessageQDef(inverseGeometry_demo_move_positions, 16, s_GEO_ToolPosition_Descartes);
  inverseGeometry_demo_move_positionsHandle = osMessageCreate(osMessageQ(inverseGeometry_demo_move_positions), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  xQueueSend(demo_move_positionsHandle, (void*)(&Pos1), 0);
  xQueueSend(demo_move_positionsHandle, (void*)(&Pos2), 0);
  xQueueSend(demo_move_positionsHandle, (void*)(&Pos3), 0);
  xQueueSend(demo_move_positionsHandle, (void*)(&Pos4), 0);
  xQueueSend(demo_move_positionsHandle, (void*)(&Pos5), 0);
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
  HAL_GPIO_WritePin(controller_LED_GPIO_Port, controller_LED_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : controller_mode_switch_Pin */
  GPIO_InitStruct.Pin = controller_mode_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(controller_mode_switch_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : controller_LED_Pin */
  GPIO_InitStruct.Pin = controller_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(controller_LED_GPIO_Port, &GPIO_InitStruct);

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
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
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
void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
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
void stepperMotor_init ()
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
		as_stepper_motors[KAR_MC_MOTORID_FI] = (s_MC_StepperMotor) {
				.id = KAR_MC_MOTORID_FI,
				.dir = KAR_MC_DIR_UNDEFINED,
				.allowedDir = KAR_MC_ALLOWDIR_BOTHDIR,
				.motorState = KAR_MC_STATE_STOPPED,

				.currPos = U32_KAR_MC_MAXPOS_FI/2,
				.nextPos = 0,

				.TIM_CH = TIM_CHANNEL_2,	//PE11
				.TIM = &htim1,

				.ENA = (s_GEN_GPIO){.GPIO_Port = motor_fi_ENA_GPIO_Port, .GPIO_Pin = motor_fi_ENA_Pin},
				.DIR = (s_GEN_GPIO){.GPIO_Port = motor_fi_DIR_GPIO_Port, .GPIO_Pin = motor_fi_DIR_Pin},
		};
		as_stepper_motors[KAR_MC_MOTORID_Z] = (s_MC_StepperMotor) {
				.id = KAR_MC_MOTORID_Z,
				.dir = KAR_MC_DIR_UNDEFINED,
				.allowedDir = KAR_MC_ALLOWDIR_BOTHDIR,
				.motorState = KAR_MC_STATE_STOPPED,

				.currPos = U32_KAR_MC_MAXPOS_Z/2,
				.nextPos = 0,

				.TIM_CH = TIM_CHANNEL_1,	//PA0
				.TIM = &htim2,

				.ENA = (s_GEN_GPIO){.GPIO_Port = motor_z_ENA_GPIO_Port, .GPIO_Pin = motor_z_ENA_Pin},
				.DIR = (s_GEN_GPIO){.GPIO_Port = motor_z_DIR_GPIO_Port, .GPIO_Pin = motor_z_DIR_Pin},
		};
		as_stepper_motors[KAR_MC_MOTORID_R] = (s_MC_StepperMotor) {
				.id = KAR_MC_MOTORID_R,
				.dir = KAR_MC_DIR_UNDEFINED,
				.allowedDir = KAR_MC_ALLOWDIR_BOTHDIR,
				.motorState = KAR_MC_STATE_STOPPED,

				.currPos = U32_KAR_MC_MAXPOS_R/2,
				.nextPos = 0,

				.TIM_CH = TIM_CHANNEL_1,	//PA6
				.TIM = &htim3,

				.ENA = (s_GEN_GPIO){.GPIO_Port = motor_r_ENA_GPIO_Port, .GPIO_Pin = motor_r_ENA_Pin},
				.DIR = (s_GEN_GPIO){.GPIO_Port = motor_r_DIR_GPIO_Port, .GPIO_Pin = motor_r_DIR_Pin},
		};

		/* Set the COM pins to GND */
		HAL_GPIO_WritePin(motor_fi_COM_GPIO_Port, motor_fi_COM_Pin, 0);
		HAL_GPIO_WritePin(motor_r_COM_GPIO_Port, motor_r_COM_Pin , 0);
		HAL_GPIO_WritePin(motor_z_COM_GPIO_Port, motor_z_COM_Pin, 0);

		/* Enable all motor */
		HAL_GPIO_WritePin(as_stepper_motors[KAR_MC_MOTORID_R].ENA.GPIO_Port, as_stepper_motors[KAR_MC_MOTORID_R].ENA.GPIO_Pin, 1);	//enable = HIGH
		HAL_GPIO_WritePin(as_stepper_motors[KAR_MC_MOTORID_Z].ENA.GPIO_Port, as_stepper_motors[KAR_MC_MOTORID_Z].ENA.GPIO_Pin , 1);	// enable = HIGH
		HAL_GPIO_WritePin(as_stepper_motors[KAR_MC_MOTORID_FI].ENA.GPIO_Port, as_stepper_motors[KAR_MC_MOTORID_FI].ENA.GPIO_Pin , 0);  // enable = GND
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
void homing_sequence ()
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

/* USER CODE END 4 */

/* USER CODE BEGIN Header_indicator_blinking_f */
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
/* USER CODE END Header_indicator_blinking_f */
void indicator_blinking_f(void const * argument)
{
  /* USER CODE BEGIN 5 */
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
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_control_viaGPIO_f */
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
/* USER CODE END Header_control_viaGPIO_f */
void control_viaGPIO_f(void const * argument)
{
  /* USER CODE BEGIN control_viaGPIO_f */

	s_GEN_GPIO r_pos_button = (s_GEN_GPIO){.GPIO_Port = motor_r_positive_button_GPIO_Port, .GPIO_Pin = motor_r_positive_button_Pin};
	s_GEN_GPIO r_neg_button = (s_GEN_GPIO){.GPIO_Port = motor_r_negative_button_GPIO_Port, .GPIO_Pin = motor_r_negative_button_Pin};
	s_GEN_GPIO fi_pos_button = (s_GEN_GPIO){.GPIO_Port = motor_fi_positive_button_GPIO_Port, .GPIO_Pin = motor_fi_positive_button_Pin};
	s_GEN_GPIO fi_neg_button = (s_GEN_GPIO){.GPIO_Port = motor_fi_negative_button_GPIO_Port, .GPIO_Pin = motor_fi_negative_button_Pin};
	s_GEN_GPIO z_pos_button = (s_GEN_GPIO){.GPIO_Port = motor_z_positive_button_GPIO_Port, .GPIO_Pin = motor_z_positive_button_Pin};
	s_GEN_GPIO z_neg_button = (s_GEN_GPIO){.GPIO_Port = motor_z_negative_button_GPIO_Port, .GPIO_Pin = motor_z_negative_button_Pin};

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
			vTaskResume(demo_moveHandle);
			vTaskSuspend(NULL);
		}

		/* Axis control functions */
		u8_MC_ControlMotor_viaGPIO_f(as_stepper_motors, KAR_MC_MOTORID_R, as_limit_switches, r_pos_button, r_neg_button);
		u8_MC_ControlMotor_viaGPIO_f(as_stepper_motors, KAR_MC_MOTORID_FI, as_limit_switches, fi_pos_button, fi_neg_button);
		u8_MC_ControlMotor_viaGPIO_f(as_stepper_motors, KAR_MC_MOTORID_Z, as_limit_switches, z_pos_button, z_neg_button);
	}
  /* USER CODE END control_viaGPIO_f */
}

/* USER CODE BEGIN Header_demo_move_f */
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
/* USER CODE END Header_demo_move_f */
void demo_move_f(void const * argument)
{
  /* USER CODE BEGIN demo_move_f */

	// Indicating on the board, that this task has the control
	HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port, GreenLed_LD3_Pin, 1);

	// Filling up the motors' struct with parameters
	stepperMotor_init();

	// Moving the arm to the zero position and initialize the coordinate system
	homing_sequence();

	s_GEO_ToolPosition_Cylinder next_pos = (s_GEO_ToolPosition_Cylinder){.fi = 0, .z = 0, .r = 0};
	int32_t z_steps_to_make = 0, r_steps_to_make = 0, fi_steps_to_make = 0;
	int32_t z_tmp = 0, r_tmp = 0, fi_tmp = 0;
	uint8_t z = 0u, fi = 0u, r = 0u;

	/* Infinite control loop */
	for (;;)
	{
		/* Switching beetwin modes */
		if (L_GPIO_TASK_CODE == s_program_status.task_id)
		{
			vTaskResume(control_viaGPIOHandle);
			vTaskSuspend(NULL);
		}

		if (xQueueReceive(demo_move_positionsHandle, (void*)&next_pos, 1))
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

			/* Stopping for a moment to make moving a little bit cooler */
			osDelay(1000);
			r = z = fi = 0;
		}

		/* Put the position back to the queue */
		xQueueSend(demo_move_positionsHandle, (void*)&next_pos, 0);
	}
  /* USER CODE END demo_move_f */
}

/* USER CODE BEGIN Header_inverseGeometry_demo_f */
/*
 *===================================================================*
 * Function name: inverseGeometry_demo_f
 *-------------------------------------------------------------------
 * Description:
 *
 *
 * INPUT: none
 *
 * OUTPUT: none
 *-------------------------------------------------------------------
 */
/* USER CODE END Header_inverseGeometry_demo_f */
void inverseGeometry_demo_f(void const * argument)
{
  /* USER CODE BEGIN inverseGeometry_demo_f */

  /* TODO */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END inverseGeometry_demo_f */
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

	s_program_status.task_id = L_NO_TASK_CODE;

	vTaskSuspend(control_viaGPIOHandle);
	vTaskSuspend(demo_moveHandle);

	v_MC_StopAllMotor_f(as_stepper_motors);

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
