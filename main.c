/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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
#include "motors.h"
#include <stdio.h>
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void StartRobotControlTask(void const * argument);
void StartCommunicationTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static volatile bool motorHorizontal_RUN = 0;
static volatile bool motorVertical_RUN = 0;
static volatile bool motorRadial_RUN = 0;
static volatile bool motorHorizontal_DIR = 0;
static volatile bool motorVertical_DIR = 0;
static volatile bool motorRadial_DIR = 0;

//static volatile uint8_t buttonOK = 1;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	StepperMotor stepperMotors[3];
	// BOTTOM motor
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

	// TOP motor
	stepperMotors[MOTOR_Z_ID] = (StepperMotor) {
			.ID = MOTOR_Z_ID,
			.allowedDir = MOTORALLOW_BOTHDIR,
			.motorState = MOTORSTATE_STOPPED,

			.TIM_CH = TIM_CHANNEL_3,	//PE13
			.TIM = &htim1,

			.enablePORT = motor_z_ENA_GPIO_Port,	//PC14
			.enablePIN = motor_z_ENA_Pin,

			.dirPORT = motor_z_DIR_GPIO_Port,	//PC13
			.dirPIN = motor_z_DIR_Pin,
	};

	// MIDDLE motor
	stepperMotors[MOTOR_R_ID] = (StepperMotor) {
			.ID = MOTOR_R_ID,
			.allowedDir = MOTORALLOW_BOTHDIR,
			.motorState = MOTORSTATE_STOPPED,

			.TIM_CH = TIM_CHANNEL_1,	//PE9
			.TIM = &htim1,

			.enablePORT = motor_r_ENA_GPIO_Port,  //PE15
			.enablePIN = motor_r_ENA_Pin,

			.dirPORT = motor_r_DIR_GPIO_Port,		//PE12
			.dirPIN = motor_r_DIR_Pin,
	};
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
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM9_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //Enable all motor
  HAL_GPIO_WritePin(stepperMotors[MOTOR_R_ID].enablePORT,stepperMotors[MOTOR_R_ID].enablePIN , 1);		//enable = HIGH
  HAL_GPIO_WritePin(stepperMotors[MOTOR_Z_ID].enablePORT,stepperMotors[MOTOR_Z_ID].enablePIN , 1);		// enable = HIGH
  HAL_GPIO_WritePin(stepperMotors[MOTOR_FI_ID].enablePORT,stepperMotors[MOTOR_FI_ID].enablePIN , 0);    // enable = GND

  //Set the COM pins to GND
  HAL_GPIO_WritePin(motor_fi_COM_GPIO_Port, motor_fi_COM_Pin, 0);
  HAL_GPIO_WritePin(motor_r_COM_GPIO_Port, motor_r_COM_Pin , 0);
  HAL_GPIO_WritePin(motor_z_COM_GPIO_Port, motor_z_COM_Pin, 0);

  //Stop all motors before execution
  HAL_TIM_PWM_Stop_IT(&htim1, stepperMotors[MOTOR_FI_ID].TIM_CH);
  HAL_TIM_PWM_Stop_IT(&htim1, stepperMotors[MOTOR_R_ID].TIM_CH);
  HAL_TIM_PWM_Stop_IT(&htim1, stepperMotors[MOTOR_Z_ID].TIM_CH);

  //Used in the first section of the code
  //I am changing the direction of the motors by toggle this variable
  //	uint8_t direction = 0;


  // Belongs to the first section
  // It prepares the motors for running alone: direction value is a variable
  //  	HAL_GPIO_WritePin(stepperMotors[MOTOR_R_ID].dirPORT, stepperMotors[MOTOR_R_ID].dirPIN, direction);
  //  	HAL_GPIO_WritePin(stepperMotors[MOTOR_Z_ID].dirPORT, stepperMotors[MOTOR_Z_ID].dirPIN, direction);
  //  	HAL_GPIO_WritePin(stepperMotors[MOTOR_FI_ID].dirPORT, stepperMotors[MOTOR_FI_ID].dirPIN, direction);
  //  	HAL_TIM_PWM_Start_IT(&htim1, stepperMotors[MOTOR_R_ID].TIM_CH);
  //  	HAL_TIM_PWM_Start_IT(&htim1, stepperMotors[MOTOR_Z_ID].TIM_CH);
  //  	HAL_TIM_PWM_Start_IT(&htim1, stepperMotors[MOTOR_FI_ID].TIM_CH);


  while (1)
  {


/*
 * Moving without buttons (first section)
 * TODO output capture módban ha megadott tick-et lépett a motor megszakítással leállítani
	  HAL_Delay(1000);

	  direction = !direction;
	  HAL_GPIO_WritePin(stepperMotors[MOTOR_R_ID].dirPORT, stepperMotors[MOTOR_R_ID].dirPIN, direction);
	  HAL_GPIO_WritePin(stepperMotors[MOTOR_Z_ID].dirPORT, stepperMotors[MOTOR_Z_ID].dirPIN, direction);
	  HAL_GPIO_WritePin(stepperMotors[MOTOR_FI_ID].dirPORT, stepperMotors[MOTOR_FI_ID].dirPIN, direction);

	  //HAL_Delay(1000);

	  //HAL_TIM_PWM_Stop_IT(&htim1, stepperMotors[MOTOR_R_ID].TIM_CH);
	  //HAL_TIM_PWM_Stop_IT(&htim1, stepperMotors[MOTOR_Z_ID].TIM_CH);
	  //HAL_TIM_PWM_Stop_IT(&htim1, stepperMotors[MOTOR_FI_ID].TIM_CH);
*/




	  //Moving with buttons (second section)
	  //MŰKÖDIK
	  //R tengely
	  if(HAL_GPIO_ReadPin(motor_r_positive_button_GPIO_Port, motor_r_positive_button_Pin)){
		  HAL_GPIO_WritePin(stepperMotors[MOTOR_R_ID].dirPORT, stepperMotors[MOTOR_R_ID].dirPIN, MOTORDIR_POSITIVE);

		  HAL_TIM_PWM_Start_IT(&htim1, stepperMotors[MOTOR_R_ID].TIM_CH);
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 1);
	  }
	  else if(HAL_GPIO_ReadPin(motor_r_negative_button_GPIO_Port, motor_r_negative_button_Pin)) {
		  HAL_GPIO_WritePin(stepperMotors[MOTOR_R_ID].dirPORT, stepperMotors[MOTOR_R_ID].dirPIN, MOTORDIR_NEGATIVE);

		  HAL_TIM_PWM_Start_IT(&htim1, stepperMotors[MOTOR_R_ID].TIM_CH);
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 1);
	  }
	  else {
		  HAL_TIM_PWM_Stop_IT(&htim1, stepperMotors[MOTOR_R_ID].TIM_CH);
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 0);
	  }


	  //FI tengely
	  if(HAL_GPIO_ReadPin(motor_fi_positive_button_GPIO_Port, motor_fi_positive_button_Pin)){
		  HAL_GPIO_WritePin(stepperMotors[MOTOR_FI_ID].dirPORT, stepperMotors[MOTOR_FI_ID].dirPIN, MOTORDIR_POSITIVE);

		  HAL_TIM_PWM_Start_IT(&htim1, stepperMotors[MOTOR_FI_ID].TIM_CH);
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 1);
	  }
	  else if(HAL_GPIO_ReadPin(motor_fi_negative_button_GPIO_Port, motor_fi_negative_button_Pin)) {
		  HAL_GPIO_WritePin(stepperMotors[MOTOR_FI_ID].dirPORT, stepperMotors[MOTOR_FI_ID].dirPIN, MOTORDIR_NEGATIVE);

		  HAL_TIM_PWM_Start_IT(&htim1, stepperMotors[MOTOR_FI_ID].TIM_CH);
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 1);
	  }
	  else {
		  HAL_TIM_PWM_Stop_IT(&htim1, stepperMotors[MOTOR_FI_ID].TIM_CH);
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 0);
	  }



	  //Z tengely: ROSSZ A NYOMÓGOMBJA!!!!
	  if(HAL_GPIO_ReadPin(motor_z_positive_button_GPIO_Port, motor_z_positive_button_Pin)){
		  HAL_GPIO_WritePin(stepperMotors[MOTOR_Z_ID].dirPORT, stepperMotors[MOTOR_Z_ID].dirPIN, MOTORDIR_POSITIVE);

		  HAL_TIM_PWM_Start_IT(&htim1, stepperMotors[MOTOR_Z_ID].TIM_CH);
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 1);
	  }
	  else if(HAL_GPIO_ReadPin(motor_z_negative_button_GPIO_Port, motor_z_negative_button_Pin)) {
		  HAL_GPIO_WritePin(stepperMotors[MOTOR_Z_ID].dirPORT, stepperMotors[MOTOR_Z_ID].dirPIN, MOTORDIR_NEGATIVE);

		  HAL_TIM_PWM_Start_IT(&htim1, stepperMotors[MOTOR_Z_ID].TIM_CH);
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 1);
	  }
	  else {
		  HAL_TIM_PWM_Stop_IT(&htim1, stepperMotors[MOTOR_Z_ID].TIM_CH);
		  HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 0);
	  }



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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 1680-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 12;
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
  sConfigOC.Pulse = 399;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 499;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim2.Init.Prescaler = 1680-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 419;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8400-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim9, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, motor_z_DIR_Pin|motor_z_ENA_Pin|motor_z_COM_Pin|motor_fi_DIR_Pin
                          |motor_fi_ENA_Pin|motor_fi_COM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, motor_r_DIR_Pin|motor_r_COM_Pin|motor_r_ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GreenLed_LD3_Pin|RedLed_LD4_Pin, GPIO_PIN_RESET);

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
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/* Elkeseredett próbálkozás egy semaphor-hoz hasonló valaminek a megírására
 * Inkább hagytuk mert nem tudtuk mit csinálunk
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(buttonOK){
		if ((GPIO_Pin & motor_r_positive_button_Pin) & !((motorHorizontal_DIR == MOTORDIR_NEGATIVE) & motorHorizontal_RUN)){
			motorHorizontal_RUN != motorHorizontal_RUN;
			motorHorizontal_DIR = MOTORDIR_POSITIVE;
			HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 1);
		}
		if ((GPIO_Pin & motor_r_negative_button_Pin) & !((motorHorizontal_DIR == MOTORDIR_POSITIVE) & motorHorizontal_RUN)){
			motorHorizontal_RUN != motorHorizontal_RUN;
			motorHorizontal_DIR = MOTORDIR_NEGATIVE;
			HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 0);
		}
		if ((GPIO_Pin & motor_z_positive_button_Pin) & !((motorVertical_DIR == MOTORDIR_NEGATIVE) & motorVertical_RUN)){
			motorVertical_RUN != motorVertical_RUN;
			motorVertical_DIR = MOTORDIR_POSITIVE;
			HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 1);
		}
		if ((GPIO_Pin & motor_z_negative_button_Pin) & !((motorVertical_DIR == MOTORDIR_POSITIVE) & motorVertical_RUN)){
			motorVertical_RUN != motorVertical_RUN;
			motorVertical_DIR = MOTORDIR_NEGATIVE;
			HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 0);
		}
		if ((GPIO_Pin & motor_fi_positive_button_Pin) & !((motorRadial_DIR == MOTORDIR_NEGATIVE) & motorRadial_RUN)){
			motorRadial_RUN != motorRadial_RUN;
			motorRadial_DIR = MOTORDIR_POSITIVE;
			HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin , 1);
		}
		if ((GPIO_Pin & motor_fi_negative_button_Pin) & !((motorRadial_DIR == MOTORDIR_POSITIVE) & motorRadial_RUN)){
			motorRadial_RUN != motorRadial_RUN;
			motorRadial_DIR = MOTORDIR_NEGATIVE;
			HAL_GPIO_WritePin(GreenLed_LD3_GPIO_Port,GreenLed_LD3_Pin, 0);
		}

		buttonOK = 0;
		HAL_TIM_Base_Start_IT(&htim9);
	}
}
*/
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRobotControlTask */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobotControlTask */
void StartRobotControlTask(void const * argument)
{
  /* USER CODE BEGIN StartRobotControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRobotControlTask */
}

/* USER CODE BEGIN Header_StartCommunicationTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommunicationTask */
void StartCommunicationTask(void const * argument)
{
  /* USER CODE BEGIN StartCommunicationTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCommunicationTask */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
