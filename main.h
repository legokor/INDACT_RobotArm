/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#pragma once
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MotorVertical_DIR_Pin GPIO_PIN_13
#define MotorVertical_DIR_GPIO_Port GPIOC
#define MotorVertical_ENA_Pin GPIO_PIN_14
#define MotorVertical_ENA_GPIO_Port GPIOC
#define MotorVertical_COM_Pin GPIO_PIN_15
#define MotorVertical_COM_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define UART2_TX_STM_Pin GPIO_PIN_2
#define UART2_TX_STM_GPIO_Port GPIOA
#define UART2_RX_STM_Pin GPIO_PIN_3
#define UART2_RX_STM_GPIO_Port GPIOA
#define ADC1_IN4_reservedforfutureuse_Pin GPIO_PIN_4
#define ADC1_IN4_reservedforfutureuse_GPIO_Port GPIOA
#define SPI1_SCK_GPIOEXTENDER_Pin GPIO_PIN_5
#define SPI1_SCK_GPIOEXTENDER_GPIO_Port GPIOA
#define Encoder1MotorR1_Pin GPIO_PIN_6
#define Encoder1MotorR1_GPIO_Port GPIOA
#define SPI1_MOSI_GPIOEXTENDER_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIOEXTENDER_GPIO_Port GPIOA
#define MotorRadial_PUL_Pin GPIO_PIN_9
#define MotorRadial_PUL_GPIO_Port GPIOE
#define MotorHorizontal_PUL_Pin GPIO_PIN_11
#define MotorHorizontal_PUL_GPIO_Port GPIOE
#define MotorRadial_DIR_Pin GPIO_PIN_12
#define MotorRadial_DIR_GPIO_Port GPIOE
#define MotorVertical_PUL_Pin GPIO_PIN_13
#define MotorVertical_PUL_GPIO_Port GPIOE
#define MotorRadial_COM_Pin GPIO_PIN_14
#define MotorRadial_COM_GPIO_Port GPIOE
#define MotorRadial_ENA_Pin GPIO_PIN_15
#define MotorRadial_ENA_GPIO_Port GPIOE
#define UART3_TX_reservedforfutureuse_Pin GPIO_PIN_10
#define UART3_TX_reservedforfutureuse_GPIO_Port GPIOB
#define UART3_RX_reservedforfutureuse_Pin GPIO_PIN_11
#define UART3_RX_reservedforfutureuse_GPIO_Port GPIOB
#define STEP_gripperMotorThumb_Pin GPIO_PIN_12
#define STEP_gripperMotorThumb_GPIO_Port GPIOD
#define STEP_gripperMotorIndexMiddle_Pin GPIO_PIN_13
#define STEP_gripperMotorIndexMiddle_GPIO_Port GPIOD
#define STEP_gripperMotorRingPinky_Pin GPIO_PIN_14
#define STEP_gripperMotorRingPinky_GPIO_Port GPIOD
#define MotorRadial_PositiveButton_Pin GPIO_PIN_2
#define MotorRadial_PositiveButton_GPIO_Port GPIOG
#define MotorRadial_PositiveButton_EXTI_IRQn EXTI2_IRQn
#define MotorRadial_NegativeButton_Pin GPIO_PIN_3
#define MotorRadial_NegativeButton_GPIO_Port GPIOG
#define MotorRadial_NegativeButton_EXTI_IRQn EXTI3_IRQn
#define MotorHorizontal_PositiveButton_Pin GPIO_PIN_4
#define MotorHorizontal_PositiveButton_GPIO_Port GPIOG
#define MotorHorizontal_PositiveButton_EXTI_IRQn EXTI4_IRQn
#define MotorHorizontal_NegativeButton_Pin GPIO_PIN_5
#define MotorHorizontal_NegativeButton_GPIO_Port GPIOG
#define MotorHorizontal_NegativeButton_EXTI_IRQn EXTI9_5_IRQn
#define MotorVertical_PositiveButton_Pin GPIO_PIN_6
#define MotorVertical_PositiveButton_GPIO_Port GPIOG
#define MotorVertical_PositiveButton_EXTI_IRQn EXTI9_5_IRQn
#define MotorVertical_NegativeButton_Pin GPIO_PIN_7
#define MotorVertical_NegativeButton_GPIO_Port GPIOG
#define MotorVertical_NegativeButton_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_PG8_Interrupt_reservedforfutureuse_Pin GPIO_PIN_8
#define GPIO_PG8_Interrupt_reservedforfutureuse_GPIO_Port GPIOG
#define GPIO_PG8_Interrupt_reservedforfutureuse_EXTI_IRQn EXTI9_5_IRQn
#define Encoder2MotorR1_Pin GPIO_PIN_7
#define Encoder2MotorR1_GPIO_Port GPIOC
#define MotorHorizontal_DIR_Pin GPIO_PIN_8
#define MotorHorizontal_DIR_GPIO_Port GPIOC
#define UART1_TX_STLINK_PC_Pin GPIO_PIN_9
#define UART1_TX_STLINK_PC_GPIO_Port GPIOA
#define UART1_RX_STLINK_PC_Pin GPIO_PIN_10
#define UART1_RX_STLINK_PC_GPIO_Port GPIOA
#define MotorHorizontal_ENA_Pin GPIO_PIN_11
#define MotorHorizontal_ENA_GPIO_Port GPIOC
#define MotorHorizontal_COM_Pin GPIO_PIN_12
#define MotorHorizontal_COM_GPIO_Port GPIOC
#define EmergencyButton_Pin GPIO_PIN_11
#define EmergencyButton_GPIO_Port GPIOG
#define EmergencyButton_EXTI_IRQn EXTI15_10_IRQn
#define GreenLed_LD3_Pin GPIO_PIN_13
#define GreenLed_LD3_GPIO_Port GPIOG
#define RedLed_LD4_Pin GPIO_PIN_14
#define RedLed_LD4_GPIO_Port GPIOG
#define SPI1_MISO_GPIOEXTENDER_Pin GPIO_PIN_4
#define SPI1_MISO_GPIOEXTENDER_GPIO_Port GPIOB
#define I2C1_SCL_reservedforfutureuse_Pin GPIO_PIN_6
#define I2C1_SCL_reservedforfutureuse_GPIO_Port GPIOB
#define I2C1_SDA_reservedforfutureuse_Pin GPIO_PIN_7
#define I2C1_SDA_reservedforfutureuse_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
