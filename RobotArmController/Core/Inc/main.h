/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define limswitch_r_null_Pin GPIO_PIN_2
#define limswitch_r_null_GPIO_Port GPIOE
#define limswitch_r_null_EXTI_IRQn EXTI2_IRQn
#define limswitch_r_max_Pin GPIO_PIN_3
#define limswitch_r_max_GPIO_Port GPIOE
#define limswitch_r_max_EXTI_IRQn EXTI3_IRQn
#define limswich_fi_null_Pin GPIO_PIN_4
#define limswich_fi_null_GPIO_Port GPIOE
#define limswich_fi_null_EXTI_IRQn EXTI4_IRQn
#define limswitch_fi_max_Pin GPIO_PIN_5
#define limswitch_fi_max_GPIO_Port GPIOE
#define limswitch_fi_max_EXTI_IRQn EXTI9_5_IRQn
#define controller_mode_switch_Pin GPIO_PIN_6
#define controller_mode_switch_GPIO_Port GPIOE
#define controller_mode_switch_EXTI_IRQn EXTI9_5_IRQn
#define motor_z_DIR_Pin GPIO_PIN_13
#define motor_z_DIR_GPIO_Port GPIOC
#define motor_z_ENA_Pin GPIO_PIN_14
#define motor_z_ENA_GPIO_Port GPIOC
#define motor_z_COM_Pin GPIO_PIN_15
#define motor_z_COM_GPIO_Port GPIOC
#define motor_z_negative_button_Pin GPIO_PIN_0
#define motor_z_negative_button_GPIO_Port GPIOF
#define motor_z_positive_button_Pin GPIO_PIN_1
#define motor_z_positive_button_GPIO_Port GPIOF
#define motor_r_negative_button_Pin GPIO_PIN_2
#define motor_r_negative_button_GPIO_Port GPIOF
#define motor_r_positive_button_Pin GPIO_PIN_3
#define motor_r_positive_button_GPIO_Port GPIOF
#define motor_fi_negative_button_Pin GPIO_PIN_4
#define motor_fi_negative_button_GPIO_Port GPIOF
#define motor_fi_positive_button_Pin GPIO_PIN_5
#define motor_fi_positive_button_GPIO_Port GPIOF
#define controller_LED_Pin GPIO_PIN_6
#define controller_LED_GPIO_Port GPIOF
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define motor_z_PUL_Pin GPIO_PIN_0
#define motor_z_PUL_GPIO_Port GPIOA
#define UART2_TX_STM_Pin GPIO_PIN_2
#define UART2_TX_STM_GPIO_Port GPIOA
#define UART2_RX_STM_Pin GPIO_PIN_3
#define UART2_RX_STM_GPIO_Port GPIOA
#define motor_r_PUL_Pin GPIO_PIN_6
#define motor_r_PUL_GPIO_Port GPIOA
#define motor_fi_PUL_Pin GPIO_PIN_11
#define motor_fi_PUL_GPIO_Port GPIOE
#define motor_r_DIR_Pin GPIO_PIN_12
#define motor_r_DIR_GPIO_Port GPIOE
#define motor_r_COM_Pin GPIO_PIN_14
#define motor_r_COM_GPIO_Port GPIOE
#define motor_r_ENA_Pin GPIO_PIN_15
#define motor_r_ENA_GPIO_Port GPIOE
#define motor_fi_DIR_Pin GPIO_PIN_8
#define motor_fi_DIR_GPIO_Port GPIOC
#define UART1_TX_STLINK_PC_Pin GPIO_PIN_9
#define UART1_TX_STLINK_PC_GPIO_Port GPIOA
#define UART1_RX_STLINK_PC_Pin GPIO_PIN_10
#define UART1_RX_STLINK_PC_GPIO_Port GPIOA
#define motor_fi_ENA_Pin GPIO_PIN_11
#define motor_fi_ENA_GPIO_Port GPIOC
#define motor_fi_COM_Pin GPIO_PIN_12
#define motor_fi_COM_GPIO_Port GPIOC
#define EmergencyButton_Pin GPIO_PIN_11
#define EmergencyButton_GPIO_Port GPIOG
#define EmergencyButton_EXTI_IRQn EXTI15_10_IRQn
#define GreenLed_LD3_Pin GPIO_PIN_13
#define GreenLed_LD3_GPIO_Port GPIOG
#define RedLed_LD4_Pin GPIO_PIN_14
#define RedLed_LD4_GPIO_Port GPIOG
#define limswitch_z_null_Pin GPIO_PIN_0
#define limswitch_z_null_GPIO_Port GPIOE
#define limswitch_z_null_EXTI_IRQn EXTI0_IRQn
#define limswitch_z_max_Pin GPIO_PIN_1
#define limswitch_z_max_GPIO_Port GPIOE
#define limswitch_z_max_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
