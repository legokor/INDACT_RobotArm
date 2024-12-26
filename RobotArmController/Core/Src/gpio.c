/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA0/WKUP   ------> S_TIM2_CH1_ETR
     PA3   ------> S_TIM2_CH4
     PB10   ------> S_TIM2_CH3
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Wifi_CH_PD_Pin|Wifi_RST_Pin|Z_Dir_Pin|Z_En_Pin
                          |R_Dir_Pin|R_En_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_D_C_Pin|LCD_R_W_Pin|LCD_EN_Pin|LCD_D0_Pin
                          |LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|Phi_Dir_Pin|Phi_En_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6|Theta_Dir_Pin|Theta_En_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Wifi_CH_PD_Pin */
  GPIO_InitStruct.Pin = Wifi_CH_PD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Wifi_CH_PD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Wifi_RST_Pin Z_Dir_Pin Z_En_Pin R_Dir_Pin
                           R_En_Pin */
  GPIO_InitStruct.Pin = Wifi_RST_Pin|Z_Dir_Pin|Z_En_Pin|R_Dir_Pin
                          |R_En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D_C_Pin LCD_R_W_Pin LCD_D0_Pin LCD_D1_Pin
                           LCD_D2_Pin LCD_D3_Pin LCD_D4_Pin LCD_D5_Pin
                           LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_D_C_Pin|LCD_R_W_Pin|LCD_D0_Pin|LCD_D1_Pin
                          |LCD_D2_Pin|LCD_D3_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_EN_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Theta_button_CCW_Pin Theta_button_CW_Pin Servo1_button_curl_Pin Servo1_button_straighten_Pin */
  GPIO_InitStruct.Pin = Theta_button_CCW_Pin|Theta_button_CW_Pin|Servo1_button_curl_Pin|Servo1_button_straighten_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Servo1_PWM_Pin Servo2_PWM_Pin */
  GPIO_InitStruct.Pin = Servo1_PWM_Pin|Servo2_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Phi_button_CCW_Pin Phi_button_CW_Pin */
  GPIO_InitStruct.Pin = Phi_button_CCW_Pin|Phi_button_CW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin Phi_Dir_Pin Phi_En_Pin
                           LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|Phi_Dir_Pin|Phi_En_Pin
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Z_button_up_Pin Z_button_down_Pin R_button_out_Pin R_button_in_Pin */
  GPIO_InitStruct.Pin = Z_button_up_Pin|Z_button_down_Pin|R_button_out_Pin|R_button_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Door_SW_Pin Z_Pos_gate_1_Pin Z_Pos_gate_2_Pin R_Pos_gate_1_Pin
                           R_pos_gate_2_Pin */
  GPIO_InitStruct.Pin = Door_SW_Pin|Z_Pos_gate_1_Pin|Z_Pos_gate_2_Pin|R_Pos_gate_1_Pin
                          |R_pos_gate_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : Servo3_PWM_Pin */
  GPIO_InitStruct.Pin = Servo3_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(Servo3_PWM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Servo2_button_curl_Pin Servo2_button_straighten_Pin Servo3_button_curl_Pin Servo3_button_straighten_Pin */
  GPIO_InitStruct.Pin = Servo2_button_curl_Pin|Servo2_button_straighten_Pin|Servo3_button_curl_Pin|Servo3_button_straighten_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG6 Theta_Dir_Pin Theta_En_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|Theta_Dir_Pin|Theta_En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Phi_Limit_sw_1_Pin Phi_Limit_sw_2_Pin Z_Limit_sw_1_Pin Z_Limit_sw_2_Pin
                           R_Limit_sw_1_Pin R_Limit_sw_2_Pin */
  GPIO_InitStruct.Pin = Phi_Limit_sw_1_Pin|Phi_Limit_sw_2_Pin|Z_Limit_sw_1_Pin|Z_Limit_sw_2_Pin
                          |R_Limit_sw_1_Pin|R_Limit_sw_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
