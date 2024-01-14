/**
 * @file KAR_GEN_interface.h
 *
 * ""
 *
 * $Author: Erdei Sándor
 * $Date: 2023-07-30
 * $Revision: $
 *
 * @copyright LEGO kör - INDACT project
 */

/*# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
 #																												#
 #		* Detailed description *																				#
 #																												#
 # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # */

#ifndef KAR_GEN_INTERFACE_H
#define KAR_GEN_INTERFACE_H

/****************************************************************************************************************
 * 																												*
 * Includes																										*
 * 																												*
 *****************************************************************************************************************/

#include "stm32f4xx_hal.h"

/****************************************************************************************************************
 * 																												*
 * Public Types																									*
 * 																												*
 *****************************************************************************************************************/

typedef struct
{
	GPIO_TypeDef* GPIO_Port;
	uint16_t GPIO_Pin;
}s_GEN_GPIO;

typedef struct
{
	uint8_t homing_state;
	uint8_t task_id;
	uint8_t error_code;
}s_GEN_ProgramStatus;

/****************************************************************************************************************
 * 																												*
 * Public function definitions																					*
 * 																												*
 *****************************************************************************************************************/

/*
 *===================================================================*
 * Function name: i32_GEN_AbsoluteValue_f
 *-------------------------------------------------------------------
 * Description: Absolute value function for int32_t type
 *-------------------------------------------------------------------
 */
static inline int32_t i32_GEN_AbsoluteValue_f(int32_t number)
{
	return ((number > 0) ? (number) : (-number));
}

#endif		// KAR_GEN_INTERFACE_H

/* End of file KAR_GEN_interface.h */

