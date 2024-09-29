/**
 * @file KAR_MC_handler.c
 *
 * ""
 *
 * $Author: Erdei Sándor
 * $Date: 2023-07-8
 * $Revision: $
 *
 * @copyright: LEGO kör
 */

/*# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
 #																												#
 #		Detailed description																					#
 #																												#
 # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # */

/****************************************************************************************************************
 * 																												*
 * Includes																										*
 * 																												*
 *****************************************************************************************************************/

#include "KAR_MC_handler.h"
#include "KAR_GEO_interface.h"

/****************************************************************************************************************
 * 																												*
 * Local function definitions																					*
 * 																												*
 *****************************************************************************************************************/

/*
 *===================================================================*
 * Function name: u8_MC_SetMotorDir_f
 *-------------------------------------------------------------------
 * Description:
 * Set manually the direction of a motor.
 * Overwrites stepper_motors[motor_id].dir value and sets the GPIO pin of the motor driver's DIR input.
 * If allowedDir prohibits a direction, output value will be e_MC_ErrorCode_FrobiddenState and stepper_motors[motor_id].dir won't be written.
 *
 * INPUT:
 * @param (s_MC_StepperMotor*) stepper_motors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate : MC_MOTORID_FI/Z/R
 * @param (uint8_t) direction: the direction define you want to set : MC_DIR_UNDEFINED/NEGATIVE/POSITIVE
 *
 * OUTPUT:
 * @retval (e_MC_ErrorCode_t) errorCode:
 *-------------------------------------------------------------------
 */
e_MC_ErrorCode_t u8_MC_SetMotorDir_f(s_MC_StepperMotor *stepper_motors, uint8_t motor_id, uint8_t direction)
{
	uint8_t dir;
	e_MC_ErrorCode_t errorCode = e_MC_ErrorCode_OK;

	if ((MC_NUMBER_OF_MOTORS - 1) < motor_id)
	{
		return e_MC_ErrorCode_WrongParameter;
	}

	switch(direction) {
		case MC_DIR_UNDEFINED:
			dir = MC_DIR_UNDEFINED;
			break;
		case MC_DIR_POSITIVE:
			if (stepper_motors[motor_id].allowedDir & (uint8_t)(2u))
			{
				dir = MC_DIR_POSITIVE;
			}
			else
			{
				errorCode = e_MC_ErrorCode_FrobiddenState;
			}
			break;
		case MC_DIR_NEGATIVE:
			if (stepper_motors[motor_id].allowedDir & (uint8_t)(1u))
			{
				dir = MC_DIR_NEGATIVE;
			}
			else
			{
				errorCode = e_MC_ErrorCode_FrobiddenState;
			}
			break;
		default:
			errorCode = e_MC_ErrorCode_WrongParameter;
			break;
	}

	if (!errorCode)
	{
		stepper_motors[motor_id].dir = dir;
		HAL_GPIO_WritePin(stepper_motors[motor_id].DIR.GPIO_Port, stepper_motors[motor_id].DIR.GPIO_Pin, stepper_motors[motor_id].dir);
	}

	return errorCode;
}

/*
 *===================================================================*
 * Function name: u8_MC_setAllMotorDir_f
 *-------------------------------------------------------------------
 * Description:
 * Set all of the motors' dir to the given direction. It calls u8_MC_SetMotorDir_f().
 *
 * INPUT:
 * @param (s_MC_StepperMotor*) stepper_motors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) direction: the direction define you want to set : U8_MC_DIR_UNDEFINED/NEGATIVE/POSITIVE
 *
 * OUTPUT:
 * @retval (e_MC_ErrorCode_t) errorCode:
 *-------------------------------------------------------------------
 */
e_MC_ErrorCode_t u8_MC_setAllMotorDir_f(s_MC_StepperMotor *stepper_motors, uint8_t direction)
{
	e_MC_ErrorCode_t errorCode = e_MC_ErrorCode_OK;

	for(uint8_t idx = 0; idx < MC_NUMBER_OF_MOTORS; idx++)
	{
		errorCode = u8_MC_SetMotorDir_f(stepper_motors, idx, direction);
		if (errorCode)
		{
			break;
		}
	}

	return errorCode;
}

/*
 *===================================================================*
 * Function name: u8_MC_MotorON_f
 *-------------------------------------------------------------------
 * Description:
 * Starts timer PWM signal and sets the motor state to U8_MC_STATE_RUNNING.
 *
 * INPUT:
 * @param (s_MC_StepperMotor*) stepper_motors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate : U8_MC_MOTORID_FI/Z/R
 *
 * OUTPUT:
 * @retval (e_MC_ErrorCode_t) errorCode:
 *-------------------------------------------------------------------
 */
e_MC_ErrorCode_t u8_MC_MotorON_f(s_MC_StepperMotor *stepper_motors, uint8_t motor_id)
{
	if ((MC_NUMBER_OF_MOTORS-1) < motor_id)
	{
		return e_MC_ErrorCode_WrongParameter;
	}

	if (MC_DIR_UNDEFINED != stepper_motors[motor_id].dir)
	{
		HAL_TIM_PWM_Start_IT(stepper_motors[motor_id].TIM, stepper_motors[motor_id].TIM_CH);
		stepper_motors[motor_id].motorState = MC_STATE_RUNNING;
	}
	else
	{
		return e_MC_ErrorCode_FrobiddenState;
	}
	return e_MC_ErrorCode_OK;
}


/*
 *===================================================================*
 * Function name: u8_MC_AllMotorON_f
 *-------------------------------------------------------------------
 * Description:
 * Starts the three motors. It calls u8_MC_MotorON_f().
 *
 * INPUT:
 * @param (s_MC_StepperMotor*) stepper_motors: Pointer to the array, that contains the motors' structs.
 *
 * OUTPUT:
 * @retval (e_MC_ErrorCode_t) errorCode:
 *-------------------------------------------------------------------
 */
e_MC_ErrorCode_t u8_MC_AllMotorON_f(s_MC_StepperMotor *stepper_motors)
{
	e_MC_ErrorCode_t errorCode = e_MC_ErrorCode_OK;

	for (uint8_t idx = 0; idx < MC_NUMBER_OF_MOTORS; idx++)
	{
		errorCode = u8_MC_MotorON_f(stepper_motors, idx);
		if (errorCode)
		{
			break;
		}
	}
	return errorCode;
}

/****************************************************************************************************************
 * 																												*
 * Global function definitions																					*
 * 																												*
 *****************************************************************************************************************/

/*
 *===================================================================*
 * Function name: u8_MC_setAllMotorDir_TowardsDesiredPos_f
 *-------------------------------------------------------------------
 * Description:
 * Set the motor directions according to the next position where the tool will go.
 * It calculates which way the motor has to rotate and calls u8_MC_SetMotorDir_f() function
 * three times to set the calculated directions to the motors.
 *
 * INPUT:
 * @param (s_MC_StepperMotor*) stepper_motors: Pointer to the array, that contains the motors' structs.
 *
 * OUTPUT:
 * @retval (e_MC_ErrorCode_t) errorCode:
 *-------------------------------------------------------------------
 */
e_MC_ErrorCode_t u8_MC_setAllMotorDir_TowardsDesiredPos_f(s_MC_StepperMotor *stepper_motors)
{
	e_MC_ErrorCode_t errorCode = e_MC_ErrorCode_OK;

	int32_t differences[MC_NUMBER_OF_MOTORS];
	differences[0] = (stepper_motors[MC_MOTORID_PHI].nextPos - stepper_motors[MC_MOTORID_PHI].currPos);
	differences[1] = (stepper_motors[MC_MOTORID_Z].nextPos - stepper_motors[MC_MOTORID_Z].currPos);
	differences[2] = (stepper_motors[MC_MOTORID_R].nextPos - stepper_motors[MC_MOTORID_R].currPos);

	for(uint8_t idx = 0; idx < MC_NUMBER_OF_MOTORS; idx++)
	{
		if(0 == differences[idx])
		{
			errorCode = u8_MC_SetMotorDir_f(stepper_motors, idx, MC_DIR_UNDEFINED);
		}
		else if (0 > differences[idx])
		{
			errorCode = u8_MC_SetMotorDir_f(stepper_motors, idx, MC_DIR_NEGATIVE);
		}
		else
		{
			errorCode = u8_MC_SetMotorDir_f(stepper_motors, idx, MC_DIR_POSITIVE);
		}

		if (errorCode)
		{
			break;
		}
	}

	return errorCode;
}

/*
 *===================================================================*
 * Function name: u8_MC_StartMotor_f
 *-------------------------------------------------------------------
 * Description:
 * Starts to rotate the motor in the given direction.
 *
 * INPUT:
 * @param (s_MC_StepperMotor*) stepper_motors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate : U8_MC_MOTORID_FI/Z/R
 * @param (uint8_t) direction: the direction define you want to set : U8_MC_DIR_UNDEFINED/NEGATIVE/POSITIVE
 *
 * OUTPUT:
 * @retval (e_MC_ErrorCode_t) errorCode:
 *-------------------------------------------------------------------
 */
inline e_MC_ErrorCode_t u8_MC_StartMotor_f(s_MC_StepperMotor *stepper_motors, uint8_t motor_id, uint8_t direction)
{
	e_MC_ErrorCode_t errorCode = e_MC_ErrorCode_OK;

	errorCode = u8_MC_SetMotorDir_f(stepper_motors, motor_id, direction);

	if (!errorCode){
		errorCode = u8_MC_MotorON_f(stepper_motors, motor_id);
	}

	return errorCode;
}

/*
 *===================================================================*
 * Function name: u8_MC_StartAllMotor_f
 *-------------------------------------------------------------------
 * Description:
 * Starts to rotate all of the motors in the given direction.
 *
 * INPUT:
 * @param (s_MC_StepperMotor*) stepper_motors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate : U8_MC_MOTORID_FI/Z/R
 * @param (uint8_t) direction: the direction define you want to set : U8_MC_DIR_UNDEFINED/NEGATIVE/POSITIVE
 *
 * OUTPUT:
 * @retval (e_MC_ErrorCode_t) errorCode:
 *-------------------------------------------------------------------
 */
inline e_MC_ErrorCode_t u8_MC_StartAllMotor_f(s_MC_StepperMotor *stepper_motors, uint8_t direction)
{
	e_MC_ErrorCode_t errorCode = e_MC_ErrorCode_OK;

	for(uint8_t idx = 0; idx < MC_NUMBER_OF_MOTORS; idx++)
	{
		errorCode = u8_MC_StartMotor_f(stepper_motors, idx, direction);
		if (errorCode)
		{
			break;
		}
	}

	return errorCode;
}

/*
 *===================================================================*
 * Function name: v_MC_StopMotor_f
 *-------------------------------------------------------------------
 * Description:
 * Stops the PWM signal of the motor and sets the motor state to U8_MC_STATE_STOPPED.
 *
 * INPUT:
 * @param (s_MC_StepperMotor*) stepper_motors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate : U8_MC_MOTORID_FI/Z/R
 *
 * OUTPUT: none
 *-------------------------------------------------------------------
 */
inline void v_MC_StopMotor_f(s_MC_StepperMotor *stepper_motors, uint8_t motor_id)
{
	HAL_TIM_PWM_Stop_IT(stepper_motors[motor_id].TIM, stepper_motors[motor_id].TIM_CH);
	stepper_motors[motor_id].motorState = MC_STATE_STOPPED;
}

/*
 *===================================================================*
 * Function name: v_MC_StopAllMotor_f
 *-------------------------------------------------------------------
 * Description:
 * Stops the PWM signal of the three motors and sets the motors' state to U8_MC_STATE_STOPPED.
 *
 * INPUT:
 * @param (s_MC_StepperMotor*) stepper_motors: Pointer to the array, that contains the motors' structs.
 *
 * OUTPUT: none
 *-------------------------------------------------------------------
 */
inline void v_MC_StopAllMotor_f(s_MC_StepperMotor *stepperMotors)
{
	v_MC_StopMotor_f(stepperMotors, MC_MOTORID_PHI);
	v_MC_StopMotor_f(stepperMotors, MC_MOTORID_Z);
	v_MC_StopMotor_f(stepperMotors, MC_MOTORID_R);
}

/*
 *===================================================================*
 * Function name: u8_MC_ControlMotor_viaGPIO_f
 *-------------------------------------------------------------------
 * Description:
 * Controls a motor with two GPIO pin.
 *
 * INPUT:
 * @param (s_MC_StepperMotor*) stepper_motors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate : U8_MC_MOTORID_FI/Z/R
 * @param (s_GEO_LimitSwitch_Cylinder*) limit_switches: Array of limit switch flags. GPIO IT handler sets these flags whenever a limit switch turns on/off.
 * @param (s_MH_GPIO) positive_button:
 * @param (s_MH_GPIO) negative_button:
 *
 * OUTPUT:
 * @retval (e_MC_ErrorCode_t) errorCode:
 *-------------------------------------------------------------------
 */
e_MC_ErrorCode_t u8_MC_ControlMotor_viaGPIO_f (s_MC_StepperMotor *stepper_motors, uint8_t motor_id, s_GEO_LimitSwitch* limit_switches,
											   s_GEN_GPIO positive_button, s_GEN_GPIO negative_button)
{
	e_MC_ErrorCode_t errorCode = e_MC_ErrorCode_OK;

	GPIO_PinState pos_button = HAL_GPIO_ReadPin(positive_button.GPIO_Port, positive_button.GPIO_Pin);
	GPIO_PinState neg_button = HAL_GPIO_ReadPin(negative_button.GPIO_Port, negative_button.GPIO_Pin);

	if (pos_button && neg_button)
	{
		v_MC_StopMotor_f(stepper_motors, motor_id);
	}
	else if (pos_button)
	{
		if (!limit_switches[motor_id].max_point)
		{
			errorCode = u8_MC_StartMotor_f(stepper_motors, motor_id, MC_DIR_POSITIVE);
		}
		else
		{
			v_MC_StopMotor_f(stepper_motors, motor_id);
		}
	}
	else if (neg_button)
	{
		if (!limit_switches[motor_id].null_point)
		{
			errorCode = u8_MC_StartMotor_f(stepper_motors, motor_id, MC_DIR_NEGATIVE);
		}
		else
		{
			v_MC_StopMotor_f(stepper_motors, motor_id);
		}
	}
	else
	{
		v_MC_StopMotor_f(stepper_motors, motor_id);
	}

	return errorCode;
}



e_MC_ErrorCode_t u8_MC_HandlePS2Dir_f (s_MC_StepperMotor *stepper_motors, uint8_t motor_id, s_GEO_LimitSwitch* limit_switches,
											   uint8_t pos_button_stat, uint8_t neg_button_stat)
{
	e_MC_ErrorCode_t errorCode = e_MC_ErrorCode_OK;

	if (pos_button_stat && neg_button_stat)
	{
		v_MC_StopMotor_f(stepper_motors, motor_id);
	}
	else if (pos_button_stat)
	{
		if (!limit_switches[motor_id].max_point)
		{
			errorCode = u8_MC_StartMotor_f(stepper_motors, motor_id, MC_DIR_POSITIVE);
		}
		else
		{
			v_MC_StopMotor_f(stepper_motors, motor_id);
		}
	}
	else if (neg_button_stat)
	{
		if (!limit_switches[motor_id].null_point)
		{
			errorCode = u8_MC_StartMotor_f(stepper_motors, motor_id, MC_DIR_NEGATIVE);
		}
		else
		{
			v_MC_StopMotor_f(stepper_motors, motor_id);
		}
	}
	else
	{
		v_MC_StopMotor_f(stepper_motors, motor_id);
	}

	return errorCode;
}

/* End of file KAR_motor_handler.c */
