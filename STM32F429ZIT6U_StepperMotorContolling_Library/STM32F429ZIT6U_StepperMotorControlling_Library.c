/**
  ******************************************************************************
  * @file     STM32F429ZIT6U_StepperMotorControlling_Library.c
  * @author   Erdei Sándor
  * @version  V1.0
  * @date     04/03/2023 20:54:09
  * @brief    The library was created for LEGO Kör\INDACT project.
  *           It handles the three stepper motors of the INDACT robot arm, alias KARCSI.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "StepperMotorControll.h"
/** Functions ----------------------------------------------------------------*/
/*
 * math.h do not have an abs() function, so I wrote my own.
 */
uint32_t absVal(int32_t number){
	if(number < 0) return -(number);
	else return number;
}


/**
 * Set manually the direction of a motor.
 * Overwrites stepperMotors[ID].dir value and sets the GPIO pin of the motor driver's DIR input.
 * If allowedDir prohibits a direction, output value will be 1 and dir wont be overwritten.
 *
 * INPUT:
 * @param (StepperMotor*) stepperMotors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate : MOTOR_R/FI/Z_ID
 * @param (uint8_t) direction: the direction define you want to set : MOTORDIR_POSITIVE/NEGATIVE/UNDEFINED
 *
 * OUTPUT:
 * @retval (uint8_t): ZERO: no error has occourd
 *                    ONE: the given direction is prohibited for the motor at the moment
 */
uint8_t setMotorDir(StepperMotor *stepperMotors, uint8_t motor_id, uint8_t direction)
{
	uint8_t return_value;

	if(MOTORDIR_UNDEFINED == direction){
		stepperMotors[motor_id].dir = MOTORDIR_UNDEFINED;
		return_value = 0;
	}
	else if(MOTORDIR_POSITIVE == direction || MOTORDIR_NEGATIVE == direction){
		switch (stepperMotors[motor_id].allowedDir) {
			case MOTORALLOW_BOTHDIR: {
				stepperMotors[motor_id].dir = direction;
				return_value = 0;
				break;
			}
			case MOTORALLOW_POSDIR: {
				if(MOTORDIR_POSITIVE == direction){
					stepperMotors[motor_id].dir = MOTORDIR_POSITIVE;
					return_value = 0;
				}
				else return_value = 1;
				break;
			}
			case MOTORALLOW_NEGDIR: {
				if(MOTORDIR_NEGATIVE == direction){
					stepperMotors[motor_id].dir = MOTORDIR_NEGATIVE;
					return_value = 0;
				}
				else return_value = 1;
				break;
			}
			default: {
				return_value = 1;
				break;
			}
		}
	}
	else return_value = 1;

	HAL_GPIO_WritePin(stepperMotors[motor_id].dirPORT, stepperMotors[motor_id].dirPIN, stepperMotors[motor_id].dir);

	return return_value;
}


/**
 * Set all of the motors' direction to the given direction. It calls setMotorDir() function three times.
 *
 * INPUT:
 * @param (StepperMotor*) stepperMotors: Pointer to the array, that contains the motors' structs.
 * @param  (uint8_t) direction: the direction define you want to set : MOTORDIR_POSITIVE/NEGATIVE/UNDEFINED
 *
 * OUTPUT:
 * @retval (uint8_t): ZERO: no error has occourd
 *                    b0000 1000: the FI axis' direction has not been set correctly.
 *                    b0000 0100: the Z axis' direction has not been set correctly.
 *                    b0000 0010: the R axis' direction has not been set correctly.
 * If multiple errors happened, the return value contains all of them: for example the return value b0000 1010 means
 * that two errors happened - one while setting FI motor and one while setting R motor.
 */
uint8_t setAllMotorDir(StepperMotor *stepperMotors, uint8_t direction){
	uint8_t return_value;

	for(uint8_t idx = 0; idx < NUMBER_OF_MOTORS; idx++){
		return_value = setMotorDir(stepperMotors, idx, direction);
		return_value = return_value << 1;
	}

	return return_value;
}


/**
 * Set the motor directions according to the next position where the tool will go.
 * It calculates from @param curr_pos and @param next_pos which way the motor has to rotate and
 * calls setMotorDir() function three times to set the calculated directions to the motors.
 *
 * INPUT:
 * @param (StepperMotor*) stepperMotors: Pointer to the array, that contains the motors' structs.
 * @param (ToolPosition) curr_pos: structure of the current position of the arm
 * @param (ToolPosition) next_pos: structure of the position we want the arm to go to
 *
 * OUTPUT:
 * @retval (uint8_t): ZERO: no error has occourd
 *                    b0000 1000: the FI axis' direction has not been set correctly.
 *                    b0000 0100: the Z axis' direction has not been set correctly.
 *                    b0000 0010: the R axis' direction has not been set correctly.
 * If multiple errors happened, the return value contains all of them: for example the return value b0000 1010 means
 * that two errors happened - one while setting FI motor and one while setting R motor.
 */
uint8_t setAllMotorDirTowardsDesiredPos(StepperMotor *stepperMotors, ToolPosition curr_pos, ToolPosition next_pos)
{
	uint8_t return_value;

	int32_t differences[3];
	differences[0] = (int32_t)(next_pos.fi - curr_pos.fi);
	differences[1] = (int32_t)(next_pos.z - curr_pos.z);
	differences[2] = (int32_t)(next_pos.r - curr_pos.r);

	for(uint8_t idx = 0; idx < 3; idx++){
		if(0 == differences[idx]){
			return_value = setMotorDir(stepperMotors, idx, MOTORDIR_UNDEFINED);
		}
		else if(0 > differences[idx]){
			return_value = setMotorDir(stepperMotors, idx, MOTORDIR_NEGATIVE);
		}
		else{
			return_value = setMotorDir(stepperMotors, idx, MOTORDIR_POSITIVE);
		}

		return_value = return_value << 1;
	}
	return return_value;
}


/**
 * DO NOT CALL THIS FUNCTION IN YOUR CODE
 */
void should_not_be_called__motorON(StepperMotor *stepperMotors, uint8_t motor_id)
{
	if((MOTORDIR_POSITIVE == stepperMotors[motor_id].dir) || (MOTORDIR_NEGATIVE == stepperMotors[motor_id].dir)){
		HAL_TIM_PWM_Start_IT(stepperMotors[motor_id].TIM, stepperMotors[motor_id].TIM_CH);
		stepperMotors[motor_id].motorState = MOTORSTATE_RUNNING;
	}
	return;
}


/**
 * DO NOT CALL THIS FUNCTION IN YOUR CODE
 */
#define motorON should_not_be_called__motorON
void should_not_be_called__allMotorON(StepperMotor *stepperMotors)
{
	for (uint8_t idx = 0; idx < NUMBER_OF_MOTORS; idx++)
		motorON(stepperMotors, idx);
	return;
}
#undef motorON


/**
 * Sets the direction of the motor: If it's succesful, then it sets the GPIO pin of the motor controller DIR signal,
 * sets the motor status to MOTORSTATE_RUNNING and starts the motor, if it is not succesful then returns with error
 * and the motor stays still.
 *
 * INPUT:
 * @param (StepperMotor*) stepperMotors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate : MOTOR_R/FI/Z_ID
 * @param (uint8_t) direction: the direction define you want to set : MOTORDIR_POSITIVE/NEGATIVE/UNDEFINED
 *
 * OUTPUT:
 * @retval (uint8_t): ZERO: no error has occourd
 *                    ONE: the given direction is prohibited for the motor at the moment
 *                         or you have wanted to set MOTORDIR_UNDEFINED direction.
 */
#define motorON should_not_be_called__motorON
uint8_t startMotor(StepperMotor *stepperMotors, uint8_t motor_id, uint8_t direction){
	uint8_t return_value;

	uint8_t setdir_error = setMotorDir(stepperMotors, motor_id, direction);

	if(setdir_error || (MOTORDIR_UNDEFINED == direction)){
		return_value = 1;
	}
	else{
		motorON(stepperMotors, motor_id);
		return_value = 0;
	}

	return return_value;
}
#undef motorON


/**
 * Calls startMotor() function three times.
 * Sets the directions of the motors: If they are succesful, then it sets the GPIO pins of the motor controllers DIR signals,
 * sets the motor statuses to MOTORSTATE_RUNNING and starts the motors.
 * If any of the direction settings are not succesful then returns with the error byte and all of the motors stays still.
 *
 * INPUT:
 * @param (StepperMotor*) stepperMotors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) direction: the direction define you want to set : MOTORDIR_POSITIVE/NEGATIVE/UNDEFINED
 *
 * OUTPUT:
 * @retval (uint8_t): ZERO: no error has occourd
 *                    b0000 1000: the given direction is prohibited for the FI motor at the moment
 *                                or you have wanted to set MOTORDIR_UNDEFINED direction.
 *                    b0000 0100: the given direction is prohibited for the Z motor at the moment
 *                                or you have wanted to set MOTORDIR_UNDEFINED direction.
 *                    b0000 0010: the given direction is prohibited for the R motor at the moment
 *                                or you have wanted to set MOTORDIR_UNDEFINED direction.
 * If multiple errors happened, the return value contains all of them: for example the return value b0000 1010 means
 * that two errors happened - one while setting FI motor and one while setting R motor.
 */
uint8_t startAllMotor(StepperMotor *stepperMotors, uint8_t direction){
	uint8_t return_value, start_error;

	for(uint8_t idx = 0; idx < NUMBER_OF_MOTORS; idx++){
		start_error = startMotor(stepperMotors, idx, direction);
		start_error = start_error << 1;
	}

	if(start_error){
		return_value = start_error;
	}
	else{
		return_value = 0;
	}

	return return_value;
}


/**
 * Stops the PWM signal of the motor and sets the motor state to MOTORSTATE_STOPPED.
 *
 * INPUT:
 * @param (StepperMotor*) stepperMotors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate : MOTOR_R/FI/Z_ID
 *
 * OUTPUT: none
 */
void stopMotor(StepperMotor *stepperMotors, uint8_t motor_id)
{
	HAL_TIM_PWM_Stop_IT(stepperMotors[motor_id].TIM, stepperMotors[motor_id].TIM_CH);
	stepperMotors[motor_id].motorState = MOTORSTATE_STOPPED;
	return;
}


/**
 * Calls stopMotor() function three times.
 * Stops th PWM signals of the motors and sets the motor statees to MOTORSTATE_STOPPED.
 *
 * INPUT:
 * @param (StepperMotor*) stepperMotors: Pointer to the array, that contains the motors' structs.
 *
 * OUTPUT: none
 */
void stopAllMotor(StepperMotor *stepperMotors)
{
	for (uint8_t idx = 0; idx < NUMBER_OF_MOTORS; idx++)
		if(MOTORSTATE_RUNNING == stepperMotors[idx].motorState)
			stopMotor(stepperMotors, idx);
	return;
}


/**
 * Controls a motor with two GPIO pin: One pin drives the motor to positive direction, the other drives it to the opposite direction.
 *
 * INPUT:
 * @param (GPIO_PinState*) limit_switches: Array of limit switch flags. GPIO IT handler sets these flags whenever a limit switch turns on/off.
 * @param (GPIO_PIN) positive_button: Struct of the button, which drives the motor to the positive direction.
 * @param (GPIO_PIN) negative_button: Struct of the button, which drives the motor to the negative direction.
 * @param (StepperMotor*) stepperMotors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate with these buttons : MOTOR_R/FI/Z_ID
 *
 * OUTPUT:
 * @retval (uint8_t): ZERO: The function stopped the motor (pressing both positive and negative button or none of them)
 *                          or cannot start it (limit switch is on or some direction is forbidden).
 *                    ONE: The function started the motor.
 */
uint8_t controlMotor_viaGPIO (GPIO_PinState* limit_switches, GPIO_TypeDef* positive_button_Port, uint16_t positive_button_Pin, GPIO_TypeDef* negative_button_Port, uint16_t negative_button_Pin, StepperMotor *stepperMotors, uint8_t motor_id)
{
	//stopped: return_value == 0, started: return_value == 1
	uint8_t return_value;

	GPIO_PinState pos_button = HAL_GPIO_ReadPin(positive_button_Port, positive_button_Pin);
	GPIO_PinState neg_button = HAL_GPIO_ReadPin(negative_button_Port, negative_button_Pin);

	if(pos_button && neg_button){
	  stopMotor(stepperMotors, motor_id);
	  return_value = 0;
	}
	else if(pos_button){
		if(!limit_switches[((motor_id+1)*2)-1]){
			if(!startMotor(stepperMotors, motor_id, MOTORDIR_POSITIVE))
				return_value = 1;
			else return_value = 0;
		}
		else{
			stopMotor(stepperMotors, motor_id);
			return_value = 0;
		}
	}
	else if(neg_button){
		if(!limit_switches[(motor_id+1)*2]){
			if(!startMotor(stepperMotors, motor_id, MOTORDIR_NEGATIVE))
				return_value = 1;
			else return_value = 0;
		}
		else{
			stopMotor(stepperMotors, motor_id);
			return_value = 0;
		}
	}
	else {
	  stopMotor(stepperMotors, motor_id);
	  return_value = 0;
	}
	return return_value;
}
