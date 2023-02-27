/*
 * motors.c
 *
 *  Created on: 8 Oct 2019
 *      Author: antal
 *      This file handles the THREE stepper motor of the system.
 *      Some functions have guards for the THREE motor. Don't attach more without rewriting the code!
 */


#include "motors.h"
#include "main.h"
#include "math.h"



/* extern global variables */
extern TIM_HandleTypeDef htim1;

/* Private variables */
static StepperMotor stepperMotors[NUMBER_OF_MOTORS]; //MOTOR_ROADIAL_ID, MOTOR_HORIZONTAL_ID, MOTOR_VERTICAL_ID

/* BEGIN static functions */
//static uint8_t maxOfThree(uint8_t value1, uint8_t value2, uint8_t value3);
/* END static functions */


/*
 * @brief Sets the direction of the motor received in @param1
 * to the desiredDirection received in @param2
 * if @param2 is MOTORDIR_TODESIREDPOS that means, we want to set the direction
 * so that the motor goes into the desired position (preset on the motorProperty)
 * before setting the desired direction the function checks if that direction is enabled
 */
uint8_t setMotorDirection(StepperMotor *stepperMotors, uint8_t motor_id, uint8_t direction)
{
	//OK = 0; error = 1;
	uint8_t return_value;

	if(MOTORALLOW_BOTHDIR == stepperMotors[motor_id].allowedDir){
		stepperMotors[motor_id].dir = direction;
		return return_value = 0;
	}
	else if(MOTORALLOW_POSDIR == stepperMotors[motor_id].allowedDir){

	}
	else if(MOTORALLOW_NEGDIR == stepperMotors[motor_id].allowedDir){

	}
	else if(MOTORALLOW_NODIR == stepperMotors[motor_id].allowedDir){
		stepperMotors[motor_id].dir = MOTORDIR_UNDEFINED;
		return return_value = 1;
	}
	else return return_value = 1;


	/*
	uint8_t retVal = 0u;
	if(MOTORDIR_TODESIREDPOS == desiredDirection)
	{
		if (stepperMotors[ID].desiredPos > stepperMotors[ID].currPos)
		{
			// reinvoke the function with new paramters
			return retVal = setMotorDirection(ID, MOTORDIR_POSITIVE);
		}
		else
		{
			// reinvoke the function with new paramters
			return retVal = setMotorDirection(ID, MOTORDIR_NEGATIVE);
		}
	}
	else if (MOTORDIR_POSITIVE == desiredDirection)
	{
		if((stepperMotors[ID].allowedDir & MOTORALLOW_POSDIR))
		{
			stepperMotors[ID].dir = MOTORDIR_POSITIVE;
		}
		else
		{
			stepperMotors[ID].dir = MOTORDIR_UNDEFINED;
			retVal |= (uint8_t)0x01;
		}
	}
	else if (MOTORDIR_NEGATIVE == desiredDirection)
	{
		if((stepperMotors[ID].allowedDir & MOTORALLOW_NEGDIR))
		{
			stepperMotors[ID].dir = MOTORDIR_NEGATIVE;
		}
		else
		{
			stepperMotors[ID].dir = MOTORDIR_UNDEFINED;
			retVal |= (uint8_t)0x02;
		}
	}
	else if (MOTORDIR_UNDEFINED == desiredDirection)
	{
		// this one is probably not used
		stepperMotors[ID].dir = MOTORDIR_UNDEFINED;
		retVal |= (uint8_t)0x03;
	}
	else
	{
		stepperMotors[ID].dir = MOTORDIR_UNDEFINED;
		retVal |= (uint8_t)0x04;
	}
	*/

	// Write the output
	HAL_GPIO_WritePin(stepperMotors[motor_id].dirPORT, stepperMotors[motor_id].dirPIN, direction);

	return return_value;
}


/*
 * Set the motor directions according to the needs
 *	@retVal |=
 *		0b0000 0000(0x00): no error in setting the direction
 *		0b0000 0001(0x01): error in setting the direction @MotorR1
 *		0b0000 0010(0x02): error in setting the direction @MotorPH
 *		0b0000 0100(0x04): error in setting the direction @MotorPV
 */
uint32_t setAllDirectionsTowardsDesiredPos()
{
	/*
	uint32_t retVal = 0u;

	if( NUMBER_OF_MOTORS == 3u)
	{
		retVal = (setMotorDirection(MOTOR_FI_ID, MOTORDIR_TODESIREDPOS) << 16);
		retVal = (setMotorDirection(MOTOR_Z_ID, MOTORDIR_TODESIREDPOS) << 8);
		retVal = setMotorDirection(MOTOR_R_ID, MOTORDIR_TODESIREDPOS);
	}
	else { retVal = 0xFFFFFFFF; }
	return retVal;
	*/
}

/*
 * Nem szükséges szoftveres helyzetlekérdezés
 * Timerekkel és megszakításokkal lesz megoldva
 */
/*
bool posReached(const uint8_t ID)
{
	bool retVal = true;
	if (stepperMotors[ID].currPos != stepperMotors[ID].desiredPos)
	{
		retVal = false;
	}

	return retVal;
}

bool posAllReached()
{
	bool retVal = true;
	uint8_t tempMotorNumbers = NUMBER_OF_MOTORS;

	if (tempMotorNumbers == 3u)
	{
		for(uint8_t idx = 0; idx < tempMotorNumbers; idx++)
		{
			retVal &= posReached(idx);
		}
	}
	else { retVal = false; }
	return retVal;
}
*/


/*
 * Sets the state of the motor to running and starts timer PWM with IT
 */
void startMotor(StepperMotor *stepperMotors, uint8_t motor_id)
{
	stepperMotors[motor_id].motorState = MOTORSTATE_RUNNING;
	HAL_TIM_PWM_Start_IT(stepperMotors[motor_id].TIM, stepperMotors[motor_id].TIM_CH);
	return;
}

/*
 * Check out: startMotor
 */
void startAllMotors(StepperMotor *stepperMotors, uint8_t motor_id)
{
	for (uint8_t cyc_idx = 0; cyc_idx < NUMBER_OF_MOTORS; cyc_idx++)
		startMotor(stepperMotors, cyc_idx);
	return;
}


/*
 * In a nutshell: RCR delays the timer IT.
 * The RC register value does not effect the frequency of the PWM signal.
 * However it delays the interrupt which occurs every time when the timer reaches the counter period.
 * This disables IT ENA, until the timer reaches the counter period RCR times.
 *
 * Example:
 * Timer1.CNT = 10; Timer1.RCR = 3;
 *
 * Let's start to count:
 * 0 . 1 . 2 ... 8 . 9 -> no IT // RCR-1
 * 0 . 1 . 2 ... 8 . 9 -> no IT // RCR-2
 * 0 . 1 . 2 ... 8 . 9 -> no IT // RCR-3
 * 0 . 1 . 2 ... 8 . 9 -> IT starts
 *
 * STM32 Nucleo F334 Reference Manual: page 694
 *
 * TODO: Kéne egy függvény aminek fordulatszámot adok meg, kiszámolja a freki és prescaler alapján
 *       a helyes RCRValue-t és felülírja
 */
void changeMotorSpeed(StepperMotor *stepperMotors, uint8_t motor_id, uint8_t RCRValue)
{
	stepperMotors[motor_id].TIM->Init.RepetitionCounter = RCRValue;
	if (HAL_TIM_Base_Init(stepperMotors[motor_id].TIM) != HAL_OK)
	{
		Error_Handler();
	}
	return;
}


/*
 * Sets the state of the motor to stopped and stops timer PWM
 */
void stopMotor(StepperMotor *stepperMotors, uint8_t motor_id)
{
	stepperMotors[motor_id].motorState = MOTORSTATE_STOPPED;
	HAL_TIM_PWM_Stop_IT(stepperMotors[motor_id].TIM, stepperMotors[motor_id].TIM_CH);
	return;
}


/*
 * Check out: stopMotor.
 */
void stopAllMotors(StepperMotor *stepperMotors, uint8_t motor_id)
{
	for (uint8_t cyc_idx = 0; cyc_idx < NUMBER_OF_MOTORS; cyc_idx++)
		if(MOTORSTATE_RUNNING == stepperMotors[cyc_idx].motorState)
			stopMotor(stepperMotors, cyc_idx);
	return;
}


/*
 * Controls a motor with two GPIO pin.
 * One pin drives the motor to positive direction, the other drives it to the opposite direction.
 *
 */
uint8_t motorControlViaGPIO (GPIO_TypeDef* pos_button_port, uint16_t pos_button_pin, GPIO_TypeDef* neg_button_port, uint16_t neg_button_pin, StepperMotor *stepperMotors, uint8_t motor_id)
{
	//stopped -> 0, started -> 1
	uint8_t return_value;

	GPIO_PinState pos_button = HAL_GPIO_ReadPin(pos_button_port, pos_button_pin);
	GPIO_PinState neg_button = HAL_GPIO_ReadPin(neg_button_port, neg_button_pin);

	if(pos_button && neg_button){
	  HAL_TIM_PWM_Stop_IT(stepperMotors[motor_id].TIM, stepperMotors[motor_id].TIM_CH);
	  return_value = 0;
	}
	else if(pos_button){
	  HAL_GPIO_WritePin(stepperMotors[motor_id].dirPORT, stepperMotors[motor_id].dirPIN, MOTORDIR_POSITIVE);
	  startMotor(stepperMotors, motor_id);
	  //HAL_TIM_PWM_Start_IT(stepperMotors[motor_id].TIM, stepperMotors[motor_id].TIM_CH);
	  return_value = 1;
	}
	else if(neg_button) {
	  HAL_GPIO_WritePin(stepperMotors[motor_id].dirPORT, stepperMotors[motor_id].dirPIN, MOTORDIR_NEGATIVE);
	  startMotor(stepperMotors, motor_id);
	  return_value = 1;
	}
	else {
	  HAL_TIM_PWM_Stop_IT(stepperMotors[motor_id].TIM, stepperMotors[motor_id].TIM_CH);
	  return_value = 0;
	}
	return return_value;
}


/*
 * @Utility function

static uint8_t maxOfThree(uint8_t value1, uint8_t value2, uint8_t value3)
{
	uint8_t max = 0;
	max = (value1 > value2) ? value1 : value2;
	max = (max > value3) ? max : value3;

	return max;
}
*/

/**************************/
/* these are being under consideration to be transferred to another .h.c file


 * @brief Calculates RCR value
 * RCR value could be bigger then uint8 but then it should be handled in other place

uint8_t calcRcrValue(uint8_t *RCRoverflow, StepperMotor* motors)
{
	uint8_t RCRValue;
	//distance between current pos and desired pos - abs value
	uint32_t xTempu32 = abs(motors[MOTOR_RADIAL_ID].desiredPos - motors[MOTOR_RADIAL_ID].currPos);
	uint32_t yTempu32 = abs(motors[MOTOR_HORIZONTAL_ID].desiredPos - motors[MOTOR_HORIZONTAL_ID].currPos);
	uint32_t zTempu32 = abs(motors[MOTOR_VERTICAL_ID].desiredPos - motors[MOTOR_VERTICAL_ID].currPos);

	//count the total steps of the 3 CH in one RCR cycle
	RCRRemainingValue = (uint8_t)xTempu32 + (uint8_t)yTempu32 + (uint8_t)zTempu32;
	//testRCRRemainingValue = (uint8_t)xTempu32 + (uint8_t)yTempu32 + (uint8_t)zTempu32;
	if ((xTempu32 > UINT8T_MAXV) || (yTempu32 > UINT8T_MAXV) || (zTempu32 > UINT8T_MAXV))
	{
		*RCRoverflow = 1;
		RCRValue = 255;
	}
	else
	{
		*RCRoverflow = 0;
		RCRValue = maxOfThree((uint8_t)xTempu32, (uint8_t)yTempu32, (uint8_t)zTempu32);
	}

	return RCRValue;
}
*/

/*
 * @brief increments the motor's current position to the corresponding direction
void incrementSTMotorPos(const uint8_t ID)
{
	uint32_t tCurrPos = getSTMotorCurrPos(ID);
	uint8_t tDir = getSTMotorDir(ID);
	tCurrPos = (tDir == MOTORDIR_POSITIVE) ? (tCurrPos + 1) : (tCurrPos - 1);
	setSTMotorCurrPos(ID, tCurrPos);
}
 */


