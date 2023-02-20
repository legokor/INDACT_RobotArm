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
uint8_t setMotorDirection(const uint8_t ID, const uint8_t desiredDirection)
{
	uint8_t retVal = 0u;
	if(MOTORDIR_TODESIREDPOS == desiredDirection)
	{
		if (stepperMotors[ID].desiredPos > stepperMotors[ID].currPos)
		{
			/* reinvoke the function with new paramters */
			return retVal = setMotorDirection(ID, MOTORDIR_POSITIVE);
		}
		else
		{
			/* reinvoke the function with new paramters */
			return retVal = setMotorDirection(ID, MOTORDIR_NEGATIVE);
		}
	}
	else if (MOTORDIR_POSITIVE == desiredDirection)
	{
		if((stepperMotors[ID].allowedDir & MOTORALLOW_POSDIR) /*>> (MOTORALLOW_POSDIR-1u)*/)
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
		if((stepperMotors[ID].allowedDir & MOTORALLOW_NEGDIR) /*>> (MOTORALLOW_NEGDIR-1u)*/)
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
		/* this one is probably not used */
		stepperMotors[ID].dir = MOTORDIR_UNDEFINED;
		retVal |= (uint8_t)0x03;
	}
	else
	{
		stepperMotors[ID].dir = MOTORDIR_UNDEFINED;
		retVal |= (uint8_t)0x04;
	}

	/* Write the necessary output */
	if (0u == retVal)
	{
		HAL_GPIO_WritePin(stepperMotors[ID].dirPORT, stepperMotors[ID].dirPIN, desiredDirection);
	}

	return retVal;
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
	uint32_t retVal = 0u;

	if( NUMBER_OF_MOTORS == 3u)
	{
		retVal = (setMotorDirection(MOTOR_FI_ID, MOTORDIR_TODESIREDPOS) << 16);
		retVal = (setMotorDirection(MOTOR_Z_ID, MOTORDIR_TODESIREDPOS) << 8);
		retVal = setMotorDirection(MOTOR_R_ID, MOTORDIR_TODESIREDPOS);
	}
	else { retVal = 0xFFFFFFFF; }
	return retVal;
}

/*
 * @brief Check if the current position is the desired position
 * erős TODO
 */
bool posReached(const uint8_t ID)
{
	bool retVal = true;
	if (stepperMotors[ID].currPos != stepperMotors[ID].desiredPos)
	{
		retVal = false;
	}

	return retVal;
}
/*
 * @brief checks if each motor has reached its desired position
 */
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


/*
 * @brief starts the PWM of the motor
 */
void startMotorPWM(const uint8_t ID)
{
	stepperMotors[ID].motorState = MOTORSTATE_RUNNING;
	HAL_TIM_PWM_Start_IT(&htim1, stepperMotors[ID].TIM_CH);
}

/*
 * This function starts the 3 PWM signal generation for the 3 main motors
 */
void startAllMotorPWMs()
{
	uint8_t tempMotorNumber = NUMBER_OF_MOTORS;

	for (uint8_t idx = 0; idx < tempMotorNumber; idx++)
	{
		if (!posReached(idx))
		{
			startMotorPWM(idx);
		}
	}
}

/*
 * TODO RCRValue állítja a timer számláló regiszter maximumát
 * Kéne egy függvény aminek fordulatszámot adok meg, kiszámolja a freki és prescaler alapján
 * a helyes RCRValue-t és felülírja
 */
void changeMotorSpeed(const uint8_t RCRValue)
{
	/* this could be more general, but the 3 motor is never likely to change timer-base
	 * and now they all shave timer1, this function works well like this */
	htim1.Init.RepetitionCounter = RCRValue;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
}



/*
 * @brief This function stops the motor received in the parameter
 */
void stopMotorPWM(const uint8_t ID)
{
	stepperMotors[ID].motorState = MOTORSTATE_STOPPED;
	HAL_TIM_PWM_Stop_IT(&htim1, stepperMotors[ID].TIM_CH);
}


/*
 * @brief This function checks if any of the motors have to be stopped based on the current and desired position
 */
void stopAllMotorBasedPos(StepperMotor* motors)
{
	for (uint8_t idx = 0; idx < NUMBER_OF_MOTORS; idx++)
	{
		if ((MOTORSTATE_RUNNING == motors[idx].motorState) && posReached(idx))
		{
			stopMotorPWM(idx);
		}
	}
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


