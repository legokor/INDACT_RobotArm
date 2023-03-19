/**
  ******************************************************************************
  * @file     StepperMotorControll.h
  * @author   Erdei Sándor
  * @version  V1.0
  * @date     04/03/2023 20:54:09
  * @brief    The library was created for LEGO Kör\INDACT project.
  *           It handles the three stepper motors of the INDACT robot arm, alias KARCSI.
  ******************************************************************************
*/

#pragma once
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// DEFINES -----------------------------------------------------------------------------//
/*
 * Motor state defines.
 */
#define MOTORSTATE_RUNNING (uint8_t)1u
#define MOTORSTATE_STOPPED (uint8_t)0u

/*
 * On the Z axis the positive direction is downward.
 * On the R axis positive means the tool is going away from the center.
 * On the FI axis positive means the clockwise rotation.
 * Undefined direction means a stop. When the motor has undefined direction it cannot start to move.
 */
#define MOTORDIR_UNDEFINED	(uint8_t)2u
#define MOTORDIR_POSITIVE 	(uint8_t)1u
#define MOTORDIR_NEGATIVE 	(uint8_t)0u

/*
 * StepperMotor.allowedDir value defines which way a motor can move.
 * With these, I can forbid certain movements of the motors when the arm is at a limit of it's coordinate system.
 */
#define MOTORALLOW_BOTHDIR	(uint8_t)3u //0b0000_0011
#define MOTORALLOW_POSDIR	(uint8_t)2u	//0b0000_0010
#define MOTORALLOW_NEGDIR	(uint8_t)1u	//0b0000_0001
#define MOTORALLOW_NODIR	(uint8_t)0u	//0b0000_0000

/*
 * Motor identifiers
 */
#define MOTOR_FI_ID			(uint8_t)0u
#define MOTOR_Z_ID			(uint8_t)1u
#define MOTOR_R_ID			(uint8_t)2u

/*
 * Number of stepper motors on the arm
 */
#define NUMBER_OF_MOTORS	(uint8_t)3u

/*
 * Length of the axis, measured in steps of the stepper motor
 */
#define MOTOR_FI_MAXPOS 	(uint32_t)13874
#define MOTOR_Z_MAXPOS 		(uint32_t)49231
#define MOTOR_R_MAXPOS 		(uint32_t)1048

/*
 * When the arm is controlled via wifi, a command from the web means a certain displacement on the specified axis.
 * The length of the movement defined by these constants.
 */
#define MOTOR_FI_MICROSTEP 	(uint32_t)1387
#define MOTOR_Z_MICROSTEP	(uint32_t)4923
#define MOTOR_R_MICROSTEP 	(uint32_t)104


// ENUMS -------------------------------------------------------------------------------//
/*
 * These are general commands for the robot.
 * The source of these could be a remote controller, a webpage or anything.
 */
typedef enum {
	INVALID = 0,
	Z_UP = 1,
	Z_DOWN = 2,
	R_FORWARD = 3,
	R_BACKWARD = 4,
	FI_COUNTERCLOCKWISE = 5,
	FI_CLOCKWISE = 6,
	HOMING = 7,
	CHANGE_COORDINATES = 8
} MovementCommands;

// STRUCTS -----------------------------------------------------------------------------//
/*
 * Struct for contain GPIO port and pin information in a convinient way.
 */
typedef struct{
	GPIO_TypeDef* GPIO_Port;
	uint16_t GPIO_Pin;
}GPIO_PIN;

/*
 * The arm moves in a cylindrical coordinate system.
 * To define points in this space requires three coordinates:
 *  - angular coordinate - fi
 *  - height - z
 *  - radial distance from origin - r
 */
typedef struct {
	uint32_t fi;
	uint32_t z;
	uint32_t r;
} ToolPosition;


/* motor typedef */
/*
 * Every stepper motor has a struct like this to contain the relevant information bout it in one place.
 *
 */
typedef struct{
	uint8_t ID;
	volatile uint32_t currPos;		//actual
	volatile uint32_t desiredPos;
	volatile uint8_t dir;   		//MOTORDIR_NEGATIVE, MOTORDIR_POSITIVE, MOTORDIR_UNDEFINED
	uint8_t allowedDir;				//MOTORALLOW_BOTHDIR, MOTORALLOW_POSDIR, MOTORALLOW_NEGDIR, MOTORALLOW_NODIR
	uint8_t motorState;     		//MOTORSTATE_RUNNING, MOTORSTATE_STOPPED

	uint32_t TIM_CH;				//TIM_CHANNEL_x
	TIM_HandleTypeDef* TIM; 		//&htimx

	GPIO_TypeDef* enablePORT;
	uint16_t enablePIN;

	GPIO_TypeDef* dirPORT;
	uint16_t dirPIN;
} StepperMotor;


// FUNCTIONS --------------------------------------------------------------------//

/*
 * math.h do not have an abs() function, so I wrote my own.
 */
uint32_t absVal(int32_t number);


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
uint8_t setMotorDir(StepperMotor *stepperMotors, uint8_t motor_id, uint8_t direction);


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
uint8_t setAllMotorDir(StepperMotor *stepperMotors, uint8_t direction);


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
uint8_t setAllMotorDirTowardsDesiredPos(StepperMotor *stepperMotors, ToolPosition curr_pos, ToolPosition next_pos);


/**
 * DO NOT CALL THIS FUNCTION IN YOUR CODE
 */
void should_not_be_called__motorON(StepperMotor *stepperMotors, uint8_t motor_id);


/**
 * DO NOT CALL THIS FUNCTION IN YOUR CODE
 */
void should_not_be_called__allMotorON(StepperMotor *stepperMotors);


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
uint8_t startMotor(StepperMotor *stepperMotors, uint8_t motor_id, uint8_t direction);


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
uint8_t startAllMotor(StepperMotor *stepperMotors, uint8_t direction);

/**
 * Stops the PWM signal of the motor and sets the motor state to MOTORSTATE_STOPPED.
 *
 * INPUT:
 * @param (StepperMotor*) stepperMotors: Pointer to the array, that contains the motors' structs.
 * @param (uint8_t) motor_id: ID of the motor, that you want to manipulate : MOTOR_R/FI/Z_ID
 *
 * OUTPUT: none
 */
void stopMotor(StepperMotor *stepperMotors, uint8_t motor_id);

/**
 * Calls stopMotor() function three times.
 * Stops th PWM signals of the motors and sets the motor statees to MOTORSTATE_STOPPED.
 *
 * INPUT:
 * @param (StepperMotor*) stepperMotors: Pointer to the array, that contains the motors' structs.
 *
 * OUTPUT: none
 */
void stopAllMotor(StepperMotor *stepperMotors);

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
uint8_t controlMotor_viaGPIO (GPIO_PinState* limit_switches, GPIO_TypeDef* positive_button_Port, uint16_t positive_button_Pin,
							  GPIO_TypeDef* negative_button_Port, uint16_t negative_button_Pin, StepperMotor *stepperMotors, uint8_t motor_id);
