/*
 *  Created on: 04.03.2023
 *  Author: Erdei Sandor
 *
 *  This file handles the THREE stepper motor of the INDACT robot arm, alias KARCSI.
 */


#pragma once
#include <stdio.h>
#include <stdbool.h>

/*
 * motor state defines
 * These will be useful for forbid movement by incident.
 */
#define MOTORSTATE_RUNNING (uint8_t)1u
#define MOTORSTATE_STOPPED (uint8_t)0u

/*
 * On the Z axis the positive direction is upward.
 * On the R axis positive means the direction of the tool.
 * On the FI axis positive means the counter-clockwise rotation.
 *
 * Undefined direction represents a stop or a non-crucial error state,
 * which occurs from software regulations.
 */
#define MOTORDIR_UNDEFINED	(uint8_t)2u
#define MOTORDIR_POSITIVE 	(uint8_t)1u
#define MOTORDIR_NEGATIVE 	(uint8_t)0u

/*
 * Software regulations is required for the correct operation.
 * These can prohibit movement when the tool is at a limit
 * or when an error occurs.
 */
#define MOTORALLOW_BOTHDIR	(uint8_t)3u //0b0000_0011
#define MOTORALLOW_POSDIR	(uint8_t)2u	//0b0000_0010
#define MOTORALLOW_NEGDIR	(uint8_t)1u	//0b0000_0001
#define MOTORALLOW_NODIR	(uint8_t)0u	//0b0000_0000

/*
 * Constants for identify the motors
 */
#define MOTOR_FI_ID			(uint8_t)0u
#define MOTOR_Z_ID			(uint8_t)1u
#define MOTOR_R_ID			(uint8_t)2u
#define NUMBER_OF_MOTORS	(uint8_t)3u


/*
 * These are general commands for the robot.
 * The source of these could be a remote controller, a webpage or anything.
 */
typedef enum {
	Z_STOP = 0,
	Z_UP = 1,
	Z_DOWN = 2,

	R_STOP = 3,
	R_FORWARD = 4,
	R_BACKWARD = 5,

	FI_STOP = 6,
	FI_COUNTERCLOCKWISE = 7,
	FI_CLOCKWISE = 8
} MovementCommands;


/*
 * Position of the tool.
 * The arm will be positioned by this struct.
 */
typedef struct {
	uint32_t fi;
	uint32_t z;
	uint32_t r;
} ToolPosition;


/* motor typedef */
typedef struct{
	uint8_t ID;
	volatile uint32_t currPos;
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


//Function declarations
/*
 * Set manually the direction of a motor.
 * If allowedDir prohibits a direction, output value will be 1 and dir wont be overwritten.
 * OK: retval = 0; ERROR: retval = 1;
 */
uint8_t setMotorDir(StepperMotor *stepperMotors, uint8_t motor_id, uint8_t direction);

/*
 * Set the motor directions according to the next position where the tool will go.
 *
 * Return value represents the errors:
 * Return value equals to zero. -> The function closed without error.
 * A 1 placed in the fourth position of return_value. -> the FI axis' direction has not been set correctly.
 * A 1 in the third position. -> The Z axis' direction has not been set correctly.
 * A 1 in the second position. -> The R axis' direction has not been set correctly.
 * For example the return value b0000 1010 means, that an error happened while setting FI and R axis' direction.
 */
uint8_t setAllMotorDirTowardsDesiredPos(StepperMotor *stepperMotors, ToolPosition curr_pos, ToolPosition next_pos);

/*
 * Sets the state of the motor to running and starts timer PWM with IT
 */
void motorON(StepperMotor *stepperMotors, uint8_t motor_id);

/*
 * Check out: startMotor
 */
void allMotorON(StepperMotor *stepperMotors);

/*
 * Properly starting a motor
 * It checks for forbidden directions, sets the direction pin and the state variable in the stepperMotor struct.
 */
uint8_t startMotor(StepperMotor *stepperMotors, uint8_t motor_id, uint8_t direction);


/*
 * Properly starting all of the motors.
 * It checks for forbidden directions, sets the direction pin and the state variable in the stepper motors.
 *
 * Return value represents the errors:
 * Return value equals to zero. -> The function closed without error.
 * A 1 placed in the fourth position of return_value. -> the FI axis' direction has not been set correctly.
 * A 1 in the third position. -> The Z axis' direction has not been set correctly.
 * A 1 in the second position. -> The R axis' direction has not been set correctly.
 * For example the return value b0000 1010 means, that an error happened while setting FI and R axis' direction.
 */
uint8_t startAllMotor(StepperMotor *stepperMotors, uint8_t direction);


/*
 * Stops timer PWM and sets the state of the motor to stopped.
 */
void motorOFF(StepperMotor *stepperMotors, uint8_t motor_id);

/*
 * It brings the three motors to a stop.
 * Check out: stopMotor() function.
 */
void allMotorOFF(StepperMotor *stepperMotors);

void command_Z_stop (StepperMotor *stepperMotors);
void command_Z_up (StepperMotor *stepperMotors);
void command_Z_down (StepperMotor *stepperMotors);

void command_R_stop (StepperMotor *stepperMotors);
void command_R_forward (StepperMotor *stepperMotors);
void command_R_backward (StepperMotor *stepperMotors);

void command_FI_stop (StepperMotor *stepperMotors);
void command_FI_counterclockwise (StepperMotor *stepperMotors);
void command_FI_clockwise (StepperMotor *stepperMotors);


/*
 * This drives all of the motors to the
 * The funcion overwrites the @current_position variable. When all of the motors reaches their limit, @current_position
 * gets all zero value.
 * The @limit_switches array must contain 6 element:
 * limit_switches[0] : FI axis' zero point (when the arm is rotated all the way counter-clockwise)
 * limit_switches[1] : FI axis' maximal positive excursion (rotated all the way clockwise)
 * limit_switches[2] : Z axis' zero point (lower switch)
 * limit_switches[3] : Z axis' maximal positiv excursion (upper sitch)
 * limit_switches[4] : R axis' zero ponit (when the arm is fully retracted)
 * limit_switches[5] : R axis' maximal positive excursion (the arm reaches the furthest)
 */
uint8_t command_Homing(StepperMotor *stepperMotors, ToolPosition *current_position, GPIO_PinState *limit_switches);

/*
 * Controls a motor with two GPIO pin.
 * One pin drives the motor to positive direction, the other drives it to the opposite direction.
 *
 */
uint8_t controlMotor_viaGPIO (GPIO_TypeDef* pos_button_port, uint16_t pos_button_pin, GPIO_TypeDef* neg_button_port, uint16_t neg_button_pin, StepperMotor *stepperMotors, uint8_t motor_id);

void controlMotor_viaUART(StepperMotor *stepperMotors, MovementCommands command);

