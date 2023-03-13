/*
 *  Created on: 04.03.2023
 *  Author: Erdei Sandor
 *
 *  This file handles the THREE stepper motor of the INDACT robot arm, alias KARCSI.
 */


#pragma once
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

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
 *
 */
#define MOTOR_FI_MAXPOS 	(uint32_t)7500u
#define MOTOR_Z_MAXPOS 		(uint32_t)6500u
#define MOTOR_R_MAXPOS 		(uint32_t)600u
#define MOTOR_FI_MICROSTEP 	(uint32_t)375u
#define MOTOR_Z_MICROSTEP	(uint32_t)325u
#define MOTOR_R_MICROSTEP 	(uint32_t)30u


/*
 * These are general commands for the robot.
 * The source of these could be a remote controller, a webpage or anything.
 */
typedef enum {
	X_INVALID = 0,

	Z_STOP = 1,
	Z_UP = 2,
	Z_DOWN = 3,

	R_STOP = 4,
	R_FORWARD =5,
	R_BACKWARD = 6,

	FI_STOP = 7,
	FI_COUNTERCLOCKWISE = 8,
	FI_CLOCKWISE = 9,

	X_HOMING = 10,
	X_CHANGE_COORDINATES = 11
} MovementCommands;


/*
 * Type for the requests that the WiFi module can send to the controller.
 */
typedef enum
{
    INVALID, 			/**< INVALID Invalid request*/
    AXIS_A_PLUS, 		/**< AXIS_A_PLUS Move the robot in the positive direction of the A axis */
    AXIS_A_MINUS, 		/**< AXIS_A_MINUS Move the robot in the negative direction of the A axis */
    AXIS_B_PLUS, 		/**< AXIS_B_PLUS Move the robot in the positive direction of the B axis */
    AXIS_B_MINUS, 		/**< AXIS_B_MINUS Move the robot in the negative direction of the B axis */
    AXIS_C_PLUS, 		/**< AXIS_C_PLUS Move the robot in the positive direction of the C axis */
    AXIS_C_MINUS, 		/**< AXIS_C_MINUS Move the robot in the negative direction of the C axis */
    HOMING, 			/**< HOMING Begin homing sequence */
    CHANGE_COORDINATES, /**< CHANGE_COORDINATES Change the coordinate system of the robot arm */
}RequestType;


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
 * Starts all three motors.
 * Check out: motorON() function.
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
 * Check out: motorOFF() function.
 */
void allMotorOFF(StepperMotor *stepperMotors);

/*
 * The WIFI module use a different enum declaration for it's commands then me.
 * Thats why I need to "translate" them.
 */
MovementCommands translate_incomingInstruction(RequestType incoming_instruction);

/*
 * Controls a motor with two GPIO pin.
 * One pin drives the motor to positive direction, the other drives it to the opposite direction.
 */
uint8_t controlMotor_viaGPIO (GPIO_TypeDef* pos_button_port, uint16_t pos_button_pin, GPIO_TypeDef* neg_button_port, uint16_t neg_button_pin, StepperMotor *stepperMotors, uint8_t motor_id);


