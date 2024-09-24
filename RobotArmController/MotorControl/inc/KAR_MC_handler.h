/**
 * @file KAR_MC_handler.h
 *
 * "Motor controlling library for Karcsi"
 *
 * $Author: Erdei Sándor
 * $Date: 2023-07-28
 * $Revision: $
 *
 * @copyright LEGO kör - INDACT project
 */

/*# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
 #																												#
 #		* Detailed description *																				#
 #																												#
 # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # */

#ifndef KAR_MC_HANDLER_H
#define KAR_MC_HANDLER_H

/****************************************************************************************************************
 * 																												*
 * Includes																										*
 * 																												*
 *****************************************************************************************************************/

#include "KAR_GEO_interface.h"
#include "KAR_GEN_interface.h"

/****************************************************************************************************************
 * 																												*
 * Public Macros																								*
 * 																												*
 *****************************************************************************************************************/

#define MC_NUMBER_OF_MOTORS						(3u)

#define MC_STATE_RUNNING 						(1u)
#define MC_STATE_STOPPED 						(0u)

/*
 * On the Z axis the positive direction is downward.
 * On the R axis positive means the tool is going away from the center.
 * On the FI axis positive means the clockwise rotation.
 * Undefined direction means a stop. When the motor has undefined direction it cannot start to move.
 */
#define MC_DIR_UNDEFINED						(2u)
#define MC_DIR_POSITIVE 						(0u)
#define MC_DIR_NEGATIVE 						(1u)

/*
 * StepperMotor.allowedDir value defines which way a motor can move - able to restrict movement.
 */
#define MC_ALLOWDIR_BOTHDIR						(3u)
#define MC_ALLOWDIR_POSDIR						(2u)
#define MC_ALLOWDIR_NEGDIR						(1u)
#define MC_ALLOWDIR_NODIR						(0u)

#define MC_MOTORID_PHI							(0u)
#define MC_MOTORID_Z							(1u)
#define MC_MOTORID_R							(2u)

/*
 * Length of the axis, measured in steps of the stepper motor.
 */
#define MC_MAXPOS_PHI 							(int32_t)13874u
#define MC_MAXPOS_Z 							(int32_t)49231u
#define MC_MAXPOS_R 							(int32_t)1048u

/****************************************************************************************************************
 * 																												*
 * Public Types																									*
 * 																												*
 *****************************************************************************************************************/

typedef enum
{
	e_MC_ErrorCode_OK = 0,
	e_MC_ErrorCode_WrongParameter = 1,
	e_MC_ErrorCode_FrobiddenState = 2,
} e_MC_ErrorCode_t;

/* TODO : used in INDACT x LEAP project
typedef enum
{
	e_MC_ActionCode_Homing = 0x01,
	e_MC_ActionCode_ChangeCoordinates = 0x02,
	e_MC_ActionCode_Z_Up = 0x03,
	e_MC_ActionCode_Z_Down = 0x04,
	e_MC_ActionCode_R_Forward = 0x05,
	e_MC_ActionCode_R_Backward = 0x06,
	e_MC_ActionCode_FI_Counterclockwise = 0x07,
	e_MC_ActionCode_FI_Clockwise = 0x08,
} e_MC_ActionCode_t;
*/

/*
 * Every stepper motor has a struct like this to contain the relevant information about it in one place.
 */
typedef struct
{
	uint8_t id;
	uint8_t dir;   					//U8_MC_DIR_UNDEFINED, U8_MC_DIR_POSITIVE, U8_MC_DIR_NEGATIVE
	uint8_t allowedDir;				//U8_MC_ALLOWDIR_BOTHDIR, U8_MC_ALLOWDIR_POSDIR, U8_MC_ALLOWDIR_NEGDIR, U8_MC_ALLOWDIR_NODIR
	uint8_t motorState;     		//U8_MC_STATE_RUNNING, U8_MC_STATE_STOPPED

	int32_t currPos;
	int32_t nextPos;

	uint32_t TIM_CH;				//TIM_CHANNEL_x
	TIM_HandleTypeDef* TIM; 		//&htimx

	s_GEN_GPIO ENA;					//motor enable GPIO pin
	s_GEN_GPIO DIR;					//motor direction GPIO pin
} s_MC_StepperMotor;

/****************************************************************************************************************
 * 																												*
 * Public function declarations																					*
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
e_MC_ErrorCode_t u8_MC_setAllMotorDir_TowardsDesiredPos_f(s_MC_StepperMotor *stepper_motors);

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
e_MC_ErrorCode_t u8_MC_StartMotor_f(s_MC_StepperMotor *stepper_motors, uint8_t motor_id, uint8_t direction);

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
e_MC_ErrorCode_t u8_MC_StartAllMotor_f(s_MC_StepperMotor *stepper_motors, uint8_t direction);

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
void v_MC_StopMotor_f(s_MC_StepperMotor *stepper_motors, uint8_t motor_id);

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
void v_MC_StopAllMotor_f(s_MC_StepperMotor *stepperMotors);

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
											   s_GEN_GPIO positive_button, s_GEN_GPIO negative_button);

#endif		// KAR_MC_HANDLER_H

/* End of file KAR_MC_handler.h */
