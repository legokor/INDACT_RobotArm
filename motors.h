/*
 * motors.h
 *
 *  Created on: 8 Oct 2019
 *      Author: antal
 */

#pragma once

#include "main.h"
#include <stdio.h>
#include <stdbool.h>

/* BEGIN defines */
/* motor defines */
#define MOTORSTATE_RUNNING (uint8_t)1u
#define MOTORSTATE_STOPPED (uint8_t)0u
#define MOTORDIR_TODESIREDPOS	(uint8_t)3u
#define MOTORDIR_UNDEFINED	(uint8_t)2u
#define MOTORDIR_POSITIVE 	(uint8_t)1u
#define MOTORDIR_NEGATIVE 	(uint8_t)0u
#define MOTORALLOW_BOTHDIR	(uint8_t)3u //0b0000_0011
#define MOTORALLOW_POSDIR	(uint8_t)2u	//0b0000_0010
#define MOTORALLOW_NEGDIR	(uint8_t)1u	//0b0000_0001
#define MOTORALLOW_NODIR	(uint8_t)0u	//0b0000_0000
#define MOTORALLOW_MASK		(uint8_t)0b00000011 //3u

#define MOTOR_FI_ID			(uint8_t)0u
#define MOTOR_Z_ID			(uint8_t)1u
#define MOTOR_R_ID			(uint8_t)2u
#define NUMBER_OF_MOTORS	(uint8_t)3u

/* Positions */ //??????
#define MAXPOSITIONS (uint8_t)12u

/* general */
#define UINT8T_MAXV (uint8_t)255u

/* END defines */

/* BEGIN type definitions */
typedef struct {
	uint32_t x;
	uint32_t y;
	uint32_t z;
} GrapplerPosition;


/* motor typedef */
typedef struct{
	uint8_t ID;
	volatile uint32_t currPos;
	volatile uint32_t desiredPos;
	volatile uint8_t dir;   // MOTORDIR_NEGATIVE, MOTORDIR_POSITIVE, MOTORDIR_UNDEFINED
	uint8_t allowedDir;		//MOTORALLOW_BOTHDIR, MOTORALLOW_POSDIR, MOTORALLOW_NEGDIR, MOTORALLOW_NODIR
	uint8_t motorState;     //MOTORSTATE_RUNNING, MOTORSTATE_STOPPED

	//these are needed, so more general functions can be implemented
	//COM is not listed here, as that shall always be 0
	uint32_t TIM_CH;	//0:0x00000000, 1:0x..04, 2:0x..08, :0x0..0C
	TIM_HandleTypeDef* TIM; //address of the used timer

	GPIO_TypeDef* enablePORT;
	uint16_t enablePIN;

	GPIO_TypeDef* dirPORT;
	uint16_t dirPIN;
} StepperMotor;
/* END type definitions */

/* BEGIN API function definitions */
uint8_t setMotorDirection(const uint8_t ID, const uint8_t desiredDirection);
uint32_t setAllDirectionsTowardsDesiredPos();
bool posReached(const uint8_t ID);
bool posAllReached();
void startMotorPWM(const uint8_t ID);
void startAllMotorPWMs();
void changeMotorSpeed(const uint8_t RCRValue);
void stopMotorPWM(const uint8_t ID);
void stopAllMotorBasedPos();
//void incrementSTMotorPos(const uint8_t ID);
/* END API function definitions */


//uint8_t calcRcrValue(uint8_t *RCRoverflow);
//void incrementSTMotorPos(const uint8_t ID)
