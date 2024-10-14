#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_

#include "KAR_MC_handler.h"

extern s_MC_StepperMotor stepper_motors[MC_NUMBER_OF_MOTORS];

void initStepperMotors(void);

#endif /* STEPPER_MOTOR_H_ */
