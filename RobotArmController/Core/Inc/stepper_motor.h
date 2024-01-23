#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_

#include "MotorControl/KAR_MC_handler.h"
#include "MotorControl/KAR_GEO_interface.h"

extern s_MC_StepperMotor as_stepper_motors[KAR_MC_NUMBER_OF_MOTORS];
extern s_GEO_LimitSwitch as_limit_switches[KAR_MC_NUMBER_OF_MOTORS];

/*
 * =============================================================================
 * Function name: initStepperMotors
 *------------------------------------------------------------------------------
 * Description:
 * This function gives starting values to the as_stepper_motor and
 * as_limit_switches array elements.
 *
 * INPUT: none
 *
 * OUTPUT: none
 *------------------------------------------------------------------------------
 */
void initStepperMotors(void);

#endif /* STEPPER_MOTOR_H_ */
