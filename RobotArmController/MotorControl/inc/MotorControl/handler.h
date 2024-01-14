#ifndef MOTORCONTROL_HANDLER_H_
#define MOTORCONTROL_HANDLER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

typedef struct MC_Gpio
{
    GPIO_TypeDef *port;
    uint16_t pin;
} MC_Gpio_t;

typedef enum MC_State
{
    MC_State_Stopped = 0,
    MC_State_Running,
    MC_State_Error
} MC_State_t;

typedef enum MC_MotorId
{
    MC_MotorId_R = 0,
    MC_MotorId_Phi,
    MC_MotorId_Z
} MC_MotorId_t;

typedef enum MC_ErrorCode
{
    MC_ErrorCode_Ok = 0,
    MC_ErrorCode_WrongParameter,
    MC_ErrorCode_ForbiddenState,
} MC_ErrorCode_t;

typedef int32_t MC_Step_t;

#define MC_NUMBER_OF_MOTORS 3

#define MC_DIRECTION_PIN_STATE_POSITIVE (GPIO_PIN_SET)
#define MC_DIRECTION_PIN_STATE_NEGATIVE (GPIO_PIN_RESET)

// 1.8° = 0.03141593rad
#define MOTOR_STEP_ANGLE_R (0.03141593)
// 3.5arcmin = 0.05833333° = 0.00101811rad
#define MOTOR_STEP_ANGLE_PHI (0.00101811)
// 1.8° = 0.03141593rad
#define MOTOR_STEP_ANGLE_Z (0.03141593)

// 3.95cm = 0.0395m (/rad)
#define JOINT_TRANSFORM_FACTOR_R (0.0395)
// 1rad (/rad)
#define JOINT_TRANSFORM_FACTOR_PHI (1.0)
// 2mm / (2 * pi) = 0.00031831m (/rad)
#define JOINT_TRANSFORM_FACTOR_Z (0.00031831)

#define MOTOR_TOTAL_STEP_R ((MC_Step_t)1048)
#define MOTOR_TOTAL_STEP_PHI ((MC_Step_t)13874)
#define MOTOR_TOTAL_STEP_Z ((MC_Step_t)49231)

// [rad]
#define MOTOR_RANGE_R (MOTOR_TOTAL_STEP_R * MOTOR_STEP_ANGLE_R)
#define MOTOR_RANGE_PHI (MOTOR_TOTAL_STEP_PHI * MOTOR_STEP_ANGLE_PHI)
#define MOTOR_RANGE_Z (MOTOR_TOTAL_STEP_Z * MOTOR_STEP_ANGLE_Z)

// SI
#define JOINT_RANGE_R (MOTOR_RANGE_R * JOINT_TRANSFORM_FACTOR_R)
#define JOINT_RANGE_PHI (MOTOR_RANGE_PHI * JOINT_TRANSFORM_FACTOR_PHI)
#define JOINT_RANGE_Z (MOTOR_RANGE_Z * JOINT_TRANSFORM_FACTOR_Z)

typedef struct MC_Timer
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t frequency;
} MC_Timer_t;

typedef struct MC_StepperMotor
{
    // Hardware
    MC_Timer_t timer;
    MC_Gpio_t enablePin;
    MC_Gpio_t directionPin;

    MC_MotorId_t id;

    // Hardware constants
    double impulse_width;
    double step_angle;

    // Software constants
    bool positiveDirectionAllowed;
    bool negativeDirectionAllowed;

    // State
    MC_Step_t position_steps;
    int direction;

    MC_State_t motorState;
} MC_StepperMotor_t;

MC_ErrorCode_t MC_SetMotorDirection(MC_StepperMotor_t *stepper_motor, int direction);

MC_ErrorCode_t MC_StartMotor(MC_StepperMotor_t *stepper_motor);

MC_ErrorCode_t MC_StartMotors(MC_StepperMotor_t *stepper_motors, size_t num_motors);

void MC_StopMotor(MC_StepperMotor_t *stepper_motor);

void MC_StopMotors(MC_StepperMotor_t *stepper_motors, size_t num_motors);

#endif /* MOTORCONTROL_HANDLER_H_ */
