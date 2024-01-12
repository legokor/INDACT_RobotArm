#ifndef MOTORCONTROL_SPEED_CONTROL_H_
#define MOTORCONTROL_SPEED_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include <stm32f4xx_hal.h>

#define TIMER_PRESCALER_MIN (1.0)
#define TIMER_PRESCALER_MAX (65536.0)
#define TIMER_PERIOD_MIN (1.0)
#define TIMER_PERIOD_MAX (65536.0)

// 1.8° = 0.03141593rad
#define MOTOR_STEP_ANGLE_R (0.03141593)
// 3.5arcmin = 0.05833333° = 0.00101811rad
#define MOTOR_STEP_ANGLE_PHI (0.00101811)
// 1.8° = 0.03141593rad
#define MOTOR_STEP_ANGLE_Z (0.03141593)

#define MOTOR_THEORETICAL_SPEED_MIN(_HMOTOR_) \
        ((_HMOTOR_->timer->frequency * _HMOTOR_->step_angle) \
        / (TIMER_PRESCALER_MAX * TIMER_PERIOD_MAX))
#define MOTOR_THEORETICAL_SPEED_MAX(_HMOTOR_) \
        ((_HMOTOR_->timer->frequency * _HMOTOR_->step_angle) \
        / (TIMER_PRESCALER_MIN * TIMER_PERIOD_MIN))

// 3.95cm = 0.0395m (/rad)
#define JOINT_TRANSFORM_FACTOR_R (0.0395)
// 1rad (/rad)
#define JOINT_TRANSFORM_FACTOR_PHI (1.0)
// 2mm / (2 * pi) = 0.00031831m (/rad)
#define JOINT_TRANSFORM_FACTOR_Z (0.00031831)

#define JOINT_THEORETICAL_SPEED_MIN(_HJOINT_) \
        (MOTOR_THEORETICAL_SPEED_MIN(_HJOINT_->motor) \
        * _HJOINT_->transform_factor)
#define JOINT_THEORETICAL_SPEED_MAX(_HJOINT_) \
        (MOTOR_THEORETICAL_SPEED_MAX(_HJOINT_->motor) \
        * _HJOINT_->transform_factor)

// Dummy classes that contain the necessary data for the speed control implementation.

typedef struct Timer
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t frequency;
} Timer_t;

typedef struct Motor
{
    // Controller settings
    Timer_t *timer;

    // Constant parameters
    double impulse_width;
    double step_angle;

    // State variables
    double position;
    double speed;
} Motor_t;

typedef struct Joint
{
    // Controller settings
    Motor_t *motor;

    // Constant parameters
    double transform_factor;

    // State variables
    double position;
    double speed;
} Joint_t;

// TODO: Acceleration?

bool Motor_SetSpeed(Motor_t *this, double speed);

bool Joint_SetSpeed(Joint_t *this, double speed);

#endif /* MOTORCONTROL_SPEED_CONTROL_H_ */
