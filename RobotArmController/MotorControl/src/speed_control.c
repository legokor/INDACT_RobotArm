#include "MotorControl/speed_control.h"

#include <math.h>

bool Motor_SetSpeed(Motor_t *this, double speed)
{
    // Determine base parameters
    const uint32_t timer_base_frequency = this->timer->frequency;
    const double motor_step_angle = this->step_angle;

    if ((speed < 0.0) || (speed > MOTOR_THEORETICAL_SPEED_MAX(this)))
    {
        return false;
    }

    if ((speed >= 0.0) && (speed < MOTOR_THEORETICAL_SPEED_MIN(this)))
    {
        // If the speed is smaller than can be set with the timer, set the
        // compare to an unreachable value and the speed to 0.
        __HAL_TIM_SET_COMPARE(this->timer->htim, this->timer->channel, 0xffff);
        this->speed = 0.0;
        return true;
    }

    // Set prescaler for minimal prescaler value
    double timer_prescaler = ceil((timer_base_frequency * motor_step_angle) / (TIMER_PERIOD_MAX * speed));
    __HAL_TIM_SET_PRESCALER(this->timer->htim, (uint16_t)(timer_prescaler - 1));

    // Set period for the most accurate impulse frequency
    double timer_period = (timer_base_frequency * motor_step_angle) / (timer_prescaler * speed);
    __HAL_TIM_SET_AUTORELOAD(this->timer->htim, (uint16_t)(timer_period - 1));

    // Set compare for constant impulse width
    double timer_compare = ceil((this->impulse_width * timer_base_frequency) / timer_prescaler);
    __HAL_TIM_SET_COMPARE(this->timer->htim, this->timer->channel, (uint16_t)(timer_compare - 1));

    // Calcuate actual motor speed with the previously set values
    this->speed = (timer_base_frequency * motor_step_angle) / (timer_prescaler * timer_period);

    return true;
}

bool Joint_SetSpeed(Joint_t *this, double speed)
{
    // {joint speed} = {motor speed} * {joint transform factor}
    double motor_speed = speed / this->transform_factor;
    if (Motor_SetSpeed(this->motor, motor_speed))
    {
        this->speed = this->motor->speed * this->transform_factor;
        return true;
    }
    return false;
}
