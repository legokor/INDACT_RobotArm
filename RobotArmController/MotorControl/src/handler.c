#include "MotorControl/handler.h"

MC_ErrorCode_t MC_SetMotorDirection(MC_StepperMotor_t *stepper_motor, int direction)
{
    if (stepper_motor == NULL)
    {
        return MC_ErrorCode_WrongParameter;
    }

    MC_ErrorCode_t ec = MC_ErrorCode_Ok;

    GPIO_PinState ps = GPIO_PIN_RESET;
    if ((direction < 0) && stepper_motor->negativeDirectionAllowed)
    {
        ps = MC_DIRECTION_PIN_STATE_NEGATIVE;
    }
    else if ((direction > 0) && stepper_motor->positiveDirectionAllowed)
    {
        ps = MC_DIRECTION_PIN_STATE_POSITIVE;
    }
    else
    {
        ec = MC_ErrorCode_ForbiddenState;
    }

    if (ec == MC_ErrorCode_Ok)
    {
        stepper_motor->direction = direction;
        HAL_GPIO_WritePin(stepper_motor->directionPin.port, stepper_motor->directionPin.pin, ps);
    }

    return ec;
}

MC_ErrorCode_t MC_StartMotor(MC_StepperMotor_t *stepper_motor)
{
    if (stepper_motor == NULL)
    {
        return MC_ErrorCode_WrongParameter;
    }

    MC_ErrorCode_t ec = MC_SetMotorDirection(stepper_motor, direction);

    if (ec == MC_ErrorCode_Ok)
    {
        HAL_TIM_PWM_Start_IT(stepper_motor->timer.htim, stepper_motor->timer.channel);
        stepper_motor->motorState = MC_State_Running;
    }
    else
    {
        stepper_motor->motorState = MC_State_ForbiddenState;
    }

    return errorCode;
}

MC_ErrorCode_t MC_StartMotors(MC_StepperMotor_t *stepper_motors, size_t num_motors)
{
    if (stepper_motors == NULL)
    {
        return MC_ErrorCode_WrongParameter;
    }

    MC_ErrorCode_t ec = MC_ErrorCode_Ok;
    for (size_t i = 0; (i < num_motors) && (ec == MC_ErrorCode_Ok); i++)
    {
        ec = MC_StartMotor(&stepper_motors[i]);
    }

    return ec;
}

void MC_StopMotor(MC_StepperMotor_t *stepper_motor)
{
    HAL_TIM_PWM_Stop_IT(stepper_motor->timer.htim, stepper_motor->timer.channel);
    stepper_motor->motorState = MC_State_Stopped;
}

void MC_StopMotors(MC_StepperMotor_t *stepper_motors, size_t num_motors)
{
    for (size_t i = 0; i < num_motors; i++)
    {
        MC_StopMotor(&stepper_motors[i]);
    }
}
