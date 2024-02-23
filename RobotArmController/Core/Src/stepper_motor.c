#include "stepper_motor.h"

#include "main.h"
#include "tim.h"

// TODO: Solve naming conflict
#include "translation.h"

s_MC_StepperMotor stepper_motors[MC_NUMBER_OF_MOTORS];

void initStepperMotors()
{
    // Init stepper motor structures
    stepper_motors[MC_MOTORID_PHI] = (s_MC_StepperMotor){
            .id = MC_MOTORID_PHI,
            .dir = MC_DIR_UNDEFINED,
            .allowedDir = MC_ALLOWDIR_BOTHDIR,
            .motorState = MC_STATE_STOPPED,
            .currPos = MC_MAXPOS_PHI / 2, .nextPos = 0,
            .TIM_CH = TIM_CHANNEL_1,
            .TIM = &htim10,
            .ENA = {
                    .GPIO_Port = motor_fi_ENA_GPIO_Port,
                    .GPIO_Pin = motor_fi_ENA_Pin
            },
            .DIR = {
                    .GPIO_Port = motor_fi_DIR_GPIO_Port,
                    .GPIO_Pin = motor_fi_DIR_Pin
            },
    };
    stepper_motors[MC_MOTORID_Z] = (s_MC_StepperMotor){
            .id = MC_MOTORID_Z,
            .dir = MC_DIR_UNDEFINED,
            .allowedDir = MC_ALLOWDIR_BOTHDIR,
            .motorState = MC_STATE_STOPPED,
            .currPos = MC_MAXPOS_Z / 2,
            .nextPos = 0,
            .TIM_CH = TIM_CHANNEL_1,
            .TIM = &htim9,
            .ENA = {
                    .GPIO_Port = motor_z_ENA_GPIO_Port,
                    .GPIO_Pin = motor_z_ENA_Pin
            },
            .DIR = {
                    .GPIO_Port = motor_z_DIR_GPIO_Port,
                    .GPIO_Pin = motor_z_DIR_Pin
            },
    };
    stepper_motors[MC_MOTORID_R] = (s_MC_StepperMotor){
            .id = MC_MOTORID_R,
            .dir = MC_DIR_UNDEFINED,
            .allowedDir = MC_ALLOWDIR_BOTHDIR,
            .motorState = MC_STATE_STOPPED,
            .currPos = MC_MAXPOS_R / 2, .nextPos = 0,
            .TIM_CH = TIM_CHANNEL_2,
            .TIM = &htim9,
            .ENA = {
                    .GPIO_Port = motor_r_ENA_GPIO_Port,
                    .GPIO_Pin = motor_r_ENA_Pin
            },
            .DIR = {
                    .GPIO_Port = motor_r_DIR_GPIO_Port,
                    .GPIO_Pin = motor_r_DIR_Pin
            },
    };

    // Enable all motors
    HAL_GPIO_WritePin(
            stepper_motors[MC_MOTORID_R].ENA.GPIO_Port,
            stepper_motors[MC_MOTORID_R].ENA.GPIO_Pin,
            GPIO_PIN_RESET);
    HAL_GPIO_WritePin(
            stepper_motors[MC_MOTORID_PHI].ENA.GPIO_Port,
            stepper_motors[MC_MOTORID_PHI].ENA.GPIO_Pin,
            GPIO_PIN_RESET);
    HAL_GPIO_WritePin(
            stepper_motors[MC_MOTORID_Z].ENA.GPIO_Port,
            stepper_motors[MC_MOTORID_Z].ENA.GPIO_Pin,
            GPIO_PIN_RESET);
}
