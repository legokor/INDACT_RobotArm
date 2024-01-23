#include "stepper_motor.h"

#include "main.h"
#include "tim.h"

s_MC_StepperMotor as_stepper_motors[KAR_MC_NUMBER_OF_MOTORS];
s_GEO_LimitSwitch as_limit_switches[KAR_MC_NUMBER_OF_MOTORS];

void initStepperMotors()
{
    // Init limit switches
    as_limit_switches[KAR_MC_MOTORID_PHI].max_point = HAL_GPIO_ReadPin(
            limswitch_fi_max_GPIO_Port,
            limswitch_fi_max_Pin);
    as_limit_switches[KAR_MC_MOTORID_PHI].null_point = HAL_GPIO_ReadPin(
            limswich_fi_null_GPIO_Port,
            limswich_fi_null_Pin);
    as_limit_switches[KAR_MC_MOTORID_Z].max_point = HAL_GPIO_ReadPin(
            limswitch_z_max_GPIO_Port,
            limswitch_z_max_Pin);
    as_limit_switches[KAR_MC_MOTORID_Z].null_point = HAL_GPIO_ReadPin(
            limswitch_z_null_GPIO_Port,
            limswitch_z_null_Pin);
    as_limit_switches[KAR_MC_MOTORID_R].max_point = HAL_GPIO_ReadPin(
            limswitch_r_max_GPIO_Port,
            limswitch_r_max_Pin);
    as_limit_switches[KAR_MC_MOTORID_R].null_point = HAL_GPIO_ReadPin(
            limswitch_r_null_GPIO_Port,
            limswitch_r_null_Pin);

    // Init stepper motors
    as_stepper_motors[KAR_MC_MOTORID_PHI] = (s_MC_StepperMotor){
            .id = KAR_MC_MOTORID_PHI,
            .dir = KAR_MC_DIR_UNDEFINED,
            .allowedDir = KAR_MC_ALLOWDIR_BOTHDIR,
            .motorState = KAR_MC_STATE_STOPPED,
            .currPos = U32_KAR_MC_MAXPOS_FI / 2, .nextPos = 0,
            .TIM_CH = TIM_CHANNEL_2,    //PE11
            .TIM = &htim1,
            .ENA = {
                    .GPIO_Port = motor_fi_ENA_GPIO_Port,
                    .GPIO_Pin = motor_fi_ENA_Pin
            },
            .DIR = {
                    .GPIO_Port = motor_fi_DIR_GPIO_Port,
                    .GPIO_Pin = motor_fi_DIR_Pin
            },
    };
    as_stepper_motors[KAR_MC_MOTORID_Z] = (s_MC_StepperMotor){
            .id = KAR_MC_MOTORID_Z,
            .dir = KAR_MC_DIR_UNDEFINED,
            .allowedDir = KAR_MC_ALLOWDIR_BOTHDIR,
            .motorState = KAR_MC_STATE_STOPPED,
            .currPos = U32_KAR_MC_MAXPOS_Z / 2,
            .nextPos = 0,
            .TIM_CH = TIM_CHANNEL_1,    //PA0
            .TIM = &htim2,
            .ENA = {
                    .GPIO_Port = motor_z_ENA_GPIO_Port,
                    .GPIO_Pin = motor_z_ENA_Pin
            },
            .DIR = {
                    .GPIO_Port = motor_z_DIR_GPIO_Port,
                    .GPIO_Pin = motor_z_DIR_Pin
            },
    };
    as_stepper_motors[KAR_MC_MOTORID_R] = (s_MC_StepperMotor){
            .id = KAR_MC_MOTORID_R,
            .dir = KAR_MC_DIR_UNDEFINED,
            .allowedDir = KAR_MC_ALLOWDIR_BOTHDIR,
            .motorState = KAR_MC_STATE_STOPPED,
            .currPos = U32_KAR_MC_MAXPOS_R / 2, .nextPos = 0,
            .TIM_CH = TIM_CHANNEL_1,    //PA6
            .TIM = &htim3,
            .ENA = {
                    .GPIO_Port = motor_r_ENA_GPIO_Port,
                    .GPIO_Pin = motor_r_ENA_Pin
            },
            .DIR = {
                    .GPIO_Port = motor_r_DIR_GPIO_Port,
                    .GPIO_Pin = motor_r_DIR_Pin
            },
    };

    /* Set the COM pins to GND */
    HAL_GPIO_WritePin(motor_fi_COM_GPIO_Port, motor_fi_COM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_r_COM_GPIO_Port, motor_r_COM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_z_COM_GPIO_Port, motor_z_COM_Pin, GPIO_PIN_RESET);

    /* Enable all motor */
    HAL_GPIO_WritePin(
            as_stepper_motors[KAR_MC_MOTORID_R].ENA.GPIO_Port,
            as_stepper_motors[KAR_MC_MOTORID_R].ENA.GPIO_Pin,
            GPIO_PIN_SET);  //enable = HIGH
    HAL_GPIO_WritePin(
            as_stepper_motors[KAR_MC_MOTORID_Z].ENA.GPIO_Port,
            as_stepper_motors[KAR_MC_MOTORID_Z].ENA.GPIO_Pin,
            GPIO_PIN_SET); // enable = HIGH
    HAL_GPIO_WritePin(
            as_stepper_motors[KAR_MC_MOTORID_PHI].ENA.GPIO_Port,
            as_stepper_motors[KAR_MC_MOTORID_PHI].ENA.GPIO_Pin,
            GPIO_PIN_RESET);  // enable = GND
}
