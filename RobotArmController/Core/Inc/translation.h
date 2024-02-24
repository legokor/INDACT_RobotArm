#ifndef TRANSLATION_H_
#define TRANSLATION_H_

// Limitswitches
#define lsw_r_null_Pin (R_Limit_sw_2_Pin)
#define lsw_r_null_GPIO_Port (R_Limit_sw_2_GPIO_Port)
#define lsw_r_max_Pin (R_Limit_sw_1_Pin)
#define lsw_r_max_GPIO_Port (R_Limit_sw_1_GPIO_Port)

#define lsw_phi_null_Pin (Phi_Limit_sw_2_Pin)
#define lsw_phi_null_GPIO_Port (Phi_Limit_sw_2_GPIO_Port)
#define lsw_phi_max_Pin (Phi_Limit_sw_1_Pin)
#define lsw_phi_max_GPIO_Port (Phi_Limit_sw_1_GPIO_Port)

#define lsw_z_null_Pin (Z_Limit_sw_2_Pin)
#define lsw_z_null_GPIO_Port (Z_Limit_sw_2_GPIO_Port)
#define lsw_z_max_Pin (Z_Limit_sw_1_Pin)
#define lsw_z_max_GPIO_Port (Z_Limit_sw_1_GPIO_Port)

// LEDs
#define RedLed_LD3_Pin (LD3_Pin)
#define RedLed_LD3_GPIO_Port (LD3_GPIO_Port)
#define GreenLed_LD1_Pin (LD1_Pin)
#define GreenLed_LD1_GPIO_Port (LD1_GPIO_Port)

// Buttons
#define controller_mode_switch_Pin (BTN1_Pin)
#define controller_mode_switch_GPIO_Port (BTN1_GPIO_Port)

#define motor_r_positive_button_Pin (R_button_out_Pin)
#define motor_r_positive_button_GPIO_Port (R_button_out_GPIO_Port)
#define motor_r_negative_button_Pin (R_button_in_Pin)
#define motor_r_negative_button_GPIO_Port (R_button_in_GPIO_Port)

#define motor_fi_positive_button_Pin (Phi_button_CCW_Pin)
#define motor_fi_positive_button_GPIO_Port (Phi_button_CCW_GPIO_Port)
#define motor_fi_negative_button_Pin (Phi_button_CW_Pin)
#define motor_fi_negative_button_GPIO_Port (Phi_button_CW_GPIO_Port)

#define motor_z_positive_button_Pin (Z_button_up_Pin)
#define motor_z_positive_button_GPIO_Port (Z_button_up_GPIO_Port)
#define motor_z_negative_button_Pin (Z_button_down_Pin)
#define motor_z_negative_button_GPIO_Port (Z_button_down_GPIO_Port)

// Motor controller signals
#define motor_r_ENA_Pin (R_En_Pin)
#define motor_r_ENA_GPIO_Port (R_En_GPIO_Port)
#define motor_r_DIR_Pin (R_Dir_Pin)
#define motor_r_DIR_GPIO_Port (R_Dir_GPIO_Port)

#define motor_fi_ENA_Pin (Phi_En_Pin)
#define motor_fi_ENA_GPIO_Port (Phi_En_GPIO_Port)
#define motor_fi_DIR_Pin (Phi_Dir_Pin)
#define motor_fi_DIR_GPIO_Port (Phi_Dir_GPIO_Port)

#define motor_z_ENA_Pin (Z_En_Pin)
#define motor_z_ENA_GPIO_Port (Z_En_GPIO_Port)
#define motor_z_DIR_Pin (Z_Dir_Pin)
#define motor_z_DIR_GPIO_Port (Z_Dir_GPIO_Port)

#endif /* TRANSLATION_H_ */
