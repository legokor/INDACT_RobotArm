#ifndef LIMITSWITCH_H_
#define LIMITSWITCH_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f7xx_hal.h"

#include "MotorControl/KAR_MC_handler.h"
#include "MotorControl/KAR_GEO_interface.h"

#define LSW_R_NULL_ACTIVE (GPIO_PIN_RESET)
#define LSW_R_MAX_ACTIVE (GPIO_PIN_RESET)
#define LSW_PHI_NULL_ACTIVE (GPIO_PIN_RESET)
#define LSW_PHI_MAX_ACTIVE (GPIO_PIN_RESET)
#define LSW_Z_NULL_ACTIVE (GPIO_PIN_RESET)
#define LSW_Z_MAX_ACTIVE (GPIO_PIN_RESET)

extern s_GEO_LimitSwitch limit_switches[MC_NUMBER_OF_MOTORS];

void initLimitswitches();

#endif /* LIMITSWITCH_H_ */
