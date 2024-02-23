#include "limitswitch.h"

#include "main.h"
// TODO: Fix naming conflict
#include "translation.h"

s_GEO_LimitSwitch limit_switches[MC_NUMBER_OF_MOTORS];

void initLimitswitches()
{
    // Init limit switch structures
    limit_switches[MC_MOTORID_R].null_point = HAL_GPIO_ReadPin(
            lsw_r_null_GPIO_Port,
            lsw_r_null_Pin) == LSW_R_NULL_ACTIVE;
    limit_switches[MC_MOTORID_R].max_point = HAL_GPIO_ReadPin(
            lsw_r_max_GPIO_Port,
            lsw_r_max_Pin) == LSW_R_MAX_ACTIVE;
    limit_switches[MC_MOTORID_PHI].null_point = HAL_GPIO_ReadPin(
            lsw_phi_null_GPIO_Port,
            lsw_phi_null_Pin) == LSW_PHI_NULL_ACTIVE;
    limit_switches[MC_MOTORID_PHI].max_point = HAL_GPIO_ReadPin(
            lsw_phi_max_GPIO_Port,
            lsw_phi_max_Pin) == LSW_PHI_MAX_ACTIVE;
    limit_switches[MC_MOTORID_Z].null_point = HAL_GPIO_ReadPin(
            lsw_z_null_GPIO_Port,
            lsw_z_null_Pin) == LSW_Z_NULL_ACTIVE;
    limit_switches[MC_MOTORID_Z].max_point = HAL_GPIO_ReadPin(
            lsw_z_max_GPIO_Port,
            lsw_z_max_Pin) == LSW_Z_MAX_ACTIVE;
}
