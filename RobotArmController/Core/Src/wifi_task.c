/**
 * @brief Implementation of the Wifi-Task.
 *
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2024-12-26
 */

#include "wifi_task.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "common_defs.h"
#include "logger.h"
#include "mechanical_conf.h"
#include "stepper_motor.h"
#include "KAR_MC_handler.h"

extern QueueHandle_t nextPositionQueueHandle;

static const char *find_param_value(const char *message, const char *param_name);
static void send_new_position(const char *btn);

void handleButtonAction(const char *args)
{
    // Use the default GUI of the Wi-Fi controller which provides simple buttons for moving the
    // motors of the robotarm.
    char btn[3] = {'\0'};

    const char *arg_text = "btn=";
    const size_t arg_text_length = 4;

    char *p = strstr(args, arg_text);
    // The argument was found and it has value
    if ((p != NULL) && (strlen(p) >= (arg_text_length + 2)))
    {
        strncpy(btn, p + arg_text_length, 2);
        send_new_position(btn);
    }
    else
    {
        strcpy(btn, "xx");
    }

    logInfo("Button action: %s", btn);
}

void handleCoordCylAction(const char *args)
{
    const char *param_r = "r";
    const char *param_phi = "phi";
    const char *param_z = "z";

    PositionCylindrical_t position;
    const char *param_value_ptr = NULL;
    int param_value = 0;

    param_value_ptr = find_param_value(args, param_r);
    if (param_value_ptr != NULL)
    {
        sscanf(param_value_ptr, "%d", &param_value);
        position.r = (param_value <= 0) ? 0 : (param_value > MOTOR_TOTAL_STEP_R) ? MOTOR_TOTAL_STEP_R : param_value;
    }
    else
    {
        position.r = param_value = 0;
    }

    param_value_ptr = find_param_value(args, param_phi);
    if (param_value_ptr != NULL)
    {
        sscanf(param_value_ptr, "%d", &param_value);
        position.phi = (param_value <= 0) ? 0 : (param_value > MOTOR_TOTAL_STEP_PHI) ? MOTOR_TOTAL_STEP_PHI : param_value;
    }
    else
    {
        position.phi = param_value = 0;
    }

    param_value_ptr = find_param_value(args, param_z);
    if (param_value_ptr != NULL)
    {
        sscanf(param_value_ptr, "%d", &param_value);
        position.z = (param_value <= 0) ? 0 : (param_value > MOTOR_TOTAL_STEP_Z) ? MOTOR_TOTAL_STEP_Z : param_value;
    }
    else
    {
        position.z = param_value = 0;
    }

    logInfo("Leap: (%ld, %ld, %ld)", position.r, position.phi, position.z);

    xQueueSend(nextPositionQueueHandle, &position, 10);
}

static const char *find_param_value(const char *message, const char *param_name)
{
    // TODO: This is just an arbitrary number. Further investigation needed!
    #define MAX_PARAM_NAME_LENGTH 32

    char pattern[MAX_PARAM_NAME_LENGTH + 2] = {'\0'};
    snprintf(pattern, MAX_PARAM_NAME_LENGTH + 2, "%s=", param_name);
    char *start = strstr(message, pattern);
    if (!start)
    {
        return NULL;
    }
    return start + strlen(pattern);
}

static void send_new_position(const char *btn)
{
    const size_t btn_id_length = 2;
    const char *r_plus_btn = "rp";
    const char *r_minus_btn = "rm";
    const char *phi_plus_btn = "fp";
    const char *phi_minus_btn = "fm";
    const char *z_plus_btn = "zp";
    const char *z_minus_btn = "zm";
    const char *homing_btn = "hx";

    PositionCylindrical_t position;
    // Disable OS and interrupts while creating local copy of global data.
    vPortEnterCritical();
    position.r = stepper_motors[MC_MOTORID_R].currPos;
    position.phi = stepper_motors[MC_MOTORID_PHI].currPos;
    position.z = stepper_motors[MC_MOTORID_Z].currPos;
    vPortExitCritical();

    const MC_Step_t d = 100;

    if (strncmp(btn, r_plus_btn, btn_id_length) == 0)
    {
        position.r += d;
        if (position.r > MC_MAXPOS_R)
        {
            position.r = MC_MAXPOS_R;
        }
    }
    else if (strncmp(btn, r_minus_btn, btn_id_length) == 0)
    {
        position.r -= d;
        if (position.r < 0)
        {
            position.r = 0;
        }
    }
    else if (strncmp(btn, phi_plus_btn, btn_id_length) == 0)
    {
        position.phi += d;
        if (position.phi > MC_MAXPOS_PHI)
        {
            position.phi = MC_MAXPOS_PHI;
        }
    }
    else if (strncmp(btn, phi_minus_btn, btn_id_length) == 0)
    {
        position.phi -= d;
        if (position.phi < 0)
        {
            position.phi = 0;
        }
    }
    else if (strncmp(btn, z_plus_btn, btn_id_length) == 0)
    {
        position.z += d;
        if (position.z > MC_MAXPOS_Z)
        {
            position.z = MC_MAXPOS_Z;
        }
    }
    else if (strncmp(btn, z_minus_btn, btn_id_length) == 0)
    {
        position.z -= d;
        if (position.z < 0)
        {
            position.z = 0;
        }
    }
    else if (strncmp(btn, homing_btn, btn_id_length) == 0)
    {
        position.r = 0;
        position.phi = 0;
        position.z = 0;
    }

    xQueueSend(nextPositionQueueHandle, &position, 10);
}
