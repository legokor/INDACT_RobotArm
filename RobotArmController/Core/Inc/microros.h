/**
 * @brief Micro-ROS related variables and functions.
 *
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2024-10-14
 */

#ifndef MICROROS_H_
#define MICROROS_H_

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t microrosTaskHandle;

void setup_microros_rtos(void);

#endif /* MICROROS_H_ */
