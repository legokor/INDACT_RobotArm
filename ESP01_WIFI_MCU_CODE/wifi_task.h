/**
 *******************************************************************************
 * @file wifi_task.h
 *
 * @date 2023. febr. 8.
 * @author Varga Péter
 *******************************************************************************
 * @brief
 * 
 *******************************************************************************
 */

#ifndef WIFI_TASK_H_
#define WIFI_TASK_H_

#include "WIFI.h"

/** Instance of the WIFI class. Declared in wifi_task.cpp.*/
extern ESP01::WIFI wifi;

/**
 * @brief Start the WIFI reception task.
 */
void wifi_StartReceiveTask(void);

/**
 * @brief Function that has to be called in the UART receive callback function when the
 *          corresponding UART interrupt occurs.
 */
void wifi_UART_RxCpltCallback(void);

#endif /* WIFI_TASK_H_ */
