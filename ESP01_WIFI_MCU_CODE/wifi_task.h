/**
 ***************************************************************************************************
 * @file wifi_task.h
 *
 * @date 2023. 02. 08.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief This file provides a simple interface for handling the setup and RTOS related parts of the
 *          WIFI.
 ***************************************************************************************************
 */

#ifndef WIFI_TASK_H_
#define WIFI_TASK_H_

/**
 * @brief Initialize the memory used by the WIFI and register the WIFI's receiving task entry
 *          function at the kernel.
 */
void WIFI_InitRTOS(void);

/**
 * @brief Entry function for the WIFI's receiving task.
 * @details This task starts the data recetion through UART interrupts, and calls the WIFI's data
 *          reception function in a loop.
 * @param argument UNUSED
 */
void WIFI_StartReceiveTask(void *argument);

/**
 * @brief Callback function that has to be called from the UART receive callback function when the
 *          WIFI has to receive the data.
 * @details
 * This function is going to save the received character (byte) and restart the data
 * reception.
 * Example usage:
   <pre>
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
   {
       if (huart->Instance == WIFI_UART_HANDLE->Instance)
       {
           WIFI_UART_RxCpltCallback();
       }
   }
   </pre>
 */
void WIFI_UART_RxCpltCallback(void);

/**
 * @brief Entry function for the WIFI's default setup task.
 * @details This task is going to first attempt synchroniation between the WiFi module and this
 *          controller, then it is going to attemp to set up an access point with the default values
 *          defined by the WiFi module.
 *          After completing the setup, the task is going to delete itself.
 * @param argument UNUSED
 */
void WIFI_StartDefaultSetupTask(void *argument);

#endif /* WIFI_TASK_H_ */
