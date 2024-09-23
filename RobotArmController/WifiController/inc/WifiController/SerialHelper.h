#ifndef WIFICONTROLLER_SERIALHELPER_H_
#define WIFICONTROLLER_SERIALHELPER_H_

#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"

#include "WifiController/Common.h"

/**
 * @brief Initializes the serial helper.
 * 
 * @param huart Handle of the UART peripherial that is used for the
 *     communication with the Wi-Fi module.
 * @return Error code.
 * 
 * @note This function must be called before any other function in this file.
 */
WC_ErrorCode_t WifiController_SerialHelper_Init(UART_HandleTypeDef *huart);

/**
 * @brief Deletes the serial helper.
 */
void WifiController_SerialHelper_Delete();

/**
 * @brief Sends a message through the serial port.
 * 
 * @param message The message to send.
 * @return Error code.
 * 
 * @note This function is blocking.
 */
WC_ErrorCode_t WifiController_SerialHelper_SendMessage(const char *message);

/**
 * @brief Reads a message from the serial port.
 * 
 * @param buffer The buffer where the message will be stored.
 * @param max_length The maximum length of the message.
 * @return Error code.
 * 
 * @note This function is blocking.
 */
WC_ErrorCode_t WifiController_SerialHelper_ReadMessage(char *buffer, int max_length);

/**
 * @brief Callback function that is called when a byte is received through the serial port.
 * 
 * @param pxHigherPriorityTaskWoken If calling this function causes a task to
 *     leave the Blocked state, and the unblocked task has a priority higher
 *     than the currently executing task (the task that was interrupted), then,
 *     internally, this function will set the value to pdTRUE. The value should
 *     be set to pdFALSE before it is passed into the function.
 *
 * @details This function should be called from the UART receive interrupt handler (HAL_UART_RxCpltCallback).
 *     It is used to implement the read message function.
 */
void WifiController_SerialHelper_UartRxCallbackFromISR(BaseType_t *const pxHigherPriorityTaskWoken);

#endif /* WIFICONTROLLER_SERIALHELPER_H_ */
