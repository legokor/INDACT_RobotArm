#ifndef WIFICONTROLLER_SERIALHELPER_H_
#define WIFICONTROLLER_SERIALHELPER_H_

#include "WifiController/Common.h"
#include "WifiController/SerialHelperConfig.h"

/**
 * @brief Initializes the serial helper.
 * 
 * @return Error code.
 * 
 * @note This function must be called before any other function in this file.
 */
WifiController_ErrorCode_t WifiController_SerialHelper_Init();

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
WifiController_ErrorCode_t WifiController_SerialHelper_SendMessage(const char *message);

/**
 * @brief Reads a message from the serial port.
 * 
 * @param buffer The buffer where the message will be stored.
 * @param max_length The maximum length of the message.
 * @return Error code.
 * 
 * @note This function is blocking.
 */
WifiController_ErrorCode_t WifiController_SerialHelper_ReadMessage(char *buffer, int max_length);

/**
 * @brief Callback function that is called when a byte is received through the serial port.
 * 
 * @param huart The UART handle.
 * 
 * @details This function should be called from the UART receive interrupt handler (HAL_UART_RxCpltCallback).
 *     It is used to implement the read message function.
 */
void WifiController_SerialHelper_UartRxCallback(UART_HandleTypeDef *huart);

#endif /* WIFICONTROLLER_SERIALHELPER_H_ */
