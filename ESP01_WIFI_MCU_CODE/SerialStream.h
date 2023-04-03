/**
 ***************************************************************************************************
 * @file SerialStream.h
 *
 * @date 2023. 02. 07.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief This file provides a simple interface for the serial communication between this controller
 *          and the WiFi module.
 ***************************************************************************************************
 */

#ifndef SERIALSTREAM_H_
#define SERIALSTREAM_H_

#include "usart.h"

// I want to use the stm32l4xx_hal_uart.h file (include below) but the compiler does not compile the
// project if I include it. Including this file would make the inclusion of usart.h unnecessary.
// #include "stm32l4xx_hal_uart.h"

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Function declarations
// /////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initialize the serial stream.
 * @param huart Handle of the UART peripherial that the serial stream uses
 * @param buffer_size Size of the serial stream reception buffer in bytes
 * @param msg_max_size Maximum size of a message in bytes
 */
void SerialStream_InitSerialStream(UART_HandleTypeDef *huart, size_t rx_buffer_size, size_t msg_max_size);

/**
 * @brief Delete the serial stream.
 */
void SerialStream_DeleteSerialStream(void);

/**
 * @brief Send a string of characters in a message frame.
 * @param msg String to be sent
 * @return 1 if the message was sent successfuly, 0 otherwise
 */
uint8_t SerialStream_SendMessage(const char *msg);

/**
 * @brief Receive a string of characters in a message frame.
 * @param msg Buffer for the received characters
 * @return 1 if a message was successfully received, 0 otherwise
 */
uint8_t SerialStream_ReceiveMessage(char *msg);

/**
 * @brief Save a character to the serial buffer. This method has to be called from an interrupt
 *          service function.
 * @param ch Character to be saved
 */
void SerialStream_SaveCharFromISR(char ch);

/**
 * @brief Get wether the serial stream buffer is ready or not.
 * @return 1 if the stream buffer is ready, 0 otherwise
 */
uint8_t SerialStream_IsBufferReady(void);

#endif /* SERIALSTREAM_H_ */
