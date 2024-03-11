#ifndef HARDWARESUPPORT_SERIAL_H_
#define HARDWARESUPPORT_SERIAL_H_

#if defined( __cplusplus )
extern "C" {
#endif /* defined( __cplusplus ) */

#include "stm32f7xx_hal.h"

#include <FreeRTOS.h>
#include <semphr.h>
#include <stream_buffer.h>

#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Serial port abstraction
 * 
 * @details This structure is used to abstract a serial port. It is used to
 *     read and write data to the serial port. It also contains a buffer
 *     to store received data.
 * @note The fields of this structure should not be accessed directly. Use
 *     the functions provided to access the data.
 */
typedef struct HardwareSupport_Serial
{
    UART_HandleTypeDef *huart; /**< UART handle. */

    StreamBufferHandle_t rxBuffer; /**< Receive buffer. */
    size_t rxBufferSize; /**< Receive buffer size. */
    SemaphoreHandle_t txSemaphore; /**< Transmit semaphore. */

    volatile char rxChar; /**< Storage for the receive interrupt. */
} HardwareSupport_Serial_t;

/**
 * @brief Initialize the serial port.
 * 
 * @param this Pointer to the serial port structure.
 * @param huart Pointer to the UART handle.
 * @param rx_buffer_size Size of the receive buffer.
 * @return True if the serial port was initialized successfully, false otherwise.
 */
bool HardwareSupport_Serial_Init(HardwareSupport_Serial_t *this, UART_HandleTypeDef *huart, size_t rx_buffer_size);

/**
 * @brief Deinitialize the serial port.
 * 
 * @param this Pointer to the serial port structure.
 */
void HardwareSupport_Serial_Delete(HardwareSupport_Serial_t *this);

/**
 * @brief Read data from the serial port.
 * 
 * @param this Pointer to the serial port structure.
 * @param buffer Pointer to the buffer to store the data.
 * @param length Length of the buffer.
 * @return Number of bytes read.
 */
size_t HardwareSupport_Serial_Read(HardwareSupport_Serial_t *this, char *buffer, size_t length);

/**
 * @brief Write data to the serial port.
 * 
 * @param this Pointer to the serial port structure.
 * @param buffer Pointer to the buffer containing the data.
 * @param length Length of the buffer.
 * 
 * @details This function is guarded by a semaphore to prevent concurrent
 *     access to the UART peripheral.
 */
void HardwareSupport_Serial_Write(HardwareSupport_Serial_t *this, const char *buffer, size_t length);

/**
 * @brief Get the number of bytes available in the receive buffer.
 * 
 * @param this Pointer to the serial port structure.
 * @return Number of bytes available in the receive buffer.
 */
size_t HardwareSupport_Serial_GetAvailable(HardwareSupport_Serial_t *this);

/**
 * @brief Callback function for the UART receive interrupt.
 * 
 * @param this Pointer to the serial port structure.
 * @param pxHigherPriorityTaskWoken If calling this function causes a task to
 *     leave the Blocked state, and the unblocked task has a priority higher
 *     than the currently executing task (the task that was interrupted), then,
 *     internally, this function will set the value to pdTRUE. The value should
 *     be set to pdFALSE before it is passed into the function.
 * 
 * @details This function should be called from the UART receive interrupt.
 *     The function will store the received character in the receive buffer.
 *     This should only be called if the interrupt was caused by the UART
 *     peripheral that is used by this serial port.
 */
void HardwareSupport_Serial_UartRxCallbackFromISR(HardwareSupport_Serial_t *this, BaseType_t *const pxHigherPriorityTaskWoken);

#if defined( __cplusplus )
}
#endif /* defined( __cplusplus ) */

#endif /* HARDWARESUPPORT_SERIAL_H_ */
