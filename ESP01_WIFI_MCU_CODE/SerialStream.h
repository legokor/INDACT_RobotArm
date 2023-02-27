/**
 *******************************************************************************
 * @file SerialStream.h
 *
 * @date Feb 7, 2023
 * @author Varga  Péter
 *******************************************************************************
 * @brief Declaration of a class that handles serial communication through UART.
 *
 *******************************************************************************
 */

#ifndef SERIALSTREAM_H_
#define SERIALSTREAM_H_

#include <stdexcept>
#include "usart.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "stream_buffer.h"

namespace ESP01
{

/**
 * @brief Class to handle the data transmission and reception on a serial communication channel.
 */
class SerialStream
{
public:
    /** Poninter of the handle of the UART. */
    UART_HandleTypeDef *const huart;

private:
    StreamBufferHandle_t rxBuffer;

    const size_t rxBufferSize;
    const size_t rxTriggerLevel = 1;

    const char *msgEndMarker = "\r\n";

public:
    /**
     * @brief Constructor.
     * @param huart Poninter of the handle of the UART
     * @param buffer_size Size of the serial buffer in bytes
     */
    SerialStream(UART_HandleTypeDef *huart, size_t buffer_size);
    /** Destructor. */
    virtual ~SerialStream();

    /**
     * @brief Send a string of characters terminated by \r\n through UART.
     * @param msg String of characters to be sent
     * @param size Number of characters to be sent
     */
    void sendMessage(const char *msg, size_t size);
    /**
     * @brief Receive a string of characters terminated by \r\n through UART.
     * @param msg Buffer for the received characters
     * @param msg_max_size Maximum number of the received characters, has to be at most the size of
     *          the buffer - 1
     * @return True if a message was successfully received otherwise false
     */
    bool receiveMessage(char *msg, size_t msg_max_size);

    /**
     * @brief Save a character to the serial buffer. This method has to be called from an interrupt
     *          service function.
     * @param ch Character to be saved
     */
    void sendCharFromISR(char ch);
};

} /* namespace ESP01 */

#endif /* SERIALSTREAM_H_ */
