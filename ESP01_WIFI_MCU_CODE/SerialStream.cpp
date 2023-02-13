/**
 *******************************************************************************
 * @file SerialStream.cpp
 *
 * @date Feb 7, 2023
 * @author Varga Péter
 *******************************************************************************
 * @brief
 * 
 *******************************************************************************
 */

#include "SerialStream.h"

#include <string.h>

#define SERIAL_TX_TIMEOUT 20
#define STREAM_RX_BLOCK_TIME_MS 20

namespace ESP01
{

SerialStream::SerialStream(UART_HandleTypeDef *huart, size_t buffer_size)
        : huart(huart), rxBufferSize(buffer_size)
{
    rxBuffer = xStreamBufferCreate(rxBufferSize, rxTriggerLevel);
    configASSERT(rxBuffer != NULL);
}

SerialStream::~SerialStream()
{
    vStreamBufferDelete(rxBuffer);
}

void SerialStream::sendMessage(const char *msg, size_t size)
{
    if (HAL_UART_Transmit(huart, (const uint8_t*)msg, size * sizeof(char), SERIAL_TX_TIMEOUT)
            != HAL_OK)
    {
        throw std::runtime_error("Serial transmission failed.");
    }
}

bool SerialStream::receiveMessage(char *msg, size_t msg_max_size)
{
    const TickType_t block_time = pdMS_TO_TICKS(STREAM_RX_BLOCK_TIME_MS);

    char rch[2] = { 0 };
    size_t idx = 0;

    while (idx <= msg_max_size + 2)
    {
        // Save the last received character
        rch[1] = rch[0];
        // Receive the new character
        if (xStreamBufferReceive(rxBuffer, (void*)rch, sizeof(char), block_time) == sizeof(char))
        {
            if (idx > 1)
            {
                // Check wether the end marker characters arrived or not
                if ((rch[0] == msgEndMarker[0]) && (rch[1] == msgEndMarker[1]))
                {
                    // Close the string
                    msg[idx - 2] = '\0';
                    // Reception successful
                    return true;
                }
                // Save the new character to the message buffer space
                msg[idx - 2] = rch[1];
            }
            idx++;
        }
    }

    // Close the string, it is empty
    msg[0] = '\0';
    // Reception not successful
    return false;
}

void SerialStream::sendCharFromISR(char ch)
{
    xStreamBufferSendFromISR(rxBuffer, (const void*)(&ch), sizeof(char), NULL);
}

} /* namespace ESP01 */
