/**
 ***************************************************************************************************
 * @file SerialStream.c
 *
 * @date 2023. 02. 07.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief Implemetation of SerialStream.h.
 ***************************************************************************************************
 */

#include "SerialStream.h"

#include <stdlib.h>
#include <string.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "stream_buffer.h"

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Macros
// /////////////////////////////////////////////////////////////////////////////////////////////////

#define SERIAL_TX_TIMEOUT_MS 200

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Global variables
// /////////////////////////////////////////////////////////////////////////////////////////////////

UART_HandleTypeDef *huart_serial;

static size_t messageMaxSize;

char *transmitBuffer;

static size_t rxBufferSize = 0;
static StreamBufferHandle_t rxBuffer;

const char msgBeginMarker[] = "+_";
const char msgEndMarker[] = "\r\n";

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Function definitions
// /////////////////////////////////////////////////////////////////////////////////////////////////

void SerialStream_InitSerialStream(UART_HandleTypeDef *huart, size_t rx_buffer_size,
        size_t msg_max_size)
{
    huart_serial = huart;
    rxBufferSize = rx_buffer_size;
    messageMaxSize = msg_max_size;

    transmitBuffer = (char*)malloc((messageMaxSize + 5) * sizeof(char));
    configASSERT(transmitBuffer != NULL);

    rxBuffer = xStreamBufferCreate(rxBufferSize, sizeof(char));
    configASSERT(rxBuffer != NULL);
}

void SerialStream_DeleteSerialStream(void)
{
    rxBufferSize = 0;

    vStreamBufferDelete(rxBuffer);

    free(transmitBuffer);
}

uint8_t SerialStream_SendMessage(const char *msg)
{
    size_t msg_size = strlen(msg);

    // Check the size of the message
    if (msg_size > messageMaxSize)
    {
        return 0;
    }

    // Create the message frame
    transmitBuffer[0] = '\0';
    strcat(transmitBuffer, msgBeginMarker);
    strcat(transmitBuffer, msg);
    strcat(transmitBuffer, msgEndMarker);

    // Send the message
    if (HAL_UART_Transmit(huart_serial, (uint8_t*)transmitBuffer, msg_size + 4,
    SERIAL_TX_TIMEOUT_MS)
            == HAL_OK)
    {
        // Transmission successful
        return 1;
    }

    // Transmission failed
    return 0;
}

uint8_t SerialStream_ReceiveMessage(char *msg)
{
    char temp[2] = { 0 };
    uint8_t message_started = 0;
    size_t i = 0;

    while (i < messageMaxSize)
    {
        temp[1] = temp[0];
        // Receive the new character
        if (xStreamBufferReceive(rxBuffer, (void*)temp, sizeof(char), portMAX_DELAY)
                == sizeof(char))
        {
            // Check the beginig condition of the message
            if ((temp[1] == msgBeginMarker[0]) && (temp[0] == msgBeginMarker[1]))
            {
                message_started = 1;
                i = 0;
            }
            else if (message_started == 1)
            {
                // Check the ending condition of the message
                if ((temp[1] == msgEndMarker[0]) && (temp[0] == msgEndMarker[1]))
                {
                    // Close the string
                    msg[i - 1] = '\0';

                    // Reception successful
                    return 1;
                }
                else
                {
                    // Save the received character to the receive buffer space
                    msg[i] = temp[0];
                    i++;
                }
            }
        }
    }

    // Close the string, it is empty
    msg[0] = '\0';
    // Reception not successful
    return 0;
}

void SerialStream_SaveCharFromISR(char ch)
{
    xStreamBufferSendFromISR(rxBuffer, (const void*)(&ch), sizeof(char), NULL);
}

uint8_t SerialStream_IsBufferReady(void)
{
    return (rxBufferSize != 0);
}
