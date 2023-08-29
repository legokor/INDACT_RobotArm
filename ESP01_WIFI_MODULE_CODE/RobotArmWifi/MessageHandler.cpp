/**
 ***************************************************************************************************
 * @file MessageHandler.cpp
 *
 * @date 2023. 04. 14.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief Implementation of MessageHandler.h.
 ***************************************************************************************************
 */

#include "MessageHandler.h"

#include <Arduino.h>

using namespace std;
using namespace ESP8266_Controller;

MessageHandler::MessageHandler(char *receive_buffer, char *message_buffer, size_t buffer_size)
    : receiveBuffer(receive_buffer), messageBuffer(message_buffer), bufferSize(buffer_size)
{
}

bool MessageHandler::receiveMessage()
{
    // Check if there is any data in the Serial buffer
    while (Serial.available() > 0)
    {
        // Shift the temporary buffer
        temp[1] = temp[0];
        temp[0] = Serial.read();

        // Check if the message has started
        if ((temp[1] == beginMarker[0]) && (temp[0] == beginMarker[1]))
        {
            index = 0;
            messageStarted = true;
        }
        else if (messageStarted == true)
        {
            // Check if the message has ended
            if ((temp[1] == endMarker[0]) && (temp[0] == endMarker[1]))
            {
                // Terminate the string
                receiveBuffer[index - 1] = '\0';
                strcpy(messageBuffer, receiveBuffer);

                index = 0;
                messageStarted = false;

                // Return true if a message was received
                return true;
            }
            else
            {
                // Copy the character to the buffer
                receiveBuffer[index] = temp[0];
                index++;

                // Check if the buffer is full
                if (index >= bufferSize)
                {
                    index = 0;
                    messageStarted = false;
                }
            }
        }
    }

    // Return false if no message was received
    return false;
}

const char *MessageHandler::getMessage(void) const
{
    return messageBuffer;
}

void MessageHandler::sendMessage(const char *message)
{
    Serial.print(beginMarker);
    Serial.print(message);
    Serial.print(endMarker);
}
