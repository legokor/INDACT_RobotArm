/**
 * @file MessageHelper.cpp
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 * 
 * @brief This file contains the implementation of the MessageHelper class.
 * 
 */

#include "MessageHelper.h"

#include <Arduino.h>

#include <sstream>

namespace robot_arm_wifi
{
    bool MessageHelper::ReceiveMessage(std::string &message)
    {
        static char temp[2];
        static bool message_started = false;
        static int index = 0;

        while (Serial.available() > 0)
        {
            // Shift buffer
            temp[1] = temp[0];
            temp[0] = Serial.read();

            // Check for beginning
            if ((temp[1] == this->BEGIN_MARKER[0]) && (temp[0] == this->BEGIN_MARKER[1]))
            {
                index = 0;
                message_started = true;
            }
            else if (message_started)
            {
                // Check for end
                if ((temp[1] == this->END_MARKER[0]) && (temp[0] == this->END_MARKER[1]))
                {
                    receiveBuffer[index - 1] = '\0';
                    message_started = false;

                    message = receiveBuffer;
                    return true;
                }

                // Store character
                receiveBuffer[index] = temp[0];
                index++;

                // Check for buffer overflow
                if (index >= this->BUFFER_SIZE)
                {
                    message_started = false;
                }
            }
        }

        return false;
    }

    void MessageHelper::SendMessage(const std::string &message)
    {
        Serial.print(this->BEGIN_MARKER);
        Serial.print(message.c_str());
        Serial.print(this->END_MARKER);
    }

    void MessageHelper::SendConfirm()
    {
        this->SendMessage(STR_CONFIRM);
    }

    void MessageHelper::SendFail()
    {
        this->SendMessage(STR_FAIL);
    }

    void MessageHelper::SendIpAddress(const std::string &ip_address)
    {
        std::ostringstream oss;
        oss << STR_IP << " " << ip_address;
        this->SendMessage(oss.str());
    }

    void MessageHelper::SendAction(const std::string &action)
    {
        std::ostringstream oss;
        oss << STR_ACTION << " " << action;
        this->SendMessage(oss.str());
    }

} // namespace robot_arm_wifi
