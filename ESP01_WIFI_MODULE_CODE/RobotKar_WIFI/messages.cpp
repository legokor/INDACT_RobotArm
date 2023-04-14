/**
 ***************************************************************************************************
 * @file messages.cpp
 *
 * @date 2023. 04. 14.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief Implementation of messages.h.
 ***************************************************************************************************
 */

#include "messages.h"

#include <regex>

#include "protocols.h"

using namespace std;

/** @brief Begin marker of the messages. */
const char msgBeginMarker[] = MESSAGE_BEGIN_MARKER;
/** @brief End marker of the messages. */
const char msgEndMarker[] = MESSAGE_END_MARKER;

bool receiveMessage(char *buffer, size_t size)
{
    static char temp[2] = {0};
    static bool msg_started = false;
    static size_t idx = 0;

    // Check if there is any data in the Serial buffer
    while (Serial.available() > 0)
    {
        // Shift the temporary buffer
        temp[1] = temp[0];
        temp[0] = Serial.read();

        // Check if the message has started
        if ((temp[1] == msgBeginMarker[0]) && (temp[0] == msgBeginMarker[1]))
        {
            idx = 0;
            msg_started = true;
        }
        else if (msg_started == true)
        {
            // Check if the message has ended
            if ((temp[1] == msgEndMarker[0]) && (temp[0] == msgEndMarker[1]))
            {
                // Terminate the string
                buffer[idx - 1] = '\0';

                idx = 0;
                msg_started = false;

                // Return true if a message was received
                return true;
            }
            else
            {
                // Copy the character to the buffer
                buffer[idx] = temp[0];
                idx++;

                // Check if the buffer is full
                if (idx >= size)
                {
                    idx = 0;
                    msg_started = false;
                }
            }
        }
    }

    // Return false if no message was received
    return false;
}

void sendMessage(const char *msg)
{
    Serial.print(msgBeginMarker);
    Serial.print(msg);
    Serial.print(msgEndMarker);
}

inline void sendConfirm(void)
{
    sendMessage(STR_CONFIRM);
}

inline void sendFail(void)
{
    sendMessage(STR_FAIL);
}
