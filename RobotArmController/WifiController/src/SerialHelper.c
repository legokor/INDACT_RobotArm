#include <WifiController/SerialHelper.h>

#include <stdbool.h>
#include <string.h>

#include "HardwareSupport/serial.h"
#include "WifiController/Common.h"
#include "WifiController/protocols.h"

static HardwareSupport_Serial_t serial;
static const char *beginMarker = MESSAGE_BEGIN_MARKER;
static const char *endMarker = MESSAGE_END_MARKER;

WC_ErrorCode_t WifiController_SerialHelper_Init()
{
    if (!HardwareSupport_Serial_Init(&serial, WifiController_SerialHelper_Config_Huart, (MESSAGE_MAX_SIZE + 3)))
    {
        return WC_ErrorCode_MEMORY_ALLOCATION;
    }
    return WC_ErrorCode_NONE;
}

void WifiController_SerialHelper_Delete()
{
    HardwareSupport_Serial_Delete(&serial);
}

WC_ErrorCode_t WifiController_SerialHelper_SendMessage(const char *message)
{
    if (strlen(message) > MESSAGE_MAX_SIZE)
    {
        return WC_ErrorCode_PARAMETER;
    }

    HardwareSupport_Serial_Write(&serial, beginMarker, MESSAGE_MARKER_SIZE);
    HardwareSupport_Serial_Write(&serial, message, strlen(message));
    HardwareSupport_Serial_Write(&serial, endMarker, MESSAGE_MARKER_SIZE);
    return WC_ErrorCode_NONE;
}

WC_ErrorCode_t WifiController_SerialHelper_ReadMessage(char *buffer, int max_length)
{
    char temp[2] = { '\0' };
    bool message_started = false;
    int index = 0;

    while (true)
    {
        // Save the last character and read a new one
        temp[1] = temp[0];
        HardwareSupport_Serial_Read(&serial, temp, 1);

        if ((temp[1] == beginMarker[0]) && (temp[0] == beginMarker[1]))
        {
            // Start of message: clear buffer, reset index and set flag
            buffer[0] = '\0';
            index = 0;
            message_started = true;
        }
        else if (message_started)
        {
            if ((temp[1] == endMarker[0]) && (temp[0] == endMarker[1]))
            {
                // End of message: clear flag and break the loop
                buffer[index - 1] = '\0';
                message_started = false;
                break;
            }
            else
            {
                // Add the character to the message buffer
                buffer[index] = temp[0];
                ++index;

                if (index >= max_length)
                {
                    return WC_ErrorCode_MESSAGE_LENGTH;
                }
            }
        }
    }

    return WC_ErrorCode_NONE;
}

void WifiController_SerialHelper_UartRxCallback()
{
    HardwareSupport_Serial_UartRxCallback(&serial);
}
