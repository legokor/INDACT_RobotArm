#ifndef MESSAGEHELPER_H_
#define MESSAGEHELPER_H_

/**
 * @file MessageHelper.h
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 *
 * @brief This file contains the declaration of the MessageHelper class.
 *
 */

#include <string>

#include "protocols.h"

namespace wifi_controller
{

    /**
     * @brief Class for handling messages.
     */
    class MessageHelper
    {
    public:
        /**
         * @brief Default constructor.
         */
        MessageHelper() = default;

        /**
         * @brief Receive a message from the serial port.
         *
         * @param message (Output) The message that was received. This is only valid if the function
         *          returns true.
         * @return True if a full message was received, false otherwise.
         */
        bool ReceiveMessage(std::string &message);

        /**
         * @brief Send a message to the serial port.
         *
         * @param message The message to be sent.
         */
        void SendMessage(const std::string &message);

        /**
         * @brief Send a confirmation message.
         */
        void SendConfirm();

        /**
         * @brief Send a fail message.
         */
        void SendFail();

        /**
         * @brief Send an IP address message.
         *
         * @param ip_address The IP address to be sent.
         */
        void SendIpAddress(const std::string &ip_address);

        /**
         * @brief Send an action message.
         *
         * @param action The action to be sent.
         */
        void SendAction(const std::string &action);

    private:
        const char *BEGIN_MARKER{MESSAGE_BEGIN_MARKER};
        const char *END_MARKER{MESSAGE_END_MARKER};

        char receiveBuffer[MESSAGE_MAX_SIZE + 2] = {'\0'};
        const int BUFFER_SIZE{MESSAGE_MAX_SIZE + 2}; // Message maximum size + end marker size

    }; // class MessageHelper

} // namespace wifi_controller

#endif // MESSAGEHELPER_H_
