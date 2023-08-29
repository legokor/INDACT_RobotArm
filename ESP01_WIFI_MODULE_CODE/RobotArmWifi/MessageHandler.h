/**
 ***************************************************************************************************
 * @file MessageHandler.h
 *
 * @date 2023. 04. 14.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief This file contains the declaration of the MessageHandler class and related data types.
 ***************************************************************************************************
 */

#ifndef MESSAGEHANDLER_H_
#define MESSAGEHANDLER_H_

#include <stddef.h>

#include "protocols.h"

namespace ESP8266_Controller
{

    /**
     * @brief Class for handling messages.
     */
    class MessageHandler
    {
    private:
        char *receiveBuffer;
        char *messageBuffer;
        const int bufferSize;

        const char *beginMarker = MESSAGE_BEGIN_MARKER;
        const char *endMarker = MESSAGE_END_MARKER;

        char temp[2];
        bool messageStarted = false;
        size_t index;

    public:
        /**
         * @brief Constructor with static buffer allocation.
         * @param receive_buffer Buffer for the incoming messages
         * @param message_buffer Buffer for the last received message
         * @param buffer_size Size of the buffers
         */
        MessageHandler(char *receive_buffer, char *message_buffer, size_t buffer_size);

        /**
         * @brief Receive a message from Serial.
         * @details The message has to be surrounded by the begin and end markers.
         * @param buffer Buffer for the message
         * @param size Size of the buffer
         * @return True if a message was received, false otherwise
         */
        bool receiveMessage();

        /**
         * @brief Get the last received message.
         * @return Last received message
         */
        const char *getMessage(void) const;

        /**
         * @brief Send a message through Serial.
         * @details The message will be surrounded by the begin and end markers.
         * @param msg Message to send
         */
        void sendMessage(const char *msg);

    }; /* class MessageHandler */

} /* namespace ESP8266_Controller */

#endif /* MESSAGEHANDLER_H_ */