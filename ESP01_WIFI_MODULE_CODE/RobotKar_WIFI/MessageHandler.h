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

namespace ESP8266_Controller
{

    /**
     * @brief Class for handling messages.
     */
    class MessageHandler
    {
    private:
        /** @brief Buffer for the incoming messages. */
        char *receiveBuffer;

        /** @brief Last received message. */
        char *messageBuffer;

        /** @brief Size of the buffers. */
        const int bufferSize;

        /** @brief Begin marker of the message. */
        const char *beginMarker;

        /** @brief End marker of the message. */
        const char *endMarker;

        /** @brief Temporary buffer for the incoming messages. */
        char temp[2];

        /** @brief Flag to indicate if a message has started. */
        bool messageStarted = false;

        /** @brief Index of the message buffer. */
        size_t index;

    public:
        /**
         * @brief Constructor.
         * @param receive_buffer Buffer for the incoming messages
         * @param message_buffer Buffer for the last received message
         * @param buffer_size Size of the buffers
         * @param begin_marker Begin marker of the message
         * @param end_marker End marker of the message
         */
        MessageHandler(char *receive_buffer, char *message_buffer, size_t buffer_size,
                       const char begin_marker[], const char end_marker[]);

        /**
         * @brief Destructor.
         */
        ~MessageHandler();

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