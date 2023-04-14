/**
 ***************************************************************************************************
 * @file messages.h
 *
 * @date 2023. 04. 14.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief
 ***************************************************************************************************
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_

/**
 * @brief Receive a message from Serial.
 * @details The message has to be surrounded by the begin and end markers.
 * @param buffer Buffer for the message
 * @param size Size of the buffer
 * @return True if a message was received, false otherwise
 */
bool receiveMessage(char *buffer, size_t size);

/**
 * @brief Send a message through Serial.
 * @details The message will be surrounded by the begin and end markers.
 * @param msg Message to send
 */
void sendMessage(const char *msg);

/**
 * @brief Send a confirmation message through Serial.
 */
inline void sendConfirm(void);

/**
 * @brief Send notification about failure through Serial.
 */
inline void sendFail(void);

#endif /* MESSAGES_H_ */