/**
 * @file protocols.h
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 *
 * @brief This file contains the protocol messages for the communication between the WiFi module and
 *          the controller.
 * 
 */

#ifndef PROTOCOLS_H_
#define PROTOCOLS_H_

/**
 * @defgroup message_markers Message markers
 * @brief These symbols that are used to mark the boundaries of a message.
 * 
 * @details These markers form a message frame that is used to identify the beginning and the end
 *          of a message. The markers are used to synchronize the communication between the WiFi
 *          module and the main controller.
 * @note The markers must be 2 characters long and they need to be different.
 * @{
 */
#define MESSAGE_BEGIN_MARKER "+_"
#define MESSAGE_END_MARKER "*_"
/** @} */

/**
 * @brief The maximum size of a message in bytes (characters), excluding the markers.
 */
#define MESSAGE_MAX_SIZE 1024
/** @} */

/**
 * @brief The maximum amount of time in milliseconds to wait for a successful connection.
 */
#define CONNECT_TIMEOUT_MS 30000

/**
 * @defgroup command_type_strings Command type strings
 * @brief Strings that identify the type of a command message.
 *
 * @{
 */
#define STR_RESET "RESET"
#define STR_SYNCHRONIZE "SYNCHRONIZE"
#define STR_CONNECT_STATION "CONNECT_STATION"
#define STR_SETUP_ACCESS_POINT "SETUP_ACCESS_POINT"
#define STR_SSID "SSID"
#define STR_PASSWORD "PASSWORD"
#define STR_CONFIGURE_LAYOUT "CONFIGURE_LAYOUT"
#define STR_UPDATE_DATA "UPDATE_DATA"
/** @} */

/**
 * @defgroup response_type_strings Response type strings
 * @brief Strings that identify the type of a response message.
 *
 * @{
 */
#define STR_CONFIRM "OK"
#define STR_FAIL "FAIL"
#define STR_IP "IP"
#define STR_ACTION "ACTION"
/** @} */

#endif // PROTOCOLS_H_
