/**
 ***************************************************************************************************
 * @file protocols.h
 *
 * @date Feb 01, 2023
 * @author Péter Varga
 ***************************************************************************************************
 * @brief This file contains the protocol messages for the communication between the WiFi module and
 * the controller.
 ***************************************************************************************************
 */

#ifndef PROTOCOLS_H_
#define PROTOCOLS_H_

/**
 * @defgroup message_markers Message markers
 * @brief The symbols that are used to mark the boundaries of a message.
 * @note The begin marker must be 2 characters long.
 * @note The end marker must be 2 characters long.
 * @note The begin marker and the end marker must be different.
 * @{
 */
#define MESSAGE_BEGIN_MARKER "+_"
#define MESSAGE_END_MARKER "*_"
/** @} */

/**
 * @defgroup string_sizes String sizes
 * @brief The sizes of the different strings that are used in the communication between the WiFi
 * module and the controller.
 * @{
 */
#define MESSAGE_MAX_SIZE 1024
#define WIFI_STRING_SIZE 32
/** @} */

/** Number of times that the synchronization code has to be sent on a synchronization attempt. */
#define SYNC_NUMBER 4

/** The maximum amount of time in milliseconds to wait for a successful connection. */
#define CONNECT_TIMEOUT_MS 30000

/**
 * @defgroup command_type_strings Command type strings
 * @brief Strings that are used to identify the type of a command message.
 * @{
 */
#define STR_RESET "RS"
#define STR_SYNCHRONIZE "SYNC"
#define STR_CONNECT_STATION "CO"
#define STR_SETUP_ACCESS_POINT "AP"
#define STR_SSID "SS"
#define STR_PASSWORD "PS"
#define STR_CONFIGURE_LAYOUT "CL"
#define STR_UPDATE_DATA "UD"
/** @} */

/**
 * @defgroup response_type_strings Response type strings
 * @brief Strings that are used to identify the type of a response message.
 * @{
 */
#define STR_SYNC_CODE "@@"
#define STR_CONFIRM "OK"
#define STR_FAIL "FA"
#define STR_IP "IP"
#define STR_ACTION "AC"
/** @} */

#endif /* PROTOCOLS_H_ */