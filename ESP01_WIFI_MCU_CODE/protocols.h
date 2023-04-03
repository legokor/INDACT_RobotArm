/**
 *******************************************************************************
 * @file protocols.h
 *
 * @date Feb 01, 2023
 * @author Varga Peter
 *******************************************************************************
 * @brief This file contains the protocol messages for the communication between
 *        the WiFi module and the controller.
 *******************************************************************************
 */

#ifndef PROTOCOLS_H_
#define PROTOCOLS_H_

#define MESSAGE_BEGIN_MARKER "+_"
#define MESSAGE_END_MARKER "\r\n"

/**
 * @brief The sizes of the different strings that the controller can send to
 *        the WiFi module.
 */
/**@{*/
#define MESSAGE_MAX_SIZE 128
#define WIFI_STRING_SIZE 32
/**@}*/

/**
 * @brief Number of times that the synchronization code has to be sent on a synchronization attempt.
 */
#define SYNC_NUMBER 4

/**
 * @brief The maximum amount of time in milliseconds to wait for a successful connection.
 */
#define CONNECT_TIMEOUT_MS 30000

/**
 * @brief Command messages from the controller to the WiFi module.
 */
/**@{*/
#define STR_RESET "RS"
#define STR_SYNCHRONIZE "SYNC"
#define STR_CONNECT_STATION "CO"
#define STR_SETUP_ACCESS_POINT "AP"
#define STR_SSID "SS"
#define STR_PASSWORD "PS"
#define STR_STATUS "ST"
#define STR_CHANGE_TO_CYLINDRICAL "CTC"
#define STR_CHANGE_TO_RECTANGULAR "CTR"
#define STR_POSITION "PO"
/**@}*/

/**
 * @brief The size of the requests that the WiFi module can send to the controller.
 */
#define MODULE_REQUEST_SIZE 2

/**@{*/
/**
 * @brief Response messages from the WiFi module to the controller.
 */
#define STR_SYNC_CODE "@@"
#define STR_CONFIRM "OK"
#define STR_FAIL "FA"
#define STR_IP "IP"
/**@}*/

/**@{*/
/**
 * @brief Request messages from the WiFi module to the controller.
 */
#define STR_AXIS_A_PLUS "ap"
#define STR_AXIS_A_MINUS "am"
#define STR_AXIS_B_PLUS "bp"
#define STR_AXIS_B_MINUS "bm"
#define STR_AXIS_C_PLUS "cp"
#define STR_AXIS_C_MINUS "cm"
#define STR_HOMING "hx"
#define STR_CHANGE_COORDINATES "kx"
/**@}*/

#endif /* PROTOCOLS_H_ */
