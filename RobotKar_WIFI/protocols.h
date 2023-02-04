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

/**@{*/
/**
 * @brief The sizes of the different strings that the controller can send to
 *        the WiFi module.
 */
#define MESSAGE_MAX_SIZE 128
#define WIFI_STRING_SIZE 32
/**@}*/

/**@{*/
/**
 * @brief Command messages from the controller to the WiFi module.
 */
#define STR_RESET "RS"
#define STR_CONNECT_STATION "CO"
#define STR_SETUP_ACCESS_POINT "AP"
#define STR_SSID "SS"
#define STR_PASSWORD "PS"
#define STR_STATUS "ST"
#define STR_CHANGE_TO_CYLINDRICAL "CTC"
#define STR_CHANGE_TO_DESCARTES "CTD"
/**@}*/

/**@{*/
/**
 * @brief Response messages from the WiFi module to the controller.
 */
#define STR_CONFIRM "OK"
#define STR_FAIL "FA"
/**@}*/

#endif /* PROTOCOLS_H_ */
