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
/**@}*/

/**@{*/
/**
 * @brief Response messages from the WiFi module to the controller.
 */
#define STR_CONFIRM "OK"
#define STR_FAIL "FA"
/**@}*/
