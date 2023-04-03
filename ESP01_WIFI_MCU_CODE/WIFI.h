/**
 ***************************************************************************************************
 * @file WIFI.h
 *
 * @date 2023. 01. 19.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief This file provides a simple to use interface for the communication with the ESP-01 WiFi
 *          module.
 ***************************************************************************************************
 */

#ifndef WIFI_H_
#define WIFI_H_

#include "usart.h"

// I want to use the stm32l4xx_hal_uart.h file (include below) but the compiler does not compile the
// project if I include it. Including this file would make the inclusion of usart.h unnecessary.
// #include "stm32l4xx_hal_uart.h"

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Type definitions
// /////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief The types of requests that the WiFi module can send to the controller.
 */
typedef enum
{
    INVALID, /**< Invalid request*/
    AXIS_A_PLUS, /**< Move the robot in the positive direction of the A axis */
    AXIS_A_MINUS, /**< Move the robot in the negative direction of the A axis */
    AXIS_B_PLUS, /**< Move the robot in the positive direction of the B axis */
    AXIS_B_MINUS, /**< Move the robot in the negative direction of the B axis */
    AXIS_C_PLUS, /**< Move the robot in the positive direction of the C axis */
    AXIS_C_MINUS, /**< Move the robot in the negative direction of the C axis */
    HOMING, /**< Begin homing sequence */
    CHANGE_COORDINATES, /**< Change the coordinate system of the robot arm */
} WIFI_RequestType;

/**
 * @brief The types of return values that the WIFI function can give.
 */
typedef enum
{
    WIFI_CONFIRM, /**< Task completed successfuly */
    WIFI_FAIL, /**< Task execution failed */
    WIFI_ERROR /**< Communication error */
} WIFI_StatusTypedef;

/**
 * @brief The possible coordinate systems that the controller can use.
 */
typedef enum
{
    RECTANGULAR, /**< Rectangular coordinate system */
    CYLINDRICAL /**< Cylindrical coordinate system */
} WIFI_CoordinateSystem;

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Function declarations
// /////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initialize the WiFi module communication interface (WIFI).
 * @param huart Handle of the UART peripherial that the WiFi module uses
 */
void WIFI_InitWIFI(UART_HandleTypeDef *huart);

/** @brief Delete the WiFi module communication interface. */
void WIFI_DeleteWIFI(void);

/**
 * @brief Get the initialization state of the WIFI.
 * @return 1 if the WIFI is initialized, 0 otherwise
 */
uint8_t WIFI_GetInitState(void);

/**
 * @brief Getter function for the WiFi SSID.
 * @return Currently used WiFi SSID if it is set, NULL otherwise
 */
const char* WIFI_GetSSID(void);

/**
 * @brief Getter function for the WiFi password.
 * @return Currently used WiFi password if it is set, NULL otherwise
 */
const char* WIFI_GetPassword(void);

/**
 * @brief Getter function for the WiFi IP address.
 * @return Currently used WiFi IP address if it is set, NULL otherwise
 */
const char* WIFI_GetIP(void);

/**
 * @brief Setter function for the WiFi SSID.
 * @details This function sets the SSID stored on this device and attempts to set this parameter
 *          inside the WiFi module too.
 * @param ssid SSID in character string format
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_SetSSID(const char *ssid);

/**
 * @brief Setter function for the WiFi password.
 * @details This function sets the password stored on this device and attempts to set this parameter
 *          inside the WiFi module too.
 * @param password Password in character string format
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_SetPassword(const char *password);

/**
 * @brief Setter function for the status message.
 * @details This function attempts to set the status message displayed on the graphical user
 *          interface of the WiFi module.
 * @param status Status message in character string format
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_SetStatusMessage(const char *status);

/**
 * @brief Attempt to change the coordinate system displayed by the WiFi module.
 * @param csys Type of the coordinate system
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_SetCoordinateSystem(WIFI_CoordinateSystem csys);

/**
 * @brief Setter function for the WiFi password.
 * @details This function attempts to set the position displayed on the graphical user interface of
 *          the WiFi module.
 * @param position Position in character string format
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_SetPosition(const char *position);

/**
 * @brief Attempt to connect to an existing WiFi network with the WiFi module.
 * @details The SSID and the password of the network have to be already set.
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_ConnectToNetwork(void);

/**
 * @brief Attempt to set up a soft access point with the WiFi module.
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_SetupAccessPoint(void);

/**
 * @brief Attempt to reset the WiFi module.
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_ResetModule(void);

/**
 * @brief Attempt synchronization with the WiFi module.
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_SynchronizeModule(void);

/**
 * @brief Receive the next request from the WiFi module.
 * @return Request from the WiFi module
 */
WIFI_RequestType WIFI_ReceiveNextRequest(void);

/**
 * @brief Receive and process data coming on the serial communicaton channel.
 * @details This is the method that has to be called in the WIFI's receiving task loop.
 */
void WIFI_Receive(void);

#endif /* WIFI_H_ */
