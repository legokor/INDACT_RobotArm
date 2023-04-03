/**
 ***************************************************************************************************
 * @file WIFI_Config.h
 *
 * @date 2023. 04. 29.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief This file contains the configuration of the WIFI.
 ***************************************************************************************************
 */

#ifndef WIFI_CONFIG_H_
#define WIFI_CONFIG_H_

#include "protocols.h"

/** Handle of the UART peripherial that the WiFi module can use. */
#define WIFI_UART_HANDLE (&huart3)

/** Size of the request queue. */
#define REQUEST_QUEUE_SIZE 32

/** Amount of time (in ms) to wait for an empty space when the request queue is full. */
#define REQUEST_QUEUE_TIMEOUT_MS 100
/** Amount of time (in ms) to wait for a reply to a message. */
#define REPLY_TIMEOUT_MS 1000
/** Amount of time (in ms) to wait for the access point setup. */
#define SETUP_TIMEOUT_MS CONNECT_TIMEOUT_MS
/** Amount of time (in ms) to wait for a synchronization code. */
#define SYNC_TIMEOUT_MS 1000

#endif /* WIFI_CONFIG_H_ */
