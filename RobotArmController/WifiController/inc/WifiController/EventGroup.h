#ifndef WIFICONTROLLER_EVENTGROUP_H_
#define WIFICONTROLLER_EVENTGROUP_H_

#include <FreeRTOS.h>
#include <event_groups.h>

#include "WifiController/Common.h"

/** Bit for OK event. */
#define WifiController_EventGroup_OkBit ((EventBits_t)(1 << 0))
/** Bit for FAIL event. */
#define WifiController_EventGroup_FailBit ((EventBits_t)(1 << 1))
/** Bit for IP event. */
#define WifiController_EventGroup_IpBit ((EventBits_t)(1 << 2))

/**
 * @brief Initializes the event group.
 * 
 * @return Error code.
 * 
 * @note This function must be called before any other function in this file.
 */
WC_ErrorCode_t WifiController_EventGroup_Init();

/**
 * @brief Deletes the event group.
 */
void WifiController_EventGroup_Delete();

/**
 * @brief Gets the event group handle.
 * 
 * @return Event group handle.
 */
EventGroupHandle_t WifiController_EventGroup_GetHandle();

#endif /* WIFICONTROLLER_EVENTGROUP_H_ */
