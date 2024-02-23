#ifndef WIFICONTROLLER_WIFICONTROLLER_H_
#define WIFICONTROLLER_WIFICONTROLLER_H_

#include "WifiController/ActionList.h"
#include "WifiController/Common.h"
#include "WifiController/IpAddress.h"
#include "WifiController/UserInterface.h"

/**
 * @file
 *
 * @details
 * Example use:
 *
 * void handleButtonAction(const char *args)
 * {
 *     // Use the default GUI of the Wi-Fi controller which provides simple buttons for moving the motors
 *     // of the robotarm.
 *     char btn[3] = { '\0' };
 *
 *     const char *arg_text = "btn=";
 *     const size_t arg_text_length = 4;
 *
 *     char *p = strstr(args, arg_text);
 *     if ((p != NULL) && (strlen(p) >= (arg_text_length + 2)))
 *     {
 *         strncpy(btn, p + arg_text_length, 2);
 *         sendNewPosition(btn);
 *     }
 *     else
 *     {
 *         strcpy(btn, "xx");
 *     }
 *
 *     logInfo("Button action: %s", btn);
 * }
 *
 * // Call after receive task is started.
 * void setupWifi()
 * {
 *     const char *ap_ssid = "indactrobot";
 *     const char *ap_password = "pirosalma";
 *
 *     // Wait for the module to start after power-up
 *     vTaskDelay(pdMS_TO_TICKS(2 * 1000));
 *
 *     if (WifiController_WifiController_ResetModule() != WC_ErrorCode_NONE)
 *     {
 *         logError("Wi-Fi reset error.");
 *         return;
 *     }
 *
 *     // Wait for the module to reset
 *     vTaskDelay(pdMS_TO_TICKS(2 * 1000));
 *
 *     if (WifiController_WifiController_SetSsid(ap_ssid) != WC_ErrorCode_NONE)
 *     {
 *         logError("Wi-Fi set SSID error.");
 *         return;
 *     }
 *
 *     if (WifiController_WifiController_SetPassword(ap_password) != WC_ErrorCode_NONE)
 *     {
 *         logError("Wi-Fi set password error.");
 *         return;
 *     }
 *
 *     if (WifiController_WifiController_BeginAccessPoint(10 * 1000) != WC_ErrorCode_NONE)
 *     {
 *         logError("Wi-Fi begin access point error.");
 *         return;
 *     }
 * }
 *
 * void wifiReceiveTask(void *pvParameters)
 * {
 *     configASSERT(WifiController_WifiController_Init() == WC_ErrorCode_NONE);
 *
 *     WifiController_ActionList_t *action_list = WifiController_WifiController_GetActionList();
 *     WifiController_ActionList_Add(action_list, "/button", handleButtonAction);
 *
 *     while (1)
 *     {
 *         WifiController_WifiController_Receive();
 *     }
 * }
 */

/**
 * @brief Initializes the WifiController.
 *
 * @return Error code.
 *
 * @note This function must be called before any other function in this file.
 */
WC_ErrorCode_t WifiController_WifiController_Init();

/**
 * @brief Deletes the WifiController.
 */
void WifiController_WifiController_Delete();

/**
 * @brief Gets the SSID.
 * 
 * @return The SSID.
 */
const char* WifiController_WifiController_GetSsid();

/**
 * @brief Gets the password.
 * 
 * @return The password.
 */
const char* WifiController_WifiController_GetPassword();

/**
 * @brief Gets the IP address.
 * 
 * @return The IP address.
 */
WifiController_IpAddress_t* WifiController_WifiController_GetIPAddress();

/**
 * @brief Get the pointer of the action list of the Wi-Fi controller.
 * @return Pointer to the action list of the Wi-Fi controller.
 */
WifiController_ActionList_t *WifiController_WifiController_GetActionList();

/**
 * @brief Set the SSID.
 * 
 * @param value The SSID to set.
 * @return Error code.
 */
WC_ErrorCode_t WifiController_WifiController_SetSsid(const char *value);

/**
 * @brief Set the password.
 * 
 * @param value The password to set.
 * @return Error code.
 */
WC_ErrorCode_t WifiController_WifiController_SetPassword(const char *value);

/**
 * @brief Reset the module.
 * 
 * @return Error code.
 */
WC_ErrorCode_t WifiController_WifiController_ResetModule();

/**
 * @brief Begin the station mode.
 * 
 * @param timeout_ms The timeout in milliseconds.
 * @return Error code.
 */
WC_ErrorCode_t WifiController_WifiController_BeginStation(int timeout_ms);

/**
 * @brief Begin the access point mode.
 * 
 * @param timeout_ms The timeout in milliseconds.
 * @return Error code.
 */
WC_ErrorCode_t WifiController_WifiController_BeginAccessPoint(int timeout_ms);

/**
 * @brief Send the user interface configuration.
 * 
 * @param ui The user interface.
 * @return Error code.
 */
WC_ErrorCode_t WifiController_WifiController_SendConfiguration(const WifiController_UserInterface_t *ui);

/**
 * @brief Send the user interface data update.
 * 
 * @param ui The user interface.
 * @return Error code.
 */
WC_ErrorCode_t WifiController_WifiController_SendDataUpdate(const WifiController_UserInterface_t *ui);

/**
 * @brief Receive a message.
 * 
 * @return Error code.
 * 
 * @details This function should be called in an infinite loop. It will block
 *     until a message is received. When a message is received, it will be
 *     processed and the appropriate action will be executed by the
 *     WifiController.
 */
WC_ErrorCode_t WifiController_WifiController_Receive();

#endif /* WIFICONTROLLER_WIFICONTROLLER_H_ */
