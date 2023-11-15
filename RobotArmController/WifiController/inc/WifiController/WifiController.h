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
 * WifiController_ActionList_t actionList;
 *
 * void handleButtonAction(const char *args)
 * {
 *     // Use the default GUI of the Wi-F controller which provides simple buttons for moving the motors
 *     // of the robotarm.
 *     char buffer[7] = {'\0'};
 *     strcat(buffer, "- ");
 *
 *     const char *arg_text = "btn=";
 *     const size_t arg_text_length = 4;
 *
 *     char *p = strstr(args, arg_text);
 *     if ((p == NULL) || (strlen(p) < (arg_text_length + 2)))
 *     {
 *         strncat(buffer, "xx", 2 + 1);
 *     }
 *     else
 *     {
 *         strncat(buffer, p + arg_text_length, 2 + 1);
 *     }
 *
 *     strcat(buffer, "\r\n");
 *
 *     HAL_UART_Transmit(USB_HUART, (uint8_t *)buffer, strlen(buffer), 200);
 * }
 *
 * void WifiSetupTask(void *pvParameters)
 * {
 *     const char *ap_ssid = "indact_wific";
 *     const char *ap_password = "pirosalma";
 *
 *     // 1.) Define actions
 *     WifiController_ActionList_Init(&actionList);
 *     WifiController_ActionList_Add(&actionList, "/button", handleButtonAction);
 *
 *     // 2.) Call WifiController initialization function
 *     configASSERT(WifiController_WifiController_Init(&actionList) == WifiController_ErrorCode_NONE);
 *
 *     // 3.) Start the receiver task
 *     xTaskCreate(WifiReceiveTask, "wifi_receive", configMINIMAL_STACK_SIZE * 4, NULL, configMAX_PRIORITIES / 4 * 3, NULL);
 *
 *     // 3.) Set parameters for the Wi-Fi module
 *     configASSERT(WifiController_WifiController_ResetModule() == WifiController_ErrorCode_NONE);
 *     // Wait for the module to reset
 *     vTaskDelay(pdMS_TO_TICKS(3 * 1000));
 *     configASSERT(WifiController_WifiController_SetSsid(ap_ssid) == WifiController_ErrorCode_NONE);
 *     configASSERT(WifiController_WifiController_SetPassword(ap_password) == WifiController_ErrorCode_NONE);
 *
 *     // 4.) Start the access point or station mode
 *     configASSERT(WifiController_WifiController_BeginAccessPoint(10 * 1000) == WifiController_ErrorCode_NONE);
 *
 *     vTaskDelete(NULL);
 * }
 *
 * void WifiReceiveTask(void *pvParameters)
 * {
 *     while (1)
 *     {
 *         WifiController_ErrorCode_t e = WifiController_WifiController_Receive();
 *         if (e != WifiController_ErrorCode_NONE)
 *         {
 *             // Handle error.
 *         }
 *     }
 * }
 */

/**
 * @brief Initializes the WifiController.
 * 
 * @param action_list The action list.
 * @return Error code.
 * 
 * @note This function must be called before any other function in this file.
 */
WifiController_ErrorCode_t WifiController_WifiController_Init(WifiController_ActionList_t *action_list);

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
 * @brief Set the SSID.
 * 
 * @param value The SSID to set.
 * @return Error code.
 */
WifiController_ErrorCode_t WifiController_WifiController_SetSsid(const char *value);

/**
 * @brief Set the password.
 * 
 * @param value The password to set.
 * @return Error code.
 */
WifiController_ErrorCode_t WifiController_WifiController_SetPassword(const char *value);

/**
 * @brief Reset the module.
 * 
 * @return Error code.
 */
WifiController_ErrorCode_t WifiController_WifiController_ResetModule();

/**
 * @brief Begin the station mode.
 * 
 * @param timeout_ms The timeout in milliseconds.
 * @return Error code.
 */
WifiController_ErrorCode_t WifiController_WifiController_BeginStation(int timeout_ms);

/**
 * @brief Begin the access point mode.
 * 
 * @param timeout_ms The timeout in milliseconds.
 * @return Error code.
 */
WifiController_ErrorCode_t WifiController_WifiController_BeginAccessPoint(int timeout_ms);

/**
 * @brief Send the user interface configuration.
 * 
 * @param ui The user interface.
 * @return Error code.
 */
WifiController_ErrorCode_t WifiController_WifiController_SendConfiguration(const WifiController_UserInterface_t *ui);

/**
 * @brief Send the user interface data update.
 * 
 * @param ui The user interface.
 * @return Error code.
 */
WifiController_ErrorCode_t WifiController_WifiController_SendDataUpdate(const WifiController_UserInterface_t *ui);

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
WifiController_ErrorCode_t WifiController_WifiController_Receive();

#endif /* WIFICONTROLLER_WIFICONTROLLER_H_ */
