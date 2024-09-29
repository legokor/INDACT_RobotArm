#ifndef WIFICONTROLLER_USERINTERFACE_H_
#define WIFICONTROLLER_USERINTERFACE_H_

#include <stdbool.h>

#include "WifiController/Common.h"
#include "WifiController/ControlTable.h"
#include "WifiController/TextField.h"

/**
 * @brief Structure representing the user interface.
 * 
 * @note The fields of this structure should not be accessed directly. Use the provided functions instead.
 */
typedef struct WifiController_UserInterface
{
    char *title; /**< The title of the page. */
    char *pageHeader; /**< The header of the page. */
    WifiController_TextFieldList_t textFieldTop; /**< The top text field. */
    WifiController_TextFieldList_t textFieldBottom; /**< The bottom text field. */
    WifiController_ControlTable_t controlTable; /**< The control table. */
    char *userScript; /**< The user script. */
} WifiController_UserInterface_t;

/**
 * @brief Initializes the user interface.
 * 
 * @param this The user interface.
 * @return True if the user interface was initialized successfully, false otherwise.
 * 
 * @note This function must be called before any other function of the user interface.
 * @note The memory for the WifiController_UserInterface_t struct must be allocated before calling this function.
 */
bool WifiController_UserInterface_Init(WifiController_UserInterface_t *this);

/**
 * @brief Deinitialize the user interface.
 * 
 * @param this The user interface.
 * 
 * @note This function must be called when the user interface is no longer needed.
 */
void WifiController_UserInterface_Delete(WifiController_UserInterface_t *this);

/**
 * @brief Set the title.
 * 
 * @param this The user interface.
 * @param value The title to set.
 * @return True if the title was set successfully, false otherwise.
 */
bool WifiController_UserInterface_SetTitle(WifiController_UserInterface_t *this, const char *value);

/**
 * @brief Set the page header.
 * 
 * @param this The user interface.
 * @param value The page header to set.
 * @return True if the page header was set successfully, false otherwise.
 */
bool WifiController_UserInterface_SetPageHeader(WifiController_UserInterface_t *this, const char *value);

/**
 * @brief Set the user script.
 * 
 * @param this The user interface.
 * @param value The user script to set.
 * @return True if the user script was set successfully, false otherwise.
 */
bool WifiController_UserInterface_SetUserScript(WifiController_UserInterface_t *this, const char *value);

/**
 * @brief Get the configuration as a JSON string.
 * 
 * @param this The user interface.
 * @param buffer The buffer to store the JSON string in.
 * @param max_length The maximum length of the buffer.
 * @return The number of characters written to the buffer.
 * 
 * @details The "configuration" is the data that is sent to the client when the client connects to the server.
 */
int WifiController_UserInterface_GetConfigurationJSON(const WifiController_UserInterface_t *this, char *buffer, int max_length);

/**
 * @brief Get the data as a JSON string.
 * 
 * @param this The user interface.
 * @param buffer The buffer to store the JSON string in.
 * @param max_length The maximum length of the buffer.
 * @return The number of characters written to the buffer.
 * 
 * @details The "data" is the data that is sent to the client when the client requests an update.
 */
int WifiController_UserInterface_GetDataJSON(const WifiController_UserInterface_t *this, char *buffer, int max_length);

#endif /* WIFICONTROLLER_USERINTERFACE_H_ */
