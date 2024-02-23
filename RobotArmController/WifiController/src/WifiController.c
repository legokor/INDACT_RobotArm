#include "WifiController/WifiController.h"

#include <stdbool.h>
#include <string.h>

#include <WifiController/EventGroup.h>
#include <WifiController/protocols.h>
#include "WifiController/SerialHelper.h"

#define SSID_MIN_LENGTH 3
#define SSID_MAX_LENGTH 32
#define PASSWORD_MIN_LENGTH 8
#define PASSWORD_MAX_LENGTH 63

#define RESPONSE_TYPE_MAX_LENGTH 16

static bool initialized = false;

static char transmitBuffer[MESSAGE_MAX_SIZE + 1];
static char receiveBuffer[MESSAGE_MAX_SIZE + 1];
static char responseTypeBuffer[RESPONSE_TYPE_MAX_LENGTH + 1];
static char responseParamsBuffer[MESSAGE_MAX_SIZE + 1];

static char ssid[SSID_MAX_LENGTH + 1] = { '\0' };
static char password[PASSWORD_MAX_LENGTH + 1] = { '\0' };
static WifiController_IpAddress_t ipAddress;
static int responseTimeout = 1000;
static WifiController_ActionList_t actionList;

static WC_ErrorCode_t wait_confirm();
static WC_ErrorCode_t wait_ip_address(int timeout_ms);
static bool check_ssid(const char *value);
static bool check_password(const char *value);
static WC_ErrorCode_t process_message(const char *message);
static WC_ErrorCode_t execute_message(const char *cmd, const char *params);
static WC_ErrorCode_t call_action(const char *params);

WC_ErrorCode_t WifiController_WifiController_Init()
{
    WC_ErrorCode_t error_code = WC_ErrorCode_UNKNOWN;

    error_code = WifiController_SerialHelper_Init();
    if (error_code != WC_ErrorCode_NONE)
    {
        return error_code;
    }

    error_code = WifiController_EventGroup_Init();
    if (error_code != WC_ErrorCode_NONE)
    {
        return error_code;
    }

    WifiController_IpAddress_Init(&ipAddress);
    WifiController_ActionList_Init(&actionList);

    initialized = true;
    return WC_ErrorCode_NONE;
}

void WifiController_WifiController_Delete()
{
    initialized = false;
    WifiController_ActionList_Delete(&actionList);
    WifiController_SerialHelper_Delete();
}

const char* WifiController_WifiController_GetSsid()
{
    return ssid;
}

const char* WifiController_WifiController_GetPassword()
{
    return password;
}

WifiController_IpAddress_t* WifiController_WifiController_GetIPAddress()
{
    return &ipAddress;
}

WifiController_ActionList_t *WifiController_WifiController_GetActionList()
{
    return &actionList;
}

WC_ErrorCode_t WifiController_WifiController_SetSsid(const char *value)
{
    if (!check_ssid(value))
    {
        return WC_ErrorCode_PARAMETER;
    }

    strncpy(ssid, value, SSID_MAX_LENGTH);

    strcpy(transmitBuffer, STR_SSID);
    strcat(transmitBuffer, " ");
    strcat(transmitBuffer, value);
    WC_ErrorCode_t error_code = WifiController_SerialHelper_SendMessage(transmitBuffer);
    if (error_code != WC_ErrorCode_NONE)
    {
        return error_code;
    }
    return wait_confirm();
}

WC_ErrorCode_t WifiController_WifiController_SetPassword(const char *value)
{
    if (!check_password(value))
    {
        return WC_ErrorCode_PARAMETER;
    }

    strcpy(password, value);

    strcpy(transmitBuffer, STR_PASSWORD);
    strcat(transmitBuffer, " ");
    strcat(transmitBuffer, value);
    WC_ErrorCode_t error_code = WifiController_SerialHelper_SendMessage(transmitBuffer);
    if (error_code != WC_ErrorCode_NONE)
    {
        return error_code;
    }
    return wait_confirm();
}

WC_ErrorCode_t WifiController_WifiController_ResetModule()
{
    WifiController_SerialHelper_SendMessage(STR_RESET);
    return wait_confirm();
}

WC_ErrorCode_t WifiController_WifiController_BeginStation(int timeout_ms)
{
    WifiController_SerialHelper_SendMessage(STR_CONNECT_STATION);

    WC_ErrorCode_t error_code = wait_confirm();
    if (error_code != WC_ErrorCode_NONE)
    {
        return error_code;
    }

    return wait_ip_address(timeout_ms);
}

WC_ErrorCode_t WifiController_WifiController_BeginAccessPoint(int timeout_ms)
{
    WifiController_SerialHelper_SendMessage(STR_SETUP_ACCESS_POINT);

    WC_ErrorCode_t error_code = wait_confirm();
    if (error_code != WC_ErrorCode_NONE)
    {
        return error_code;
    }

    return wait_ip_address(timeout_ms);
}

WC_ErrorCode_t WifiController_WifiController_SendConfiguration(const WifiController_UserInterface_t *ui)
{
    const char *command = STR_CONFIGURE_LAYOUT;
    size_t command_length = strlen(command);
    strcpy(transmitBuffer, command);
    strcat(transmitBuffer, " ");
    WifiController_UserInterface_GetConfigurationJSON(
            ui,
            transmitBuffer + command_length + 1,
            MESSAGE_MAX_SIZE - command_length - 1);
    WifiController_SerialHelper_SendMessage(transmitBuffer);
    return wait_confirm();
}

WC_ErrorCode_t WifiController_WifiController_SendDataUpdate(const WifiController_UserInterface_t *ui)
{
    const char *command = STR_UPDATE_DATA;
    size_t command_length = strlen(command);
    strcpy(transmitBuffer, command);
    strcat(transmitBuffer, " ");
    WifiController_UserInterface_GetDataJSON(
            ui,
            transmitBuffer + command_length + 1,
            MESSAGE_MAX_SIZE - command_length - 1);
    WifiController_SerialHelper_SendMessage(transmitBuffer);
    return wait_confirm();
}

WC_ErrorCode_t WifiController_WifiController_Receive()
{
    if (!initialized)
    {
        return WC_ErrorCode_NOT_INITIALZED;
    }

    WC_ErrorCode_t error_code = WC_ErrorCode_UNKNOWN;

    error_code = WifiController_SerialHelper_ReadMessage(receiveBuffer, MESSAGE_MAX_SIZE);
    if (error_code != WC_ErrorCode_NONE)
    {
        return error_code;
    }

    error_code = process_message(receiveBuffer);

    return error_code;
}

static WC_ErrorCode_t wait_confirm()
{
    EventBits_t bits = xEventGroupWaitBits(WifiController_EventGroup_GetHandle(),
            WifiController_EventGroup_OkBit | WifiController_EventGroup_FailBit,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(responseTimeout));

    if (bits & WifiController_EventGroup_OkBit)
    {
        return WC_ErrorCode_NONE;
    }
    else if (bits & WifiController_EventGroup_FailBit)
    {
        return WC_ErrorCode_CONTROLLER;
    }
    return WC_ErrorCode_TIMEOUT;
}

static WC_ErrorCode_t wait_ip_address(int timeout_ms)
{
    EventBits_t bits = xEventGroupWaitBits(WifiController_EventGroup_GetHandle(),
            WifiController_EventGroup_IpBit | WifiController_EventGroup_FailBit,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(responseTimeout));

    if (bits & WifiController_EventGroup_IpBit)
    {
        return WC_ErrorCode_NONE;
    }
    else if (bits & WifiController_EventGroup_FailBit)
    {
        return WC_ErrorCode_CONTROLLER;
    }
    return WC_ErrorCode_TIMEOUT;
}

static bool check_ssid(const char *value)
{
    size_t length = strlen(value);
    return (length >= SSID_MIN_LENGTH) && (length <= SSID_MAX_LENGTH);
}

static bool check_password(const char *value)
{
    size_t length = strlen(value);
    return (length >= PASSWORD_MIN_LENGTH) && (length <= PASSWORD_MAX_LENGTH);
}

static WC_ErrorCode_t process_message(const char *message)
{
    WC_ErrorCode_t error_code = WC_ErrorCode_NONE;
    const char *p = strchr(message, ' ');
    int type_length = 0;
    if (p != NULL)
    {
        type_length = p - message;
        if (type_length > RESPONSE_TYPE_MAX_LENGTH)
        {
            return WC_ErrorCode_CONTROLLER;
        }
        strncpy(responseTypeBuffer, message, p - message);
        strcpy(responseParamsBuffer, p + 1);

        error_code = execute_message(responseTypeBuffer, responseParamsBuffer);
    }
    else
    {
        type_length = strlen(message);
        if (type_length > RESPONSE_TYPE_MAX_LENGTH)
        {
            return WC_ErrorCode_CONTROLLER;
        }
        strncpy(responseTypeBuffer, message, type_length + 1);

        error_code = execute_message(responseTypeBuffer, NULL);
    }
    return error_code;
}

static WC_ErrorCode_t execute_message(const char *cmd, const char *params)
{
    WC_ErrorCode_t error_code = WC_ErrorCode_NONE;
    if (strcmp(cmd, STR_ACTION) == 0)
    {
        if (params == NULL)
        {
            error_code = WC_ErrorCode_CONTROLLER;
        }
        else
        {
            error_code = call_action(params);
        }
    }
    else if (strcmp(cmd, STR_CONFIRM) == 0)
    {
        xEventGroupSetBits(WifiController_EventGroup_GetHandle(), WifiController_EventGroup_OkBit);
    }
    else if (strcmp(cmd, STR_FAIL) == 0)
    {
        xEventGroupSetBits(WifiController_EventGroup_GetHandle(), WifiController_EventGroup_FailBit);
    }
    else if (strcmp(cmd, STR_IP) == 0)
    {
        if (params == NULL)
        {
            WifiController_IpAddress_SetAddress(&ipAddress, 0);
        }
        else if (!WifiController_IpAddress_FromString(&ipAddress, params))
        {
            WifiController_IpAddress_SetAddress(&ipAddress, 0);
            error_code = WC_ErrorCode_CONTROLLER;
        }
        xEventGroupSetBits(WifiController_EventGroup_GetHandle(), WifiController_EventGroup_IpBit);
    }
    else
    {
        error_code = WC_ErrorCode_CONTROLLER;
    }
    return error_code;
}

static WC_ErrorCode_t call_action(const char *params)
{
    size_t path_length = 0;
    size_t args_length = 0;
    const char *p = strchr(params, '?');
    if (p != NULL)
    {
        path_length = p - params;
        args_length = strlen(p + 1);
    }
    else
    {
        path_length = strlen(params);
        // args_length = 0;
    }

    // Allocate memory for and copy path {
    char *path = (char*)pvPortMalloc(sizeof(char) * (path_length + 1));
    if (path == NULL)
    {
        return WC_ErrorCode_MEMORY_ALLOCATION;
    }
    strncpy(path, params, path_length);
    path[path_length] = '\0';
    // }

    // Allocate memory for and copy args {
    char *args = (char*)pvPortMalloc(sizeof(char) * (args_length + 1));
    if (args == NULL)
    {
        vPortFree(path);
        return WC_ErrorCode_MEMORY_ALLOCATION;
    }
    strncpy(args, p + 1, args_length);
    args[args_length] = '\0';
    // }

    // Find action in list and call handler function {
    int index = WifiController_ActionList_FindByPath(&actionList, path);
    const WifiController_Action_t *action = WifiController_ActionList_At(&actionList, index);
    if (action != NULL)
    {
        action->handler(args);
    }
    // }

    vPortFree(path);
    vPortFree(args);
    return WC_ErrorCode_NONE;
}
