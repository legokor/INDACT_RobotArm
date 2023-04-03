/**
 ***************************************************************************************************
 * @file WIFI.c
 *
 * @date 2023. 01. 19.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief Implementation of WIFI.h.
 ***************************************************************************************************
 */

#include "WIFI.h"

#include <string.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"

#include "SerialStream.h"

#include "WIFI_Config.h"

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Macros
// /////////////////////////////////////////////////////////////////////////////////////////////////

#define EVENT_BIT_CONFIRM       (((uint32_t)1) << 0)
#define EVENT_BIT_FAIL          (((uint32_t)1) << 1)
#define EVENT_BIT_IP            (((uint32_t)1) << 2)
#define EVENT_BIT_SYNC          (((uint32_t)1) << 3)

#define PARAM_STATE_BIT_SSID        (((uint32_t)1) << 0)
#define PARAM_STATE_BIT_PASSWORD    (((uint32_t)1) << 1)
#define PARAM_STATE_BIT_IP          (((uint32_t)1) << 2)

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Type definitions
// /////////////////////////////////////////////////////////////////////////////////////////////////

// TODO Szépíteni
typedef struct
{
    const char *str;
    const WIFI_RequestType rqst;
} WIFI_RequestStruct;

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Global variables
// /////////////////////////////////////////////////////////////////////////////////////////////////

/** Stroage space for the WIFI initialization state. */
static uint8_t initState = 0;

/** Handle of the request queue. */
QueueHandle_t requestQueue;

/** Handle of the response event group. */
EventGroupHandle_t responseEventGroup;

/** Storage space for the parameter states. */
static uint32_t paramState = 0;

/** SSID of the WIFI. */
char ssid[WIFI_STRING_SIZE + 1] = { 0 };
/** Password of the WIFI. */
char password[WIFI_STRING_SIZE + 1] = { 0 };
/** IP address of the WIFI. */
char ipAddress[WIFI_STRING_SIZE + 1] = { 0 };

// TODO Szépíteni
static const WIFI_RequestStruct request_table[] =
        {
                { .str = STR_AXIS_A_PLUS, .rqst = AXIS_A_PLUS },
                { .str = STR_AXIS_A_MINUS, .rqst = AXIS_A_MINUS },
                { .str = STR_AXIS_B_PLUS, .rqst = AXIS_B_PLUS },
                { .str = STR_AXIS_B_MINUS, .rqst = AXIS_B_MINUS },
                { .str = STR_AXIS_C_PLUS, .rqst = AXIS_C_PLUS },
                { .str = STR_AXIS_C_MINUS, .rqst = AXIS_C_MINUS },
                { .str = STR_HOMING, .rqst = HOMING },
                { .str = STR_CHANGE_COORDINATES, .rqst = CHANGE_COORDINATES }
        };

// TODO Szépíteni
static const size_t request_table_size = 8;

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Function declarations
// /////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Transmit a message through the serial communication channel.
 * @param msg Message to be sent in string format.
 * @param size Size of a message in characters.
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_SerialTransmitMessage(const char *msg, uint16_t size);

/**
 * @brief Process a message.
 * @param msg Message in string format to be processed
 * @return 1 if the message was successfuly processed, 0 otherwise
 */
uint8_t WIFI_ProcessMessage(const char *msg);
/**
 * @brief Decode and save a request.
 * @param rqst String to be decoded
 * @return 1 if the string was identified as a request and was successfuly saved, 0 otherwise
 */
uint8_t WIFI_DecodeRequest(const char *rqst);
/**
 * @brief Decode and process response.
 * @param response String to be decoded and processed
 * @return 1 if the string was identified as a response and was succesfuly processed, 0 otherwise
 */
uint8_t WIFI_DecodeResponse(const char *response);

/**
 * @brief Process a generic reply of the WiFi module.
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_CheckReply(void);

/**
 * @brief Set a parameter of the WiFi module.
 * @param param_name Name of the parameter
 * @param size_name Size of the name in characters
 * @param param_value Desired value of the parameter
 * @param size_value Size of the desired value in charactes
 * @return WIFI status
 */
WIFI_StatusTypedef WIFI_SetModuleParam(
        const char *param_name, uint16_t size_name,
        const char *param_value, uint16_t size_value);

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Function definitions
// /////////////////////////////////////////////////////////////////////////////////////////////////

void WIFI_InitWIFI(UART_HandleTypeDef *huart)
{
    paramState = 0;

    SerialStream_InitSerialStream(huart, MESSAGE_MAX_SIZE * 2, MESSAGE_MAX_SIZE);

    responseEventGroup = xEventGroupCreate();
    configASSERT(responseEventGroup != NULL);

    requestQueue = xQueueCreate(REQUEST_QUEUE_SIZE, sizeof(WIFI_RequestType));
    configASSERT(requestQueue != NULL);

    initState = 1;
}

void WIFI_DeleteWIFI(void)
{
    initState = 0;
    paramState = 0;

    SerialStream_DeleteSerialStream();

    vEventGroupDelete(responseEventGroup);

    vQueueDelete(requestQueue);
}

uint8_t WIFI_GetInitState(void)
{
    return initState;
}

const char* WIFI_GetSSID(void)
{
    if (paramState & PARAM_STATE_BIT_SSID)
    {
        return ssid;
    }
    else
    {
        return NULL;
    }
}

const char* WIFI_GetPassword(void)
{
    if (paramState & PARAM_STATE_BIT_PASSWORD)
    {
        return password;
    }
    else
    {
        return NULL;
    }
}

const char* WIFI_GetIP(void)
{
    if (paramState & PARAM_STATE_BIT_IP)
    {
        return ipAddress;
    }
    else
    {
        return NULL;
    }
}

WIFI_StatusTypedef WIFI_SetSSID(const char *new_ssid)
{
    // Save the SSID
    strncpy(ssid, new_ssid, WIFI_STRING_SIZE);

    // Set the SSID with the appropriate command
    const char *cmd_msg = STR_SSID;
    WIFI_StatusTypedef ret_val = WIFI_SetModuleParam(cmd_msg, strlen(cmd_msg), new_ssid,
            strlen(new_ssid));

    if (ret_val == WIFI_CONFIRM)
    {
        paramState |= PARAM_STATE_BIT_SSID;
    }

    return ret_val;
}

WIFI_StatusTypedef WIFI_SetPassword(const char *new_password)
{
    // Save the password
    strncpy(password, new_password, WIFI_STRING_SIZE);

    // Set the password with the appropriate command
    const char *cmd_msg = STR_PASSWORD;
    WIFI_StatusTypedef ret_val = WIFI_SetModuleParam(cmd_msg, strlen(cmd_msg), new_password,
            strlen(new_password));

    if (ret_val == WIFI_CONFIRM)
    {
        paramState |= PARAM_STATE_BIT_PASSWORD;
    }

    return ret_val;
}

WIFI_StatusTypedef WIFI_SetStatusMessage(const char *new_status)
{
    // Set the status with the appropriate command
    const char *cmd_msg = STR_STATUS;
    return WIFI_SetModuleParam(cmd_msg, strlen(cmd_msg), new_status, strlen(new_status));
}

WIFI_StatusTypedef WIFI_SetPosition(const char *new_position)
{
    // Set the position with the appropriate command
    const char *cmd_msg = STR_POSITION;
    return WIFI_SetModuleParam(cmd_msg, strlen(cmd_msg), new_position, strlen(new_position));
}

WIFI_StatusTypedef WIFI_SetCoordinateSystem(WIFI_CoordinateSystem csys)
{
    const char *cmd_msg_cyl = STR_CHANGE_TO_CYLINDRICAL;
    const char *cmd_msg_rec = STR_CHANGE_TO_RECTANGULAR;

    WIFI_StatusTypedef cmd_ret_val = WIFI_ERROR;

    // Set the coordinate system with the appropriate command
    switch (csys)
    {
    case CYLINDRICAL:
        cmd_ret_val = WIFI_SerialTransmitMessage(cmd_msg_cyl, strlen(cmd_msg_cyl));
        break;
    case RECTANGULAR:
        cmd_ret_val = WIFI_SerialTransmitMessage(cmd_msg_rec, strlen(cmd_msg_rec));
        break;
    default:
        return WIFI_FAIL;
    }

    if (cmd_ret_val != WIFI_CONFIRM)
    {
        return cmd_ret_val;
    }

    return WIFI_CheckReply();
}

WIFI_StatusTypedef WIFI_ConnectToNetwork()
{
    // Check if the password for the external network is set
    if (!(paramState & PARAM_STATE_BIT_SSID) || !(paramState & PARAM_STATE_BIT_PASSWORD))
    {
        // Necessary parameters are not set
        return WIFI_FAIL;
    }

    // Send the connection command
    const char *cmd_msg = STR_CONNECT_STATION;
    WIFI_SerialTransmitMessage(cmd_msg, strlen(cmd_msg));

    WIFI_StatusTypedef cmd_ret_val = WIFI_CheckReply();
    if (cmd_ret_val != WIFI_CONFIRM)
    {
        return cmd_ret_val;
    }

    // Wait for the IP address
    EventBits_t ev_bits = xEventGroupWaitBits(responseEventGroup, (EVENT_BIT_IP | EVENT_BIT_FAIL),
    pdTRUE, pdFALSE, pdMS_TO_TICKS(CONNECT_TIMEOUT_MS + 100));

    if (ev_bits & EVENT_BIT_IP)
    {
        // IP arrived, connection successful
        paramState |= PARAM_STATE_BIT_IP;
        return WIFI_CONFIRM;
    }
    else if (ev_bits & EVENT_BIT_FAIL)
    {
        // Connection failed
        return WIFI_FAIL;
    }
    else
    {
        // Communication error
        return WIFI_ERROR;
    }
}

WIFI_StatusTypedef WIFI_SetupAccessPoint(void)
{
    // Send the setup command
    const char *cmd_msg = STR_SETUP_ACCESS_POINT;
    WIFI_SerialTransmitMessage(cmd_msg, strlen(cmd_msg));

    WIFI_StatusTypedef cmd_ret_val = WIFI_CheckReply();
    if (cmd_ret_val != WIFI_CONFIRM)
    {
        return cmd_ret_val;
    }

    // Wait for the IP address
    EventBits_t ev_bits = xEventGroupWaitBits(responseEventGroup, (EVENT_BIT_IP | EVENT_BIT_FAIL),
    pdTRUE, pdFALSE, pdMS_TO_TICKS(SETUP_TIMEOUT_MS + 100));

    if (ev_bits & EVENT_BIT_IP)
    {
        // IP arrived, access point setup successful
        paramState |= PARAM_STATE_BIT_IP;
        return WIFI_CONFIRM;
    }
    else if (ev_bits & EVENT_BIT_FAIL)
    {
        // Access point setup failed
        return WIFI_FAIL;
    }
    else
    {
        // Communication error
        return WIFI_ERROR;
    }
}

WIFI_StatusTypedef WIFI_ResetModule(void)
{
    // Send the reset command
    const char *cmd_msg = STR_RESET;
    WIFI_SerialTransmitMessage(cmd_msg, strlen(cmd_msg));

    WIFI_StatusTypedef ret_val = WIFI_CheckReply();
    if (ret_val == WIFI_CONFIRM)
    {
        paramState = 0;
    }

    return ret_val;
}

WIFI_StatusTypedef WIFI_SynchronizeModule(void)
{
    // Clear all response events
    xEventGroupClearBits(responseEventGroup, 0xFFFF);

    // Send the synchronization command
    const char *cmd_msg = STR_SYNCHRONIZE;
    WIFI_SerialTransmitMessage(cmd_msg, strlen(cmd_msg));

    // Wait for the necessary number of synchronization responses
    for (uint32_t i = 0; i < SYNC_NUMBER; i++)
    {
        EventBits_t ev_bits = xEventGroupWaitBits(responseEventGroup, (EVENT_BIT_SYNC), pdTRUE,
        pdFALSE, pdMS_TO_TICKS(SYNC_TIMEOUT_MS));

        if (!(ev_bits & EVENT_BIT_SYNC))
        {
            // Synchronization failed
            return WIFI_ERROR;
        }
    }

    // Synchronization successful
    return WIFI_CONFIRM;
}

WIFI_RequestType WIFI_ReceiveNextRequest(void)
{
    WIFI_RequestType rqst;
    // Wait for a Request to appear in the queue
    if (xQueueReceive(requestQueue, (void*)(&rqst), portMAX_DELAY) != pdTRUE)
    {
        return INVALID;
    }
    // Return the request
    return rqst;
}

void WIFI_Receive(void)
{
    // Buffer for the incoming message
    static char msg[MESSAGE_MAX_SIZE + 1] = { 0 };

    // Receive a message
    if (SerialStream_ReceiveMessage(msg) == 0)
    {
        // No message arrived
        return;
    }

    // Process the message
    WIFI_ProcessMessage(msg);
}

uint8_t WIFI_DecodeRequest(const char *rqst)
{
    // Check all the acceptable values for the request
    for (size_t i = 0; i < request_table_size; i++)
    {
        if (strcmp(rqst, request_table[i].str) == 0)
        {
            // If a request is found, save it to the queue
            xQueueSendToBack(requestQueue, (const void* )(&(request_table[i].rqst)),
                    pdMS_TO_TICKS(REQUEST_QUEUE_TIMEOUT_MS));
            return 1;
        }
    }

    // The message is not an acceptable request
    return 0;
}

uint8_t WIFI_DecodeResponse(const char *response)
{
    if (strcmp(response, STR_CONFIRM) == 0)
    {
        xEventGroupSetBits(responseEventGroup, (EVENT_BIT_CONFIRM));
    }
    else if (strcmp(response, STR_FAIL) == 0)
    {
        xEventGroupSetBits(responseEventGroup, (EVENT_BIT_FAIL));
    }
    else if (strcmp(response, STR_SYNC_CODE) == 0)
    {
        xEventGroupSetBits(responseEventGroup, (EVENT_BIT_SYNC));
    }
    else
    {
        // Check IP
        const char str_ip[] = STR_IP;
        const size_t size_str_ip = strlen(str_ip);

        // Match IP pattern
        uint8_t is_ip = 1;
        for (size_t i = 0; (response[i] != '\0') && (i < size_str_ip); i++)
        {
            if (response[i] != str_ip[i])
            {
                is_ip = 0;
                break;
            }
        }
        if (is_ip && (response[size_str_ip] == ' '))
        {
            // Save new IP
            strncpy(ipAddress, response + size_str_ip + 1, WIFI_STRING_SIZE);

            xEventGroupSetBits(responseEventGroup, (EVENT_BIT_IP));
        }
        else
        {
            return 0;
        }
    }

    return 1;
}

WIFI_StatusTypedef WIFI_SerialTransmitMessage(const char *msg, uint16_t size)
{
    // Check for appropriate size
    if (size > MESSAGE_MAX_SIZE)
    {
        // The size is too big
        return WIFI_FAIL;
    }

    // Send the message
    if (SerialStream_SendMessage(msg))
    {
        // Communication error
        return WIFI_ERROR;
    }

    return WIFI_CONFIRM;
}

WIFI_StatusTypedef WIFI_SetModuleParam(
        const char *param_name, uint16_t size_name,
        const char *param_value, uint16_t size_value)
{
    static char buffer[MESSAGE_MAX_SIZE + 1];

    uint16_t size = size_name + size_value + 1;

    // Check the size of the message
    if (size > MESSAGE_MAX_SIZE + 1)
    {
        return WIFI_FAIL;
    }

    // Prepare and send the command to the WiFi module
    buffer[0] = '\0';
    strcat(buffer, param_name);
    strcat(buffer, " ");
    strcat(buffer, param_value);
    WIFI_SerialTransmitMessage(buffer, size);

    return WIFI_CheckReply();
}

uint8_t WIFI_ProcessMessage(const char *msg)
{
    // Try processing the message as a response or as a request
    if (WIFI_DecodeResponse(msg) || WIFI_DecodeRequest(msg))
    {
        return 1;
    }
    else
    {
        // The message cannot be processed
        return 0;
    }
}

WIFI_StatusTypedef WIFI_CheckReply()
{
    EventBits_t ev_bits = xEventGroupWaitBits(responseEventGroup,
            (EVENT_BIT_CONFIRM | EVENT_BIT_FAIL), pdTRUE, pdFALSE, pdMS_TO_TICKS(REPLY_TIMEOUT_MS));

    if (ev_bits & EVENT_BIT_CONFIRM)
    {
        // Success
        return WIFI_CONFIRM;
    }
    else if (ev_bits & EVENT_BIT_FAIL)
    {
        // Failure
        return WIFI_FAIL;
    }
    else
    {
        // Communication error
        return WIFI_ERROR;
    }
}
