/**
 *******************************************************************************
 * @file WIFI.cpp
 *
 * @date Jan 19, 2023
 * @author Varga Péter
 *******************************************************************************
 * @brief Implementation of WIFI.h.
 *
 *******************************************************************************
 */

#include "WIFI.h"

#include <string.h>

#include "../ESP01_WIFI_MODULE_CODE/RobotKar_WIFI/protocols.h"

#define BLOCK_TIME_MS 20
#define BLOCK_TIME_TICK (pdMS_TO_TICKS(BLOCK_TIME_MS))

#define REPLY_TIMEOUT_MS 200
#define SETUP_TIMEOUT_MS CONNECT_TIMEOUT_MS
#define SYNC_TIMEOUT_MS REPLY_TIMEOUT_MS

using namespace std;

namespace ESP01
{

WIFI::WIFI(UART_HandleTypeDef *huart)
{
    serial = new SerialStream(huart, MESSAGE_MAX_SIZE + 1);
    configASSERT(serial != NULL);

    responseEventGroup = xEventGroupCreate();
    configASSERT(responseEventGroup != NULL);

    requestQueue = xQueueCreate(requestQueueSize, sizeof(RequestType));
    configASSERT(requestQueue != NULL);
}

WIFI::~WIFI()
{
    delete (serial);
    vEventGroupDelete(responseEventGroup);
    vQueueDelete(requestQueue);
}

std::string WIFI::getSSID(void)
{
    return ssid;
}

std::string WIFI::getPassword(void)
{
    return password;
}

std::string WIFI::getIP(void)
{
    return ip_address;
}

void WIFI::setSSID(const std::string ssid)
{
    // Save the SSID
    this->ssid = ssid;

    // Set the SSID with the appropriate command
    const char *cmd_msg = STR_SSID;
    setModuleParam(cmd_msg, strlen(cmd_msg), ssid.c_str(), ssid.size());

    ssid_set = true;
}

void WIFI::setPassword(const std::string password)
{
    // Save the password
    this->password = password;

    // Set the password with the appropriate command
    const char *cmd_msg = STR_PASSWORD;
    setModuleParam(cmd_msg, strlen(cmd_msg), password.c_str(), password.size());

    password_set = true;
}

void WIFI::setStatusMessage(const std::string status)
{
    // Set the status with the appropriate command
    const char *cmd_msg = STR_STATUS;
    setModuleParam(cmd_msg, strlen(cmd_msg), status.c_str(), status.size());
}

void WIFI::setCoordinateSystem(const CoordinateSystem csys)
{
    const char *cmd_msg_cyl = STR_CHANGE_TO_CYLINDRICAL;
    const char *cmd_msg_rec = STR_CHANGE_TO_RECTANGULAR;

    // Set the coordinate system with the appropriate command
    switch (csys)
    {
    case CoordinateSystem::CYLINDRICAL:
        serialTransmitMessage(cmd_msg_cyl, strlen(cmd_msg_cyl));
        break;
    case CoordinateSystem::RECTANGULAR:
        serialTransmitMessage(cmd_msg_rec, strlen(cmd_msg_rec));
        break;
    default:
        return;
    }

    if (checkReply() == false)
    {
        throw runtime_error("Setting of the coordinate system failed.");
    }
}

void WIFI::setPosition(const std::string position)
{
    // Set the position with the appropriate command
    const char *cmd_msg = STR_POSITION;
    setModuleParam(cmd_msg, strlen(cmd_msg), position.c_str(), position.size());
}

void WIFI::connectToNetwork()
{
    // Check if the password for the external network is set
    if (!(ssid_set && password_set))
    {
        // Necessary parameters are not set
        throw logic_error(
                "The SSID and password of the WiFi network has to be set before connection.");
    }

    // Send the connection command
    const char *cmd_msg = STR_CONNECT_STATION;
    serialTransmitMessage(cmd_msg, strlen(cmd_msg));

    if (checkReply() == false)
    {
        throw runtime_error("Connecting to the WiFi network failed.");
    }

    // Wait for the IP address
    TickType_t connect_timeout = pdMS_TO_TICKS(CONNECT_TIMEOUT_MS + 100);
    EventBits_t ev_bits = xEventGroupWaitBits(responseEventGroup, (ip_bit | fail_bit), pdTRUE,
    pdFALSE, connect_timeout);

    if (ev_bits & ip_bit)
    {
        // IP arrived, connection successful
        ip_set = true;
    }
    else if (ev_bits & fail_bit)
    {
        // Connection failed
        string error_msg = "The WiFi module could not connect to the specified network.";
        error_msg += " SSID: ";
        error_msg += ssid;
        error_msg += ", password: ";
        error_msg += password;

        throw runtime_error(error_msg);
    }
    else
    {
        // Communication error
        throw runtime_error("Communication with the WiFi module failed.");
    }
}

void WIFI::setupAccessPoint(void)
{
    // Send the setup command
    const char *cmd_msg = STR_SETUP_ACCESS_POINT;
    serialTransmitMessage(cmd_msg, strlen(cmd_msg));

    if (checkReply() == false)
    {
        throw runtime_error("Setting up the WiFi access point failed.");
    }

    // Wait for the IP address
    TickType_t setup_timeout = pdMS_TO_TICKS(SETUP_TIMEOUT_MS + 100);
    EventBits_t ev_bits = xEventGroupWaitBits(responseEventGroup, (ip_bit | fail_bit), pdTRUE,
    pdTRUE, setup_timeout);

    if (ev_bits & ip_bit)
    {
        // IP arrived, access point setup successful
        ip_set = true;
    }
    else if (ev_bits & fail_bit)
    {
        // Access point setup failed
        string error_msg =
                "The WiFi module could not set up an access point with the specified parameters.";
        error_msg += " SSID: ";
        error_msg += ssid;
        error_msg += ", password: ";
        error_msg += password;

        throw runtime_error(error_msg);
    }
    else
    {
        // Communication error
        throw runtime_error("Communication with the WiFi module failed.");
    }
}

void WIFI::resetModule(void)
{
    // Send the reset command
    const char *cmd_msg = STR_RESET;
    serialTransmitMessage(cmd_msg, strlen(cmd_msg));

    if (checkReply() == false)
    {
        throw runtime_error("Module reset failed.");
    }

    ssid_set = false;
    password_set = false;
    ip_set = false;
}

bool WIFI::synchronizeModule(void)
{
    const TickType_t sync_timeout = pdMS_TO_TICKS(SYNC_TIMEOUT_MS);

    // Clear all response events
    xEventGroupClearBits(responseEventGroup, 0xFFFF);

    // Send the synchronization command
    const char *cmd_msg = STR_SYNCHRONIZE;
    serialTransmitMessage(cmd_msg, strlen(cmd_msg));

    // Wait for the necessary number of synchronization responses
    for (uint32_t i = 0; i < SYNC_NUMBER; i++)
    {
        EventBits_t ev_bits = xEventGroupWaitBits(responseEventGroup, (sync_bit), pdTRUE, pdFALSE,
                sync_timeout);

        if (!(ev_bits & sync_bit))
        {
            // Synchronization failed
            return false;
        }
    }

    // Synchronization successful
    return true;
}

RequestType WIFI::receiveNextRequest(void)
{
    RequestType rqst;
    // Wait for a Request to appear in the queue
    while (xQueueReceive(requestQueue, (void*)(&rqst), 200) != pdTRUE)
    {
        // Do nothing
    }
    // Return the request
    return rqst;
}

void WIFI::receive(void)
{
    // Buffer for the incoming message
    char msg[MESSAGE_MAX_SIZE + 1] = { 0 };

    // Receive a message
    if (!serial->receiveMessage(msg, MESSAGE_MAX_SIZE))
    {
        // No message arrived
        return;
    }

    // Process the message
    if (processMessage(msg) == false)
    {
        // If the process was not successful, then throw an exeption with the appropriate type and
        // message
        throw runtime_error("An invalid message arrived.");
    }
}

typedef struct
{
    const char *str;
    const RequestType rqst;
} RequestStruct;

const RequestStruct requests[] =
        {
                { .str = STR_AXIS_A_PLUS, .rqst = RequestType::AXIS_A_PLUS },
                { .str = STR_AXIS_A_MINUS, .rqst = RequestType::AXIS_A_MINUS },
                { .str = STR_AXIS_B_PLUS, .rqst = RequestType::AXIS_B_PLUS },
                { .str = STR_AXIS_B_MINUS, .rqst = RequestType::AXIS_B_MINUS },
                { .str = STR_AXIS_C_PLUS, .rqst = RequestType::AXIS_C_PLUS },
                { .str = STR_AXIS_C_MINUS, .rqst = RequestType::AXIS_C_MINUS },
                { .str = STR_HOMING, .rqst = RequestType::HOMING },
                { .str = STR_CHANGE_COORDINATES, .rqst = RequestType::CHANGE_COORDINATES }
        };
const size_t size_requests = 8;

bool WIFI::decodeRequest(const char *rqst)
{
    // Check all the acceptable values for the request
    for (size_t i = 0; i < size_requests; i++)
    {
        if (strcmp(rqst, requests[i].str) == 0)
        {
            // If a request is found, save it to the queue
            xQueueSendToBack(requestQueue, (const void* )(requests[i].rqst), BLOCK_TIME_TICK);
            return true;
        }
    }

    // The message is not an acceptable request
    return false;
}

void WIFI::serialTransmitMessage(const char *msg, uint16_t size)
{
    // Check for appropriate size
    if (size > MESSAGE_MAX_SIZE)
    {
        return;
    }

    // Send the message
    serial->sendMessage(msg, size);
}

bool WIFI::decodeResponse(const char *resp)
{
    const char str_sync_code[] = STR_SYNC_CODE;

    if (strcmp(resp, STR_CONFIRM) == 0)
    {
        xEventGroupSetBits(responseEventGroup, (ok_bit));
    }
    else if (strcmp(resp, STR_FAIL) == 0)
    {
        xEventGroupSetBits(responseEventGroup, (fail_bit));
    }
    else if (strcmp(resp, str_sync_code) == 0)
    {
        xEventGroupSetBits(responseEventGroup, (sync_bit));
    }
    else
    {
        // Check IP
        const char *str_ip = STR_IP;
        const size_t size_str_ip = strlen(str_ip);

        bool is_ip = true;
        for (size_t i = 0; i < size_str_ip; i++)
        {
            if (resp[i] != str_ip[i])
            {
                is_ip = false;
                break;
            }
        }
        if (is_ip && (resp[size_str_ip] == ' '))
        {
            // Save new IP
            ip_address = resp + size_str_ip + 1;

            xEventGroupSetBits(responseEventGroup, (ip_bit));
        }
        else
        {
            return false;
        }
    }

    return true;
}

void WIFI::setModuleParam(
        const char *param_name, uint16_t size_name,
        const char *param_value, uint16_t size_value)
{
    char buffer[MESSAGE_MAX_SIZE + 1];

    // Prepare and send the command to the module
    buffer[0] = '\0';
    strcat(buffer, param_name);
    strcat(buffer, " ");
    strcat(buffer, param_value);
    serialTransmitMessage(buffer, strlen(buffer));

    if (checkReply() == false)
    {
        // Operation failed
        string error_msg;
        error_msg += "The \"";
        error_msg += param_name;
        error_msg += "\" parameter could not be set to the specified value \"";
        error_msg += param_value;
        error_msg += "\".";

        throw runtime_error(error_msg);
    }
}

bool WIFI::processMessage(const char *msg)
{
    // Try processing the message as a response or as a request
    if (decodeResponse(msg) || decodeRequest(msg))
    {
        return true;
    }

    // The message cannot be processed
    return false;
}

bool WIFI::checkReply()
{
    TickType_t connect_timeout = pdMS_TO_TICKS(REPLY_TIMEOUT_MS);
    EventBits_t ev_bits = xEventGroupWaitBits(responseEventGroup, (ok_bit | fail_bit), pdTRUE,
    pdFALSE, connect_timeout);

    if (ev_bits & ok_bit)
    {
        // Success
        return true;
    }
    else if (ev_bits & fail_bit)
    {
        // Failure
        return false;
    }
    else
    {
        // Communication error
        throw runtime_error("Communication with the WiFi module failed.");
    }
}

} /* namespace ESP01 */
