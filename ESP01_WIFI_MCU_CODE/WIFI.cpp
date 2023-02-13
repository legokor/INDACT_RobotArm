/**
 *******************************************************************************
 * @file WIFI.cpp
 *
 * @date Jan 19, 2023
 * @author Varga Péter
 *******************************************************************************
 * @brief
 * 
 *******************************************************************************
 */

#include "WIFI.h"

#include <string.h>

#include "../ESP01_WIFI_MODULE_CODE/protocols.h"

#define BLOCK_TIME_MS 20
#define BLOCK_TIME_TICK (pdMS_TO_TICKS(BLOCK_TIME_MS))

using namespace std;

namespace ESP01
{

WIFI::WIFI(UART_HandleTypeDef *huart)
{
    serial = new SerialStream(huart, MESSAGE_MAX_SIZE + 1);
    configASSERT(serial != NULL);

    messageQueue = xQueueCreate(messageQueueSize, sizeof(string));
    configASSERT(messageQueue != NULL);

    requestQueue = xQueueCreate(requestQueueSize, sizeof(RequestType));
    configASSERT(requestQueue != NULL);
}

WIFI::~WIFI()
{
    delete (serial);
    vQueueDelete(messageQueue);
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

void WIFI::setSSID(std::string ssid)
{
    this->ssid = ssid;

    const char *cmd_msg = STR_SSID;
    setModuleParam(cmd_msg, strlen(cmd_msg), ssid.c_str(), ssid.size());

    ssid_set = true;
}

void WIFI::setPassword(std::string password)
{
    this->password = password;

    const char *cmd_msg = STR_PASSWORD;
    setModuleParam(cmd_msg, strlen(cmd_msg), password.c_str(), password.size());

    password_set = true;
}

void WIFI::setStatusMessage(std::string status)
{
    const char *cmd_msg = STR_STATUS;
    setModuleParam(cmd_msg, strlen(cmd_msg), status.c_str(), status.size());
}

void WIFI::setCoordinateSystem(CoordinateSystem csys)
{
    const char *cmd_msg_cyl = STR_CHANGE_TO_CYLINDRICAL;
    const char *cmd_msg_rec = STR_CHANGE_TO_RECTANGULAR;

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

    demandConfirm();
}

bool WIFI::connectToNetwork()
{
    if (!(ssid_set && password_set))
    {
        return false;
    }

    const char *cmd_msg = STR_CONNECT_STATION;

    serialTransmitMessage(cmd_msg, strlen(cmd_msg));
    demandConfirm();

    ip_address = serialReceiveMessage();
    if (ip_address == STR_FAIL)
    {
        throw runtime_error("Connecting to the WiFi network failed.");
    }

    return true;
}

void WIFI::setupAccessPoint(void)
{
    const char *cmd_msg = STR_SETUP_ACCESS_POINT;

    serialTransmitMessage(cmd_msg, strlen(cmd_msg));
    demandConfirm();

    ip_address = serialReceiveMessage();
    if (ip_address == STR_FAIL)
    {
        throw runtime_error("Setting up the WiFi access point failed.");
    }
}

void WIFI::resetModule(void)
{
    const char *cmd_msg = STR_RESET;

    serialTransmitMessage(cmd_msg, strlen(cmd_msg));
    demandConfirm();
}

RequestType WIFI::receiveNextRequest(void)
{
    RequestType rqst;
    while (xQueueReceive(requestQueue, (void*)(&rqst), 20) != pdTRUE)
    {
    }
    return rqst;
}

void WIFI::receive(void)
{
    char msg[MESSAGE_MAX_SIZE + 1] = { 0 };

    if (!serial->receiveMessage(msg, MESSAGE_MAX_SIZE))
    {
        return;
    }

    RequestType rqst = decodeRequest(msg);

    if (rqst == RequestType::INVALID)
    {
        string str_rqst = msg;
        xQueueSendToBack(messageQueue, (const void* )(&str_rqst), BLOCK_TIME_TICK);
    }
    else
    {
        xQueueSendToBack(requestQueue, (const void* )(&rqst), BLOCK_TIME_TICK);
    }
}

RequestType WIFI::decodeRequest(const char *rqst)
{
    if (strcmp(rqst, STR_AXIS_A_PLUS) == 0)
    {
        return RequestType::AXIS_A_PLUS;
    }
    else if (strcmp(rqst, STR_AXIS_A_MINUS) == 0)
    {
        return RequestType::AXIS_A_MINUS;
    }
    else if (strcmp(rqst, STR_AXIS_B_PLUS) == 0)
    {
        return RequestType::AXIS_B_PLUS;
    }
    else if (strcmp(rqst, STR_AXIS_B_MINUS) == 0)
    {
        return RequestType::AXIS_B_MINUS;
    }
    else if (strcmp(rqst, STR_AXIS_C_PLUS) == 0)
    {
        return RequestType::AXIS_C_PLUS;
    }
    else if (strcmp(rqst, STR_AXIS_C_MINUS) == 0)
    {
        return RequestType::AXIS_C_MINUS;
    }
    else if (strcmp(rqst, STR_HOMING) == 0)
    {
        return RequestType::HOMING;
    }
    else if (strcmp(rqst, STR_CHANGE_COORDINATES) == 0)
    {
        return RequestType::CHANGE_COORDINATES;
    }
    else
    {
        return RequestType::INVALID;
    }
}

void WIFI::serialTransmitMessage(const char *msg, uint16_t size)
{
    if (size > MESSAGE_MAX_SIZE)
    {
        return;
    }

    serial->sendMessage(msg, size);
}

std::string WIFI::serialReceiveMessage(void)
{
    string str;
    if (xQueueReceive(messageQueue, (void*)(&str), BLOCK_TIME_TICK) == pdFAIL)
    {
        return string();
    }
    return string(str);
}

void WIFI::setModuleParam(
        const char *param_name, uint16_t size_name,
        const char *param_value, uint16_t size_value)
{
    serialTransmitMessage(param_name, size_name);
    demandConfirm();

    serialTransmitMessage(param_value, size_value);
    if (!checkReply())
    {
        string error_msg;
        error_msg += "The \"";
        error_msg += param_name;
        error_msg += "\" parameter could not be set to the specified value \"";
        error_msg += param_value;
        error_msg += "\".";

        throw runtime_error(error_msg);
    }
}

bool WIFI::checkReply()
{
    string reply = serialReceiveMessage();
    if (reply == STR_CONFIRM)
    {
        return true;
    }
    else if (reply == STR_FAIL)
    {
        return false;
    }
    else
    {
        throw runtime_error("Communication with the WiFi module failed.");
    }
}

void WIFI::demandConfirm()
{
    if (!checkReply())
    {
        throw runtime_error("The WiFi module failed to complete the task.");
    }
}

} /* namespace ESP01 */
