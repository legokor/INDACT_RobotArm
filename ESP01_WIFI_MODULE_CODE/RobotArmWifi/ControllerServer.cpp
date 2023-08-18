/**
 ***************************************************************************************************
 * @file ControllerServer.cpp
 *
 * @date 2023. 04. 14.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief Implementation of ControllerServer.h.
 ***************************************************************************************************
 */

#include "ControllerServer.h"

using namespace std;
using namespace ESP8266_Controller;

/** @brief Main HTML page of the GUI. */
static const char robotkar_gui_main_cstring[] PROGMEM = {
#include "./cstring_text/robotkar_gui_main_cstring.h"
};

/** @brief Back-To-Index page. */
static const char robotkar_back_to_index_cstring[] PROGMEM = {
#include "./cstring_text/robotkar_back_to_index_cstring.h"
};

/** @brief Default data in JSON format. */
static const char default_data_cstring[] PROGMEM = {
#include "./cstring_text/default_data_cstring.h"
};

/** @brief Default configuration in JSON format. */
static const char default_configuration_cstring[] PROGMEM = {
#include "./cstring_text/default_configuration_cstring.h"
};

ControllerServer::ControllerServer(char *type_buffer, size_t type_buffer_size, char *parameter_buffer, size_t parameter_buffer_size)
    : parameterBuffer(parameter_buffer), typeBufferSize(parameter_buffer_size),
      typeBuffer(type_buffer), parameterBufferSize(type_buffer_size)
{
    // Set default values
    layoutConfigurationJSON = FPSTR(default_configuration_cstring);
    dataUpdateJSON = FPSTR(default_data_cstring);
}

ControllerServer::~ControllerServer()
{
}

RequestType ControllerServer::matchRequest(const char *request)
{
    // Separate the request into two parts
    if (!extractParts(request))
    {
        // If the request cannot be separated, it is invalid
        return RequestType::INVALID;
    }

    RequestType type = getRequestType(typeBuffer);

    // Extract parameters
    if ((type == RequestType::ACTION) && (parameterBuffer[0] == '\0'))
    {
        // No parameters for action
        return RequestType::INVALID;
    }

    return type;
}

bool ControllerServer::extractParts(const char *request)
{
    // Find the first part of the request
    char *first_begin = strchr(request, '/');

    // Check if the first part is present
    if (first_begin == NULL)
    {
        return false;
    }

    // Find the end of the first part
    char *first_end = strpbrk(first_begin + 1, "/ ");

    if (first_end == NULL)
    {
        return false;
    }

    // Check the size of the first part
    if (first_end - first_begin - 1 > typeBufferSize)
    {
        return false;
    }

    if (first_end[0] == '/')
    {
        // Find the end of the second part
        char *second_end = strchr(first_end + 1, ' ');

        if (second_end == NULL)
        {
            return false;
        }

        // Check the size of the second part
        if (second_end - first_end - 1 > parameterBufferSize)
        {
            return false;
        }

        // Copy the first part to the type buffer
        strncpy(typeBuffer, first_begin + 1, first_end - first_begin - 1);
        typeBuffer[first_end - first_begin - 1] = '\0';

        // Copy the second part to the parameters buffer
        strncpy(parameterBuffer, first_end + 1, second_end - first_end - 1);
        parameterBuffer[second_end - first_end - 1] = '\0';
    }
    else
    {
        // Copy the first part to the type buffer
        strncpy(typeBuffer, first_begin + 1, first_end - first_begin - 1);
        typeBuffer[first_end - first_begin - 1] = '\0';

        // No parameters
        parameterBuffer[0] = '\0';
    }

    return true;
}

void ControllerServer::serverSendResponse(WiFiClient &client, RequestType type)
{
    switch (type)
    {
    case RequestType::INDEX:
        // Send the index page
        client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"));
        client.print(FPSTR(robotkar_gui_main_cstring));
        break;

    case RequestType::INVALID:
        // Send the bad request page
        client.print(F("HTTP/1.1 400 Bad Request\r\nContent-Type: text/html\r\n\r\n"));
        client.print(FPSTR(robotkar_back_to_index_cstring));
        break;

    case RequestType::DATA:
        // Send the data update in JSON format
        client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n"));
        client.print(dataUpdateJSON);
        break;

    case RequestType::CONFIG:
        // Send the layout configuration in JSON format
        client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n"));
        client.print(layoutConfigurationJSON);
        break;

    case RequestType::ACTION:
        // Send the OK response
        client.print(F("HTTP/1.1 200 OK\r\n\r\n\r\n"));
        break;

    default:
        break;
    }
}

const char *ControllerServer::getParameters() const
{
    return parameterBuffer;
}

void ControllerServer::setLayoutConfiguration(const char *configuration)
{
    layoutConfigurationJSON = configuration;
}

void ControllerServer::setDataUpdate(const char *data)
{
    dataUpdateJSON = data;
}

RequestType ControllerServer::getRequestType(const char *request)
{
    // Check the first part of the request
    if ((strcmp(request, "") == 0) || (strcmp(request, "index") == 0))
    {
        return RequestType::INDEX;
    }
    else if (strcmp(request, "data") == 0)
    {
        return RequestType::DATA;
    }
    else if (strcmp(request, "config") == 0)
    {
        return RequestType::CONFIG;
    }
    else if (strcmp(request, "action") == 0)
    {
        return RequestType::ACTION;
    }

    // No match, return invalid
    return RequestType::INVALID;
}
