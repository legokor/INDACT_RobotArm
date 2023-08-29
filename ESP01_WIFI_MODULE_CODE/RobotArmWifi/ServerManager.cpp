/**
 * @file ServerManager.cpp
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 * 
 * @brief This file contains the implementation of the ServerManager class.
 * 
 */

#include "ServerManager.h"

static const char robotarm_gui_main_cstring[] PROGMEM = {
#include "cstring_text/robotarm_gui_main_cstring.h"
};

static const char robotarm_back_to_index_cstring[] PROGMEM = {
#include "cstring_text/robotarm_back_to_index_cstring.h"
};

static const char default_data_cstring[] PROGMEM = {
#include "cstring_text/default_data_cstring.h"
};

static const char default_configuration_cstring[] PROGMEM = {
#include "cstring_text/default_configuration_cstring.h"
};

namespace robot_arm_wifi
{

    ServerManager::ServerManager(std::function<void(const std::string &)> action_callback)
        : actionCallback(action_callback)
    {
        this->wifiServer.begin();
    }

    void ServerManager::SetLayoutConfiguration(const std::string &layout_configuration)
    {
        this->layoutConfiguration = layoutConfiguration;
    }

    void ServerManager::SetDataUpdate(const std::string &data_update)
    {
        this->dataUpdate = dataUpdate;
    }

    void ServerManager::SetActionCallback(std::function<void(const std::string &)> action_callback)
    {
        this->actionCallback = action_callback;
    }

    void ServerManager::Run()
    {
        auto client = wifiServer.accept();

        if (!client)
        {
            return;
        }

        client.setTimeout(5000);

        String request = client.readStringUntil('\r');
        RequestType type = this->MatchRequest(request.c_str());

        while (client.available())
        {
            client.read();
        }

        this->SendResponse(client, type);

        if (type == RequestType::ACTION)
        {
            this->actionCallback(this->action);
        }
    }

    ServerManager::RequestType ServerManager::MatchRequest(const std::string &request)
    {
        if ((request.find("GET / ") != std::string::npos) || (request.find("GET /index") != std::string::npos))
        {
            return RequestType::INDEX;
        }
        else if (request.find("GET /data") != std::string::npos)
        {
            return RequestType::DATA;
        }
        else if (request.find("GET /config") != std::string::npos)
        {
            return RequestType::CONFIG;
        }
        else if (request.find("GET /action") != std::string::npos)
        {
            auto start = request.find("/action");
            auto end = request.find(" HTTP/1.1");

            if ((start != std::string::npos) && (end != std::string::npos))
            {
                this->action = request.substr(start + 7, end - start - 7);
                return RequestType::ACTION;
            }
        }

        return RequestType::INVALID;
    }

    void ServerManager::SendResponse(WiFiClient &client, ServerManager::RequestType type)
    {
        switch (type)
        {
        case RequestType::INDEX:
            // Send the index page
            client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"));
            client.print(FPSTR(robotarm_gui_main_cstring));
            break;

        case RequestType::INVALID:
            // Send the bad request page
            client.print(F("HTTP/1.1 400 Bad Request\r\nContent-Type: text/html\r\n\r\n"));
            client.print(FPSTR(robotarm_back_to_index_cstring));
            break;

        case RequestType::CONFIG:
            // Send the layout configuration in JSON format
            client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n"));
            if (this->layoutConfiguration.empty())
            {
                client.print(FPSTR(default_configuration_cstring));
            }
            else
            {
                client.print(this->layoutConfiguration.c_str());
            }
            break;

        case RequestType::DATA:
            // Send the data update in JSON format
            client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n"));
            if (this->dataUpdate.empty())
            {
                client.print(FPSTR(default_data_cstring));
            }
            else
            {
                client.print(this->dataUpdate.c_str());
            }
            break;

        case RequestType::ACTION:
            // Send the OK response
            client.print(F("HTTP/1.1 200 OK\r\n\r\n\r\n"));
            break;
        }
    }

} // namespace robot_arm_wifi
