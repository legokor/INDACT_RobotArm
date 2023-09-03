/**
 * @file ServerManager.cpp
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 *
 * @brief This file contains the implementation of the ServerManager class.
 *
 */

#include "ServerManager.h"

#include <sstream>
#include <cstddef>

#include <Arduino.h>
#include <LittleFS.h>

namespace wifi_controller
{

    ServerManager::ServerManager(const std::function<void(const std::string &)> &action_callback)
        : actionCallback(action_callback)
    {
        LittleFS.begin();

        server.on("/", HTTP_GET, std::bind(&ServerManager::handleIndex, this, std::placeholders::_1));
        server.on("/config", HTTP_GET, std::bind(&ServerManager::handleConfig, this, std::placeholders::_1));
        server.on("/data", HTTP_GET, std::bind(&ServerManager::handleData, this, std::placeholders::_1));
        server.on("/action*", HTTP_GET, std::bind(&ServerManager::handleAction, this, std::placeholders::_1));
        server.onNotFound(std::bind(&ServerManager::handleNotFound, this, std::placeholders::_1));
        server.begin();
    }

    void ServerManager::SetLayoutConfiguration(const std::string &layout_configuration)
    {
        this->layoutConfiguration = layout_configuration;
    }

    void ServerManager::SetDataUpdate(const std::string &data_update)
    {
        this->dataUpdate = data_update;
    }

    void ServerManager::SetActionCallback(const std::function<void(const std::string &)> &action_callback)
    {
        this->actionCallback = action_callback;
    }

    void ServerManager::handleIndex(AsyncWebServerRequest *request)
    {
        request->send(LittleFS, "/gui_main.html", "text/html");
    }

    void ServerManager::handleConfig(AsyncWebServerRequest *request)
    {
        if (this->layoutConfiguration.empty())
        {
            request->send(LittleFS, "/config.json", "application/json");
        }
        else
        {
            request->send(200, "application/json", this->layoutConfiguration.c_str());
        }
    }

    void ServerManager::handleData(AsyncWebServerRequest *request)
    {
        if (this->dataUpdate.empty())
        {
            request->send(LittleFS, "/data.json", "application/json");
        }
        else
        {
            request->send(200, "application/json", this->dataUpdate.c_str());
        }
    }

    void ServerManager::handleAction(AsyncWebServerRequest *request)
    {
        // The request object does not contain the full URL (with GET parameters) so the URL has to be reconstructed
        // for the action callback just without the "/action" prefix.
        std::ostringstream oss;
        oss << request->url().substring(7).c_str();
        for (std::size_t i = 0; i < request->args(); ++i)
        {
            if (i == 0)
            {
                oss << "?";
            }
            else
            {
                oss << "&";
            }
            oss << request->argName(i).c_str() << "=" << request->arg(i).c_str();
        }

        this->actionCallback(oss.str());
        request->send(200);
    }

    void ServerManager::handleNotFound(AsyncWebServerRequest *request)
    {
        request->send(LittleFS, "/back_to_index.html", "text/html");
    }

} // namespace wifi_controller
