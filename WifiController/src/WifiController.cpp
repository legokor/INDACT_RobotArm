/**
 * @file WifiController.cpp
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 * 
 * @brief This file contains the implementation of the WifiController class.
 * 
 */

#include "WifiController.h"

#include <Arduino.h>

#include <sstream>

#include "board_configuration.h"
#include "protocols.h"

namespace wifi_controller
{

    WifiController::WifiController()
        : serverManager(std::bind(&WifiController::actionCallback, this, std::placeholders::_1))
    {
        this->commandManager.AddCommand(STR_RESET, std::bind(&WifiController::resetHandler, this, std::placeholders::_1));
        this->commandManager.AddCommand(STR_SYNCHRONIZE, std::bind(&WifiController::synchronizeHandler, this, std::placeholders::_1));
        this->commandManager.AddCommand(STR_CONNECT_STATION, std::bind(&WifiController::connectStationHandler, this, std::placeholders::_1));
        this->commandManager.AddCommand(STR_SETUP_ACCESS_POINT, std::bind(&WifiController::setupAccessPointHandler, this, std::placeholders::_1));
        this->commandManager.AddCommand(STR_SSID, std::bind(&WifiController::ssidHandler, this, std::placeholders::_1));
        this->commandManager.AddCommand(STR_PASSWORD, std::bind(&WifiController::passwordHandler, this, std::placeholders::_1));
        this->commandManager.AddCommand(STR_CONFIGURE_LAYOUT, std::bind(&WifiController::configureLayoutHandler, this, std::placeholders::_1));
        this->commandManager.AddCommand(STR_UPDATE_DATA, std::bind(&WifiController::updateDataHandler, this, std::placeholders::_1));
    }

    void WifiController::Loop()
    {
        std::string message;
        if (this->messageHelper.ReceiveMessage(message))
        {
            Command command;
            std::string data;
            if (this->commandManager.MatchCommand(message, command, data))
            {
                this->messageHelper.SendConfirm();
                command.Execute(data);
            }
            else
            {
                this->messageHelper.SendFail();
            }
        }
    }

    void WifiController::resetHandler(const std::string &data)
    {
        delay(10);
        ESP.reset();
    }

    void WifiController::synchronizeHandler(const std::string &data)
    {
        messageHelper.SendConfirm();
    }

    void WifiController::connectStationHandler(const std::string &data)
    {
        if (this->ssid.empty() || this->password.empty())
        {
            this->messageHelper.SendFail();
            return;
        }

        WiFi.mode(WIFI_STA);
        WiFi.begin(this->ssid.c_str(), this->password.c_str());

        // Wait for connection
        unsigned long start_time = millis();
        while ((WiFi.status() != WL_CONNECTED) && (millis() - start_time) <= CONNECT_TIMEOUT_MS)
        {
            // Blink the LED while waiting for connection
            delay(250);
            digitalWrite(EXTERNAL_LED, LED_ON);
            delay(250);
            digitalWrite(EXTERNAL_LED, LED_OFF);
        }

        if (WiFi.status() == WL_CONNECTED)
        {
            this->messageHelper.SendIpAddress(WiFi.localIP().toString().c_str());
            // Give visible feedback to the user
            digitalWrite(EXTERNAL_LED, LED_ON);
        }
        else
        {
            WiFi.disconnect();
            this->messageHelper.SendFail();
        }
    }

    void WifiController::setupAccessPointHandler(const std::string &data)
    {
        WiFi.mode(WIFI_AP);

        // Start the access point
        if (WiFi.softAP(
                ssid.empty() ? DEFAULT_AP_SSID.c_str() : ssid.c_str(),
                password.empty() ? NULL : password.c_str()))
        {
            this->messageHelper.SendIpAddress(WiFi.softAPIP().toString().c_str());
            // Give visible feedback to the user
            digitalWrite(EXTERNAL_LED, LED_ON);
        }
        else
        {
            WiFi.disconnect();
            this->messageHelper.SendFail();
        }
    }

    void WifiController::ssidHandler(const std::string &data)
    {
        if (data.length() <= 32)
        {
            this->ssid = data;
        }
    }

    void WifiController::passwordHandler(const std::string &data)
    {
        if (data.length() >= 8)
        {
            this->password = data;
        }
    }

    void WifiController::configureLayoutHandler(const std::string &data)
    {
        this->serverManager.SetLayoutConfiguration(data);
    }

    void WifiController::updateDataHandler(const std::string &data)
    {
        this->serverManager.SetDataUpdate(data);
    }

    void WifiController::actionCallback(const std::string &action)
    {
        this->messageHelper.SendAction(action);
    }

} // namespace wifi_controller
