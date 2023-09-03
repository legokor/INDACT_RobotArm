#ifndef WIFICONTROLLER_H_
#define WIFICONTROLLER_H_

/**
 * @file WifiController.h
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 * 
 * @brief This file contains the declaration of the WifiController class.
 * 
 */

#include <string>

#include <ESP8266WiFi.h>

#include "Command.hpp"
#include "CommandManager.hpp"
#include "MessageHelper.h"
#include "ServerManager.h"

namespace wifi_controller
{

    /**
     * @brief This class is responsible for running the wifi controller.
     */
    class WifiController
    {
    public:
        /**
         * @brief Simple constructor.
         */
        WifiController();

        /**
         * @brief This function has to be called in the Arduino loop function.
         */
        void Loop();

    private:
        std::string ssid;
        std::string password;
        const std::string DEFAULT_AP_SSID{"RobotArmWifi"};

        CommandManager commandManager;
        ServerManager serverManager;
        MessageHelper messageHelper;

        void resetHandler(const std::string &data);
        void synchronizeHandler(const std::string &data);
        void connectStationHandler(const std::string &data);
        void setupAccessPointHandler(const std::string &data);
        void ssidHandler(const std::string &data);
        void passwordHandler(const std::string &data);
        void configureLayoutHandler(const std::string &data);
        void updateDataHandler(const std::string &data);

        void actionCallback(const std::string &action);

    }; // class WifiController

} // namespace wifi_controller

#endif // WIFICONTROLLER_H_
