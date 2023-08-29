#ifndef SERVERMANAGER_H_
#define SERVERMANAGER_H_

/**
 * @file ServerManager.h
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 * 
 * @brief This file contains the declaration of the ServerManager class.
 * 
 */

#include <ESP8266WiFi.h>

#include <string>
#include <functional>

namespace robot_arm_wifi
{

    class ServerManager
    {
    public:
        ServerManager(std::function<void(const std::string &)> action_callback);

        void SetLayoutConfiguration(const std::string &layout_configuration);
        void SetDataUpdate(const std::string &data_update);
        void SetActionCallback(std::function<void(const std::string &)> action_callback);

        void Run();

    private:
        enum class RequestType
        {
            INVALID,
            INDEX,
            DATA,
            CONFIG,
            ACTION
        };

        WiFiServer wifiServer{80};

        std::string layoutConfiguration;
        std::string dataUpdate;
        std::string action;
        std::function<void(const std::string &)> actionCallback;

        RequestType MatchRequest(const std::string &request);
        void SendResponse(WiFiClient &client, RequestType type);

    }; // class ServerManager

} // namespace robot_arm_wifi

#endif // SERVERMANAGER_H_
