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

    /**
     * @brief Class for handling the server.
     */
    class ServerManager
    {
    public:
        /**
         * @brief Constructor with action callback initialization.
         *
         * @param action_callback The callback function to be called when an action is received.
         */
        ServerManager(std::function<void(const std::string &)> action_callback);

        /**
         * @brief Set the layout configuration.
         *
         * @param layout_configuration The layout configuration to be set.
         */
        void SetLayoutConfiguration(const std::string &layout_configuration);

        /**
         * @brief Set the data update.
         *
         * @param data_update The data update to be set.
         */
        void SetDataUpdate(const std::string &data_update);

        /**
         * @brief Set the action callback function.
         *
         * @param action_callback The callback function to be called when an action is received.
         */
        void SetActionCallback(std::function<void(const std::string &)> action_callback);

        /**
         * @brief Run the server.
         */
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
