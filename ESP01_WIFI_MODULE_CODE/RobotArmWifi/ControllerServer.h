/**
 ***************************************************************************************************
 * @file ControllerServer.h
 *
 * @date 2023. 04. 14.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief This file contains the declaration of the ControllerServer class and related data types.
 ***************************************************************************************************
 */

#ifndef CONTROLLERSERVER_H_
#define CONTROLLERSERVER_H_

#include <ESP8266WiFi.h>

namespace ESP8266_Controller
{

    /**
     * @brief Enum for the request types
     */
    enum class RequestType
    {
        INVALID, /**< Invalid request */
        INDEX,   /**< Index request */
        DATA,    /**< Data request */
        CONFIG,  /**< Configuration request */
        ACTION   /**< Action request */
    };

    /**
     * @brief Class for handling the server.
     */
    class ControllerServer
    {
    private:
        String layoutConfigurationJSON;
        String dataUpdateJSON;

        char *typeBuffer;
        size_t typeBufferSize;
        char *parameterBuffer;
        size_t parameterBufferSize;

    public:
        /**
         * @brief Constructor with static buffer allocation.
         * @param type_buffer Buffer for the type of the request
         * @param type_buffer_size Size of the type buffer
         * @param parameter_buffer Buffer for the parameters of the request
         * @param parameter_buffer_size Size of the parameter buffer
         */
        ControllerServer(char *type_buffer, size_t type_buffer_size, char *parameter_buffer, size_t parameter_buffer_size);

        /**
         * @brief Extracts the request type and parameters from a given request.
         * @param request A string containing the first line of an HTTP request
         * @return Type of the request, can be anything specified in the RequestType enum
         */
        RequestType matchRequest(const char *request);

        /**
         * @brief Sends the response to the client.
         * @param client Client to send the response to
         * @param type Type of the request to send the response for
         */
        void serverSendResponse(WiFiClient &client, RequestType type);

        /**
         * @brief Gets the parameters of the last request.
         * @return Parameters of the request
         */
        const char *getParameters() const;

        /**
         * @brief Sets the layout configuration.
         * @param configuration Layout configuration in JSON format
         */
        void setLayoutConfiguration(const char *configuration);

        /**
         * @brief Sets the data update.
         * @param data Data update in JSON format
         */
        void setDataUpdate(const char *data);

    private:
        /**
         * @brief Gets the request type from a given request.
         * @param request String containing the request
         * @return Request type
         */
        RequestType getRequestType(const char *request);

        /**
         * @brief Extract the type and parameters from the request.
         * @param request The request
         * @return True if the extraction was successful, false otherwise
         */
        bool extractParts(const char *request);

    }; /* class Server */

} /* namespace CustomServer */

#endif /* CONTROLLERSERVER_H_ */
