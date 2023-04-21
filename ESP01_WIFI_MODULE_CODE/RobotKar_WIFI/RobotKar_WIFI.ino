/**
 ***************************************************************************************************
 * @file RobotKar_WIFI.ino
 *
 * @date Jan 20, 2023
 * @author Varga Peter
 ***************************************************************************************************
 * @brief This program provides wireless controls for the robot arm.
 * @details
 ***************************************************************************************************
 */

#include <ESP8266WiFi.h>
#include <string.h>

#include "board_configuration.h"

#include "ControllerServer.h"
#include "MessageHandler.h"
#include "CommandHandler.h"

#include "protocols.h"

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Macro definitions
// /////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief Default SSID for the Soft Access Point. */
#define DEF_AP_SSID "ESP8266_Controller"

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Using statements
// /////////////////////////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace ESP8266_Controller;

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Typedef-s, global constants and variables
// /////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief Buffer for the incoming messages. */
char receiveBuffer[MESSAGE_MAX_SIZE + 1] = {0};

/** @brief Buffer for the complete messages. */
char messageBuffer[MESSAGE_MAX_SIZE + 1] = {0};

/** @brief SSID for WiFi connection. */
char ssid[WIFI_STRING_SIZE + 1] = {0};

/** @brief Password for WiFi connection. */
char password[WIFI_STRING_SIZE + 1] = {0};

/** @brief Buffer for the server type. */
char serverTypeBuffer[MESSAGE_MAX_SIZE + 1] = {0};

/** @brief Buffer for the server parameters. */
char serverParameterBuffer[MESSAGE_MAX_SIZE + 1] = {0};

/** @brief Buffer for a new message. */
char newMessageBuffer[MESSAGE_MAX_SIZE + 1] = {0};

/** @brief Instance of the wifiServer. */
WiFiServer wifiServer(80);

/** @brief Instance of the server. */
ControllerServer server(serverTypeBuffer, sizeof(serverTypeBuffer), serverParameterBuffer, sizeof(serverParameterBuffer));

/** @brief Instance of the message handler. */
MessageHandler messageHandler(receiveBuffer, messageBuffer, sizeof(messageBuffer), MESSAGE_BEGIN_MARKER, MESSAGE_END_MARKER);

/**
 * @defgroup command_handler_functions Command handler functions
 * @brief This group contains the command handler functions.
 * @details Each command handler has the type of CommandHandlerFunction. The command handler
 * functions are called when their corresponding command is received from the main controller.
 * @{
 */
void resetHandler(const char *data);
void synchronizeHandler(const char *data);
void connectStationHandler(const char *data);
void setupAccessPointHandler(const char *data);
void ssidHandler(const char *data);
void passwordHandler(const char *data);
void configureLayoutHandler(const char *data);
void updateDataHandler(const char *data);
/** @} */

/** @brief Array of the commands. */
const Command commandArray[] = {
    {STR_RESET, resetHandler},
    {STR_SYNCHRONIZE, synchronizeHandler},
    {STR_CONNECT_STATION, connectStationHandler},
    {STR_SETUP_ACCESS_POINT, setupAccessPointHandler},
    {STR_SSID, ssidHandler},
    {STR_PASSWORD, passwordHandler},
    {STR_CONFIGURE_LAYOUT, configureLayoutHandler},
    {STR_UPDATE_DATA, updateDataHandler}};

/** @brief Instance of the command handler. */
CommandHandler commandHandler(commandArray, sizeof(commandArray) / sizeof(Command));

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Inline functions
// /////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Sends a confirmation message to the main controller.
 */
inline void sendConfirm(void)
{
    messageHandler.sendMessage(STR_CONFIRM);
}

/**
 * @brief Sends a fail message to the main controller.
 */
inline void sendFail(void)
{
    messageHandler.sendMessage(STR_FAIL);
}

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino core setup() and loop()
// /////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Arduino core setup.
 */
void setup()
{
    pinMode(EXTERNAL_LED, OUTPUT);
    digitalWrite(EXTERNAL_LED, LED_OFF);

    Serial.begin(115200);

    // Start the wifiServer
    wifiServer.begin();
}

/**
 * @brief Arduino core loop.
 */
void loop()
{
    // Handle the messages from the controller
    handle_messages();

    // Let the WiFi task run
    yield();

    // Handle the connected clients
    handle_clients();
}

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Main handler functions
// /////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Receives and replys to the incoming messages, if there is a command
 *        then executes it.
 */
void handle_messages(void)
{
    // The rest of this function only runs if a full message arrived at Serial.
    if (!messageHandler.receiveMessage())
    {
        return;
    }

    // Check if the message is a command
    if (commandHandler.matchCommand(messageHandler.getMessage()))
    {
        // Notify the main controller that the command is received
        sendConfirm();

        // Execute the command
        commandHandler.executeCommand();
    }
    else
    {
        // Notify the main controller that the command is not received
        sendFail();
    }
}

/**
 * @brief Checks if a client has connected, and if so, handles the request.
 */
void handle_clients(void)
{
    // Check if a client has connected
    WiFiClient client = wifiServer.accept();
    if (!client)
    {
        return;
    }

    // Wait until the client sends some data
    client.setTimeout(5000);

    // Read the first line of the request
    String request_string = client.readStringUntil('\r');
    
    // Match the request
    RequestType type = server.matchRequest(request_string.c_str());

    // read/ignore the rest of the request
    while (client.available())
    {
        client.read();
    }

    // Send the response to the client
    server.serverSendResponse(client, type);

    // Send the message to the controller
    if (type == RequestType::ACTION)
    {
        newMessageBuffer[0] = '\0';
        strcat(newMessageBuffer, STR_ACTION);
        strcat(newMessageBuffer, " ");
        strcat(newMessageBuffer, server.getParameters());
        messageHandler.sendMessage(newMessageBuffer);
    }
}

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Command handler function definitions
// /////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Reset the ESP8266.
 * @details This function is called when the reset command is received from the main
 * controller.
 * @param data Unused
 */
void resetHandler(const char *data)
{
    // Remove unused parameter warning
    (void)data;

    // Reset the ESP8266
    ESP.reset();
}

/**
 * @brief Synchronize the ESP8266.
 * @details This function is called when the synchronize command is received from the main
 * controller. The response is sent back to the main controller in the specied format.
 * @param data Unused
 */
void synchronizeHandler(const char *data)
{
    // Remove unused parameter warning
    (void)data;

    // Send the synchronize message
    for (size_t i = 0; i < SYNC_NUMBER; i++)
    {
        messageHandler.sendMessage(STR_SYNC_CODE);
    }
}

/**
 * @brief Connect to the WiFi network.
 * @details This function is called when the connect station command is received from the main
 * controller. If the connection is successful, the response is going to be the IP address.
 * @param data Unused
 */
void connectStationHandler(const char *data)
{
    // Remove unused parameter warning
    (void)data;

    // If the SSID and password is not set, then do nothing and notify the controller about the
    // unsuccessful connection
    if ((ssid[0] == 0) || (password[0] == 0))
    {
        sendFail();
        return;
    }

    // Set the WiFi mode to Station
    WiFi.mode(WIFI_STA);
    // Start the connection process
    WiFi.begin(ssid, password);

    // Wait for connection and give visible feedback to the user about the process
    unsigned long start_time = millis();
    while ((WiFi.status() != WL_CONNECTED) && (millis() - start_time) <= CONNECT_TIMEOUT_MS)
    {
        // Wait while blinking the LED
        delay(250);
        digitalWrite(EXTERNAL_LED, LED_ON);
        delay(250);
        digitalWrite(EXTERNAL_LED, LED_OFF);
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        // Send the IP address to the controller
        newMessageBuffer[0] = '\0';
        strcat_P(newMessageBuffer, PSTR(STR_IP));
        strcat(newMessageBuffer, " ");
        strcat(newMessageBuffer, WiFi.localIP().toString().c_str());
        messageHandler.sendMessage(newMessageBuffer);

        // Give visible feedback to the user
        digitalWrite(EXTERNAL_LED, LED_ON);
    }
    else
    {
        // Stop the connection process
        WiFi.disconnect();

        // Send the fail message to the controller
        sendFail();
    }
}

/**
 * @brief Setup the Soft Access Point.
 * @details This function is called when the setup access point command is received from the main
 * controller. If the setup is successful, the response is going to be the IP address.
 * @param data Unused
 */
void setupAccessPointHandler(const char *data)
{
    // Remove unused parameter warning
    (void)data;

    // Set the mode to Soft Access Point
    WiFi.mode(WIFI_AP);

    // Start the access point
    if (WiFi.softAP((ssid[0] == 0) ? DEF_AP_SSID : ssid, (password[0] == 0) ? NULL : password))
    {
        // Send the IP address to the controller
        newMessageBuffer[0] = '\0';
        strcat_P(newMessageBuffer, PSTR(STR_IP));
        strcat(newMessageBuffer, " ");
        strcat(newMessageBuffer, WiFi.softAPIP().toString().c_str());
        messageHandler.sendMessage(newMessageBuffer);

        // Give visible feedback to the user
        digitalWrite(EXTERNAL_LED, LED_ON);
    }
    else
    {
        // Stop the connection process
        WiFi.disconnect();

        // Send the fail message to the controller
        sendFail();
    }
}

/**
 * @brief Set the SSID for the WiFi connection.
 * @details This function is called when the SSID command is received from the main controller.
 * @param data SSID
 */
void ssidHandler(const char *data)
{
    // Set the new SSID
    if (data == NULL)
    {
        ssid[0] = 0;
    }
    else
    {
        strcpy(ssid, data);
    }
}

/**
 * @brief Set the password for the WiFi connection.
 * @details This function is called when the password command is received from the main controller.
 * @param data Password
 */
void passwordHandler(const char *data)
{
    // Set the new password
    if (data == NULL)
    {
        password[0] = 0;
    }
    else
    {
        strcpy(password, data);
    }
}

/**
 * @brief Configure the layout of the board.
 * @details This function is called when the configure layout command is received from the main
 * controller.
 * @param data Layout configuration
 */
void configureLayoutHandler(const char *configuration)
{
    server.setLayoutConfiguration(configuration);
}

/**
 * @brief Update the data of the board.
 * @details This function is called when the update data command is received from the main
 * controller.
 * @param data Data
 */
void updateDataHandler(const char *data)
{
    server.setDataUpdate(data);
}
