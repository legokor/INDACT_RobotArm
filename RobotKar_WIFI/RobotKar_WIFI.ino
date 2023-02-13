/**
 *******************************************************************************
 * @file RobotKar_WIFI.ino
 *
 * @date Jan 20, 2023
 * @author Varga Peter
 *******************************************************************************
 * @brief This program provides wireless controls for the robot arm.
 * @details 
 *******************************************************************************
 */

#include <ESP8266WiFi.h>
#include <string.h>

#include "robotkar_gui_html.h"
#include "control_codes.h"
#include "protocols.h"

// ////////////////////////////////////////////////////////////////////////////
// Macro definitions
// ////////////////////////////////////////////////////////////////////////////

/**@{*/
/**
 * @brief Default settings for the WiFi soft access point.
 */
#define DEF_AP_SSID "INDACT_robotkar"
#define DEF_AP_PSW "pirosalma"
/**@}*/

/**
 * @brief Number of the GPIO pin that is being used by this program to give
 *        visual feedback to the user by turning on and off an external LED.
 */
#define EXT_LED 1

/**
 * @brief The maximum amount of time in milliseconds that the module can wait
 *        for a successful connection (WiFi station connection).
 */
#define CONNECT_TIMEOUT_MS 30000

// ////////////////////////////////////////////////////////////////////////////
// Typedef-s, global constants and variables
// ////////////////////////////////////////////////////////////////////////////

/** @brief Buffer for the incoming messages. */
char msg_buffer[MESSAGE_MAX_SIZE + 2] = { 0 };
/** @brief Size of the message buffer. */
const unsigned int messageBufferSize = MESSAGE_MAX_SIZE + 2;

/** @brief SSID for WiFi connection. */
char ssid[WIFI_STRING_SIZE + 1] = { 0 };
/** @brief Password for WiFi connection. */
char password[WIFI_STRING_SIZE + 1] = { 0 };
/** @brief Status message in html format. */
char status[MESSAGE_MAX_SIZE + 1] = { 0 };

/**
 * @brief Store the names of the 3 axis of a coordinate system.
 */
typedef struct {
  const char* axis_1;
  const char* axis_2;
  const char* axis_3;
} CoordSystem;

/** @brief Cylindrical coordiante system. */
const CoordSystem cylindrical = {
  .axis_1 = "r",
  .axis_2 = "&phi;",
  .axis_3 = "z"
};
/** @brief Rectangular coordiante system. */
const CoordSystem rectangular = {
  .axis_1 = "x",
  .axis_2 = "y",
  .axis_3 = "z"
};
/** @brief Pointer of the displayed coordinate system. */
const CoordSystem* coordSystem = &cylindrical;

/** @brief Instance of the server. */
WiFiServer server(80);

// ////////////////////////////////////////////////////////////////////////////
// Arduino core setup() and loop()
// ////////////////////////////////////////////////////////////////////////////

/**
 * @brief Arduino core setup.
 */
void setup() {
  pinMode(EXT_LED, OUTPUT);
  digitalWrite(EXT_LED, LOW);

  Serial.begin(115200);

  // Start the server
  server.begin();
}

/**
 * @brief Arduino core loop.
 */
void loop() {
  // Handle the messages from the controller
  handle_messages();

  // Let the WiFi tasks run
  yield();

  // Handle the connected clients
  handle_clients();
}

// ////////////////////////////////////////////////////////////////////////////
// Main handler functions
// ////////////////////////////////////////////////////////////////////////////

/**
 * @brief Receives and replys to the incoming messages, if there is a command
 *        then executes it.
 */
void handle_messages(void) {
  // The rest of this function only runs if a full message arrived at Serial.
  if (!receiveMessage(msg_buffer, messageBufferSize)) {
    return;
  }

  // A message can set one of the string variables of the module if the
  // controller sent a request asking for that to happen.
  static bool catchMessage = false;
  static char* messageTo;
  if (catchMessage) {
    catchMessage = false;

    if ((messageTo == ssid) || (messageTo == password) && (strlen(msg_buffer) >= WIFI_STRING_SIZE)) {
      sendFail();
      return;
    }

    strcpy(messageTo, msg_buffer);
    sendConfirm();
    return;
  }

  if (strcmp_P(msg_buffer, PSTR(STR_RESET)) == 0) {
    sendConfirm();
    ESP.reset();

  } else if (strcmp_P(msg_buffer, PSTR(STR_CONNECT_STATION)) == 0) {
    sendConfirm();
    connectToNetwork();

  } else if (strcmp_P(msg_buffer, PSTR(STR_SETUP_ACCESS_POINT)) == 0) {
    sendConfirm();
    setupAccessPoint();

  } else if (strcmp_P(msg_buffer, PSTR(STR_SSID)) == 0) {
    sendConfirm();
    // Receive the new SSID
    catchMessage = true;
    messageTo = ssid;

  } else if (strcmp_P(msg_buffer, PSTR(STR_PASSWORD)) == 0) {
    sendConfirm();
    // Receive the new password
    catchMessage = true;
    messageTo = password;

  } else if (strcmp_P(msg_buffer, PSTR(STR_STATUS)) == 0) {
    sendConfirm();
    // Receive the new status
    catchMessage = true;
    messageTo = status;

  } else if (strcmp_P(msg_buffer, PSTR(STR_CHANGE_TO_CYLINDRICAL)) == 0) {
    sendConfirm();
    // Change the displayed coordinate system
    coordSystem = &cylindrical;

  } else if (strcmp_P(msg_buffer, PSTR(STR_CHANGE_TO_RECTANGULAR)) == 0) {
    sendConfirm();
    // Change the displayed coordinate system
    coordSystem = &rectangular;

  } else {
    sendFail();
  }
}

/**
 * @brief Checks wether a client is connected or not, and if there is a
 *        connection then handles the requests and closes the connection.
 */
void handle_clients(void) {
  // Check if a client has connected
  WiFiClient client = server.accept();
  if (!client) {
    return;
  }

  client.setTimeout(5000);

  // Read the first line of the request
  String request_string = client.readStringUntil('\r');

  // Match the request
  RequestType req_type = matchRequest(request_string);

  if (!((req_type == RequestType::INVALID) || (req_type == RequestType::INDEX))) {
    // Send the request to the controller
    char message[MODULE_REQUEST_SIZE + 1];
    requestToMessage(req_type, message);
    Serial.println(message);
  }

  // read/ignore the rest of the request
  while (client.available()) {
    client.read();
  }

  // Send the response to the client
  if (req_type == RequestType::INVALID) {
    client.print(F("HTTP/1.1 400 Bad Request\r\nContent-Type: text/html\r\n\r\n"));
    client.print(FPSTR(back_to_index_page));

  } else if (req_type == RequestType::CHANGE_COORDINATES) {
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"));
    client.print(FPSTR(wait_for_change_page));

  } else {
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"));
    sendControlPage(client);
  }
}

// ////////////////////////////////////////////////////////////////////////////
// Functions for handling WiFi and server tasks
// ////////////////////////////////////////////////////////////////////////////

/**
 * @brief Connect to an existing WiFi network, and print the IP address on
 *        Serial. If the connection attempt is not successful, then print a
 *        failure message on Serial.
 */
void connectToNetwork(void) {
  // If the SSID and password is not set, then do nothing and notify the
  // controller about the unsuccessful connection
  if ((ssid[0] == 0) || (password[0] == 0)) {
    sendFail();
    return;
  }

  // Start the connection process
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait for connection and give visible feedback to the user about the
  // process
  unsigned long start_time = millis();
  while ((WiFi.status() != WL_CONNECTED)
         && (millis() - start_time) <= CONNECT_TIMEOUT_MS) {
    delay(250);
    digitalWrite(EXT_LED, HIGH);
    delay(250);
    digitalWrite(EXT_LED, LOW);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
    digitalWrite(EXT_LED, HIGH);

  } else {
    WiFi.disconnect();
    sendFail();
  }
}

/**
 * @brief Create a soft access point, and print the IP address on Serial. If
 *        the connection attempt is not successful, then print a failure
 *        message on Serial.
 */
void setupAccessPoint(void) {
  WiFi.mode(WIFI_AP);

  if (WiFi.softAP((ssid[0] == 0) ? DEF_AP_SSID : ssid,
                  (password[0] == 0) ? DEF_AP_PSW : password)) {
    Serial.println(WiFi.softAPIP());
    digitalWrite(EXT_LED, HIGH);

  } else {
    sendFail();
  }
}

/**
 * @brief Send the control page to the client.
 * @param client A reference to the client object.
 */
void sendControlPage(WiFiClient& client) {
  client.print(FPSTR(control_page_section_1));

  // Show the user that the request arrived and the page updated.
  static bool b;
  if (b) {
    client.print(F("O"));

  } else {
    client.print(F("X"));
  }
  b = !b;

  client.print(F("<br>"));
  client.print(status);

  client.print(FPSTR(control_page_section_2));
  client.print(coordSystem->axis_1);

  client.print(FPSTR(control_page_section_3));
  client.print(coordSystem->axis_2);

  client.print(FPSTR(control_page_section_4));
  client.print(coordSystem->axis_3);

  client.print(FPSTR(control_page_section_5));
}

// ////////////////////////////////////////////////////////////////////////////
// Functions for serial communication tasks
// ////////////////////////////////////////////////////////////////////////////

/**
 * @brief Receive a string terminated by one CR and one NL character on the
 *        serial channel. To receive a full message, this function has to be
 *        called until it returns true.
 * @param buffer The place of the received string
 * @param size Size of the buffer, has to be at least the length of the
 *        expected message + 2
 * @return True if the terminating character arrived or the buffer got full,
 *        else false
 */
bool receiveMessage(char* buffer, uint32_t size) {
  const char* endMarker = "\r\n";
  static uint32_t idx = 0;

  while (Serial.available() > 0) {
    buffer[idx] = Serial.read();

    if ((idx > 0)
        && (buffer[idx - 1] == endMarker[0]) && (buffer[idx] == endMarker[1])) {
      buffer[idx - 1] = '\0';
      idx = 0;
      return true;
    }

    idx++;
    if (idx >= size) {
      buffer[size - 1] = '\0';
      idx = 0;
      return true;
    }
  }

  return false;
}

/**
 * @brief Send a confirmation message trough Serial.
 */
inline void sendConfirm(void) {
  Serial.println(F(STR_CONFIRM));
}

/**
 * @brief Send a notification about failure through Serial.
 */
inline void sendFail(void) {
  Serial.println(F(STR_FAIL));
}
