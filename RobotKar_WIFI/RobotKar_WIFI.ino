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

// #define DEBUG 0

#ifndef DEF_AP_SSID
#define DEF_AP_SSID "INDACT_robotkar"
#define DEF_AP_PSW "pirosalma"
#endif

#define EXT_LED 1
#define MSG_BUFFER_SIZE (128 + 2)
#define WIFI_STRING_SIZE 32

#define CONNECT_TIMEOUT_MS 30000

/** @brief Buffer for the incoming messages. */
char msg_buffer[MSG_BUFFER_SIZE] = { 0 };

/** @brief SSID for WiFi connection. */
char ssid[WIFI_STRING_SIZE] = { 0 };
/** @brief Password for WiFi connection. */
char password[WIFI_STRING_SIZE] = { 0 };
/** @brief Status message in html format. */
char status[MSG_BUFFER_SIZE] = { 0 };

/** @brief Instance of the server. */
WiFiServer server(80);

void setup() {
  pinMode(EXT_LED, OUTPUT);
  digitalWrite(EXT_LED, LOW);

  Serial.begin(115200);
#ifdef DEBUG
  Serial.setDebugOutput(true);

  Serial.println();
#endif

  // Start the server
  server.begin();
}

void loop() {
  // Handle the messages from the controller
  handle_messages();

  // Let the WiFi tasks run
  yield();

  // Handle the connected clients
  handle_clients();
}

/**
 * @brief Receives and replys to the incoming messages, if there is a command
 *        then executes it.
 */
void handle_messages(void) {
  // The rest of this function only runs if a full message arrived at Serial.
  if (!receiveMessage(msg_buffer, MSG_BUFFER_SIZE)) {
    return;
  }
#ifdef DEBUG
  Serial.println(msg_buffer);
#endif

  // A message can set one of the string variables of the module if the
  // controller sent a request asking for that to happen.
  static bool catchMessage = false;
  static char* messageTo;
  if (catchMessage) {
    catchMessage = false;

    if ((messageTo == ssid) || (messageTo == password) && (strlen(msg_buffer) + 1 >= WIFI_STRING_SIZE)) {
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

#ifndef DEBUG
  if (!((req_type == RequestType::INVALID) || (req_type == RequestType::INDEX))) {
#endif
    // Send the request to the controller
    char message[5];
    requestToMessage(req_type, message);
    Serial.print(message);
#ifndef DEBUG
  }
#endif

  // read/ignore the rest of the request
  while (client.available()) {
    client.read();
  }

  // Send the response to the client
  if (req_type == RequestType::INVALID) {
    client.print(F("HTTP/1.1 400 Bad Request\r\nContent-Type: text/html\r\n\r\n"));
    client.print(FPSTR(back_to_index_page));
    return;
  }

  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"));
  client.print(FPSTR(control_page_before_status));
  static bool b;
  if (b) {
    client.print(F("UwU"));

  } else {
    client.print(F("OwO"));
  }
  b = !b;
  client.print(F("<br>"));
  client.print(status);
  client.print(FPSTR(control_page_after_status));
}

/**
 * @brief Connect to an existing WiFi network, and print the IP address on
 *        Serial. If the connection attempt is not successful, then print a "no
 *        connection" message on Serial.
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

  // Wait for connection and give visible feedback to the user about connecting process
  unsigned long start_time = millis();
  while ((WiFi.status() != WL_CONNECTED)
         && (millis() - start_time) <= CONNECT_TIMEOUT_MS) {

    delay(250);
    digitalWrite(EXT_LED, HIGH);
    delay(250);
    digitalWrite(EXT_LED, LOW);

#ifdef DEBUG
    Serial.print(". ");
#endif
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
    digitalWrite(EXT_LED, HIGH);

  } else {
    WiFi.disconnect();
    sendFail();
  }

#ifdef DEBUG
  WiFi.printDiag(Serial);
  Serial.println();
#endif
}

/**
 * @brief Create a soft access point, and print the IP address on Serial. If
 *        the connection attempt is not successful, then print a "no
 *        connection" message on Serial.
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

#ifdef DEBUG
  WiFi.printDiag(Serial);
  Serial.println();
#endif
}

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
