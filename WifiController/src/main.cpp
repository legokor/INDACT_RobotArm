/**
 * @file RobotArmWifi.ino
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2023-08-29
 *
 * @brief This file contains the main Arduino program.
 *
 */

#include "board_configuration.h"
#include "WifiController.h"

// Define DEBUG to enable debug printing
#define DEBUG

#ifdef DEBUG

#define DEBUG_PRINT_INTERVAL_MS (1000 * 8) // 8 seconds

#endif // DEBUG

wifi_controller::WifiController wifiController;

// Arduino setup function
void setup()
{
    pinMode(EXTERNAL_LED, OUTPUT);
    digitalWrite(EXTERNAL_LED, LED_OFF);

    Serial.begin(115200);
    Serial.println();
}

// Arduino loop function
void loop()
{
    // Call the WifiController loop function
    wifiController.Loop();

#ifdef DEBUG
    // Periodically print debug information
    static unsigned long last_millis = 0;
    if (millis() - last_millis > DEBUG_PRINT_INTERVAL_MS)
    {
        last_millis = millis();

        Serial.println("////////////////////////////////////////");

        // Print time
        Serial.print("Time: ");
        Serial.println(last_millis);

        // Print memory usage
        Serial.print("Free heap: ");
        Serial.println(ESP.getFreeHeap());

        // Print WiFi status
        Serial.print("WiFi status: ");
        Serial.println(WiFi.status());

        Serial.println("////////////////////////////////////////");
    }
#endif // DEBUG
}
