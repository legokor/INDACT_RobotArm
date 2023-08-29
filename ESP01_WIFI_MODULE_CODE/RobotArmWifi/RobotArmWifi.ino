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

// Set DEBUG to 1 to enable debug output
#define DEBUG 1

robot_arm_wifi::WifiController wifiController;

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

#if DEBUG == 1
    // Periodically print debug information
    static unsigned long last_millis = 0;
    if (millis() - last_millis > 10000)
    {
        last_millis = millis();

        Serial.println("////////////////////////////////////////");

        // Print time
        Serial.print("Time: ");
        Serial.println(millis());

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
