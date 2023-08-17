# INDACT Wi-Fi module

*This is the documentation of the Wi-Fi module for the INDACT robotic arm project.*

## About the project

The Wi-Fi module provides a wireless control interface for the INDACT robotic arm. It is based on an ESP8266 Wi-Fi module and the Arduino framework.

The module is designed to be easily integrated into the INDACT robotic arm. It is connected to the arm's control board via a serial interface. The module can be powered from the arm's control board or from an external power supply.

The module provides a web interface for controlling the arm. The web interface is served from the module itself. The module can also be controlled via a custom web API.

## Hardware requirements

To use the Wi-Fi module, the following hardware is needed:

- ESP8266 Wi-Fi module
- Power supply for the Wi-Fi module
- 2 wires for UART communication with the main control board
- 1 optional LED for status indication
- Other components needed for the specific Wi-Fi module that is used

For the INDACT project an ESP-01 module is used, while the power supply is provided by the arm's controller board. The module is connected to the control board via a 2-wire UART interface.

## Software requirements

The Wi-Fi module is based on the Arduino framework. The following libraries are used:

- [ESP8266 Arduino Core](https://github.com/esp8266/Arduino "ESP8266 Arduino Core on GitHub")

For uploading the firmware to the module, the most convenient way is to use the [Arduino IDE](https://www.arduino.cc/en/software "Arduino IDE"). The Arduino IDE can be used to install the ESP8266 Arduino Core library as well. For the development of the firmware, the [Visual Studio Code](https://code.visualstudio.com/ "Visual Studio Code") editor was used with the [Arduino extension](https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino "Arduino extension for Visual Studio Code").

Before uploading the firmware to the module, the `RobotKar_WIFI/board_configuration.h` file should be edited to match the specific hardware configuration. Furthermore, one of the prebuild scripts (`prebuild.sh`, `prebuild.bat`) should be executed to generate C-string representations of the web interface files. The generated files are used by the firmware to serve the web interface.

## Usage

### I. Web user interface

The web user interface is served from the module itself. The module acts as a web server. The web interface can be accessed from any device that is connected to the same network as the module or directly to the module's Wi-Fi access point.

To see the web interface, open the module's IP address in a web browser. The IP address can be found in the module's serial output or in the router's DHCP client list. The default IP address for the access point is `192.168.0.4`. With the default address, the web interface can be accessed at `http://192.168.0.4/`.

The web interface provides a control panel for the arm. It was designed to be customizable so that the control interface can be adapted to the specific needs of the user. However, the layout of the control panel is fixed. The control panel consists of the following elements:

- **Page header** - The page header should contain the name of the control panel.
- **Top text field** - The top text field can be used to display any text. It can be used to display information about the current state of the arm.
- **Bottom text field** - The bottom text field can be used to display any text. It can be used to display information about the current state of the arm.
- **Control table** - The control table contains the control elements. Each row of the table contains a control element. The control elements can be buttons, sliders, or text fields. The control elements can be used to control the arm. The control elements can be configured to send custom commands to the arm's control board. The control elements can also be configured to display custom text in the top or bottom text fields.

### II. Web API

The web API can be used to control the arm from any device that is connected to the same network as the module or directly to the module's Wi-Fi access point. The web API can be accessed at `http://<module IP address>/action/`.

Any text that comes after the `/action/` part of the URL is forwarded to the main controller as is. The interpretation of the text is up to the main controller.

### III. Serial interface

TODO: Continue documentation ...
