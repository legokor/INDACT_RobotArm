/**
 *******************************************************************************
 * @file WIFI.h
 *
 * @date Jan 19, 2023
 * @author Varga Péter
 *******************************************************************************
 * @brief
 * 
 *******************************************************************************
 */

#ifndef WIFI_H_
#define WIFI_H_

#include <string>
#include <stdexcept>

#include "FreeRTOS.h"
#include "queue.h"

#include "usart.h"
#include "SerialStream.h"

namespace ESP01
{

/**
 * @brief Type for the requests that the WiFi module can send to the controller.
 */
enum class RequestType
{
    INVALID, /**< INVALID Invalid request*/
    AXIS_A_PLUS, /**< AXIS_A_PLUS Move the robot in the positive direction of the A axis */
    AXIS_A_MINUS, /**< AXIS_A_MINUS Move the robot in the negative direction of the A axis */
    AXIS_B_PLUS, /**< AXIS_B_PLUS Move the robot in the positive direction of the B axis */
    AXIS_B_MINUS, /**< AXIS_B_MINUS Move the robot in the negative direction of the B axis */
    AXIS_C_PLUS, /**< AXIS_C_PLUS Move the robot in the positive direction of the C axis */
    AXIS_C_MINUS, /**< AXIS_C_MINUS Move the robot in the negative direction of the C axis */
    HOMING, /**< HOMING Begin homing sequence */
    CHANGE_COORDINATES, /**< CHANGE_COORDINATES Change the coordinate system of the robot arm */
};
/**
 * @brief Type for the possible coordinate systems that the controller can use.
 */
enum class CoordinateSystem
{
    RECTANGULAR, /**< RECTANGULAR Rectangular coordinate system */
    CYLINDRICAL/**< CYLINDRICAL Cylindrical coordinate system */
};

/**
 * @brief A class to handle communication with the WiFi module.
 */
class WIFI
{
public:
    /** SerialStream that handles the communication on the UART that the WiFi module uses. */
    SerialStream *serial;

private:
    const size_t messageQueueSize = 16;
    QueueHandle_t messageQueue;

    const size_t requestQueueSize = 32;
    QueueHandle_t requestQueue;

    std::string ssid;
    std::string password;

    bool ssid_set = false;
    bool password_set = false;

    std::string ip_address;

public:
    /**
     * @brief Constructor.
     * @param huart Pointer of the handle of the UART that the WiFi modul uses
     */
    WIFI(UART_HandleTypeDef *huart);
    /** Destructor */
    virtual ~WIFI();

    /**
     * @brief Getter method for the WiFi SSID.
     * @return Current WiFi SSID stored in this class
     */
    std::string getSSID(void);
    /**
     * @brief Getter method for the WiFi password.
     * @return Current WiFi password stored in this class
     */
    std::string getPassword(void);
    /**
     * @brief Getter method for the WiFi IP.
     * @return Current WiFi IP stored in this class
     */
    std::string getIP(void);

    /**
     * @brief Setter method for the WiFi SSID.
     * @details This method sets the SSID stored in this class and attempts to set this parameter
     *          inside the WiFi module too.
     * @param ssid SSID in character string format
     */
    void setSSID(std::string ssid);
    /**
     * @brief Setter method for the WiFi password.
     * @details This method sets the password stored in this class and attempts to set this
     *          parameter inside the WiFi module too.
     * @param password Password in character string format
     */
    void setPassword(std::string password);
    /**
     * @brief Setter method for the WiFi password.
     * @details This method attempts to set the status message stored in the WiFi module.
     * @param status Status message in character string format
     */
    void setStatusMessage(std::string status);
    /**
     * @brief Attempt to change the coordinate system displayed by the WiFi module.
     * @param csys Type of the coordinate system
     */
    void setCoordinateSystem(CoordinateSystem csys);

    /**
     * @brief Attempt to connect to an existing WiFi network with the WiFi module. The SSID and the
     *          password of the network have to be already set.
     * @return True if the connection was successful otherwise false
     */
    bool connectToNetwork(void);
    /**
     * @brief Attempt to set up a soft access point with the WiFi module.
     */
    void setupAccessPoint(void);

    /**
     * @brief Attempt to reset the WiFi module.
     */
    void resetModule(void);

    /**
     * @brief Receive the next request from the WiFi module.
     * @return Request from the WiFi module
     */
    RequestType receiveNextRequest(void);

    /**
     * @brief Receive and process data coming on the serial communicaton channel.
     * @details This is the method that has to be called in the WIFI's receiving task loop.
     */
    void receive(void);

private:
    RequestType decodeRequest(const char *rqst);

    void serialTransmitMessage(const char *msg, uint16_t size);
    std::string serialReceiveMessage(void);

    void setModuleParam(
            const char *param_name, uint16_t size_name,
            const char *param_value, uint16_t size_value);
    bool checkReply(void);
    void demandConfirm(void);
};

} /* namespace ESP01 */

#endif /* WIFI_H_ */
