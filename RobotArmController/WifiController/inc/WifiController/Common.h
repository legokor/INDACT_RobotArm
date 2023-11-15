#ifndef WIFICONTROLLER_COMMON_H_
#define WIFICONTROLLER_COMMON_H_

/**
 * @brief WifiController error codes.
 */
typedef enum WifiController_ErrorCode
{
    WifiController_ErrorCode_NONE = 0, /**< No error. */
    WifiController_ErrorCode_COMMUNICATION, /**< Communication error. */
    WifiController_ErrorCode_CONTROLLER, /**< Controller error. */
    WifiController_ErrorCode_MEMORY_ALLOCATION, /**< Memory allocation error. */
    WifiController_ErrorCode_MESSAGE_LENGTH, /**< Message length error. */
    WifiController_ErrorCode_NOT_INITIALZED, /**< Not initialized error. */
    WifiController_ErrorCode_PARAMETER, /**< Parameter error. */
    WifiController_ErrorCode_TIMEOUT, /**< Timeout error. */
    WifiController_ErrorCode_UNKNOWN /**< Unknown error. */
} WifiController_ErrorCode_t;

#endif /* WIFICONTROLLER_COMMON_H_ */
