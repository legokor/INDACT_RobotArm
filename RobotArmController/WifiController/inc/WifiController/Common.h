#ifndef WIFICONTROLLER_COMMON_H_
#define WIFICONTROLLER_COMMON_H_

/**
 * @brief WifiController error codes.
 */
typedef enum WC_ErrorCode
{
    WC_ErrorCode_NONE = 0, /**< No error. */
    WC_ErrorCode_COMMUNICATION, /**< Communication error. */
    WC_ErrorCode_CONTROLLER, /**< Controller error. */
    WC_ErrorCode_MEMORY_ALLOCATION, /**< Memory allocation error. */
    WC_ErrorCode_MESSAGE_LENGTH, /**< Message length error. */
    WC_ErrorCode_NOT_INITIALZED, /**< Not initialized error. */
    WC_ErrorCode_PARAMETER, /**< Parameter error. */
    WC_ErrorCode_TIMEOUT, /**< Timeout error. */
    WC_ErrorCode_UNKNOWN /**< Unknown error. */
} WC_ErrorCode_t;

#endif /* WIFICONTROLLER_COMMON_H_ */
