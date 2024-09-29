#ifndef WIFICONTROLLER_IPADDRESS_H_
#define WIFICONTROLLER_IPADDRESS_H_

#include <FreeRTOS.h>
#include <semphr.h>

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Structure to hold an IP address and provide thread-safe access to it.
 * 
 * @note The fields of this structure should not be accessed directly. Use the provided functions instead.
 */
typedef struct WifiController_IpAddress
{
    uint32_t address; /**< The IP address */
    SemaphoreHandle_t mutex; /**< Mutex to protect the IP address */
} WifiController_IpAddress_t;

/**
 * @brief Enumeration of the octets of an IP address.
 */
typedef enum WifiController_IpAddress_Octet
{
    WifiController_IpAddress_Octet_1, /**< The first octet */
    WifiController_IpAddress_Octet_2, /**< The second octet */
    WifiController_IpAddress_Octet_3, /**< The third octet */
    WifiController_IpAddress_Octet_4 /**< The fourth octet */
} WifiController_IpAddress_Octet_t;

/**
 * @brief Initializes the IP address.
 * 
 * @param this The IP address struct.
 * @return True if the IP address was initialized successfully, false otherwise.
 * 
 * @note This function must be called before any other function of the IP address.
 * @note The memory for the WifiController_IpAddress_t struct must be allocated before calling this function.
 */
bool WifiController_IpAddress_Init(WifiController_IpAddress_t *this);

/**
 * @brief Deinitialize the IP address.
 * 
 * @param this The IP address struct.
 * 
 * @note This function must be called when the IP address is no longer needed.
 */
void WifiController_IpAddress_Delete(WifiController_IpAddress_t *this);

/**
 * @brief Get the IP address.
 * 
 * @param this The IP address struct.
 * @return The IP address as a 32-bit unsigned integer.
 */
uint32_t WifiController_IpAddress_GetAddress(WifiController_IpAddress_t *this);

/**
 * @brief Get an octet of the IP address.
 * 
 * @param this The IP address struct.
 * @param octet The octet to get.
 * @return The octet as a 32-bit unsigned integer.
 */
uint32_t WifiController_IpAddress_GetOctet(WifiController_IpAddress_t *this, WifiController_IpAddress_Octet_t octet);

/**
 * @brief Set the IP address.
 * 
 * @param this The IP address struct.
 * @param value The IP address as a 32-bit unsigned integer.
 * @return True if the IP address was set successfully, false otherwise.
 */
bool WifiController_IpAddress_SetAddress(WifiController_IpAddress_t *this, uint32_t value);

/**
 * @brief Set an octet of the IP address.
 * 
 * @param this The IP address struct.
 * @param octet The octet to set.
 * @param value The octet as a 32-bit unsigned integer.
 * @return True if the octet was set successfully, false otherwise.
 */
bool WifiController_IpAddress_SetOctet(WifiController_IpAddress_t *this, WifiController_IpAddress_Octet_t octet, uint32_t value);

/**
 * @brief Converts the IP address to a string.
 * 
 * @param this The IP address struct.
 * @param buffer The buffer to store the string in.
 * @param max_length The maximum length of the buffer.
 * @return The number of characters written to the buffer.
 */
int WifiController_IpAddress_ToString(WifiController_IpAddress_t *this, char *buffer, int max_length);

/**
 * @brief Converts a string to an IP address.
 * 
 * @param this The IP address struct.
 * @param str The string to convert.
 * @return True if the string was converted successfully, false otherwise.
 */
bool WifiController_IpAddress_FromString(WifiController_IpAddress_t *this, const char *str);

#endif /* WIFICONTROLLER_IPADDRESS_H_ */
