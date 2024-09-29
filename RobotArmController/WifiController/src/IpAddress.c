#include "WifiController/IpAddress.h"

#include <stdio.h>
#include <string.h>

static uint32_t get_address(WifiController_IpAddress_t *this);
static uint32_t get_octet(WifiController_IpAddress_t *this, WifiController_IpAddress_Octet_t octet);
static bool set_address(WifiController_IpAddress_t *this, uint32_t value);
static bool set_octet(WifiController_IpAddress_t *this, WifiController_IpAddress_Octet_t octet, uint32_t value);
static int to_string(WifiController_IpAddress_t *this, char *buffer, int max_length);
static bool from_string(WifiController_IpAddress_t *this, const char *str);

bool WifiController_IpAddress_Init(WifiController_IpAddress_t *this)
{
    this->address = 0x00;
    this->mutex = xSemaphoreCreateMutex();
    if (this->mutex == NULL)
    {
        return false;
    }
    return true;
}

void WifiController_IpAddress_Delete(WifiController_IpAddress_t *this)
{
    vSemaphoreDelete(this->mutex);
}

uint32_t WifiController_IpAddress_GetAddress(WifiController_IpAddress_t *this)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return 0x00;
    }
    uint32_t rv = get_address(this);
    xSemaphoreGive(this->mutex);
    return rv;
}

uint32_t WifiController_IpAddress_GetOctet(WifiController_IpAddress_t *this, WifiController_IpAddress_Octet_t octet)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return 0x00000100;
    }
    uint32_t rv = get_octet(this, octet);
    xSemaphoreGive(this->mutex);
    return rv;
}

bool WifiController_IpAddress_SetAddress(WifiController_IpAddress_t *this, uint32_t value)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }
    bool rv = set_address(this, value);
    xSemaphoreGive(this->mutex);
    return rv;
}

bool WifiController_IpAddress_SetOctet(WifiController_IpAddress_t *this, WifiController_IpAddress_Octet_t octet, uint32_t value)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }
    bool rv = set_octet(this, octet, value);
    xSemaphoreGive(this->mutex);
    return rv;
}

int WifiController_IpAddress_ToString(WifiController_IpAddress_t *this, char *buffer, int max_length)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return -1;
    }
    int rv = to_string(this, buffer, max_length);
    xSemaphoreGive(this->mutex);
    return rv;
}

bool WifiController_IpAddress_FromString(WifiController_IpAddress_t *this, const char *str)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }
    bool rv = from_string(this, str);
    xSemaphoreGive(this->mutex);
    return rv;
}

static uint32_t get_address(WifiController_IpAddress_t *this)
{
    return this->address;
}

static uint32_t get_octet(WifiController_IpAddress_t *this, WifiController_IpAddress_Octet_t octet)
{
    switch (octet)
    {
    case WifiController_IpAddress_Octet_1:
        return ((this->address) >> 24) & 0xFF;
    case WifiController_IpAddress_Octet_2:
        return ((this->address) >> 16) & 0xFF;
    case WifiController_IpAddress_Octet_3:
        return ((this->address) >> 8) & 0xFF;
    case WifiController_IpAddress_Octet_4:
        return (this->address) & 0xFF;
    default:
        return 0x100;
    }
}

static bool set_address(WifiController_IpAddress_t *this, uint32_t value)
{
    this->address = value;
    return true;
}

static bool set_octet(WifiController_IpAddress_t *this, WifiController_IpAddress_Octet_t octet, uint32_t value)
{
    if (value > 0xFF)
    {
        return false;
    }

    switch (octet)
    {
    case WifiController_IpAddress_Octet_1:
        this->address = (this->address & (~(0x000000FF << 24))) | (value << 24);
        break;
    case WifiController_IpAddress_Octet_2:
        this->address = (this->address & (~(0x000000FF << 16))) | (value << 16);
        break;
    case WifiController_IpAddress_Octet_3:
        this->address = (this->address & (~(0x000000FF << 8))) | (value << 8);
        break;
    case WifiController_IpAddress_Octet_4:
        this->address = (this->address & (~(0x000000FF))) | (value);
        break;
    default:
        return false;
    }
    return true;
}

static int to_string(WifiController_IpAddress_t *this, char *buffer, int max_length)
{
    if (max_length < 15)
    {
        return -1;
    }
    return sprintf(buffer, "%lu.%lu.%lu.%lu", get_octet(this, WifiController_IpAddress_Octet_1),
            get_octet(this, WifiController_IpAddress_Octet_2),
            get_octet(this, WifiController_IpAddress_Octet_3),
            get_octet(this, WifiController_IpAddress_Octet_4));
}

static bool from_string(WifiController_IpAddress_t *this, const char *str)
{
    uint32_t octets[4];
    if (sscanf(str, "%lu.%lu.%lu.%lu", &(octets[0]), &(octets[1]), &(octets[2]), &(octets[3])) != 4)
    {
        return false;
    }
    this->address = (octets[0] << 24) | (octets[1] << 16) | (octets[2] << 8) | octets[3];
    return true;
}
