#include "HardwareSupport/serial.h"

#include <stdint.h>

bool HardwareSupport_Serial_Init(HardwareSupport_Serial_t *this, UART_HandleTypeDef *huart,
        size_t rx_buffer_size)
{
    this->huart = huart;

    this->rxBufferSize = rx_buffer_size;
    this->rxBuffer = xStreamBufferCreate(rx_buffer_size, 1);
    if (this->rxBuffer == NULL)
    {
        return false;
    }

    this->txSemaphore = xSemaphoreCreateBinary();
    if (this->txSemaphore == NULL)
    {
        vStreamBufferDelete(this->rxBuffer);
        return false;
    }

    // Enable data transmit
    xSemaphoreGive(this->txSemaphore);
    // Enable data receive
    HAL_UART_Receive_IT(this->huart, (uint8_t*)(&(this->rxChar)), 1);
    return true;
}

void HardwareSupport_Serial_Delete(HardwareSupport_Serial_t *this)
{
    vStreamBufferDelete(this->rxBuffer);
    vSemaphoreDelete(this->txSemaphore);
}

size_t HardwareSupport_Serial_Read(HardwareSupport_Serial_t *this, char *buffer, size_t length)
{
    return xStreamBufferReceive(this->rxBuffer, buffer, length, portMAX_DELAY);
}

void HardwareSupport_Serial_Write(HardwareSupport_Serial_t *this, const char *buffer, size_t length)
{
    // Guard the transmit function with a semaphore to prevent concurrent access
    xSemaphoreTake(this->txSemaphore, portMAX_DELAY);
    HAL_UART_Transmit(this->huart, (const uint8_t*)buffer, length, 500);
    xSemaphoreGive(this->txSemaphore);
}

size_t HardwareSupport_Serial_GetAvailable(HardwareSupport_Serial_t *this)
{
    return xStreamBufferBytesAvailable(this->rxBuffer);
}

void HardwareSupport_Serial_UartRxCallback(HardwareSupport_Serial_t *this)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xStreamBufferSendFromISR(this->rxBuffer, (uint8_t*)(&(this->rxChar)), 1, &xHigherPriorityTaskWoken);
    HAL_UART_Receive_IT(this->huart, (uint8_t*)(&(this->rxChar)), 1);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
