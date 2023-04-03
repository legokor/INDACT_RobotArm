/**
 ***************************************************************************************************
 * @file wifi_task.c
 *
 * @date 2023. 02. 08.
 * @author Péter Varga
 ***************************************************************************************************
 * @brief Implementation of wifi_task.h.
 ***************************************************************************************************
 */

#include "wifi_task.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include "WIFI_Config.h"
#include "WIFI.h"
#include "SerialStream.h"

#include "usart.h"

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Global variables
// /////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief Temporary storage space for the WIFI serial reception functions. */
volatile char rx_ch = 0;

// /////////////////////////////////////////////////////////////////////////////////////////////////
// Function definitions
// /////////////////////////////////////////////////////////////////////////////////////////////////

void WIFI_InitRTOS(void)
{
    // Initialize the WIFI parameters, create the RTOS structures
    WIFI_InitWIFI(WIFI_UART_HANDLE);

    // Create the receiving task
    xTaskCreate(WIFI_StartReceiveTask,
            "WIFI_Receive",
            128 * 4,
            NULL,
            2,
            NULL);
}

void WIFI_StartReceiveTask(void *argument)
{
    // Start the data reception through UART
    HAL_UART_Receive_IT(WIFI_UART_HANDLE, (uint8_t*)(&rx_ch), sizeof(char));

    for (;;)
    {
        // Receive and process data
        WIFI_Receive();
    }
}

void WIFI_UART_RxCpltCallback(void)
{
    // Save the received character to the wifi serial buffer
    SerialStream_SaveCharFromISR(rx_ch);

    // Restart the data reception
    HAL_UART_Receive_IT(WIFI_UART_HANDLE, (uint8_t*)(&rx_ch), sizeof(char));
}

void WIFI_StartDefaultSetupTask(void *argument)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Wait for WIFI initialization
    while (WIFI_GetInitState() != 1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (WIFI_SynchronizeModule() != WIFI_CONFIRM)
    {
        Error_Handler();
    }

    if (WIFI_SetupAccessPoint() != WIFI_CONFIRM)
    {
        Error_Handler();
    }

    // Delete the task when the setup is complete
    vTaskDelete(NULL);
}
