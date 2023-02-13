/**
 *******************************************************************************
 * @file wifi_task.cpp
 *
 * @date 2023. febr. 8.
 * @author Varga Péter
 *******************************************************************************
 * @brief
 * 
 *******************************************************************************
 */

#include "wifi_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include "usart.h"

#include "protocols.h"
#include "SerialStream.h"

// /////////////////////////////////////////////////////////////////////////////
// Privát makrók
// /////////////////////////////////////////////////////////////////////////////

/** @brief Reference of the UART handle used by the WIFI module. */
#define WIFI_UART_HANDLE (&huart3)

// /////////////////////////////////////////////////////////////////////////////
// Using kifejezések
// /////////////////////////////////////////////////////////////////////////////

using namespace ESP01;

// /////////////////////////////////////////////////////////////////////////////
// Privát globális változók
// /////////////////////////////////////////////////////////////////////////////

/** @brief Temporary storage space for the WIFI serial reception functions. */
static char rx_ch = 0;
/** @brief Instance of the WIFI handling class. */
WIFI wifi = WIFI(WIFI_UART_HANDLE);

// /////////////////////////////////////////////////////////////////////////////
// Függvény deklarációk
// /////////////////////////////////////////////////////////////////////////////

/**
 * @brief A function implementing the WIFI's data reception task.
 * @param pvParameters Unused
 */
static void wifi_ReceiveTask(void *pvParameters);

// /////////////////////////////////////////////////////////////////////////////
// Függvény definíciók
// /////////////////////////////////////////////////////////////////////////////

void wifi_StartReceiveTask(void)
{
    xTaskCreate(
            wifi_ReceiveTask,
            "wifi_receive",
            configMINIMAL_STACK_SIZE,
            NULL,
            tskIDLE_PRIORITY + 2,
            NULL);
}

static void wifi_ReceiveTask(void *pvParameters)
{
    // Remove warning about unused parameters
    (void)pvParameters;

    // Start UART receprion by interrupt
    HAL_UART_Receive_IT(wifi.serial->huart, (uint8_t*)(&rx_ch), sizeof(char));

    for (;;)
    {
        // Receive and process data
        wifi.receive();
    }
}

void wifi_UART_RxCpltCallback(void)
{
    // Save the received character to the wifi serial buffer
    wifi.serial->sendCharFromISR(rx_ch);

    // Restart the data reception
    HAL_UART_Receive_IT(wifi.serial->huart, (uint8_t*)(&rx_ch), sizeof(char));
}
