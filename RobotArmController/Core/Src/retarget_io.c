#include "retarget_io.h"

#include <stdio.h>

#include "usart.h"

#define USB_HUART (&huart3)

int __io_putchar(int ch)
{
    uint8_t data = (uint8_t)ch;
    HAL_StatusTypeDef status = HAL_UART_Transmit(USB_HUART, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return EOF;
    }
    return data;
}

//int __io_getchar(void)
//{
//    uint8_t data = 0;
//    HAL_StatusTypeDef status = HAL_UART_Receive(USB_HUART, &data, 1, HAL_MAX_DELAY);
//    if (status != HAL_OK)
//    {
//        return EOF;
//    }
//    return data;
//}

void initRetargetIo(void)
{
    // Turn off buffers, so I/O occurs immediately
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}
