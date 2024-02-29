#include "stm32f7xx_hal.h"

#include "main.h"

void initWifiModule()
{
    // Set reset and enable pins for operating mode
    HAL_GPIO_WritePin(Wifi_RST_GPIO_Port, Wifi_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Wifi_CH_PD_GPIO_Port, Wifi_CH_PD_Pin, GPIO_PIN_SET);
}
