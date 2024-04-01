#include "lcd.h"

#include "main.h"

Lcd16x2_t hlcd1;

void initLcd()
{
    hlcd1.pinout.rs.port = LCD_D_C_GPIO_Port;
    hlcd1.pinout.rs.pin = LCD_D_C_Pin;
    hlcd1.pinout.rw.port = LCD_R_W_GPIO_Port;
    hlcd1.pinout.rw.pin = LCD_R_W_Pin;
    hlcd1.pinout.en.port = LCD_EN_GPIO_Port;
    hlcd1.pinout.en.pin = LCD_EN_Pin;
    hlcd1.pinout.d[0].port = LCD_D0_GPIO_Port;
    hlcd1.pinout.d[0].pin = LCD_D0_Pin;
    hlcd1.pinout.d[1].port = LCD_D1_GPIO_Port;
    hlcd1.pinout.d[1].pin = LCD_D1_Pin;
    hlcd1.pinout.d[2].port = LCD_D2_GPIO_Port;
    hlcd1.pinout.d[2].pin = LCD_D2_Pin;
    hlcd1.pinout.d[3].port = LCD_D3_GPIO_Port;
    hlcd1.pinout.d[3].pin = LCD_D3_Pin;
    hlcd1.pinout.d[4].port = LCD_D4_GPIO_Port;
    hlcd1.pinout.d[4].pin = LCD_D4_Pin;
    hlcd1.pinout.d[5].port = LCD_D5_GPIO_Port;
    hlcd1.pinout.d[5].pin = LCD_D5_Pin;
    hlcd1.pinout.d[6].port = LCD_D6_GPIO_Port;
    hlcd1.pinout.d[6].pin = LCD_D6_Pin;
    hlcd1.pinout.d[7].port = LCD_D7_GPIO_Port;
    hlcd1.pinout.d[7].pin = LCD_D7_Pin;

    hlcd1.entry_mode_set = CURSOR_INCREMENT;

    hlcd1.display_control = DISPLAY_ON | CURSOR_ON;

    Lcd16x2_Init(&hlcd1);
}
