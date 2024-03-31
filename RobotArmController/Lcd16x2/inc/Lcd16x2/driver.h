#ifndef LCD16X2_DRIVER_H_
#define LCD16X2_DRIVER_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "stm32f7xx_hal.h"

/* Entry mode set flags { */
#define CURSOR_INCREMENT 0x02
#define DISPLAY_SHIFT_LEFT 0x03
#define DISPLAY_SHIFT_RIGHT 0x01
/* } Entry mode set flags */

/* Display control flags { */
#define DISPLAY_ON 0x04
#define CURSOR_ON 0x02
#define BLINK_ON 0x01
/* } Display control flags */

/**
 * @brief Structure representing a GPIO pin for the LCD 16x2 module.
 */
typedef struct Lcd16x2_GpioPin
{
    GPIO_TypeDef *port; /**< GPIO port for the pin */
    uint16_t pin;       /**< GPIO pin number */
} Lcd16x2_GpioPin_t;

/**
 * @brief Structure representing the pinout configuration for the LCD 16x2 module.
 */
typedef struct Lcd16x2_Pinout
{
    Lcd16x2_GpioPin_t rs;   /**< Register select (RS) pin */
    Lcd16x2_GpioPin_t rw;   /**< Read/Write (RW) pin */
    Lcd16x2_GpioPin_t en;   /**< Enable (EN) pin */
    Lcd16x2_GpioPin_t d[8]; /**< Data pins (D0-D7) */
} Lcd16x2_Pinout_t;

/**
 * @brief Structure representing the LCD 16x2 module.
 */
typedef struct Lcd16x2
{
    Lcd16x2_Pinout_t pinout;    /**< Pinout configuration */
    uint8_t display_control;    /**< Display control settings */
    uint8_t entry_mode_set;     /**< Entry mode settings */
} Lcd16x2_t;

/**
 * @brief Initializes the LCD module.
 * 
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD module.
 */
void Lcd16x2_Init(Lcd16x2_t *lcd);

/**
 * @brief Writes a character to the LCD module.
 * 
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD module.
 * @param ch The character to be written.
 */
void Lcd16x2_WriteChar(Lcd16x2_t *lcd, char ch);

/**
 * @brief Writes a string to the LCD module.
 * 
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD module.
 * @param str The string to be written.
 * @return The number of characters written.
 */
size_t Lcd16x2_WriteString(Lcd16x2_t *lcd, const char *str);

/**
 * @brief Writes an integer to the LCD module.
 * 
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD module.
 * @param num The integer to be written.
 * @return The number of characters written.
 */
size_t Lcd16x2_WriteInteger(Lcd16x2_t *lcd, int num);

/**
 * @brief Writes a floating-point number to the LCD module.
 * 
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD module.
 * @param num The floating-point number to be written.
 * @param decimal_points The number of decimal points to be displayed.
 * @return The number of characters written.
 */
size_t Lcd16x2_WriteFloat(Lcd16x2_t *lcd, double num, int decimal_points);

/**
 * Prints formatted text on a 16x2 LCD display.
 *
 * @param lcd The pointer to the Lcd16x2_t structure representing the LCD display.
 * @param format The format string specifying the text to be printed.
 * @param ... Additional arguments to be formatted and printed.
 * @return The number of characters printed, or a negative value if an error occurred.
 */
int Lcd16x2_Printf(Lcd16x2_t *lcd, const char *format, ...);

/**
 * @brief Clears the display of the LCD.
 *
 * This function clears the entire display of the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_ClearDisplay(Lcd16x2_t *lcd);

/**
 * @brief Returns the cursor to the home position.
 *
 * This function moves the cursor to the home position, which is the first position of the first row.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_ReturnHome(Lcd16x2_t *lcd);

/**
 * @brief Sets the cursor position on the LCD.
 *
 * This function sets the cursor position on the LCD to the specified row and column.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 * @param row The row number (0-based index) where the cursor should be set.
 * @param col The column number (0-based index) where the cursor should be set.
 * @return true if the cursor position was set successfully, false otherwise.
 */
bool Lcd16x2_SetCursor(Lcd16x2_t *lcd, size_t row, size_t col);

/**
 * @brief Turns on the display of the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_SetDisplayOn(Lcd16x2_t *lcd);

/**
 * @brief Turns off the display of the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_SetDisplayOff(Lcd16x2_t *lcd);

/**
 * @brief Turns on the cursor of the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_SetCursorOn(Lcd16x2_t *lcd);

/**
 * @brief Turns off the cursor of the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_SetCursorOff(Lcd16x2_t *lcd);

/**
 * @brief Turns on the blinking cursor on the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_SetBlinkOn(Lcd16x2_t *lcd);

/**
 * @brief Turns off the blinking cursor on the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_SetBlinkOff(Lcd16x2_t *lcd);

/**
 * @brief Shifts the cursor one position to the left on the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_ShiftCursorLeft(Lcd16x2_t *lcd);

/**
 * @brief Shifts the cursor one position to the right on the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_ShiftCursorRight(Lcd16x2_t *lcd);

/**
 * @brief Shifts the entire display one position to the left on the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_ShiftDisplayLeft(Lcd16x2_t *lcd);

/**
 * @brief Shifts the entire display one position to the right on the LCD.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD.
 */
void Lcd16x2_ShiftDisplayRight(Lcd16x2_t *lcd);

/**
 * @brief Checks if the LCD module is busy.
 *
 * This function checks if the LCD module is busy processing a command or data.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD module.
 * @return true if the LCD module is busy, false otherwise.
 */
bool Lcd16x2_IsBusy(Lcd16x2_t *lcd);

/**
 * @brief Reads the address counter of the LCD module.
 *
 * This function reads the current address counter value of the LCD module.
 *
 * @param lcd Pointer to the Lcd16x2_t structure representing the LCD module.
 * @return The address counter value of the LCD module.
 */
uint8_t Lcd16x2_ReadAddressCounter(Lcd16x2_t *lcd);

#endif /* LCD16X2_DRIVER_H_ */
