#include "Lcd16x2/driver.h"

/* Instructions { */
#define CLEAR_DISPLAY 0x01
#define RETURN_HOME 0x02
#define ENTRY_MODE_SET 0x04
#define DISPLAY_CONTROL 0x08
#define DISPLAY_CURSOR_SHIFT 0x10
#define FUNCTION_SET 0x20
#define SET_CGRAM_ADDRESS 0x40
#define SET_DDRAM_ADDRESS 0x80
/* } Instructions */

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

/* Function set flags { */
#define DATA_LENGTH_8_BITS 0x10
#define DISPLAY_LINES_2 0x08
#define FONT_5X11 0x04
/* } Function set flags */

/* State masks { */
#define BUSY_FLAG_MASK 0x80
#define ADDRESS_COUNTER_MASK 0x7F
/* } State masks */

static void write_data(Lcd16x2_t *lcd, uint8_t data);
static void write_instruction(Lcd16x2_t *lcd, uint8_t instruction);
static uint8_t read_data(Lcd16x2_t *lcd);
static uint8_t read_state(Lcd16x2_t *lcd);

void Lcd16x2_Init(Lcd16x2_t *lcd)
{
    /* Initialization sequence based on the ST7066 controller datasheet (page 19). */
    // Wait at least 15ms after power-on.
    HAL_Delay(15);
    // Set 8-bit mode.
    write_instruction(lcd, FUNCTION_SET | 0x30);
    // Wait at least 4.1ms.
    HAL_Delay(5);
    // Set 8-bit mode again.
    write_instruction(lcd, FUNCTION_SET | 0x30);
    // Wait at least 100us.
    HAL_Delay(1);
    // Set 8-bit mode one more time.
    write_instruction(lcd, FUNCTION_SET | 0x30);
    
    // Function settings: 8-bit mode, 2 display lines, 5x8 font.
    HAL_Delay(1);
    write_instruction(lcd, FUNCTION_SET | DATA_LENGTH_8_BITS | DISPLAY_LINES_2);
    
    HAL_Delay(1);
    write_instruction(lcd, DISPLAY_CONTROL | lcd->display_control);
    
    HAL_Delay(1);
    write_instruction(lcd, CLEAR_DISPLAY);
    
    HAL_Delay(1);
    write_instruction(lcd, ENTRY_MODE_SET | lcd->entry_mode_set);
}

void Lcd16x2_WriteChar(Lcd16x2_t *lcd, char ch)
{
    write_data(lcd, ch);
}

size_t Lcd16x2_WriteString(Lcd16x2_t *lcd, const char *str)
{
    size_t len = 0;
    while (*str && len < 32)
    {
        write_data(lcd, *str++);
        len++;
    }
    return len;
}

size_t Lcd16x2_WriteInteger(Lcd16x2_t *lcd, int num)
{
    char buffer[12];
    sprintf(buffer, "%d", num);
    return Lcd16x2_WriteString(lcd, buffer);
}

size_t Lcd16x2_WriteFloat(Lcd16x2_t *lcd, double num, int decimal_points)
{
    char buffer[32 + 1];
    snprintf(buffer, sizeof(buffer), "%.*f", decimal_points, num);
    return Lcd16x2_WriteString(lcd, buffer);
}

int Lcd16x2_Printf(Lcd16x2_t *lcd, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    char buffer[32 + 1];
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len >= 0)
    {
        for (int i = 0; i < len; i++)
        {
            write_data(lcd, buffer[i]);
        }
    }
    
    return len;
}

void Lcd16x2_ClearDisplay(Lcd16x2_t *lcd)
{
    write_instruction(lcd, CLEAR_DISPLAY);
}

void Lcd16x2_ReturnHome(Lcd16x2_t *lcd)
{
    write_instruction(lcd, RETURN_HOME);
}

bool Lcd16x2_SetCursor(Lcd16x2_t *lcd, size_t row, size_t col)
{
    if (row > 1 || col > 15)
    {
        return false;
    }
    
    uint8_t address = col + (row * 0x40);
    write_instruction(lcd, SET_DDRAM_ADDRESS | address);
    return true;
}

void Lcd16x2_SetDisplayOn(Lcd16x2_t *lcd)
{
    lcd->display_control |= DISPLAY_ON;
    write_instruction(lcd, DISPLAY_CONTROL | lcd->display_control);
}

void Lcd16x2_SetDisplayOff(Lcd16x2_t *lcd)
{
    lcd->display_control &= ~DISPLAY_ON;
    write_instruction(lcd, DISPLAY_CONTROL | lcd->display_control);
}

void Lcd16x2_SetCursorOn(Lcd16x2_t *lcd)
{
    lcd->display_control |= CURSOR_ON;
    write_instruction(lcd, DISPLAY_CONTROL | lcd->display_control);
}

void Lcd16x2_SetCursorOff(Lcd16x2_t *lcd)
{
    lcd->display_control &= ~CURSOR_ON;
    write_instruction(lcd, DISPLAY_CONTROL | lcd->display_control);
}

void Lcd16x2_SetBlinkOn(Lcd16x2_t *lcd)
{
    lcd->display_control |= BLINK_ON;
    write_instruction(lcd, DISPLAY_CONTROL | lcd->display_control);
}

void Lcd16x2_SetBlinkOff(Lcd16x2_t *lcd)
{
    lcd->display_control &= ~BLINK_ON;
    write_instruction(lcd, DISPLAY_CONTROL | lcd->display_control);
}

void Lcd16x2_ShiftCursorLeft(Lcd16x2_t *lcd)
{
    write_instruction(lcd, DISPLAY_CURSOR_SHIFT | 0x00);
}

void Lcd16x2_ShiftCursorRight(Lcd16x2_t *lcd)
{
    write_instruction(lcd, DISPLAY_CURSOR_SHIFT | 0x04);
}

void Lcd16x2_ShiftDisplayLeft(Lcd16x2_t *lcd)
{
    write_instruction(lcd, DISPLAY_CURSOR_SHIFT | 0x08);
}

void Lcd16x2_ShiftDisplayRight(Lcd16x2_t *lcd)
{
    write_instruction(lcd, DISPLAY_CURSOR_SHIFT | 0x0C);
}

bool Lcd16x2_IsBusy(Lcd16x2_t *lcd)
{
    return read_state(lcd) & BUSY_FLAG_MASK;
}

uint8_t Lcd16x2_ReadAddressCounter(Lcd16x2_t *lcd)
{
    return read_state(lcd) & ADDRESS_COUNTER_MASK;
}

static void write_data(Lcd16x2_t *lcd, uint8_t data)
{
    HAL_GPIO_WritePin(lcd->pinout.rs.port, lcd->pinout.rs.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(lcd->pinout.rw.port, lcd->pinout.rw.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.en.port, lcd->pinout.en.pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(lcd->pinout.d[0].port, lcd->pinout.d[0].pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[1].port, lcd->pinout.d[1].pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[2].port, lcd->pinout.d[2].pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[3].port, lcd->pinout.d[3].pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[4].port, lcd->pinout.d[4].pin, (data & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[5].port, lcd->pinout.d[5].pin, (data & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[6].port, lcd->pinout.d[6].pin, (data & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[7].port, lcd->pinout.d[7].pin, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(lcd->pinout.en.port, lcd->pinout.en.pin, GPIO_PIN_RESET);
}

static void write_instruction(Lcd16x2_t *lcd, uint8_t instruction)
{
    HAL_GPIO_WritePin(lcd->pinout.rs.port, lcd->pinout.rs.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.rw.port, lcd->pinout.rw.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.en.port, lcd->pinout.en.pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(lcd->pinout.d[0].port, lcd->pinout.d[0].pin, (instruction & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[1].port, lcd->pinout.d[1].pin, (instruction & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[2].port, lcd->pinout.d[2].pin, (instruction & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[3].port, lcd->pinout.d[3].pin, (instruction & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[4].port, lcd->pinout.d[4].pin, (instruction & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[5].port, lcd->pinout.d[5].pin, (instruction & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[6].port, lcd->pinout.d[6].pin, (instruction & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.d[7].port, lcd->pinout.d[7].pin, (instruction & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(lcd->pinout.en.port, lcd->pinout.en.pin, GPIO_PIN_RESET);
}

static uint8_t read_data(Lcd16x2_t *lcd)
{
    HAL_GPIO_WritePin(lcd->pinout.rs.port, lcd->pinout.rs.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(lcd->pinout.rw.port, lcd->pinout.rw.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(lcd->pinout.en.port, lcd->pinout.en.pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(lcd->pinout.en.port, lcd->pinout.en.pin, GPIO_PIN_RESET);

    uint8_t data = 0;
    data |= HAL_GPIO_ReadPin(lcd->pinout.d[0].port, lcd->pinout.d[0].pin) ? 0x01 : 0x00;
    data |= HAL_GPIO_ReadPin(lcd->pinout.d[1].port, lcd->pinout.d[1].pin) ? 0x02 : 0x00;
    data |= HAL_GPIO_ReadPin(lcd->pinout.d[2].port, lcd->pinout.d[2].pin) ? 0x04 : 0x00;
    data |= HAL_GPIO_ReadPin(lcd->pinout.d[3].port, lcd->pinout.d[3].pin) ? 0x08 : 0x00;
    data |= HAL_GPIO_ReadPin(lcd->pinout.d[4].port, lcd->pinout.d[4].pin) ? 0x10 : 0x00;
    data |= HAL_GPIO_ReadPin(lcd->pinout.d[5].port, lcd->pinout.d[5].pin) ? 0x20 : 0x00;
    data |= HAL_GPIO_ReadPin(lcd->pinout.d[6].port, lcd->pinout.d[6].pin) ? 0x40 : 0x00;
    data |= HAL_GPIO_ReadPin(lcd->pinout.d[7].port, lcd->pinout.d[7].pin) ? 0x80 : 0x00;

    return data;
}

static uint8_t read_state(Lcd16x2_t *lcd)
{
    HAL_GPIO_WritePin(lcd->pinout.rs.port, lcd->pinout.rs.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lcd->pinout.rw.port, lcd->pinout.rw.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(lcd->pinout.en.port, lcd->pinout.en.pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(lcd->pinout.en.port, lcd->pinout.en.pin, GPIO_PIN_RESET);

    uint8_t state = 0;
    state |= HAL_GPIO_ReadPin(lcd->pinout.d[0].port, lcd->pinout.d[0].pin) ? 0x01 : 0x00;
    state |= HAL_GPIO_ReadPin(lcd->pinout.d[1].port, lcd->pinout.d[1].pin) ? 0x02 : 0x00;
    state |= HAL_GPIO_ReadPin(lcd->pinout.d[2].port, lcd->pinout.d[2].pin) ? 0x04 : 0x00;
    state |= HAL_GPIO_ReadPin(lcd->pinout.d[3].port, lcd->pinout.d[3].pin) ? 0x08 : 0x00;
    state |= HAL_GPIO_ReadPin(lcd->pinout.d[4].port, lcd->pinout.d[4].pin) ? 0x10 : 0x00;
    state |= HAL_GPIO_ReadPin(lcd->pinout.d[5].port, lcd->pinout.d[5].pin) ? 0x20 : 0x00;
    state |= HAL_GPIO_ReadPin(lcd->pinout.d[6].port, lcd->pinout.d[6].pin) ? 0x40 : 0x00;
    state |= HAL_GPIO_ReadPin(lcd->pinout.d[7].port, lcd->pinout.d[7].pin) ? 0x80 : 0x00;

    return state;
}
