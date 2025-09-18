#include "lcd.h"
#include "stm32f1xx_hal.h"

/* Private functions */
static void LCD_Enable(void);
static void LCD_Send4Bits(uint8_t data);
static void LCD_Cmd(uint8_t cmd);

void LCD_Init(void)
{
    HAL_Delay(50); // wait for LCD power up

    // Init sequence for 4-bit mode
    LCD_Send4Bits(0x03);
    HAL_Delay(5);
    LCD_Send4Bits(0x03);
    HAL_Delay(5);
    LCD_Send4Bits(0x03);
    HAL_Delay(1);
    LCD_Send4Bits(0x02); // set 4-bit mode

    LCD_Cmd(0x28); // 2 lines, 5x8 font
    LCD_Cmd(0x0C); // display ON, cursor OFF
    LCD_Cmd(0x06); // auto increment cursor
    LCD_Clear();
}

void LCD_Clear(void)
{
    LCD_Cmd(0x01); // clear display
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t address = (row == 0) ? 0x80 : 0xC0;
    address += col;
    LCD_Cmd(address);
}

void LCD_Print(const char *str)
{
    while (*str) {
        LCD_WriteChar(*str++);
    }
}

void LCD_WriteChar(char c)
{
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_SET);
    LCD_Send4Bits(c >> 4);
    LCD_Send4Bits(c & 0x0F);
}

static void LCD_Cmd(uint8_t cmd)
{
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    LCD_Send4Bits(cmd >> 4);
    LCD_Send4Bits(cmd & 0x0F);
}

static void LCD_Send4Bits(uint8_t data)
{
    HAL_GPIO_WritePin(LCD_D4_Port, LCD_D4_Pin, (data >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_D5_Port, LCD_D5_Pin, (data >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_D6_Port, LCD_D6_Pin, (data >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_D7_Port, LCD_D7_Pin, (data >> 3) & 0x01);

    LCD_Enable();
}

static void LCD_Enable(void)
{
    HAL_GPIO_WritePin(LCD_EN_Port, LCD_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_EN_Port, LCD_EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}
