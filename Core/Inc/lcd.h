#ifndef __LCD_H__
#define __LCD_H__

#include "stm32f1xx_hal.h"

/* LCD pin mapping (4-bit mode) */
#define LCD_RS_Pin   GPIO_PIN_0
#define LCD_RS_Port  GPIOB
#define LCD_EN_Pin   GPIO_PIN_1
#define LCD_EN_Port  GPIOB

#define LCD_D4_Pin   GPIO_PIN_2
#define LCD_D4_Port  GPIOB
#define LCD_D5_Pin   GPIO_PIN_3
#define LCD_D5_Port  GPIOB
#define LCD_D6_Pin   GPIO_PIN_4
#define LCD_D6_Port  GPIOB
#define LCD_D7_Pin   GPIO_PIN_5
#define LCD_D7_Port  GPIOB

/* Public API */
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *str);
void LCD_WriteChar(char c);

#endif /* __LCD_H__ */
