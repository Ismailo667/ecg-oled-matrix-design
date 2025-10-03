#ifndef MAX7219_H
#define MAX7219_H

#include "stm32l4xx_hal.h"

#define NUMBER_OF_DIGITS 8
#define MAX7219_SYM_BLANK 0x00
#define MAX7219_CS_Pin GPIO_PIN_15
#define MAX7219_CS_GPIO_Port GPIOA

typedef enum {
    REG_NO_OP           = 0x00,
    REG_DIGIT_0         = 0x01,
    REG_DIGIT_1         = 0x02,
    REG_DIGIT_2         = 0x03,
    REG_DIGIT_3         = 0x04,
    REG_DIGIT_4         = 0x05,
    REG_DIGIT_5         = 0x06,
    REG_DIGIT_6         = 0x07,
    REG_DIGIT_7         = 0x08,
    REG_DECODE_MODE     = 0x09,
    REG_INTENSITY       = 0x0A,
    REG_SCAN_LIMIT      = 0x0B,
    REG_SHUTDOWN        = 0x0C,
    REG_DISPLAY_TEST    = 0x0F,
} MAX7219_REGISTERS;

void max7219_Init(void);
void max7219_SetIntensivity(uint8_t intensivity);
void max7219_Clean(void);
void max7219_SendData(uint8_t addr, uint8_t data);
void max7219_TurnOn(void);
void max7219_TurnOff(void);

#endif // MAX7219_H
