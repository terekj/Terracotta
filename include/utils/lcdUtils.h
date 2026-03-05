#pragma once
#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "utils/spiAVR.h"

#define SWRESET 0x01
#define SLPOUT 0x11
#define COLMOD 0x3A
#define DISPON 0x29
#define DISPOFF 0x28
#define CASET 0x2A
#define RASET 0x2B
#define RAMWR 0x2C
#define MADCTL 0x36
#define RGB_ORDER 0x08

// pin macros here
#define LCD_CS_PIN     PD7
#define LCD_DC_PIN     PD6
#define LCD_RST_PIN    PB0

#define LCD_CS_PORT    PORTD
#define LCD_DC_PORT    PORTD
#define LCD_RST_PORT   PORTB

#define LCD_CS_DDR     DDRD
#define LCD_DC_DDR     DDRD
#define LCD_RST_DDR    DDRB

#define CS_LOW()    (LCD_CS_PORT &= ~(1 << LCD_CS_PIN))
#define CS_HIGH()   (LCD_CS_PORT |=  (1 << LCD_CS_PIN))

#define DC_LOW()    (LCD_DC_PORT &= ~(1 << LCD_DC_PIN))
#define DC_HIGH()   (LCD_DC_PORT |=  (1 << LCD_DC_PIN))

#define RST_LOW()   (LCD_RST_PORT &= ~(1 << LCD_RST_PIN))
#define RST_HIGH()  (LCD_RST_PORT |=  (1 << LCD_RST_PIN))


#define COLOR_WHITE 0xFFFF
#define COLOR_BLACK 0x0000

// DECLARATIONS ONLY — NO BODIES
void lcd_init();
void Send_Command(uint8_t cmd);
void Send_Data(uint8_t data);
void HardwareReset();
void SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ST7735_init();
void fillColor(uint16_t color);
void fillRect(uint16_t color, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);