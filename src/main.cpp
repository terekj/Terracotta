#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "spiAVR.h"

// TFT LCD PINS L->R (screen facing)
// VCC GND CS RESET A0 SDA SCK LED
// 5V  GND PB2 PB0  PB1 PB3 PB5 3.3V
#define PIN_SCK                   PORTB5//SHOULD ALWAYS BE B5 ON THE ARDUINO
#define PIN_MOSI                  PORTB3//SHOULD ALWAYS BE B3 ON THE ARDUINO
#define PIN_SS                    PORTB2
#define PIN_A0					  PORTB1
#define PIN_RESET				  PORTB0

// COMMANDS
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

// PIN CONTROLLERS
#define CS_LOW() PORTB &= ~(0x01 << PIN_SS)
#define CS_HIGH() PORTB |= (0x01 << PIN_SS)

#define DC_LOW() PORTB &= ~(0x01 << PIN_A0)
#define DC_HIGH() PORTB |= (0x01 << PIN_A0)

#define RST_LOW() PORTB &= ~(0x01 << PIN_RESET)
#define RST_HIGH() PORTB |= (0x01 << PIN_RESET)

// SPI HANDLERS
void Send_Command(uint8_t cmd) {
	DC_LOW();      // set DC first (A0)
	CS_LOW();
	SPI_SEND(cmd);
	CS_HIGH();
}

void Send_Data(uint8_t d) {
	DC_HIGH();     // set DC first (A0)
	CS_LOW();
	SPI_SEND(d);
	CS_HIGH();
}

void HardwareReset() {
	RST_LOW();
	_delay_ms(100);
	RST_HIGH();
	_delay_ms(100);
}

void SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
	Send_Command(CASET);
	Send_Data(0x00);
	Send_Data(x0);
	Send_Data(0x00);
	Send_Data(x1);

	Send_Command(RASET);
	Send_Data(0x00);
	Send_Data(y0);
	Send_Data(0x00);
	Send_Data(y1);

	Send_Command(RAMWR);
}

void ST7735_init() {
	HardwareReset();
	Send_Command(SWRESET);
	_delay_ms(150);

	Send_Command(SLPOUT);
	_delay_ms(200);

	Send_Command(MADCTL);
	Send_Data(RGB_ORDER);

	Send_Command(COLMOD);
	Send_Data(0x05);   // 16-bit color
	_delay_ms(10);

	Send_Command(DISPON);
	_delay_ms(200);
}

void fillColor(uint16_t color) {
    SetAddressWindow(0, 0, 127, 127);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    CS_LOW();
    DC_HIGH();

    for (uint32_t i = 0; i < 128UL * 128UL; i++) {
        SPI_SEND(hi);
        SPI_SEND(lo);
    }

    CS_HIGH();
}

int main() {

	DDRC = 0x00;
	PORTC = 0xFF;
    // All control pins output
    DDRB |= (0x01 << PORTB0) | (0x01 << PORTB1) | (0x01 << PORTB2);

    // Default states
    PORTB |= (0x01 << PORTB2) | (1 << PORTB0) | (0x01 << PORTB1);   // RST HIGH
	SPI_INIT();
	ST7735_init();

	fillColor(0x1BC1);
	while (1) {}
}
