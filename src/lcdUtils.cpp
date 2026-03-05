#include "utils/lcdUtils.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "utils/spiAVR.h"


void lcd_init() {
    LCD_DC_DDR  |= (1 << LCD_DC_PIN);
    LCD_CS_DDR  |= (1 << LCD_CS_PIN);
    LCD_RST_DDR |= (1 << LCD_RST_PIN);

    CS_HIGH();
    DC_HIGH();
    RST_HIGH();

    ST7735_init();
}


void Send_Command(uint8_t cmd) {
    DC_LOW();
    CS_LOW();
    SPI_SEND(cmd);
    CS_HIGH();
}

void Send_Data(uint8_t d) {
    DC_HIGH();
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
    Send_Data(x0 + 2);
    Send_Data(0x00);
    Send_Data(x1 + 2);

    Send_Command(RASET);
    Send_Data(0x00);
    Send_Data(y0 + 1);
    Send_Data(0x00);
    Send_Data(y1 + 1);

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
    Send_Data(0x05);
    _delay_ms(10);

    Send_Command(DISPON);
    _delay_ms(200);
}

void fillColor(uint16_t color) {
    fillRect(color, 0, 0, 127, 127);
}
void fillRect(uint16_t color, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    SetAddressWindow(x0, y0, x1, y1);

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
