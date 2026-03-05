#include "classes/Application.h"
#include "classes/Game.h"
#include "classes/Player.h"
#include "classes/Sprite.h"

#include "utils/lcdUtils.h"
#include "utils/spiAVR.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

Application* Application::instance = nullptr;

Application::Application()
    : game(Game::getInstance()),
      player(Player::getInstance())
{
}

Application& Application::getInstance() {
    if (!instance) {
        static Application singleton;
        instance = &singleton;
    }
    return *instance;
}

void Application::drawSprite(const Sprite& sprite) {

    const uint8_t*  bitmap   = sprite.getBitmap();
    const uint16_t* colormap = sprite.getColormap();

    const int rowBytes = 80 / 4;  // 20 bytes per row

    SetAddressWindow(21, 21, 21 + 79, 21 + 79);
    CS_LOW();
    DC_HIGH();

    for (int y = 0; y < 80; y++) {
        int rowStart = y * rowBytes;

        for (int b = 0; b < rowBytes; b++) {

            uint8_t fourpx = pgm_read_byte(&bitmap[rowStart + b]);

            uint8_t p0 = (fourpx >> 6) & 0x03;
            uint8_t p1 = (fourpx >> 4) & 0x03;
            uint8_t p2 = (fourpx >> 2) & 0x03;
            uint8_t p3 = (fourpx >> 0) & 0x03;

            uint16_t c0 = pgm_read_word(&colormap[p0]);
            uint16_t c1 = pgm_read_word(&colormap[p1]);
            uint16_t c2 = pgm_read_word(&colormap[p2]);
            uint16_t c3 = pgm_read_word(&colormap[p3]);

            SPI_SEND(c0 >> 8); SPI_SEND(c0 & 0xFF);
            SPI_SEND(c1 >> 8); SPI_SEND(c1 & 0xFF);
            SPI_SEND(c2 >> 8); SPI_SEND(c2 & 0xFF);
            SPI_SEND(c3 >> 8); SPI_SEND(c3 & 0xFF);
        }
    }

    CS_HIGH();
}
