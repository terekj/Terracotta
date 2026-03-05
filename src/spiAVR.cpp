#include <avr/io.h>
#include "utils/spiAVR.h"


//B5 should always be SCK(spi clock) and B3 should always be MOSI. If you are using an
//SPI peripheral that sends data back to the arduino, you will need to use B4 as the MISO pin.
//The SS pin can be any digital pin on the arduino. Right before sending an 8 bit value with
//the SPI_SEND() funtion, you will need to set your SS pin to low. If you have multiple SPI
//devices, they will share the SCK, MOSI and MISO pins but should have different SS pins.
//To send a value to a specific device, set it's SS pin to low and all other SS pins to high.

// Outputs, pin definitions
#define SPI_SCK_PIN   PB5  // D13
#define SPI_MOSI_PIN  PB3  // D11
#define SPI_SS_HW_PIN PB2  // D10, MUST be output


//If SS is on a different port, make sure to change the init to take that into account.
void SPI_INIT() {
    DDRB |= (1 << SPI_SCK_PIN) | (1 << SPI_MOSI_PIN) | (1 << SPI_SS_HW_PIN);
    PORTB |= (1 << SPI_SS_HW_PIN);   // keep hardware SS high
    SPCR = (1 << SPE) | (1 << MSTR);
}


void SPI_SEND(uint8_t data)
{
    SPDR = data;//set data that you want to transmit
    while (!(SPSR & (1 << SPIF)));// wait until done transmitting
}
