/**
	ADS7951 - 8 channel 12 bit ADC application - for Maple Mini STM32

	This sketch reads the converted values of the 8 channels in predetermined sequence.

	Using the second SPI port (SPI_2):
	SS    <-->  PB12 <-->  BOARD_SPI2_NSS_PIN
	SCK   <-->  PB13 <-->  BOARD_SPI2_SCK_PIN
	MISO  <-->  PB14 <-->  BOARD_SPI2_MISO_PIN
	MOSI  <-->  PB15 <-->  BOARD_SPI2_MOSI_PIN
*/

#include <SPI.h>

// the settings may be different for the SPI ports
SPISettings spi2Settings(18000000, MSBFIRST, SPI_MODE2, DATA_SIZE_16BIT);

SPIClass SPI_2(2); // create an instance of the SPI Class that uses the 2nd SPI Port

/*****************************************************************************/
#define PORT_PIN_SET(port, pin)	( port->regs->BSRR = (1U << pin) )
#define PORT_PIN_CLEAR(port, pin)	( port->regs->BSRR = (1U << pin) << 16 )
/*****************************************************************************/
#define SPI2_NSS_PIN		PB12   // SPI_2 Chip Select pin
#define NSS2_PIN_SET		PORT_PIN_SET(GPIOB, 12)
#define NSS2_PIN_CLEAR	PORT_PIN_CLEAR(GPIOB, 12)
/*****************************************************************************/
#define DBG_PIN			PB4
#define DBG_PIN_SET		PORT_PIN_SET(GPIOB, 4)
#define DBG_PIN_CLEAR	PORT_PIN_CLEAR( GPIOB, 4)
/*****************************************************************************/

/*****************************************************************************/
uint16_t SPI2_transfer(uint16_t data)
{
	NSS2_PIN_CLEAR;
	uint16_t rd = SPI_2.transfer(data);
	NSS2_PIN_SET;
	return rd;
}
/*****************************************************************************/
void setup()
{
	Serial1.begin(57600);
	Serial1.println(F("ADS7951 application started...\n"));
	// Setup SPI 2
	SPI_2.beginTransaction(SPI2_NSS_PIN, spi2Settings); //Initialize the SPI_2 port.
	pinMode(SPI2_NSS_PIN, OUTPUT);
	NSS2_PIN_SET; // set NSS pin inactive
	// debug
	pinMode(DBG_PIN, OUTPUT);
	// setup ADS7951 - no need to re-configure when sampling only channel 0 continuously
	SPI2_transfer(0x0040);	// CTRL reg: use 2x Vref input range
}
/*****************************************************************************/
void loop()
{
	uint16_t dat, i;
	Serial.println();
	for (i = 0; i<10; i++) {
		dat = SPI2_transfer((uint16_t)0);	// read channel 0 value
		Serial.print(dat);
		Serial.print(',');
	}
}
