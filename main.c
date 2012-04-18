#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usiTwiSlave.h"

#define RST0 PA0
#define RST1 PA1
#define RST2 PA2
#define RST3 PA3
#define SCL PA4
#define RST4 PA5
#define SDA PA6
#define ADDR3 PA7
#define ADDR0 PB0
#define ADDR1 PB1
#define ADDR2 PB2

/*
	DDRA - port direction configuration
	PORTA - port output configuration
	PINA - port status
*/

void configPins(void) {

	// disable all MPL115A2 sensors
	DDRA |= (1 << RST0) | (1 << RST1) | (1 << RST2) | (1 << RST3) | (1 << RST4); // all 
	PORTA &= ~((1 << RST0) | (1 << RST1) | (1 << RST2) | (1 << RST3) | (1 << RST4));

	// set all ADDR pins as inputs with pullups
	DDRA ^= (1 << ADDR3);
	DDRB ^= (1 << ADDR0) | (1 << ADDR1) | ( 1 << ADDR2);
	PORTA |= (1 << ADDR3);
	PORTB |= (1 << ADDR0) | (1 << ADDR1) | ( 1 << ADDR2);
}

int main(void) {
	configPins();
	// calculate slaveAddress from state of ADDR pins, shift
	uint8_t slaveAddress = (PINA & ADDR3) << 3 | ( PINB & (ADDR0 | ADDR1 | ADDR2) );
	slaveAddress <<= 4;

	// enable interupts	
	sei(); 
	
	usiTwiSlaveInit(slaveAddress);

	for(;;) {
		if(usiTwiDataInReceiveBuffer())
		{
			uint8_t value;
			uint8_t temp = usiTwiReceiveByte();
			if (temp == 1) 
			{
				PORTB ^= _BV(PB3);
				value = 'D';
			}

			usiTwiTransmitByte(value);
		}
	}
}
