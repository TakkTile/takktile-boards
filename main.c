#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usiTwiSlave.h"

#define RST0 1 << PA0
#define RST1 1 << PA1
#define RST2 1 << PA2
#define RST3 1 << PA3
#define RST4 1 << PA5
#define ADDR3 1 << PA7
#define ADDR0 1 << PB0
#define ADDR1 1 << PB1
#define ADDR2 1 << PB2

/*
	DDRA - port direction configuration
	PORTA - port output configuration
	PINA - port status
*/

void configPins(void) {

	// disable all MPL115A2 sensors
	DDRA |= RST0 | RST1 | RST2 | RST3 | RST4; 
	PORTA &= ~(RST0 | RST1 | RST2 | RST3 | RST4);

	// set all ADDR pins as inputs with pullups
	DDRA ^= ADDR3;
	DDRB ^= ADDR0 | ADDR1 | ADDR2;
	PORTA |= ADDR3;
	PORTB |= ADDR0 | ADDR1 | ADDR2;
}

int main(void) {
	configPins();
	// calculate slaveAddress from state of ADDR pins, shift
	slaveAddress = (PINA & ADDR3) << 3 | ( PINB & (ADDR0 | ADDR1 | ADDR2) );
	slaveAddress <<= 4;

	// enable interupts	
	sei(); 
	
	usiTwiSlaveInit();

	for(;;) {
	}
}
