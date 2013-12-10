// (C) 2012 Biorobotics Lab and Nonolith Labs
// written by Ian Daniher, based off usiTwiSlave.c by Donald R. Blake
// updated by Leif Jentoft, 2013
// licensed under the terms of the GNU GPLv3+

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define RST0 1 << PA0
#define RST1 1 << PA1
#define RST2 1 << PA2
#define RST3 1 << PA3
#define RST4 1 << PA5
#define RST5 1 << PA7
#define ADDR0 1 << PB0
#define ADDR1 1 << PB1
#define ADDR2 1 << PB2

typedef enum
{
	USI_SLAVE_CHECK_ADDRESS	= 0x00,
	USI_SLAVE_END_TRX= 0x01,
} overflowState_t;

static volatile overflowState_t overflowState;

uint8_t slaveAddress;

void usiTwiSlaveInit( void ) {

	// set SCL and SDA as output
	SDA_DDR |= ( 1 << SDA_BIT );
	SCL_DDR |= ( 1 << SCL_BIT );

	// set SCL high
	SCL_PORT |= ( 1 << SCL_BIT );

	// set SDA high
	SDA_PORT |= ( 1 << SDA_BIT );

	// set SDA as input
	SDA_DDR &= ~( 1 << SDA_BIT );

	// enable Start Condition Interrupt
	// disable Overflow Interrupt
	// set USI in Two-wire mode, no USI Counter overflow hold
	// shift Register Clock Source = external, positive edge
	// 4-Bit Counter Source = external, both edges
	// no toggle clock-port pin
	USICR = ( 1 << USISIE ) |
			( 0 << USIOIE ) |
			( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
			( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
			( 0 << USITC );

	// clear all interrupt flags and reset overflow counter

	USISR = ( 1 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC );

} 


ISR( USI_STR_vect ) {

	// set default starting conditions for new TWI packageDDRA
	overflowState = USI_SLAVE_CHECK_ADDRESS;

	// set SDA as input
	SDA_DDR &= ~( 1 << SDA_BIT );

	// wait for SCL to be high and SDA to be low
	while ( ( SCL_PIN & ( 1 << SCL_BIT ) ) && !( ( SDA_PIN & ( 1 << SDA_BIT ) ) ) );

	// a stop condition did not occur
	if ( !( SDA_PIN & ( 1 << SDA_BIT ) ) ) {

		// keep Start Condition Interrupt enabled to detect RESTART
		// enable Overflow Interrupt
		// set USI in Two-wire mode, hold SCL low on USI Counter overflow
		// shift Register Clock Source = External, positive edge
		// 4-Bit Counter Source = external, both edges
		// no toggle clock-port pin
		USICR = ( 1 << USISIE ) |
				( 1 << USIOIE ) |
				( 1 << USIWM1 ) | ( 1 << USIWM0 ) |
				( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
				( 0 << USITC );

	}
	// a stop condition did occur
	else {

		// enable Start Condition Interrupt
		// disable Overflow Interrupt
		// set USI in Two-wire mode, no USI Counter overflow hold
		// shift Register Clock Source = external, positive edge
		// 4-Bit Counter Source = external, both edges
		// no toggle clock-port pin
		USICR = ( 1 << USISIE ) |
				( 0 << USIOIE ) |
				( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
				( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
				( 0 << USITC );

	}

	// clear interrupt flags - resetting the Start Condition Flag will release SCL
	// set USI to sample 8 bits (count 16 external SCL pin toggles)
	USISR = ( 1 << USISIF ) | ( 1 << USIOIF ) |
			( 1 << USIPF ) |( 1 << USIDC ) |
			( 0x0 << USICNT0);

}


ISR( USI_OVF_vect ) {

	switch ( overflowState ) {

		case USI_SLAVE_CHECK_ADDRESS: {

			uint8_t ADDR = USIDR;
			
			if ( ( ADDR == 0x0C ) || (ADDR == 0x0D) || (( (ADDR&0xF0) == (slaveAddress&0xF0) ))){
				// pin bit position determined by LSB1..3 inclusive
				uint8_t pin_bit_position = (ADDR & 0x0F) >> 1;
				// pin bitmask defaults to 1 << pin_bit_position
				uint8_t pin_bitmask = 1 << pin_bit_position;
				// if you're theoretically writing the state of the "sixth" sensor, set the bitmask to match all of them
				if ( pin_bit_position == 6 ) pin_bitmask = 0xAF;
				// RST4 is actually located at PA5
				if ( pin_bit_position == 4 ) pin_bitmask = 0x20; 
				// RST5 is actually located at PA7
				if ( pin_bit_position == 5 ) pin_bitmask = 0x80; 
				// inverted logic - if transaction is a "read", disable the corresponding sensor
				if ( ADDR & 0x01 ) PORTA &= ~pin_bitmask;
				// otherwise, enable it
				else PORTA |= pin_bitmask;
				// prep ACK
				USIDR = 0;
				// set SDA as an output
				SDA_DDR |= ( 1 << SDA_BIT );
				// reset all interrupt flags but start condition and set USI to shift out one bit
				USISR = ( 0 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC ) | ( 0x0E << USICNT0 );
				// after the oveflow interrupt is triggered after one bit is shifted, go to USI_SLAVE_END_TRX condition
				overflowState = USI_SLAVE_END_TRX;
			}
			else {
				overflowState = USI_SLAVE_END_TRX;
				return;
			}
			}
			break;

		case USI_SLAVE_END_TRX: {
			// next time overflow interrupt trips, check the address
			overflowState = USI_SLAVE_CHECK_ADDRESS; 
			// SDA as input
			SDA_DDR &= ~( 1 << SDA_BIT );

			// enable Start Condition Interrupt, disable Overflow Interrupt
			// set USI in Two-wire mode, no USI Counter overflow hold
			// shift Register Clock Source = External, positive edge
			// 4-Bit Counter Source = external, both edges
			// no toggle clock-port pin

			USICR = ( 1 << USISIE ) | ( 0 << USIOIE ) |
				( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
				( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
				( 0 << USITC );

			// reset all interrupt flags but start condition
			// set USI to shift out one bit
			USISR = ( 0 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC ) | ( 0x0 << USICNT0 );
			}
			break;

	} 

}

uint8_t getAddress(void) {
	uint8_t slaveAddress;
	uint8_t tmpAddress;
	// calculate slaveAddress from state of ADDR pins

	slaveAddress =  ( PINB & ((ADDR0) | (ADDR1) | (ADDR2)) );

        // calculate slaveAddress from state of ADDR pins
        tmpAddress = 0;
        slaveAddress ^= 0b0111; // exclusive or ... to invert the bits value.
        if (slaveAddress & 0b001) {
                tmpAddress = 0b100;
        }
        if (slaveAddress & 0b010) {
                tmpAddress |= 0b010;
        }
        if (slaveAddress & 0b100) {
        	tmpAddress |= 0b001;
        }
        slaveAddress = tmpAddress;

	slaveAddress <<= 4;

	return slaveAddress;
}

int main(void) {

	uint16_t bigAddress;

	bigAddress=0;

	// disable all MPL115A2 sensors
	DDRA |= RST0 | RST1 | RST2 | RST3 | RST4 | RST5; 
	PORTA &= ~(RST0 | RST1 | RST2 | RST3 | RST4 | RST5);

	// set all ADDR pins as inputs with pullups
	DDRB &= ~(ADDR0 | ADDR1 | ADDR2);
	PORTB |= ADDR0 | ADDR1 | ADDR2;

	for (uint8_t i = 0; i < 8; i++) {
		// first wait, and then read the address
		_delay_us(100);
		bigAddress += getAddress();
	}
	slaveAddress = bigAddress >> 3;

// ---- for Debugging ----------------------	
//	if (bigAddress % 8) {
//	if (1) {
//		PORTA |= RST0; 
//		_delay_us(5);
//		PORTA &= ~ RST0; 
//		tmp=slaveAddress;
//		tmp>>=4;
//
//		for (uint8_t i = 0; i < (tmp); i++) {
//			PORTA |= RST0; 
//			_delay_us(9);
//			PORTA &= ~ RST0; 
//		}
//	}
// --------------------------

	// enable interupts	
	sei(); 

	// instantiate usiTwiSlave code	
	usiTwiSlaveInit();

	for(;;) {}
}
