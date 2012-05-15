// (C) 2012 Biorobotics Lab and Nonolith Labs
// Written by Ian Daniher, based off usiTwiSlave.c by Donald R. Blake
// Licensed under the terms of the GNU GPLv3+

#include <avr/io.h>
#include <avr/interrupt.h>

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

typedef enum
{
	USI_SLAVE_CHECK_ADDRESS	= 0x00,
	USI_SLAVE_END_TRX= 0x01,
} overflowState_t;

static volatile overflowState_t overflowState;

uint8_t slaveAddress;

void usiTwiSlaveInit( void ) {

	// In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL
	// low when a start condition is detected or a counter overflow (only
	// for USIWM1, USIWM0 = 11).	This inserts a wait state.	SCL is released
	// by the ISRs (USI_START_vect and USI_OVERFLOW_vect).

	// Set SCL and SDA as output
	SDA_DDR |= ( 1 << SDA_BIT );
	SCL_DDR |= ( 1 << SCL_BIT );

	// set SCL high
	SCL_PORT |= ( 1 << SCL_BIT );

	// set SDA high
	SDA_PORT |= ( 1 << SDA_BIT );

	// set SDA as input
	SDA_DDR &= ~( 1 << SDA_BIT );

	USICR =
			 // enable Start Condition Interrupt
			 ( 1 << USISIE ) |
			 // disable Overflow Interrupt
			 ( 0 << USIOIE ) |
			 // set USI in Two-wire mode, no USI Counter overflow hold
			 ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
			 // shift Register Clock Source = external, positive edge
			 // 4-Bit Counter Source = external, both edges
			 ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
			 // no toggle clock-port pin
			 ( 0 << USITC );

	// clear all interrupt flags and reset overflow counter

	USISR = ( 1 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC );

} 


ISR( USI_STR_vect ) {

	// set default starting conditions for new TWI package
	overflowState = USI_SLAVE_CHECK_ADDRESS;

	// set SDA as input
	SDA_DDR &= ~( 1 << SDA_BIT );

	// wait for SCL to go low to ensure the Start Condition has completed (the
	// start detector will hold SCL low ) - if a Stop Condition arises then leave
	// the interrupt to prevent waiting forever - don't use USISR to test for Stop
	// Condition as in Application Note AVR312 because the Stop Condition Flag is
	// going to be set from the last TWI sequence
	while (
			// SCL is high
			( SCL_PIN & ( 1 << SCL_BIT ) ) &&
			// and SDA is low
			!( ( SDA_PIN & ( 1 << SDA_BIT ) ) )
	);


	if ( !( SDA_PIN & ( 1 << SDA_BIT ) ) ) {

		// a Stop Condition did not occur

		USICR =
				// keep Start Condition Interrupt enabled to detect RESTART
				( 1 << USISIE ) |
				// enable Overflow Interrupt
				( 1 << USIOIE ) |
				// set USI in Two-wire mode, hold SCL low on USI Counter overflow
				( 1 << USIWM1 ) | ( 1 << USIWM0 ) |
				// Shift Register Clock Source = External, positive edge
				// 4-Bit Counter Source = external, both edges
				( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
				// no toggle clock-port pin
				( 0 << USITC );

	}
	else {

		// a Stop Condition did occur
		USICR =
				// enable Start Condition Interrupt
				( 1 << USISIE ) |
				// disable Overflow Interrupt
				( 0 << USIOIE ) |
				// set USI in Two-wire mode, no USI Counter overflow hold
				( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
				// Shift Register Clock Source = external, positive edge
				// 4-Bit Counter Source = external, both edges
				( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
				// no toggle clock-port pin
				( 0 << USITC );

	} // end if

	USISR =
			// clear interrupt flags - resetting the Start Condition Flag will release SCL
			( 1 << USISIF ) | ( 1 << USIOIF ) |
			( 1 << USIPF ) |( 1 << USIDC ) |
			// set USI to sample 8 bits (count 16 external SCL pin toggles)
			( 0x0 << USICNT0);

}


ISR( USI_OVF_vect ) {

	switch ( overflowState ) {

		case USI_SLAVE_CHECK_ADDRESS:
			if ( (USIDR&0xF0) == (slaveAddress&0xF0) ) {
				// pin bit position determined by LSB1..3 inclusive
				uint8_t pin_bp = (USIDR & 0x0F) >> 1;
				// pin bitmask defaults to 1 << pin_bp
				uint8_t pin_bm = 1 << pin_bp;
				// if you're theoretically writing the state of the "sixth" sensor, set the bitmask to match all of them
				if ( pin_bp == 6 ) pin_bm = 0x2F;
				// RST4 is actually located at PA5
				if ( pin_bp == 4 ) pin_bm = 0x20; 
				// inverted logic - if transaction is a "read", disable the corresponding sensor
				if ( USIDR & 0x01 ) PORTA &= ~pin_bm;
				// otherwise, enable it
				else PORTA |= pin_bm;
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
			break;

		case USI_SLAVE_END_TRX:
			// next time overflow interrupt trips, check the address
			overflowState = USI_SLAVE_CHECK_ADDRESS; 
			// SDA as input
			SDA_DDR &= ~( 1 << SDA_BIT );
			USICR =
				/* enable Start Condition Interrupt, disable Overflow Interrupt */
				( 1 << USISIE ) | ( 0 << USIOIE ) |
				/* set USI in Two-wire mode, no USI Counter overflow hold */
				( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
				/* Shift Register Clock Source = External, positive edge */
				/* 4-Bit Counter Source = external, both edges */
				( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
				/* no toggle clock-port pin */
				( 0 << USITC );
			// reset all interrupt flags but start condition
			// set USI to shift out one bit
			USISR = ( 0 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC ) | ( 0x0 << USICNT0 );
			break;

	} 

}

int main(void) {
	// disable all MPL115A2 sensors
	DDRA |= RST0 | RST1 | RST2 | RST3 | RST4; 
	PORTA &= ~(RST0 | RST1 | RST2 | RST3 | RST4);

	// set all ADDR pins as inputs with pullups
	DDRA &= ~(ADDR3);
	DDRB &= ~(ADDR0 | ADDR1 | ADDR2);
	PORTA |= ADDR3;
	PORTB |= ADDR0 | ADDR1 | ADDR2;

	// calculate slaveAddress from state of ADDR pins
	slaveAddress = (PINA & ADDR3) << 3 | ( PINB & (ADDR0 | ADDR1 | ADDR2) );
	slaveAddress <<= 4;

	// enable interupts	
	sei(); 

	// instantiate usiTwiSlave code	
	usiTwiSlaveInit();

	for(;;) {}
}
