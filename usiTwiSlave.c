#include <avr/io.h>
#include <avr/interrupt.h>
#include "usiTwiSlave.h"

typedef enum
{
	USI_SLAVE_CHECK_ADDRESS	= 0x00,
	USI_SLAVE_END_TRX= 0x01,
} overflowState_t;

static volatile overflowState_t overflowState;

// initialise USI for TWI slave mode

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
				if ( USIDR & 0x01 ) {
					PORTA |= 1 << ((USIDR & 0x0F) >> 1);
				}
				else {
					PORTA &= ~(1 << ((USIDR & 0x0F) >> 1));
				} 
				overflowState = USI_SLAVE_END_TRX;
				// prep ACK
				USIDR = 0;
				// set SDA as an output
				SDA_DDR |= ( 1 << SDA_BIT );
				// reset all interrupt flags but start condition
				// set USI to shift out one bit
				USISR = ( 0 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC ) | ( 0x0E << USICNT0 );
			}
			else {
				overflowState = USI_SLAVE_END_TRX;
				return;
			}
			break;

		case USI_SLAVE_END_TRX:
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
