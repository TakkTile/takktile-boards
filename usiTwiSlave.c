#include <avr/io.h>
#include <avr/interrupt.h>
#include "usiTwiSlave.h"

static inline void SET_USI_TO_SEND_ACK( ) {
	/* prepare ACK */
	USIDR = 0;
	/* set SDA as output */
	SDA_DDR |= ( 1 << SDA_BIT );
	/* clear all interrupt flags, except Start Cond */
	USISR = 
		( 0 << USISIF ) |
		( 1 << USIOIF ) | ( 1 << USIPF ) | 
		( 1 << USIDC )| 
		/* set USI counter to shift 1 bit */
		( 0x0E << USICNT0 );
}

static inline void SET_USI_TO_TWI_START_CONDITION_MODE( ) {
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
	USISR =
		/* clear all interrupt flags, except Start Cond */
		( 0 << USISIF ) | ( 1 << USIOIF ) | ( 1 << USIPF ) |
		( 1 << USIDC ) | ( 0x0 << USICNT0 );
}

static inline void SET_USI_TO_READ_DATA( ) {
	/* set SDA as input */
	SDA_DDR &= ~( 1 << SDA_BIT );
	/* clear all interrupt flags, except Start Cond */
	USISR =
		( 0 << USISIF ) | ( 1 << USIOIF ) |
		( 1 << USIPF ) | ( 1 << USIDC ) |
		/* set USI to shift out 8 bits */
		( 0x0 << USICNT0 );
}



/********************************************************************************
								 typedef's
********************************************************************************/

typedef enum
{
	USI_SLAVE_CHECK_ADDRESS	= 0x00,
	USI_SLAVE_END_TRX= 0x01,
} overflowState_t;


static volatile overflowState_t overflowState;

/********************************************************************************
								public functions
********************************************************************************/

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



/********************************************************************************
USI Overflow ISR

Handles all the communication.

Only disabled when waiting for a new Start Condition.
********************************************************************************/

ISR( USI_OVF_vect ) {

	switch ( overflowState ) {

		// Address mode: check address and send ACK (and then USI_SLAVE_SEND_DATA) if OK, else reset USI
		case USI_SLAVE_CHECK_ADDRESS:
			if ( (USIDR&0xF0) == (slaveAddress&0xF0) ) {
				if ( USIDR & 0x01 ) {
					PORTA |= 1 << ((USIDR & 0x0F) >> 1);
				}
				else {
					PORTA &= ~(1 << ((USIDR & 0x0F) >> 1));
				} 
				overflowState = USI_SLAVE_END_TRX;
				SET_USI_TO_SEND_ACK( );
			}
			else {
				SET_USI_TO_TWI_START_CONDITION_MODE( );
			}
			break;

		// master read data mode: set USI to sample data from master, then USI_SLAVE_GET_DATA_AND_SEND_ACK
		case USI_SLAVE_END_TRX:
			overflowState = USI_SLAVE_CHECK_ADDRESS; 
			SET_USI_TO_READ_DATA( );
			break;

	} 

}
