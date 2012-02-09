#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

extern "C" 
{
	#include "usiTwiSlave.h"
}


int main(void)
{
	sei(); // enable interupts
	
	DDRB |= _BV(DDB4);
 	DDRB |= _BV(DDB3);
	PORTB |= _BV(PB4);
	PORTB |= _BV(PB3);
	

	const uint8_t slaveAddress = 0x14;
	usiTwiSlaveInit(slaveAddress);
	for(;;)
	{
		if(usiTwiDataInReceiveBuffer())
		{
			uint8_t value;
			uint8_t temp = usiTwiReceiveByte();
			if (temp == 1) 
			{
				PORTB ^= _BV(PB4);
				value = 'H';
			}
			else 
			{
				PORTB ^= _BV(PB3);
				value = 'D';
			}

			usiTwiTransmitByte(value);
		}
	}
}
