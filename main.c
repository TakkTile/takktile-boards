#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usiTwiSlave.h"


int main(void)
{
	// enable interupts	
	sei(); 
	
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
				PORTB ^= _BV(PB3);
				value = 'D';
			}

			usiTwiTransmitByte(value);
		}
	}
}
