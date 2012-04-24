#include <stdbool.h>

void    usiTwiSlaveInit( void );
void    usiTwiTransmitByte( uint8_t );
uint8_t usiTwiReceiveByte( void );
bool    usiTwiDataInReceiveBuffer( void );

uint8_t slaveAddress;
