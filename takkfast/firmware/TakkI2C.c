// (C) 2012 Biorobotics Lab and Nonolith Labs
// (C) 2011, 2012 Ian Daniher (Nonolith Labs) <ian@nonolithlabs.com>
// (C) 2012 Kevin Mehall (Nonolith Labs) <kevin@nonolithlabs.com>
// Licensed under the terms of the GNU GPLv3+

#include "TakkTile.h"

inline uint8_t calcTinyAddr(uint8_t row, uint8_t column) { return (((row)&0x0F) << 4 | (column&0x07) << 1); }
inline uint8_t calcTinyAddrFlat(uint8_t cell) { return (((cell/5)&0x0F) << 4 | ((cell%5)&0x07) << 1); }

uint8_t botherAddress(uint8_t address, bool stop){
	// Function to write address byte to I2C, returns 1 if ACK, 0 if NACK.
	// 'stop' specifies an optional stop bit on the transaction.
	// NB: Don't read from a non-existant address or the CPU will hang waiting for ACK

	// quick command mode - RIF/WIF trips on ACK
	TWIC.MASTER.CTRLB |= TWI_MASTER_QCEN_bm;
	// set address to bother
	TWIC.MASTER.ADDR = address;
	// if address ends in one, wait for a read to finish
	if (address & 1) while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
	// if address ends in zero, wait for a write to finish
	else while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	// initiate stop condition if (stop)
	if (stop) TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
	// return 1 if ACK, 0 if NACK
	return ((TWIC.MASTER.STATUS & TWI_MASTER_RXACK_bm) >> 4)^1; 
}

inline void startConversion(){
	// Initiates the analog-to-digital conversion of pressure and temperature
	// on all MPL115A2 sensors on all attached rows.
		
	// enable all MPL115A2 by writing to 0x0C
	uint8_t ACK = botherAddress(calcTinyAddr(0, 6), 1);
	// write address byte of MPL115A2
	botherAddress(0xC0, 0);
	// write 0x01 to 0x12 - start conversion of pressure & temperature
	TWIC.MASTER.DATA = 0x12;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x01;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	// end transaction
	TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
	// if you got an ACK on enable, disable all the MPL115A2s
	if (ACK == 1) botherAddress(calcTinyAddr(0, 6)^1, 1);

}

void getCalibrationData(uint8_t cell){
	// Iterate through all rows and all columns. If that cell is alive,
	// read 8 calibration bytes from 0x04 into calibrationData.
	TWIC.MASTER.CTRLC &= ~TWI_MASTER_ACKACT_bm;
	TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm;
	if ( aliveCells[cell] == 0xFF ){
		// attiny address formula
		uint8_t tinyAddr = calcTinyAddrFlat(cell); 
		// enable cell
		botherAddress(tinyAddr, 1);
		// start write to MPL115A2 
		botherAddress(0xC0, 0);
		TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm; 
		// set start address to 0
		TWIC.MASTER.DATA = 0x04;
		while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
		// end transaction
		TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
		// start read from MPL115A2
		TWIC.MASTER.ADDR = 0xC1;
		while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
		for (uint8_t byteCt = 0; byteCt < 8; byteCt++){
			ep0_buf_in[byteCt] = TWIC.MASTER.DATA;
			// if transaction isn't over, wait for ACK
			if (byteCt < 7) while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
			// if transaction is almost over, set next byte to NACK
			if (byteCt == 6) TWIC.MASTER.CTRLC |= TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
			}
		botherAddress(tinyAddr^1, 1);
	}
	else TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
}

void getSensorData(void){
	/* Iterate through all cells. If that cell is alive,
	read four data bytes from memory address 0x00 into sensorData buffer.
	If SLAVE, send sensorData buffer via DMA.
	If MASTER, send sensorData and sensorDataPrime via USB. */
	uint8_t datum = 0x00;
	for (uint8_t cell = 0; cell < 40; cell++) {
		TWIC.MASTER.CTRLC &= ~TWI_MASTER_ACKACT_bm;
		TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm;
		// attiny address formula
		if ( aliveCells[cell] == 0xFF ) { 
			uint8_t tinyAddr = calcTinyAddrFlat(cell); 
			// enable cell
			botherAddress(tinyAddr, 1);
			// start write to MPL115A2
			botherAddress(0xC0, 0);
			TWIC.MASTER.CTRLB = TWI_MASTER_SMEN_bm;	 
			// set start address to 0
			TWIC.MASTER.DATA = 0x00;
			while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
			// end transaction
			TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
			// start read from MPL115A2
			TWIC.MASTER.ADDR = 0xC1;
			while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
			// clock out four bytes
			for (uint8_t byteCt = 0; byteCt < 4; byteCt++){
				datum = TWIC.MASTER.DATA;
				sensorData[(cell<<2) + byteCt] = datum;
				// if transaction isn't over, wait for ACK
				if (byteCt < 3) while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
				// if transaction is almost over, set next byte to NACK
				if (byteCt == 2) TWIC.MASTER.CTRLC |= TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
			}
		botherAddress(tinyAddr^1, 1);
		}
		else TWIC.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc;
	}
	if (MASTER & ~timeout_or_sampling_no_longer_enabled){
		for (uint8_t i = 0; i < 160; i++) send_byte(sensorData[i]);
		for (uint8_t i = 0; i < 160; i++) send_byte(sensorDataPrime[i]);
		break_and_flush();
	}
	if (SLAVE) {
		DMA.CH0.TRFCNT = 160;
		DMA.CH0.CTRLA |= DMA_CH_ENABLE_bm;
		DMA.CH0.CTRLA |= DMA_CH_TRFREQ_bm;
	}
}


void getAliveFlat(void){
	_delay_ms(1);
	for (uint8_t cell = 0; cell < 40; cell++){
		uint8_t tinyAddr = calcTinyAddrFlat(cell);
		if (botherAddress(tinyAddr, 1) == 1) {
			_delay_us(5);
			if (botherAddress(0xC0, 1) == 1) aliveCells[cell] = 0xFF;
			_delay_us(5);
			botherAddress(tinyAddr^1, 1);
			_delay_us(5);
		}
	}
}
