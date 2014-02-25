// (C) 2012 Biorobotics Lab and Nonolith Labs
// (C) 2011, 2012 Ian Daniher (Nonolith Labs) <ian@nonolithlabs.com>
// (C) 2012 Kevin Mehall (Nonolith Labs) <kevin@nonolithlabs.com>
// Licensed under the terms of the GNU GPLv3+

#include "TakkTile.h"
#include "TakkI2C.c"

// run I2C at 1MHz
#define F_TWI	1000000
#define TWI_BAUD ((F_CPU / (2 * F_TWI)) - 5) 

ISR(TCC0_CCA_vect){
	// Timer interrupt that trips 1ms after TCC0.CNT is set to 0.
	// Clock out all data from all alive sensors, and start next conversion

	getSensorData();

	// start conversion of next block
	startConversion();

	// reset timer
	TCC0.CNT = 0;
}

int main(void){
	PORTR.DIRSET = 1 << 1;
	PORTR.OUTSET = 1 << 1;
	_delay_ms(5);
	PORTR.OUTCLR = 1 << 1;
	_delay_ms(100);


	// master-detect
	PORTE.DIRCLR = 1 << 0;
	// PE0 is daisy-chaining configuration pin
	if (PORTE.IN & (1 << 0)) { MASTER = 1; SLAVE = 0; }

	USB_ConfigureClock();
	USB_Init();
		
	if (MASTER) {
		// Enable USB interrupts
		USB.INTCTRLA = USB_BUSEVIE_bm | USB_INTLVL_MED_gc;
		USB.INTCTRLB = USB_TRNIE_bm | USB_SETUPIE_bm;
	
	}

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
	sei(); 

	// setup TWI bus for master-mode I2C comms
	TWIC.MASTER.BAUD = TWI_BAUD;
	TWIC.MASTER.CTRLA = TWI_MASTER_ENABLE_bm;  
	TWIC.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;

	// setup TCC0 for sample timing
	TCC0.CTRLA = TC_CLKSEL_DIV256_gc;
	TCC0.CTRLB = TC0_CCAEN_bm | TC_WGMODE_SINGLESLOPE_gc;
	TCC0.INTCTRLB = TC_CCAINTLVL_LO_gc;
	TCC0.CCA = 120; 
	TCC0.PER = 0;

	// config PORTE.USARTE0 for serial transmission as 2e6 8 1 n
	PORTE.DIRSET = 1 << 3;
	PORTE.DIRCLR = 1 << 2;
	USARTE0.BAUDCTRLA = 0;
	USARTE0.BAUDCTRLB = 0;
	USARTE0.CTRLC =  USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	if (MASTER) USARTE0.CTRLB |= USART_RXEN_bm;
	if (SLAVE) USARTE0.CTRLB |= USART_TXEN_bm;

	// configure general DMA settings
	DMA.CTRL = DMA_ENABLE_bm | DMA_DBUFMODE_DISABLED_gc | DMA_PRIMODE_RR0123_gc;

	if (SLAVE) {
	DMA.CH0.ADDRCTRL = DMA_CH_SRCRELOAD_TRANSACTION_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc;
	DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_USARTE0_DRE_gc;
	DMA.CH0.SRCADDR0 = ((uint32_t)(&sensorData) >> (8*0)) & 0xFF;
	DMA.CH0.SRCADDR1 = ((uint32_t)(&sensorData) >> (8*1)) & 0xFF;
	DMA.CH0.SRCADDR2 = ((uint32_t)(&sensorData) >> (8*2)) & 0xFF;
	DMA.CH0.DESTADDR0 = ((uint32_t)(&USARTE0.DATA) >> (8*0)) & 0xFF;
	DMA.CH0.DESTADDR1 = ((uint32_t)(&USARTE0.DATA) >> (8*1)) & 0xFF;
	DMA.CH0.DESTADDR2 = ((uint32_t)(&USARTE0.DATA) >> (8*2)) & 0xFF;
	DMA.CH0.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc; 
	}
	if (MASTER) {
	DMA.CH1.ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_SRCDIR_FIXED_gc | DMA_CH_DESTRELOAD_TRANSACTION_gc | DMA_CH_DESTDIR_INC_gc;
	DMA.CH1.TRIGSRC = DMA_CH_TRIGSRC_USARTE0_RXC_gc;
	DMA.CH1.SRCADDR0 = ((uint32_t)(&USARTE0.DATA) >> (8*0)) & 0xFF;
	DMA.CH1.SRCADDR1 = ((uint32_t)(&USARTE0.DATA) >> (8*1)) & 0xFF;
	DMA.CH1.SRCADDR2 = ((uint32_t)(&USARTE0.DATA) >> (8*2)) & 0xFF;
	DMA.CH1.DESTADDR0 = ((uint32_t)(&sensorDataPrime) >> (8*0)) & 0xFF;
	DMA.CH1.DESTADDR1 = ((uint32_t)(&sensorDataPrime) >> (8*1)) & 0xFF;
	DMA.CH1.DESTADDR2 = ((uint32_t)(&sensorDataPrime) >> (8*2)) & 0xFF;
	DMA.CH1.TRFCNT = 160;
	DMA.CH1.REPCNT = 0;
	DMA.CH1.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_REPEAT_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc; 
	}

	getAliveFlat();
	//getCalibrationData();

	if (SLAVE && ~MASTER) {
		TCC0.CNT = 0; 
		TCC0.PER = 1 << 15; 
		startConversion();
	}

	PORTR.OUTSET = 1 << 1;
	for (;;){
	}
}


#define xstringify(s) stringify(s)
#define stringify(s) #s

const char PROGMEM hwversion[] = xstringify(HW_VERSION);
const char PROGMEM fwversion[] = xstringify(FW_VERSION);

uint8_t usb_cmd = 0;
uint8_t cmd_data = 0;

// Add a flag to prevent other i2c-using requests from happening while sampling is enabled. ControlRequest is now from an ISR, and could happen at any time inside of GetData and friends.

/** Event handler for the library USB Control Request reception event. */
bool EVENT_USB_Device_ControlRequest(USB_Request_Header_t* req){
	// zero out ep0_buf_in
	for (uint8_t i = 0; i < 64; i++) ep0_buf_in[i] = 0;
	usb_cmd = 0;
	if ((req->bmRequestType & CONTROL_REQTYPE_TYPE) == REQTYPE_VENDOR){
		switch(req->bRequest){
			case 0x00: // Info

				if (req->wIndex == 0){
					USB_ep0_send_progmem((uint8_t*)hwversion, sizeof(hwversion));
				}else if (req->wIndex == 1){
					USB_ep0_send_progmem((uint8_t*)fwversion, sizeof(fwversion));
				}
				
				return true;

			// bother a specified I2C address, return '1' if address ACKs, '0' if NACK
			// mnemonic - 0xBotherAddress
			case 0xBA: 
				ep0_buf_in[0] = botherAddress(req->wIndex, req->wValue);
				USB_ep0_send(1);
				return true;

			// start sampling
			// mnemonic - 0xConfigure7imer
			case 0xC7:
				if (req->wIndex != 0) {
					TCC0.PER = 1 << 15;
					startConversion();
					ep0_buf_in[0] = 1;
					usb_pipe_reset(&ep_in);
					timeout_or_sampling_no_longer_enabled = 0;
					TCC0.CCA = req->wValue;
				}
				else {
					TCC0.PER = 0;
					ep0_buf_in[0] = 0;
					usb_pipe_reset(&ep_in);
					timeout_or_sampling_no_longer_enabled = 1;
				}
				TCC0.CNT = 0;
				USB_ep0_send(1);
				return true;

			// return a bitmap of alive cells 
			// mnemonic - 0x5Can
			case 0x5C: 
				getAliveFlat();
				_delay_ms(5);
				for (uint8_t i = 0; i < 40; i++) {ep0_buf_in[i] = aliveCells[i];}
				USB_ep0_send(40);
				return true;

			// robust vendor-request based data collection
			// mnemonic - 0x6etData
			case 0x6D:
				if (req->wIndex == 0xFF) {
					startConversion();
					_delay_ms(2);
					USB_ep0_send(0);
					return true;
				}
				if (req->wIndex == 0x0F) {
					getSensorData();
					USB_ep0_send(0);
					return true;
				}
				if (req->wIndex == 0x00) { 
					for (uint8_t i = 0; i < 64; i++) {ep0_buf_in[i] = sensorData[i+(req->wValue*64)];}
					USB_ep0_send(64);
					return true;
				}

			// return calibration information
			// mnemonic - 0x6etCalibration
			case 0x6C: {
				getCalibrationData(req->wIndex&0xFF);
				USB_ep0_send(8);
				return true;
				}

			// disconnect from USB, jump to bootloader	
			case 0xBB: 
				USB_enter_bootloader();
				return true;
		}
	}
	return false;
}
