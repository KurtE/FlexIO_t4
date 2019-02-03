#include "FlexSerial.h"
#define BAUDRATE 115200
#define FLEXIO1_CLOCK (480000000L/16) // Again assuming default clocks?

//#define DEBUG_FlexSerial
//#define DEBUG_digitalWriteFast(pin, state) digitalWriteFast(pin, state)
#define DEBUG_digitalWriteFast(pin, state) 

//=============================================================================
// FlexSerial::Begin
//=============================================================================
bool FlexSerial::begin(uint32_t baud) {
	// BUGBUG - may need to actual Clocks to computer baud...
	uint16_t baud_div =  (FLEXIO1_CLOCK/baud)/2 - 1;                                   
	//-------------------------------------------------------------------------
	// TX Pin setup - if requested
	//-------------------------------------------------------------------------
	if (_txPin != -1) {
		_tx_pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_txPin, _tx_flex_pin);
		if (_tx_pflex == nullptr) {
			Serial.printf("FlexSerial - Failed to map TX pin %d to FlexIO\n", _txPin);
			return false;
		}

		// BUGBUG need to handle restarts...
		IMXRT_FLEXIO_t *p = &_tx_pflex->port();

#ifdef DEBUG_FlexSerial
		Serial.printf("pin %d maps to: %x, port: %x pin %x\n", _txPin, (uint32_t)_tx_pflex, (uint32_t)p, _tx_flex_pin);
#endif		
		_tx_timer = _tx_pflex->requestTimers();
		_tx_shifter = _tx_pflex->requestShifter();
		if ((_tx_timer == 0xff) || (_tx_shifter == 0xff)) {
			_tx_pflex->freeTimers(_tx_timer);
			_tx_timer = 0xff;
			_tx_pflex->freeShifter(_tx_shifter);
			_tx_shifter = 0xff;
			Serial.println("FlexSerial - Failed to allocate TX timer or shifter");
			return false;
		}

		_tx_shifter_mask = 1;
		for (uint8_t i = _tx_shifter; i > 0; i--) _tx_shifter_mask <<= 1;

#ifdef DEBUG_FlexSerial
		Serial.printf("timer index: %d shifter index: %d mask: %x\n", _tx_timer, _tx_shifter, _tx_shifter_mask);
		// lets try to configure a tranmitter like example
		Serial.println("Before configure flexio");
#endif
		p->SHIFTCFG[_tx_shifter] = FLEXIO_SHIFTCFG_SSTOP(3) | FLEXIO_SHIFTCFG_SSTART(2); //0x0000_0032;
		p->SHIFTCTL[_tx_shifter] = FLEXIO_SHIFTCTL_PINCFG(3) | FLEXIO_SHIFTCTL_SMOD(2) |
		                              FLEXIO_SHIFTCTL_TIMSEL(_tx_timer) | FLEXIO_SHIFTCTL_PINSEL(_tx_flex_pin); // 0x0003_0002;
		p->TIMCMP[_tx_timer] = 0xf00 | baud_div; //0xF01; //0x0000_0F01;		//
		p->TIMCFG[_tx_timer] = FLEXIO_TIMCFG_TSTART | FLEXIO_TIMCFG_TSTOP(2) |
		                          FLEXIO_TIMCFG_TIMENA(2) |  FLEXIO_TIMCFG_TIMDIS(2); //0x0000_2222;
		p->TIMCTL[_tx_timer] = FLEXIO_TIMCTL_TIMOD(1) | FLEXIO_TIMCTL_TRGPOL | FLEXIO_TIMCTL_TRGSRC
		                          | FLEXIO_TIMCTL_TRGSEL(1) | FLEXIO_TIMCTL_PINSEL(_tx_flex_pin);  // 0x01C0_0001;
		p->CTRL = FLEXIO_CTRL_FLEXEN;
		//p->SHIFTSTAT = _tx_shifter_mask;   // Clear out the status.

		// Set the IO pin into FLEXIO mode
		_tx_pflex->setIOPinToFlexMode(_txPin);
		_tx_pflex->addIOHandlerCallback(this);
		// Lets print out some of the settings and the like to get idea of state
#ifdef DEBUG_FlexSerial
		Serial.printf("CCM_CDCDR: %x\n", CCM_CDCDR);
		Serial.printf("VERID:%x PARAM:%x CTRL:%x PIN: %x\n", p->PARAM, p->CTRL, p->CTRL, p->PIN);
		Serial.printf("SHIFTSTAT:%x SHIFTERR=%x TIMSTAT=%x\n", p->SHIFTSTAT, p->SHIFTERR, p->TIMSTAT);
		Serial.printf("SHIFTSIEN:%x SHIFTEIEN=%x TIMIEN=%x\n", p->SHIFTSIEN, p->SHIFTEIEN, p->TIMIEN);
		Serial.printf("SHIFTSDEN:%x SHIFTSTATE=%x\n", p->SHIFTSDEN, p->SHIFTSTATE);
		Serial.printf("SHIFTCTL:%x %x %x %x\n", p->SHIFTCTL[0], p->SHIFTCTL[1], p->SHIFTCTL[2], p->SHIFTCTL[3]);
		Serial.printf("SHIFTCFG:%x %x %x %x\n", p->SHIFTCFG[0], p->SHIFTCFG[1], p->SHIFTCFG[2], p->SHIFTCFG[3]);
		Serial.printf("TIMCTL:%x %x %x %x\n", p->TIMCTL[0], p->TIMCTL[1], p->TIMCTL[2], p->TIMCTL[3]);
		Serial.printf("TIMCFG:%x %x %x %x\n", p->TIMCFG[0], p->TIMCFG[1], p->TIMCFG[2], p->TIMCFG[3]);
		Serial.printf("TIMCMP:%x %x %x %x\n", p->TIMCMP[0], p->TIMCMP[1], p->TIMCMP[2], p->TIMCMP[3]);
#endif
	}

	//-------------------------------------------------------------------------
	// RX Pin setup - if requested
	//-------------------------------------------------------------------------
	if (_rxPin != -1) {
		_rx_pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_rxPin, _rx_flex_pin);
		if (_rx_pflex == nullptr) {
			Serial.printf("FlexSerial - Failed to map RX pin %d to FlexIO\n", _rxPin);
			return false;
		}
		IMXRT_FLEXIO_t *p = &_rx_pflex->port();
#ifdef DEBUG_FlexSerial
		Serial.printf("pin %d maps to: %x, port: %x pin %x\n", _rxPin, (uint32_t)_rx_pflex, (uint32_t)p, _rx_flex_pin);
#endif		
		_rx_timer = _rx_pflex->requestTimers();
		
		// We don't need to, but if using two shifters, try not to take shifters associated with one dma channel
		uint8_t dma_channel_to_avoid = 0xff;
		if (_tx_pflex == _rx_pflex) dma_channel_to_avoid = _tx_pflex->shiftersDMAChannel(_tx_shifter);

		_rx_shifter = _rx_pflex->requestShifter(dma_channel_to_avoid);
		if ((_rx_timer == 0xff) || (_rx_shifter == 0xff)) {
			_rx_pflex->freeTimers(_rx_timer);
			_rx_timer = 0xff;
			_rx_pflex->freeShifter(_rx_shifter);
			_rx_shifter = 0xff;
			Serial.println("FlexSerial - Failed to allocate RX timer or shifter");
			return false;
		}

		_rx_shifter_mask = 1;

		for (uint8_t i = _rx_shifter; i > 0; i--) _rx_shifter_mask <<= 1;

#ifdef DEBUG_FlexSerial
		Serial.printf("timer index: %d shifter index: %d mask: %x\n", _rx_timer, _rx_shifter, _rx_shifter_mask);
#endif
		// lets try to configure a receiver like example
		Serial.println("Before configure flexio");
		p->SHIFTCFG[_rx_shifter] = FLEXIO_SHIFTCFG_SSTOP(3) | FLEXIO_SHIFTCFG_SSTART(2); //0x0000_0032;
		p->SHIFTCTL[_rx_shifter] = FLEXIO_SHIFTCTL_TIMPOL | FLEXIO_SHIFTCTL_SMOD(1) |
		                              FLEXIO_SHIFTCTL_TIMSEL(_rx_timer) | FLEXIO_SHIFTCTL_PINSEL(_rx_flex_pin); // 0x0080_0001;

		p->TIMCMP[_rx_timer] = 0xf00 | baud_div; //0xF01; //0x0000_0F01;		//
		p->TIMCFG[_rx_timer] = FLEXIO_TIMCFG_TSTART | FLEXIO_TIMCFG_TSTOP(2) |
		                          FLEXIO_TIMCFG_TIMENA(4) | FLEXIO_TIMCFG_TIMDIS(2) |
		                          FLEXIO_TIMCFG_TIMRST(4) | FLEXIO_TIMCFG_TIMOUT(2); //0x204_2422

		p->TIMCTL[_rx_timer] = FLEXIO_TIMCTL_TIMOD(1) | FLEXIO_TIMCTL_PINPOL | FLEXIO_TIMCTL_PINSEL(_rx_flex_pin);;  // 0x0000_0081;
		p->CTRL = FLEXIO_CTRL_FLEXEN;									// make sure it is enabled. 
		//p->SHIFTSTAT = _rx_shifter_mask;   // Clear out the status.

		// Set the IO pin into FLEXIO mode
		_rx_pflex->setIOPinToFlexMode(_rxPin);
		_rx_pflex->addIOHandlerCallback(this);
		__disable_irq();
		_rx_pflex->port().SHIFTSIEN |= _rx_shifter_mask;  // enable interrupt on this one...

		__enable_irq();
		// Lets print out some of the settings and the like to get idea of state
#ifdef DEBUG_FlexSerial
		Serial.printf("CCM_CDCDR: %x\n", CCM_CDCDR);
		Serial.printf("VERID:%x PARAM:%x CTRL:%x PIN: %x\n", p->PARAM, p->CTRL, p->CTRL, p->PIN);
		Serial.printf("SHIFTSTAT:%x SHIFTERR=%x TIMSTAT=%x\n", p->SHIFTSTAT, p->SHIFTERR, p->TIMSTAT);
		Serial.printf("SHIFTSIEN:%x SHIFTEIEN=%x TIMIEN=%x\n", p->SHIFTSIEN, p->SHIFTEIEN, p->TIMIEN);
		Serial.printf("SHIFTSDEN:%x SHIFTSTATE=%x\n", p->SHIFTSDEN, p->SHIFTSTATE);
		Serial.printf("SHIFTCTL:%x %x %x %x\n", p->SHIFTCTL[0], p->SHIFTCTL[1], p->SHIFTCTL[2], p->SHIFTCTL[3]);
		Serial.printf("SHIFTCFG:%x %x %x %x\n", p->SHIFTCFG[0], p->SHIFTCFG[1], p->SHIFTCFG[2], p->SHIFTCFG[3]);
		Serial.printf("TIMCTL:%x %x %x %x\n", p->TIMCTL[0], p->TIMCTL[1], p->TIMCTL[2], p->TIMCTL[3]);
		Serial.printf("TIMCFG:%x %x %x %x\n", p->TIMCFG[0], p->TIMCFG[1], p->TIMCFG[2], p->TIMCFG[3]);
		Serial.printf("TIMCMP:%x %x %x %x\n", p->TIMCMP[0], p->TIMCMP[1], p->TIMCMP[2], p->TIMCMP[3]);
#endif
	}

	return true;
}

void FlexSerial::end(void) {
	// If the transmit was allocated free it now as well as timers and shifters.
	if (_tx_pflex) {
		_tx_pflex->freeTimers(_tx_timer);
		_tx_timer = 0xff;
		_tx_pflex->freeShifter(_tx_shifter);
		_tx_shifter = 0xff;

		_tx_pflex->removeIOHandlerCallback(this);
		_tx_pflex = nullptr;	
	}

}


void FlexSerial::flush(void) {

}

size_t FlexSerial::write(uint8_t c) {
	uint32_t head;
	head = _tx_buffer_head;
	if (++head >= TX_BUFFER_SIZE) head = 0;
	while (_tx_buffer_tail == head) {
		/*
		int priority = nvic_execution_priority();
		if (priority <= hardware->irq_priority) {
			if ((port->STAT & LPUART_STAT_TDRE)) {
				uint32_t tail = _tx_buffer_tail;
				if (++tail >= TX_BUFFER_SIZE) tail = 0;
				if (tail < tx_buffer_size_) {
					n = tx_buffer_[tail];
				} else {
					n = tx_buffer_storage_[tail-tx_buffer_size_];
				}
				port->DATA  = n;
				_tx_buffer_tail = tail;
			}
		} else if (priority >= 256) 
		*/
		{
			yield(); // wait
		} 
	}
	//digitalWrite(5, LOW);
	//Serial.printf("WR %x %d %d %d %x %x\n", c, head, tx_buffer_size_,  TX_BUFFER_SIZE, (uint32_t)tx_buffer_, (uint32_t)tx_buffer_storage_);
	_tx_buffer[_tx_buffer_head] = c;
	__disable_irq();
	//transmitting_ = 1;
	_tx_buffer_head = head;
	//port->CTRL |= LPUART_CTRL_TIE; // (may need to handle this issue)BITBAND_SET_BIT(LPUART0_CTRL, TIE_BIT);
	_tx_pflex->port().SHIFTSIEN |= _tx_shifter_mask;  // enable interrupt on this one...

	__enable_irq();
	//digitalWrite(3, LOW);
	return 1;
/*

	IMXRT_FLEXIO_t *p = &_tx_pflex->port();

	while (!(p->SHIFTSTAT & _tx_shifter_mask)) ; // wait until it says there is room for next output
	p->SHIFTBUF[_tx_shifter] = c;  // put the next byte out
	return 1;
*/
}

int FlexSerial::available(void) {
	uint32_t head, tail;

	head = _rx_buffer_head;
	tail = _rx_buffer_tail;
	if (head >= tail) return RX_BUFFER_SIZE - 1 - head + tail;
	return tail - head - 1;
}

int FlexSerial::peek(void) {
	if (_rx_buffer_head == _rx_buffer_tail) return -1;
	return _rx_buffer[_tx_buffer_tail] ;
}

int FlexSerial::read(void) {
	int return_value = -1;
	if (_rx_buffer_head != _rx_buffer_tail) {
		return_value = _rx_buffer[_rx_buffer_tail++] ;
		if (_rx_buffer_tail >= RX_BUFFER_SIZE) 
			_rx_buffer_tail = 0;
	}
	return return_value;
}

void FlexSerial::clear(void) {
	//rx_buffer_head_ = rx_buffer_tail_;
}

int FlexSerial::availableForWrite(void) {
	uint32_t head, tail;

	head = _tx_buffer_head;
	tail = _tx_buffer_tail;
	if (head >= tail) return TX_BUFFER_SIZE - 1 - head + tail;
	return tail - head - 1;
}


bool FlexSerial::call_back (FlexIOHandler *pflex) {
	DEBUG_digitalWriteFast(4, HIGH);
	// check for RX
	if (pflex == _rx_pflex) {
		if (_rx_pflex->port().SHIFTSTAT & _rx_shifter_mask) {
			DEBUG_digitalWriteFast(5, HIGH);
			uint8_t c = _rx_pflex->port().SHIFTBUFBYS[_rx_shifter] & 0xff;
			uint32_t head;
			head = _rx_buffer_head;
			if (++head >= RX_BUFFER_SIZE) head = 0;
			// don't save char if buffer is full...
			if (_tx_buffer_tail != head) {
				_rx_buffer[_rx_buffer_head] = c;
				_rx_buffer_head = head;
			}
			DEBUG_digitalWriteFast(5, LOW);
		}
	}

	// See if we we have a TX event
	if (pflex == _tx_pflex) {
		if ((_tx_pflex->port().SHIFTSTAT & _tx_shifter_mask) && (_tx_pflex->port().SHIFTSIEN & _tx_shifter_mask)) {
			DEBUG_digitalWriteFast(6, HIGH);
			if (_tx_buffer_head != _tx_buffer_tail) {
				_tx_pflex->port().SHIFTBUF[_tx_shifter] = _tx_buffer[_tx_buffer_tail++] ;
				if (_tx_buffer_tail >= TX_BUFFER_SIZE) _tx_buffer_tail = 0;
			}
			if (_tx_buffer_head == _tx_buffer_tail) {
				_tx_pflex->port().SHIFTSIEN &= ~_tx_shifter_mask;  // disable interrupt on this one...
			}
			DEBUG_digitalWriteFast(6, LOW);
		}
	}
	DEBUG_digitalWriteFast(4, LOW);
	return false;  // right now always return false... 
}
