#include "flexSPI.h"
#define BAUDRATE 115200
#define FLEXIO1_CLOCK (480000000L/16) // Again assuming default clocks?

#define DEBUG_FlexSPI
#define DEBUG_digitalWriteFast(pin, state) digitalWriteFast(pin, state)
//#define DEBUG_digitalWriteFast(pin, state) 

//=============================================================================
// FlexSPI::Begin
//=============================================================================
bool FlexSPI::begin() {
	// BUGBUG - may need to actual Clocks to computer baud...
//	uint16_t baud_div =  (FLEXIO1_CLOCK/baud)/2 - 1;                                   
	//-------------------------------------------------------------------------
	// Make sure all of the IO pins are valid flex pins on same controller
	//-------------------------------------------------------------------------
	uint8_t 		_mosi_flex_pin = 0xff;
	uint8_t 		_sck_flex_pin = 0xff;
	uint8_t 		_miso_flex_pin = 0xff;
	uint8_t 		_cs_flex_pin = 0xff;

	_pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_mosiPin, _mosi_flex_pin);
	FlexIOHandler *sck_flex = FlexIOHandler::mapIOPinToFlexIOHandler(_sckPin, _sck_flex_pin);
	FlexIOHandler *miso_flex = FlexIOHandler::mapIOPinToFlexIOHandler(_misoPin, _miso_flex_pin);
	FlexIOHandler *cs_flex = FlexIOHandler::mapIOPinToFlexIOHandler(_csPin, _cs_flex_pin);

	// Make sure all of the pins map to same Flex IO controller
	if (!_pflex  || (_pflex != sck_flex) || (_pflex != miso_flex) || (_pflex != cs_flex)) {
		Serial.printf("FlexSPI - not all pins mapped to same Flex controller\n");
		return false;
	}

	// Now reserve timers and shifters
	IMXRT_FLEXIO_t *p = &_pflex->port();

	_timer = _pflex->requestTimers(2);
	_shifter = _pflex->requestShifters(2);

	if ((_timer == 0xff) || (_shifter == 0xff)) {
		_pflex->freeTimers(_timer, 2);
		_timer = 0xff;
		_pflex->freeShifters(_shifter, 2);
		_shifter = 0xff;
		Serial.println("FlexSPI - Failed to allocate timers or shifters");
		return false;
	}

	_shifter_mask = 1;
	for (uint8_t i = _shifter; i > 0; i--) _shifter_mask <<= 1;

#ifdef DEBUG_FlexSPI
	Serial.printf("timer index: %d shifter index: %d mask: %x\n", _timer, _shifter, _shifter_mask);
	// lets try to configure a tranmitter like example
	Serial.println("Before configure flexio");
#endif
	p->SHIFTCFG[_shifter] = 0; // Start/stop disabled;
	p->SHIFTCTL[_shifter] = FLEXIO_SHIFTCTL_TIMPOL | FLEXIO_SHIFTCTL_PINCFG(3) | FLEXIO_SHIFTCTL_SMOD(2) |
	                              FLEXIO_SHIFTCTL_TIMSEL(_timer) | FLEXIO_SHIFTCTL_PINSEL(_mosi_flex_pin); // 0x0003_0002;
	p->SHIFTCFG[_shifter+1] = 0; // Start/stop disabled;
	p->SHIFTCTL[_shifter+1] =  FLEXIO_SHIFTCTL_SMOD(1) |
	                              FLEXIO_SHIFTCTL_TIMSEL(_timer) | FLEXIO_SHIFTCTL_PINSEL(_miso_flex_pin); // 0x0003_0002;

	p->TIMCMP[_timer] = 0x0f01; // (8 bits?)0x3f01; // ???0xf00 | baud_div; //0xF01; //0x0000_0F01;		//

	p->TIMCFG[_timer] = FLEXIO_TIMCFG_TIMOUT(1) | FLEXIO_TIMCFG_TSTART | FLEXIO_TIMCFG_TSTOP(2) |
	                          FLEXIO_TIMCFG_TIMENA(2) |  FLEXIO_TIMCFG_TIMDIS(2); //0x0100_2222;
	p->TIMCTL[_timer] =  FLEXIO_TIMCTL_TRGSEL(1) | FLEXIO_TIMCTL_TRGPOL | FLEXIO_TIMCTL_TRGSRC 
					| FLEXIO_TIMCTL_PINCFG(3) | FLEXIO_TIMCTL_PINSEL(_sck_flex_pin)| FLEXIO_TIMCTL_TIMOD(1);  // 0x01C0_0001;

	p->TIMCMP[_timer+1] = 0xffff; // never compare
						// 0x0000_1100 enable/desale with clock(n-1)
	p->TIMCFG[_timer+1] = FLEXIO_TIMCFG_TIMDIS(1) | FLEXIO_TIMCFG_TIMENA(1); // 0x0000_1100
					// 
	p->TIMCTL[_timer+1] =  FLEXIO_TIMCTL_PINCFG(3) | FLEXIO_TIMCTL_PINSEL(_cs_flex_pin) | 
					FLEXIO_TIMCTL_PINPOL | FLEXIO_TIMCTL_TIMOD(3);  // 0003_0383;

	// Make sure this flex IO object is enabled				
	p->CTRL = FLEXIO_CTRL_FLEXEN;
	//p->SHIFTSTAT = _shifter_mask;   // Clear out the status.

	// Set the IO pins into FLEXIO mode
	_pflex->setIOPinToFlexMode(_mosiPin);
	_pflex->setIOPinToFlexMode(_sckPin);
	_pflex->setIOPinToFlexMode(_misoPin);
	_pflex->setIOPinToFlexMode(_csPin);


	_pflex->addIOHandlerCallback(this);
	// Lets print out some of the settings and the like to get idea of state
#ifdef DEBUG_FlexSPI
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

	return true;
}

void FlexSPI::end(void) {
	// If the transmit was allocated free it now as well as timers and shifters.
	if (_pflex) {
		_pflex->freeTimers(_timer, 2);
		_timer = 0xff;
		_pflex->freeShifters(_shifter, 2);
		_shifter = 0xff;

		_pflex->removeIOHandlerCallback(this);
		_pflex = nullptr;	
	}

}

uint8_t FlexSPI::transfer(uint8_t b) 
{
	// Need to do some validation...
	uint8_t return_val = 0xff;
	_pflex->port().SHIFTBUFBBS[_shifter] = b;	// try putting out a byte.

	// Now lets wait for something to come back.
	uint8_t rx_shifter_mask = _shifter_mask << 1;	// n+1 port
	uint16_t timeout = 0xffff;	// don't completely hang
	while (!(_pflex->port().SHIFTSTAT & rx_shifter_mask) && (--timeout)) ;

	if (_pflex->port().SHIFTSTAT & rx_shifter_mask) return_val = _pflex->port().SHIFTBUFBIS[_shifter+1] & 0xff;

	return return_val;
}

bool FlexSPI::call_back (FlexIOHandler *pflex) {
//	DEBUG_digitalWriteFast(4, HIGH);
	return false;  // right now always return false... 
}
