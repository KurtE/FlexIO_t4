/* Teensyduino Core Library
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <Arduino.h>
#include "FlexIO_t4.h"
#ifndef _FLEX_SPI_H_
#define _FLEX_SPI_H_



class FlexSPI : public FlexIOHandlerCallback
{
public:
	enum {TX_BUFFER_SIZE=64, RX_BUFFER_SIZE=40};
	FlexSPI(int mosiPin, int misoPin, int sckPin,  int csPin=-1) :
		_mosiPin(mosiPin), _sckPin(sckPin), _misoPin(misoPin), _csPin(csPin) {};

	~FlexSPI() { end(); }
	bool begin();
	void end(void);

	uint8_t transfer(uint8_t b);	// transfer one byte


	// Call back from flexIO when ISR hapens
	virtual bool call_back (FlexIOHandler *pflex);


private:
	int _mosiPin;
	int _sckPin;
	int _misoPin;
	int _csPin;

	// Variables for tranmitter
	FlexIOHandler  *_pflex = nullptr;
	uint8_t 		_mosi_flex_pin = 0xff;
	uint8_t 		_sck_flex_pin = 0xff;
	uint8_t 		_miso_flex_pin = 0xff;
	uint8_t 		_cs_flex_pin = 0xff;
	uint8_t 		_timer = 0xff;
	uint8_t 		_shifter = 0xff;
	uint8_t 		_shifter_mask;
	
};
#endif //_FLEX_SPI_H_