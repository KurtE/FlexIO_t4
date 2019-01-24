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

//#define DEBUG_FlexIO

#ifndef _FLEX_IO_T4_H_
#define _FLEX_IO_T4_H_
#include <Arduino.h>
#if !defined(__IMXRT1052__) && !defined(__IMXRT1062__)
#error "Sorry, Flex IO only works on Teensy 4.x boards"
#endif
typedef struct {
	const 	uint32_t VERID;				// 0x00	(IMXRT_FLEXIO1.offset000)
	volatile uint32_t PARAM;			// 0x04	// (IMXRT_FLEXIO1.offset004)
	volatile uint32_t CTRL;				// 0x08(IMXRT_FLEXIO1.offset008)
	volatile uint32_t PIN;				// 0x0c (IMXRT_FLEXIO1.offset00C)
	volatile uint32_t SHIFTSTAT;		// 0x10 (IMXRT_FLEXIO1.offset010)
	volatile uint32_t SHIFTERR;			// 0x14(IMXRT_FLEXIO1.offset014)
	volatile uint32_t TIMSTAT;			// 0x18 (IMXRT_FLEXIO1.offset018)
	const	uint32_t UNUSED0;			// 0x1c
	volatile uint32_t SHIFTSIEN;		// 0x20 (IMXRT_FLEXIO1.offset020)
	volatile uint32_t SHIFTEIEN;		// 0x24 (IMXRT_FLEXIO1.offset024)
	volatile uint32_t TIMIEN;			// 0x28 (IMXRT_FLEXIO1.offset028)
	const	uint32_t UNUSED1;			// 0x2c
	volatile uint32_t SHIFTSDEN;		// 0x30 (IMXRT_FLEXIO1.offset030)
	const	uint32_t UNUSED2[3];		// 0x34 38 3C
	volatile uint32_t SHIFTSTATE;		// 0x40 (IMXRT_FLEXIO1.offset040)
	const	uint32_t UNUSED3[15];		// 0x44..  50... 60... 70...
	volatile uint32_t SHIFTCTL[4];		// 0x80 84 88 8C
	const	uint32_t UNUSED4[28];		// 0x90 - 0xfc
	volatile uint32_t SHIFTCFG[4];		// 0x100 104 108 10C (IMXRT_FLEXIO1.offset100)
	const	uint32_t UNUSED5[60];		// 0x110 - 0x1FC
	volatile uint32_t SHIFTBUF[4];		// 0x200 204 208 20c (IMXRT_FLEXIO1.offset200)
	const	uint32_t UNUSED6[28];		// 
	volatile uint32_t SHIFTBUFBIS[4];	// 0x280	// (IMXRT_FLEXIO1.offset280)
	const	uint32_t UNUSED7[28];		// 
	volatile uint32_t SHIFTBUFBYS[4];	// 0x300 (IMXRT_FLEXIO1.offset300)
	const	uint32_t UNUSED8[28];		// 
	volatile uint32_t SHIFTBUFBBS[4];	// 0x380 (IMXRT_FLEXIO1.offset380)
	const	uint32_t UNUSED9[28];		// 
	volatile uint32_t TIMCTL[4];		// 0x400 
	const	uint32_t UNUSED10[28];		// 
	volatile uint32_t TIMCFG[4];		// 0x480
	const	uint32_t UNUSED11[28];		// 
	volatile uint32_t TIMCMP[4];		// 0x500
	const	uint32_t UNUSED12[28+64];	// 
	volatile uint32_t SHIFTBUFNBS[4];	// 0x680
	const	uint32_t UNUSED13[28];		// 
	volatile uint32_t SHIFTBUFHWS[4];	// 0x700
	const	uint32_t UNUSED14[28];		// 
	volatile uint32_t SHIFTBUFNIS[4];	// 0x780
} IMXRT_FLEXIO_t;


// forward reference
class FlexIOHandler; 

class FlexIOHandlerCallback {
public:
	virtual bool call_back (FlexIOHandler *pflex) = 0;
};

class FlexIOHandler {
public:
	static const uint8_t CNT_FLEX_PINS = 9;
	typedef struct {
		volatile uint32_t &clock_gate_register;
		const uint32_t clock_gate_mask;
		const IRQ_NUMBER_t  flex_irq;
		void    (*flex_isr)();
		const uint8_t  io_pin[CNT_FLEX_PINS];
		const uint8_t  flex_pin[CNT_FLEX_PINS];
		const uint8_t  io_pin_mux[CNT_FLEX_PINS];
	} FLEXIO_Hardware_t;

	static const FLEXIO_Hardware_t flex1_hardware;
	static const FLEXIO_Hardware_t flex2_hardware;

  constexpr FlexIOHandler(uintptr_t myport, uintptr_t myhardware, uintptr_t callback_list)
  	: port_addr(myport), hardware_addr(myhardware), _callback_list_addr(callback_list) {
  }
  static FlexIOHandler *mapIOPinToFlexIOHandler(uint8_t pin, uint8_t &flex_pin);

	IMXRT_FLEXIO_t & port() { return *(IMXRT_FLEXIO_t *)port_addr; }
	const FLEXIO_Hardware_t & hardware() { return *(const FLEXIO_Hardware_t *)hardware_addr; }

	uint8_t requestTimers(uint8_t cnt=1);
	uint8_t requestShifters(uint8_t cnt=1);
	uint8_t requestBuffers(uint8_t cnt=1);

	void freeTimers(uint8_t n, uint8_t cnt=1);
	void freeShifters(uint8_t n, uint8_t cnt=1);
	void freeBuffers(uint8_t n, uint8_t cnt=1);

	bool setIOPinToFlexMode(uint8_t pin);
	bool addIOHandlerCallback(FlexIOHandlerCallback *callback);
	bool removeIOHandlerCallback(FlexIOHandlerCallback *callback);

	void IRQHandler(void);

protected: 
	uintptr_t 		port_addr;
	uintptr_t		 hardware_addr;
	uintptr_t		_callback_list_addr;
	uint8_t         _used_timers = 0;
	uint8_t         _used_shifters = 0;
	uint8_t         _used_buffers = 0;
	bool			_irq_initialized = false;
  
};

#endif //_FLEX_IO_T4_H_