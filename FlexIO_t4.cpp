#include "FlexIO_t4.h"

#define IMXRT_FLEXIO1_S		(*(IMXRT_FLEXIO_t *)0x401AC000)
#define IMXRT_FLEXIO2_S		(*(IMXRT_FLEXIO_t *)0x401B0000)
//#define IMXRT_FLEXIO3		(*(IMXRT_REGISTER32_t *)0x42020000) only RT1062


extern void IRQHandler_FlexIO1();
extern void IRQHandler_FlexIO2();

FlexIOHandlerCallback *flex1_Handler_callbacks[4] = {nullptr, nullptr, nullptr, nullptr};
FlexIOHandlerCallback *flex2_Handler_callbacks[4] = {nullptr, nullptr, nullptr, nullptr};

const FlexIOHandler::FLEXIO_Hardware_t FlexIOHandler::flex1_hardware = {
	CCM_CCGR5, CCM_CCGR5_FLEXIO1(CCM_CCGR_ON),
	IRQ_FLEXIO1, 
	&IRQHandler_FlexIO1,
	2,       3,    4,    5,  33,  0xff, 0xff, 0xff, 0xff,
	4,       5,    6,    7,  8,   0xff, 0xff, 0xff, 0xff,
	0x14, 0x14, 0x14, 0x14, 0x14, 0xff, 0xff, 0xff, 0xff,
};

const FlexIOHandler::FLEXIO_Hardware_t FlexIOHandler::flex2_hardware = {
	CCM_CCGR3, CCM_CCGR3_FLEXIO2(CCM_CCGR_ON),
	IRQ_FLEXIO2, 
	IRQHandler_FlexIO2,
	6,       7,    8,    9,  10,    11,   12,   13,   32,
	17,     16,   10,   11,  0,      2,    1,    3,   12,
	0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
};


static FlexIOHandler flexIO1((uintptr_t)&IMXRT_FLEXIO1_S, (uintptr_t)&FlexIOHandler::flex1_hardware, (uintptr_t)flex1_Handler_callbacks);
static FlexIOHandler flexIO2((uintptr_t)&IMXRT_FLEXIO2_S, (uintptr_t)&FlexIOHandler::flex2_hardware, (uintptr_t)flex2_Handler_callbacks);

static FlexIOHandler *flex_list[] = {&flexIO1, &flexIO2};

void IRQHandler_FlexIO1() {
	FlexIOHandlerCallback **ppfhc = flex1_Handler_callbacks;
	for (uint8_t i = 0; i < 4; i++) {
		if (*ppfhc) {
			if ((*ppfhc)->call_back(&flexIO1)) return;
		}
		ppfhc++;
	}
	flexIO1.IRQHandler();
	 asm("dsb");
}


void IRQHandler_FlexIO2() {
	FlexIOHandlerCallback **ppfhc = flex2_Handler_callbacks;
	for (uint8_t i = 0; i < 4; i++) {
		if (*ppfhc) {
			if ((*ppfhc)->call_back(&flexIO1)) return;
		}
	}
	flexIO2.IRQHandler();
	 asm("dsb");
}



FlexIOHandler *FlexIOHandler::mapIOPinToFlexIOHandler(uint8_t pin, uint8_t &flex_pin)
{
  FlexIOHandler *pflex = nullptr;

  for (uint8_t iflex = 0; iflex < (sizeof(flex_list) / sizeof(flex_list[0])); iflex++) {
    pflex = flex_list[iflex];

    for (uint8_t i = 0; i < CNT_FLEX_PINS; i++ ) {
      if (pflex->hardware().io_pin[i] == pin) {
        flex_pin = pflex->hardware().flex_pin[i];
#ifdef DEBUG_FlexIO
  		Serial.println("Enable flexio clock");
#endif
  		pflex->hardware().clock_gate_register |= pflex->hardware().clock_gate_mask;
        return pflex;
      }
    }
  }
  flex_pin = 0xff;
  return nullptr;
}

bool FlexIOHandler::setIOPinToFlexMode(uint8_t pin) {
	for (uint8_t i = 0; i < CNT_FLEX_PINS; i++ ) {
		if (hardware().io_pin[i] == pin) {
			  *(portConfigRegister(pin)) = hardware().io_pin_mux[i];
			  return true;
		}
	}
	return false;
}


// TODO: Get count of timers/shifters/buffers out of object...
// Also handle cnt > 1...
// Currently lets support 1 or 2.
uint8_t FlexIOHandler::requestTimers(uint8_t cnt) {
  uint8_t mask = (cnt == 1)? 0x1 : 0x3;
  for (uint8_t i = 0; i < (5-cnt); i++) {
    if (!(_used_timers & mask)) {
      _used_timers |= mask;
      return i;
    }
    mask <<= 1;
  }
  return 0xff;
}

uint8_t FlexIOHandler::requestShifters(uint8_t cnt) {
  uint8_t mask = (cnt == 1)? 0x1 : 0x3;
  for (uint8_t i = 0; i < (5-cnt); i++) {
    if (!(_used_shifters & mask)) {
      _used_shifters |= mask;
      return i;
    }
    mask <<= 1;
  }
  return 0xff;
}

void FlexIOHandler::IRQHandler() {
  
}

void FlexIOHandler::freeTimers(uint8_t n, uint8_t cnt) {
  uint8_t mask = (cnt == 1)? 0x1 : 0x3;
  for (;n < 0; n--) mask <<= 1;
  _used_timers &= ~mask;
}

void FlexIOHandler::freeShifters(uint8_t n, uint8_t cnt) {
  uint8_t mask = (cnt == 1)? 0x1 : 0x3;
  for (;n < 0; n--) mask <<= 1;
  _used_shifters &= ~mask;
}

bool FlexIOHandler::addIOHandlerCallback(FlexIOHandlerCallback *callback) {
	FlexIOHandlerCallback **pflex_handler = (FlexIOHandlerCallback**)_callback_list_addr;

	for (uint8_t i=0; i < 4; i++) {
		if (*pflex_handler == nullptr) {
			*pflex_handler = callback;
			// See if we need to enable the interrupt...
			if (!_irq_initialized) {
				attachInterruptVector(hardware().flex_irq, hardware().flex_isr);
				//NVIC_SET_PRIORITY(hardware().irq, hardware().irq_priority);	// maybe should put into hardware...
				NVIC_ENABLE_IRQ(hardware().flex_irq);
				_irq_initialized = true;
			}
			return true;
		}
		else if (*pflex_handler == callback) return true;	// don't need to add again... 
		pflex_handler++;	// look at next one. 
	}
	return false;
}
bool FlexIOHandler::removeIOHandlerCallback(FlexIOHandlerCallback *callback) {
	FlexIOHandlerCallback **pflex_handler = (FlexIOHandlerCallback**)_callback_list_addr;

	for (uint8_t i=0; i < 4; i++) {
		if (*pflex_handler == callback) {
			*pflex_handler = nullptr;
			return true;
		}
		pflex_handler++;	// look at next one. 
	}
	return false;
}
