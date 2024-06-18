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

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#if defined(__IMXRT1062__)
#include "FlexIO_t4.h"

//-----------------------------------------------------------------------------
// FlexIO Hardware structures
//-----------------------------------------------------------------------------
extern void IRQHandler_FlexIO1();
extern void IRQHandler_FlexIO2();
extern void IRQHandler_FlexIO3();

FlexIOHandlerCallback *flex1_Handler_callbacks[FlexIOHandler::CNT_TIMERS] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
FlexIOHandlerCallback *flex2_Handler_callbacks[FlexIOHandler::CNT_TIMERS] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
FlexIOHandlerCallback *flex3_Handler_callbacks[FlexIOHandler::CNT_TIMERS] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

//-----------------------------------------------------------------------------
// Board independant
//-----------------------------------------------------------------------------

// FlexIO1
static const volatile uint32_t *flexio_t4_pins_flexio1[] PROGMEM = {
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_00, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_01, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_02, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_03, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_07, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_09, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_10, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_11, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_26, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_27, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_28, &IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_29};
const FlexIOHandler::FLEXIO_Hardware_t FlexIOHandler::flex1_hardware = {
    CCM_CCGR5, CCM_CCGR5_FLEXIO1(CCM_CCGR_ON),
    IRQ_FLEXIO1, &IRQHandler_FlexIO1,
    0x14, // The Mux setting for this FlexIO
    DMAMUX_SOURCE_FLEXIO1_REQUEST0, DMAMUX_SOURCE_FLEXIO1_REQUEST1, DMAMUX_SOURCE_FLEXIO1_REQUEST2, DMAMUX_SOURCE_FLEXIO1_REQUEST3,
    0xff, 0xff, 0xff, 0xff, // No DMA Sources?
};

// FlexIO2
static const volatile uint32_t * flexio_t4_pins_flexio2[] PROGMEM = {
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_04, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_05, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_06, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_07, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_08, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_09, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11,
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_12, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_13, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_14, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_15, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_02, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_03, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_04, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_05, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_06, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_07, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_08, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_09, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_10, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_11, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_12, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_14, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_15};

const FlexIOHandler::FLEXIO_Hardware_t FlexIOHandler::flex2_hardware = {
    CCM_CCGR3, CCM_CCGR3_FLEXIO2(CCM_CCGR_ON),
    IRQ_FLEXIO2, &IRQHandler_FlexIO2,
    0x14, // The Mux setting for this FlexIO
    DMAMUX_SOURCE_FLEXIO2_REQUEST0, DMAMUX_SOURCE_FLEXIO2_REQUEST1, DMAMUX_SOURCE_FLEXIO2_REQUEST2, DMAMUX_SOURCE_FLEXIO2_REQUEST3,
    0xff, 0xff, 0xff, 0xff // No DMA Sources?
};

// FlexIO3
static const volatile uint32_t * flexio_t4_pins_flexio3[] PROGMEM = {
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_01, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_12, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_13, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_02, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_03, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_04, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_05, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_06, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_07, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_08, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_09, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_10, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_11, 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_12, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_14, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_15};

const FlexIOHandler::FLEXIO_Hardware_t FlexIOHandler::flex3_hardware = {
    CCM_CCGR7, CCM_CCGR7_FLEXIO3(CCM_CCGR_ON),
    IRQ_FLEXIO3, &IRQHandler_FlexIO3,
    0x19,                   // mux setting for all of the pins
    0xff, 0xff, 0xff, 0xff, // No DMA Sources?
    0xff, 0xff, 0xff, 0xff, // No DMA Sources?
};

static FlexIOHandler flexIO1((uintptr_t)&IMXRT_FLEXIO1_S, (uintptr_t)&FlexIOHandler::flex1_hardware, (uintptr_t)flex1_Handler_callbacks,
                             (uintptr_t)flexio_t4_pins_flexio1, sizeof(flexio_t4_pins_flexio1)/sizeof(flexio_t4_pins_flexio1[0]));
static FlexIOHandler flexIO2((uintptr_t)&IMXRT_FLEXIO2_S, (uintptr_t)&FlexIOHandler::flex2_hardware, (uintptr_t)flex2_Handler_callbacks,
                             (uintptr_t)flexio_t4_pins_flexio2, sizeof(flexio_t4_pins_flexio2)/sizeof(flexio_t4_pins_flexio2[0]));
static FlexIOHandler flexIO3((uintptr_t)&IMXRT_FLEXIO3_S, (uintptr_t)&FlexIOHandler::flex3_hardware, (uintptr_t)flex3_Handler_callbacks,
                             (uintptr_t)flexio_t4_pins_flexio3, sizeof(flexio_t4_pins_flexio3)/sizeof(flexio_t4_pins_flexio3[0]));

FlexIOHandler *FlexIOHandler::flexIOHandler_list[] = {&flexIO1, &flexIO2, &flexIO3};

//-----------------------------------------------------------------------------
// Interrupt functions
//-----------------------------------------------------------------------------
void IRQHandler_FlexIO1() {
    FlexIOHandlerCallback **ppfhc = flex1_Handler_callbacks;
    //	Serial.printf("FI1: %x %x %x ", FLEXIO1_SHIFTSTAT, FLEXIO1_SHIFTSIEN, FLEXIO1_SHIFTERR);
    for (uint8_t i = 0; i < FlexIOHandler::CNT_TIMERS; i++) {
        if (*ppfhc) {
            if ((*ppfhc)->call_back(&flexIO1))
                return;
        }
        ppfhc++;
    }
    flexIO1.IRQHandler();
    //	Serial.printf(" %x %x %x\n", FLEXIO1_SHIFTSTAT, FLEXIO1_SHIFTSIEN, FLEXIO1_SHIFTERR);
    asm("dsb");
}

void IRQHandler_FlexIO2() {
    FlexIOHandlerCallback **ppfhc = flex2_Handler_callbacks;
    for (uint8_t i = 0; i < FlexIOHandler::CNT_TIMERS; i++) {
        if (*ppfhc) {
            if ((*ppfhc)->call_back(&flexIO2))
                return;
        }
        ppfhc++;
    }
    flexIO2.IRQHandler();
    asm("dsb");
}

void IRQHandler_FlexIO3() {
    FlexIOHandlerCallback **ppfhc = flex3_Handler_callbacks;
    for (uint8_t i = 0; i < FlexIOHandler::CNT_TIMERS; i++) {
        if (*ppfhc) {
            if ((*ppfhc)->call_back(&flexIO3))
                return;
        }
        ppfhc++;
    }
    flexIO3.IRQHandler();
    asm("dsb");
}

//-----------------------------------------------------------------------------
// Map IO pins to their Flex object and the flex pin
//-----------------------------------------------------------------------------
FlexIOHandler *FlexIOHandler::mapIOPinToFlexIOHandler(uint8_t pin, uint8_t &flex_pin) {
#ifdef DEBUG_FlexIO
	Serial.printf("mapIOPinToFlexIOHandler(%u)\n", pin); Serial.flush();
#endif	
    FlexIOHandler *pflex = nullptr;
    if (pin == 0xff)
        return nullptr;

    for (uint8_t iflex = 0; iflex < (sizeof(flexIOHandler_list) / sizeof(flexIOHandler_list[0])); iflex++) {
        pflex = flexIOHandler_list[iflex];
        flex_pin = pflex->mapIOPinToFlexPin(pin);
        if (flex_pin != 0xff) {
            return pflex;
        }
    }
    return nullptr;
}

//-----------------------------------------------------------------------------
// Map IO pins to their Flex object and the flex pin
//-----------------------------------------------------------------------------
uint8_t FlexIOHandler::mapIOPinToFlexPin(uint8_t pin) {
    // Map the pin to its port config register which we will use to
    // look it up in the table of pins
#ifdef DEBUG_FlexIO
	Serial.printf("FlexIOHandler::mapIOPinToFlexPin(%u, %p)\n", pin, this); Serial.flush();
#endif	
    if (pin >= CORE_NUM_DIGITAL)
        return 0xff; // pin is out of range.

    volatile uint32_t *pin_mux = portConfigRegister(pin);

    // Now lets walk through all of the pins associated with this object.
    const volatile uint32_t **pin_list = pins();

    for (uint8_t i = 0; i < _pin_list_count; i++) {
        if (pin_list[i] == pin_mux) {
#ifdef DEBUG_FlexIO
            Serial.println("Enable flexio clock");
#endif
            hardware().clock_gate_register |= hardware().clock_gate_mask;

            return i;
        }
    }
    return 0xff;
}

//-----------------------------------------------------------------------------
// Map IO pins to their Flex object and the flex pin
//-----------------------------------------------------------------------------
uint8_t FlexIOHandler::mapFlexPinToIOPin(uint8_t flex_pin) {
	if (flex_pin  >= _pin_list_count) return 0xff;

	const volatile uint32_t *flex_pin_port_mux = pins()[flex_pin];


    for (uint8_t pin = 0; pin < CORE_NUM_DIGITAL; pin++) {
    	volatile uint32_t *pin_mux = portConfigRegister(pin);

    	if (pin_mux == flex_pin_port_mux) return pin;
    }
	return 0xff;
}



//-----------------------------------------------------------------------------
// Set an IO pin into Flex Mode
//-----------------------------------------------------------------------------
bool FlexIOHandler::setIOPinToFlexMode(uint8_t pin) {
    // Map the pin to its port config register which we will use to
    // look it up in the table of pins
    if (pin >= CORE_NUM_DIGITAL)
        return false; // pin is out of range.
    volatile uint32_t *pin_mux = portConfigRegister(pin);

    // Now lets walk through all of the pins associated with this object.
    const volatile uint32_t **pin_list = pins();

    for (uint8_t i = 0; i < _pin_list_count; i++) {
        if (pin_list[i] == pin_mux) {
            *(portConfigRegister(pin)) = hardware().io_pin_mux;
            return true;
        }
    }
    return false;
}

// return the index of the ojbect 1:->0 2:->1 later 3:->2
int FlexIOHandler::FlexIOIndex() {
    for (uint8_t iflex = 0; iflex < (sizeof(flexIOHandler_list) / sizeof(flexIOHandler_list[0])); iflex++) {
        if (flexIOHandler_list[iflex] == this)
            return iflex;
    }
    return -1;
}

//-----------------------------------------------------------------------------
// Request and release Timers and Shifters
//-----------------------------------------------------------------------------
// TODO: Get count of timers/shifters/buffers out of object...
// Also handle cnt > 1...
// Currently lets support 1 or 2.
uint8_t FlexIOHandler::requestTimers(uint8_t cnt) {
    uint8_t mask = (cnt == 1) ? 0x1 : 0x3;
    for (uint8_t i = 0; i < (CNT_TIMERS + 1 - cnt); i++) {
        if (!(_used_timers & mask)) {
            _used_timers |= mask;
            return i;
        }
        mask <<= 1;
    }
    return 0xff;
}

uint8_t FlexIOHandler::requestShifter(uint8_t not_dma_channel) {
    uint8_t mask = 0x1;
    for (uint8_t i = 0; i < CNT_SHIFTERS; i++) {
        if (!(_used_shifters & mask)) {
            if ((not_dma_channel == 0xff) || (hardware().shifters_dma_channel[i] != not_dma_channel)) {
                _used_shifters |= mask;
                return i;
            }
        }
        mask <<= 1;
    }
    return 0xff;
}

uint8_t FlexIOHandler::shiftersDMAChannel(uint8_t n) {
    if (n < 4)
        return hardware().shifters_dma_channel[n];
    return 0xff;
}

bool FlexIOHandler::claimTimer(uint8_t timer) {
    uint8_t mask = 1 << timer;
    if (!(_used_timers & mask)) {
        _used_timers |= mask;
        return true;
    }
    return false;
}
bool FlexIOHandler::claimShifter(uint8_t shifter) {
    uint8_t mask = 1 << shifter;
    if (!(_used_shifters & mask)) {
        _used_shifters |= mask;
        return true;
    }
    return false;
}

void FlexIOHandler::IRQHandler() {
}

void FlexIOHandler::freeTimers(uint8_t n, uint8_t cnt) {
    if (n == 0xff)
        return; // don't free if we did not allocate
    uint8_t mask = (cnt == 1) ? 0x1 : 0x3;
    for (; n < 0; n--)
        mask <<= 1;
    _used_timers &= ~mask;
}

void FlexIOHandler::freeShifter(uint8_t n) {
    if (n == 0xff)
        return; // don't free if we did not allocate
    uint8_t mask = 0x1;
    for (; n < 0; n--)
        mask <<= 1;
    _used_shifters &= ~mask;
}

//-----------------------------------------------------------------------------
// Add a call back object to be called when the IRQ is called
//-----------------------------------------------------------------------------
bool FlexIOHandler::addIOHandlerCallback(FlexIOHandlerCallback *callback) {
    FlexIOHandlerCallback **pflex_handler = (FlexIOHandlerCallback **)_callback_list_addr;

    for (uint8_t i = 0; i < CNT_TIMERS; i++) {
        if (*pflex_handler == nullptr) {
            *pflex_handler = callback;
            // See if we need to enable the interrupt...
            if (!_irq_initialized) {
                attachInterruptVector(hardware().flex_irq, hardware().flex_isr);
                // NVIC_SET_PRIORITY(hardware().irq, hardware().irq_priority);	// maybe should put into hardware...
                NVIC_ENABLE_IRQ(hardware().flex_irq);
                _irq_initialized = true;
            }
            return true;
        } else if (*pflex_handler == callback)
            return true; // don't need to add again...
        pflex_handler++; // look at next one.
    }
    return false;
}

//-----------------------------------------------------------------------------
// Remove callbacks
//-----------------------------------------------------------------------------
bool FlexIOHandler::removeIOHandlerCallback(FlexIOHandlerCallback *callback) {
    FlexIOHandlerCallback **pflex_handler = (FlexIOHandlerCallback **)_callback_list_addr;

    for (uint8_t i = 0; i < CNT_TIMERS; i++) {
        if (*pflex_handler == callback) {
            *pflex_handler = nullptr;
            return true;
        }
        pflex_handler++; // look at next one.
    }
    return false;
}

//-----------------------------------------------------------------------------
// Compute the Flex IO clock rate.
//-----------------------------------------------------------------------------

FLASHMEM
static float pll_fractional(uint32_t reg, uint32_t numerator, uint32_t denominator, uint32_t postdiv) {
    float mult = (float)(reg & 127) + (float)numerator / (float)denominator;
    float freq = 24.0e6f * mult;
    uint32_t div = (reg >> 19) & 3;
    if (div == 0)
        freq *= 0.25f;
    else if (div == 1)
        freq *= 0.5f;
    if (postdiv == 1)
        return freq * 0.5f;
    if (postdiv == 3)
        return freq * 0.25f;
    return freq;
}

FLASHMEM
uint32_t FlexIOHandler::computeClockRate() {
    // Todo: add all of this stuff into hardware()...
    uint32_t pll_clock; // = 480000000U;	// Assume PPL3_SEL
    uint32_t misc2;
    uint8_t clk_sel;
    uint8_t clk_pred;
    uint8_t clk_podf;
    if ((IMXRT_FLEXIO_t *)_port_addr == &IMXRT_FLEXIO1_S) {
        // FlexIO1...
        clk_sel = (CCM_CDCDR >> 7) & 0x3;
        clk_pred = (CCM_CDCDR >> 12) & 0x7;
        clk_podf = (CCM_CDCDR >> 9) & 0x7;
    } else {
        // FlexIO2...
        clk_sel = (CCM_CSCMR2 >> 19) & 0x3;
        clk_pred = (CCM_CS1CDR >> 9) & 0x7;
        clk_podf = (CCM_CS1CDR >> 25) & 0x7;
    }
    // TODO - look at the actual clock select
    switch (clk_sel) {
    case 0: // PLL4-Audio
        misc2 = CCM_ANALOG_MISC2;
        pll_clock = pll_fractional(CCM_ANALOG_PLL_AUDIO,
                                   CCM_ANALOG_PLL_AUDIO_NUM, CCM_ANALOG_PLL_AUDIO_DENOM,
                                   ((misc2 >> 22) & 0x02) | ((misc2 >> 15) & 0x01));
        break;
    case 1: // PLL3-PFD2
        pll_clock = 508240000U;
        break;
    case 2: // PLL5-Video
        pll_clock = pll_fractional(CCM_ANALOG_PLL_VIDEO,
                                   CCM_ANALOG_PLL_VIDEO_NUM, CCM_ANALOG_PLL_VIDEO_DENOM,
                                   (CCM_ANALOG_MISC2 >> 30) & 0x03);
        break;
    case 3: // PLL3-USB
    default:
        pll_clock = 480000000U;
        break;
    }
    return pll_clock / (uint32_t)((clk_pred + 1) * (clk_podf + 1));
}

FLASHMEM
bool FlexIOHandler::usesSameClock(const FlexIOHandler *other) {
    const bool this_is_flexio1 = ((IMXRT_FLEXIO_t *)_port_addr == &IMXRT_FLEXIO1_S);
    const bool other_is_flexio1 = ((IMXRT_FLEXIO_t *)(other->_port_addr) == &IMXRT_FLEXIO1_S);
    if (this_is_flexio1 && other_is_flexio1)
        return true;
    if (!this_is_flexio1 && !other_is_flexio1)
        return true; // flexio2 & flexio3 share clock
    return false;
}

//-----------------------------------------------------------------------------
// Try to set clock settings
//-----------------------------------------------------------------------------
FLASHMEM
void FlexIOHandler::setClockSettings(uint8_t clk_sel, uint8_t clk_pred, uint8_t clk_podf) {
    // Todo: add all of this stuff into hardware()...
    // warning this does no validation of the values passed in...
    if (clk_sel == 2) {
        // PLL4 - We may need to enable this clock!
        CCM_ANALOG_PLL_VIDEO_CLR = CCM_ANALOG_PLL_ARM_POWERDOWN | CCM_ANALOG_PLL_ARM_BYPASS;
        CCM_ANALOG_PLL_VIDEO_SET = CCM_ANALOG_PLL_ARM_ENABLE;
#ifdef DEBUG_FlexIO
        // Serial.printf("CCM_ANALOG_PLL_VIDEO: %x\n", CCM_ANALOG_PLL_VIDEO);
#endif
    }
    if ((IMXRT_FLEXIO_t *)_port_addr == &IMXRT_FLEXIO1_S) {
        // FlexIO1...
        // need to turn clock off...
        hardware().clock_gate_register &= ~hardware().clock_gate_mask;

        CCM_CDCDR = (CCM_CDCDR & ~(CCM_CDCDR_FLEXIO1_CLK_SEL(3) | CCM_CDCDR_FLEXIO1_CLK_PRED(7) | CCM_CDCDR_FLEXIO1_CLK_PODF(7))) | CCM_CDCDR_FLEXIO1_CLK_SEL(clk_sel) | CCM_CDCDR_FLEXIO1_CLK_PRED(clk_pred) | CCM_CDCDR_FLEXIO1_CLK_PODF(clk_podf);

        // turn clock back on
        hardware().clock_gate_register |= hardware().clock_gate_mask;
    } else {
        // FlexIO2...
        // need to turn clock off...
        hardware().clock_gate_register &= ~hardware().clock_gate_mask;

        CCM_CSCMR2 = (CCM_CSCMR2 & ~(CCM_CSCMR2_FLEXIO2_CLK_SEL(3))) | CCM_CSCMR2_FLEXIO2_CLK_SEL(clk_sel);
        CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_FLEXIO2_CLK_PRED(7) | CCM_CS1CDR_FLEXIO2_CLK_PODF(7))) | CCM_CS1CDR_FLEXIO2_CLK_PRED(clk_pred) | CCM_CS1CDR_FLEXIO2_CLK_PODF(clk_podf);

        // turn clock back on
        hardware().clock_gate_register |= hardware().clock_gate_mask;
    }
}

// Find the closest pair of FlexIO clock divides to an ideal desired divide ratio.
// Each divide 1-8 is returned in a 4 bit nibble.  Optionally return the error too.
FLASHMEM
static uint8_t best_flexio_div(float ideal_div, float *error) {
    static const uint8_t uniquediv[] = {
        0x88, 0x87, 0x77, 0x86, 0x76, 0x85, 0x66, 0x75,
        0x84, 0x65, 0x74, 0x55, 0x83, 0x73, 0x54, 0x63,
        0x82, 0x53, 0x72, 0x62, 0x52, 0x33, 0x81, 0x71,
        0x61, 0x51, 0x41, 0x31, 0x21, 0x11};
    unsigned int best_index = 0;
    float best_error = 1e100 /*FLT_MAX*/;
    for (unsigned int i = 0; i < sizeof(uniquediv); i++) {
        float actual_div = ((uniquediv[i] >> 4) & 15) * (uniquediv[i] & 15);
        float actual_error = fabsf(actual_div - ideal_div);
        if (actual_error < best_error) {
            best_error = actual_error;
            best_index = i;
        }
    }
    if (error)
        *error = best_error;
    return uniquediv[best_index];
}

FLASHMEM
float FlexIOHandler::setClock(float frequency) {
    // Serial.printf("setClock frequency =  %.0f\n", frequency);
    float error480, error508;
    uint8_t div480 = best_flexio_div(480.0e6f / frequency, &error480);
    uint8_t div508 = best_flexio_div(508.24e6f / frequency, &error508);
    // Serial.printf("div480 is %02X, error %.2f\n", div480, error480);
    // Serial.printf("div508 is %02X, error %.2f\n", div508, error508);
    unsigned int div1, div2;
    if (error480 <= error508) {
        div1 = (div480 >> 4) & 15;
        div2 = div480 & 15;
        setClockSettings(3, div1 - 1, div2 - 1);
        frequency = 480.0e6f / (div1 * div2);
    } else {
        div1 = (div508 >> 4) & 15;
        div2 = div508 & 15;
        setClockSettings(1, div1 - 1, div2 - 1);
        frequency = 508.24e6f / (div1 * div2);
    }
    // Serial.printf("actual frequency =  %.0f\n", frequency);
    return frequency;
}

FLASHMEM
static float best_pll_config(float frequency, uint8_t &pll_multiply, uint32_t &pll_numerator,
                             uint32_t &pll_denominator, uint8_t &pll_divs, uint8_t &flexio_divs) {
    // First choose the PLL's post divide.  Higher final frequency
    // requires the PLL to output a higher range.  Choose the
    // slowest (most divided down within the PLL) workable range.
    const float PLL_MIN = 650000000; // 650 MHz
    if (frequency > PLL_MIN / 4) {
        pll_divs = 0x21;
    } else if (frequency > PLL_MIN / 8) {
        pll_divs = 0x41;
    } else if (frequency > PLL_MIN / 16) {
        pll_divs = 0x42;
    } else {
        pll_divs = 0x44;
    }
    unsigned int pll_div = (pll_divs >> 4) * (pll_divs & 15);
    // Serial.printf(" pll_div = %u\n", pll_div);

    // Compute the "ideal" FlexIO divider which would operate the
    // PLL exactly at the center of its usable range (650 MHz to 1.3 GHz)
    float ideal_flexio_div = 975.0e6f / frequency / pll_div;
    // Serial.printf(" ideal_flexio_div = %.2f\n", ideal_flexio_div);

    // Find the closest FlexIO divider and compute the needed PLL frequency
    flexio_divs = best_flexio_div(ideal_flexio_div, NULL);
    unsigned int actual_flexio_div = ((flexio_divs >> 4) & 15) * (flexio_divs & 15);
    // Serial.printf(" actual_flexio_div = %u\n", actual_flexio_div);
    float pll_freq = frequency * pll_div * actual_flexio_div;
    // Serial.printf(" pll_freq = %.0f\n", pll_freq);

    // Compute the PLL configuration to achieve this frequency
    float pll_mult = pll_freq / 24.0e6f;
    if (pll_mult < 27.083f)
        pll_mult = 27.083f;
    if (pll_mult > 54.167f)
        pll_mult = 54.167f;
    // Serial.printf(" pll_mult = %.3f\n", pll_mult);
    float pll_mult_integer, pll_mult_fraction;
    pll_mult_fraction = modff(pll_mult, &pll_mult_integer);
    pll_multiply = pll_mult_integer;
    pll_denominator = 0x1FFFFFFF;
    pll_numerator = pll_denominator * pll_mult_fraction;
    // Serial.printf(" pll_multiply = %u\n", pll_multiply);
    // Serial.printf(" pll_numerator = %08X\n", pll_numerator);
    // Serial.printf(" pll_denominator = %08X\n", pll_denominator);

    // Return the actual frequency using all these settings
    frequency = 24.0e6f * ((float)pll_multiply + (float)pll_numerator / (float)pll_denominator) / (float)(pll_div * actual_flexio_div);
    // Serial.printf(" actual frequency = %.0f\n", frequency);
    return frequency;
}

FLASHMEM
float FlexIOHandler::setClockUsingAudioPLL(float frequency) {
    // Serial.printf("setClockUsingVideoPLL: freq = %.0f\n", frequency);
    uint8_t pll_multiply;
    uint32_t pll_numerator, pll_denominator;
    uint8_t pll_divs, flexio_divs;
    frequency = best_pll_config(frequency, pll_multiply, pll_numerator,
                                pll_denominator, pll_divs, flexio_divs);
    // TODO: handle PLL already running?
    uint8_t post_div_select = 0; // 0 means div by 4 (ref manual rev 3, page 1109)
    if ((pll_divs >> 4) == 2)
        post_div_select = 1; // 1 means div by 2
    CCM_ANALOG_PLL_AUDIO = CCM_ANALOG_PLL_AUDIO_DIV_SELECT(pll_multiply) |
                           CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(post_div_select) |
                           CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC(0);
    CCM_ANALOG_PLL_AUDIO_NUM = pll_numerator;
    CCM_ANALOG_PLL_AUDIO_DENOM = pll_denominator;
    bool msb = false, lsb = false; // 0 means div by 1 (ref manual rev 3, page 1128)
    if ((pll_divs & 15) == 2)
        lsb = true; // 1 means div by 2
    if ((pll_divs & 15) == 4)
        msb = lsb = true; // 3 means div by 4
    CCM_ANALOG_MISC2_CLR = CCM_ANALOG_MISC2_AUDIO_DIV_MSB | CCM_ANALOG_MISC2_AUDIO_DIV_LSB;
    CCM_ANALOG_MISC2_SET = (msb ? CCM_ANALOG_MISC2_AUDIO_DIV_MSB : 0) |
                           (lsb ? CCM_ANALOG_MISC2_AUDIO_DIV_LSB : 0);
    CCM_ANALOG_PLL_AUDIO_SET = CCM_ANALOG_PLL_AUDIO_ENABLE;
    setClockSettings(0, (flexio_divs >> 4) - 1, (flexio_divs & 15) - 1);
    return frequency;
}

FLASHMEM
float FlexIOHandler::setClockUsingVideoPLL(float frequency) {
    // Serial.printf("setClockUsingVideoPLL: freq = %.0f\n", frequency);
    uint8_t pll_multiply;
    uint32_t pll_numerator, pll_denominator;
    uint8_t pll_divs, flexio_divs;
    frequency = best_pll_config(frequency, pll_multiply, pll_numerator,
                                pll_denominator, pll_divs, flexio_divs);
    // TODO: handle PLL already running?
    uint8_t post_div_select = 0; // 0 means div by 4 (ref manual rev 3, page 1109)
    if ((pll_divs >> 4) == 2)
        post_div_select = 1; // 1 means div by 2
    CCM_ANALOG_PLL_VIDEO = CCM_ANALOG_PLL_VIDEO_DIV_SELECT(pll_multiply) |
                           CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(post_div_select) |
                           CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC(0);
    CCM_ANALOG_PLL_VIDEO_NUM = pll_numerator;
    CCM_ANALOG_PLL_VIDEO_DENOM = pll_denominator;
    uint8_t video_div = 0; // 0 means div by 1 (ref manual rev 3, page 1127)
    if ((pll_divs & 15) == 2)
        video_div = 1; // 1 means div by 2
    if ((pll_divs & 15) == 4)
        video_div = 3; // 3 means div by 4
    CCM_ANALOG_MISC2_CLR = CCM_ANALOG_MISC2_VIDEO_DIV(3);
    CCM_ANALOG_MISC2_SET = CCM_ANALOG_MISC2_VIDEO_DIV(video_div);
    CCM_ANALOG_PLL_VIDEO_SET = CCM_ANALOG_PLL_VIDEO_ENABLE;
    setClockSettings(2, (flexio_divs >> 4) - 1, (flexio_divs & 15) - 1);
    return frequency;
}

#endif
