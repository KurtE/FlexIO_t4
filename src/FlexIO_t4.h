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

// #define DEBUG_FlexIO

#ifndef _FLEX_IO_T4_H_
#define _FLEX_IO_T4_H_
#include <Arduino.h>
#if !defined(__IMXRT1062__)
#error "Sorry, Flex IO only works on Teensy 4.x boards"
#endif

#define FLEX_IO_HAS_FULL_PIN_MAPPING

// forward reference
class FlexIOHandler;

class FlexIOHandlerCallback {
  public:
    virtual bool call_back(FlexIOHandler *pflex) = 0;
};

// Note: T4.x Param data does not match RM.
// PARAM:2200808
class FlexIOHandler {
  public:
    static const uint8_t CNT_SHIFTERS = 8;
    static const uint8_t CNT_TIMERS = 8;
    static const uint8_t CNT_FLEX_IO_OBJECT = 3;

    typedef struct {
        volatile uint32_t &clock_gate_register;
        const uint32_t clock_gate_mask;
        const IRQ_NUMBER_t flex_irq;
        void (*flex_isr)();
        const uint8_t io_pin_mux; // The pin mux is the same for all of the pins
        const uint8_t shifters_dma_channel[CNT_SHIFTERS];
    } FLEXIO_Hardware_t;

    static const FLEXIO_Hardware_t flex1_hardware;
    static const FLEXIO_Hardware_t flex2_hardware;
    static const FLEXIO_Hardware_t flex3_hardware;

    constexpr FlexIOHandler(uintptr_t myport, uintptr_t myhardware, uintptr_t callback_list, uintptr_t pins_list, uint8_t pin_list_count)
        : _port_addr(myport), _hardware_addr(myhardware), _callback_list_addr(callback_list), _pins_addr(pins_list),  _pin_list_count(pin_list_count) {
    }

    // A static one that can map across all FlexIO controller
    static FlexIOHandler *mapIOPinToFlexIOHandler(uint8_t pin, uint8_t &flex_pin);
    static FlexIOHandler *flexIOHandler_list[3];

    // A simple one that maps within a controller
    uint8_t mapIOPinToFlexPin(uint8_t);

    // Look for IO pin that maps tos specific FlexIO pin
    uint8_t mapFlexPinToIOPin(uint8_t flex_pin);

    IMXRT_FLEXIO_t &port() { return *(IMXRT_FLEXIO_t *)_port_addr; }
    const FLEXIO_Hardware_t &hardware() { return *(const FLEXIO_Hardware_t *)_hardware_addr; }
    const volatile uint32_t **pins() { return (const volatile uint32_t **)_pins_addr; }
    uint8_t pinCount() {return _pin_list_count;}
    int FlexIOIndex(); // return the index of the ojbect 1:->0 2:->1 later 3:->2

    uint8_t requestTimers(uint8_t cnt = 1);
    uint8_t requestShifter(uint8_t not_dma_channel = 0xff);
    uint8_t shiftersDMAChannel(uint8_t n);
    bool claimTimer(uint8_t timer);
    bool claimShifter(uint8_t shifter);

    void freeTimers(uint8_t n, uint8_t cnt = 1);
    void freeShifter(uint8_t n);

    bool setIOPinToFlexMode(uint8_t pin);
    bool addIOHandlerCallback(FlexIOHandlerCallback *callback);
    bool removeIOHandlerCallback(FlexIOHandlerCallback *callback);

    uint32_t computeClockRate();
    bool usesSameClock(const FlexIOHandler *other);

    void setClockSettings(uint8_t clk_sel, uint8_t clk_pred, uint8_t clk_podf);

    float setClock(float frequency);
    float setClockUsingAudioPLL(float frequency);
    float setClockUsingVideoPLL(float frequency);

    void IRQHandler(void);

  protected:
    uintptr_t _port_addr;
    uintptr_t _hardware_addr;
    uintptr_t _callback_list_addr;
    uintptr_t _pins_addr;
    uint8_t _pin_list_count;
    uint8_t _used_timers = 0;
    uint8_t _used_shifters = 0;
    bool _irq_initialized = false;
};

#endif //_FLEX_IO_T4_H_
