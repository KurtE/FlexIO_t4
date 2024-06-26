#include "FlexIOSPI.h"
#define BAUDRATE 115200
#define FLEXIO1_CLOCK (480000000L / 16) // Again assuming default clocks?

// #define DEBUG_FlexSPI Serial
#define DEBUG_digitalWriteFast(pin, state) digitalWriteFast(pin, state)
// #define DEBUG_digitalWriteFast(pin, state)
FlexIOSPI *FlexIOSPI::_dmaActiveObjects[FlexIOHandler::CNT_FLEX_IO_OBJECT] = {nullptr, nullptr};

//=============================================================================
// FlexIOSPI::Begin
//=============================================================================
bool FlexIOSPI::begin() {
    // BUGBUG - may need to actual Clocks to computer baud...
    //	uint16_t baud_div =  (FLEXIO1_CLOCK/baud)/2 - 1;
    //-------------------------------------------------------------------------
    // Make sure all of the IO pins are valid flex pins on same controller
    //-------------------------------------------------------------------------
    _pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_mosiPin, _mosi_flex_pin);
    if (!_pflex) {
#ifdef DEBUG_FlexSPI
        Serial.printf("FlexIOSPI - Mosi pin does not map to flex controller\n");
#endif
        return false;
    }
    // Serial.printf("FlexIOSPI Begin: Mosi map %d %x %d\n", _mosiPin, (uint32_t)_pflex, _mosi_flex_pin);
    //  Lets try mapping the others to this one.
    _sck_flex_pin = _pflex->mapIOPinToFlexPin(_sckPin);
    _miso_flex_pin = _pflex->mapIOPinToFlexPin(_misoPin);

    // Lets see if they all mapped to same to same controller.
    if ((_sck_flex_pin == 0xff) || (_miso_flex_pin == 0xff)) {
#if defined(__IMXRT1062__)
        // Note this par
        if (_sck_flex_pin == 0xff) {
            _pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_sck_flex_pin, _sck_flex_pin);
        } else {
            _pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_miso_flex_pin, _miso_flex_pin);
        }
        if (!_pflex) {
#ifdef DEBUG_FlexSPI
            Serial.printf("FlexIOSPI - not all pins mapped to same Flex controller\n");
#endif
            return false;
        }

        _mosi_flex_pin = _pflex->mapIOPinToFlexPin(_mosiPin);
        _sck_flex_pin = _pflex->mapIOPinToFlexPin(_sckPin);
        _miso_flex_pin = _pflex->mapIOPinToFlexPin(_misoPin);
        if ((_sck_flex_pin == 0xff) || (_miso_flex_pin == 0xff) || (_mosi_flex_pin == 0xff)) {
#ifdef DEBUG_FlexSPI
            Serial.printf("FlexIOSPI - not all pins mapped to same Flex controller\n");
#endif
            return false;
        }
#else
        // 1052 don't have pins that map to different FLEXIO controllers
#ifdef DEBUG_FlexSPI
        Serial.printf("FlexIOSPI - not all pins mapped to same Flex controller\n");
#endif
        return false;
#endif
    }
    // FlexIOHandler *cs_flex = _pflex;
    if (_csPin != -1) {
        _cs_flex_pin = _pflex->mapIOPinToFlexPin(_csPin);
        if (_cs_flex_pin == 0xff) {
#ifdef DEBUG_FlexSPI
            Serial.printf("FlexIOSPI - not all pins(CS) mapped to same Flex controller\n");
#endif
            return false;
        }
    }

    // Now reserve timers and shifters
    IMXRT_FLEXIO_t *p = &_pflex->port();

    _timer = _pflex->requestTimers((_csPin != -1) ? 2 : 1);
    _tx_shifter = _pflex->requestShifter();
    _rx_shifter = _pflex->requestShifter(_pflex->shiftersDMAChannel(_tx_shifter));

    // If first request failed to get second different shifter on different dma channel, allocate other one on same channel
    // but DMA will not work...
    if (_rx_shifter == 0xff)
        _rx_shifter = _pflex->requestShifter();

    if ((_timer == 0xff) || (_tx_shifter == 0xff) || (_rx_shifter == 0xff)) {
        _pflex->freeTimers(_timer, (_csPin != -1) ? 2 : 1);
        _timer = 0xff;
        _pflex->freeShifter(_tx_shifter);
        _pflex->freeShifter(_rx_shifter);
        _tx_shifter = 0xff;
        _rx_shifter = 0xff;
#ifdef DEBUG_FlexSPI
        Serial.println("FlexIOSPI - Failed to allocate timers or shifters");
#endif
        return false;
    }

    _tx_shifter_mask = 1;
    for (uint8_t i = _tx_shifter; i > 0; i--)
        _tx_shifter_mask <<= 1;
    _rx_shifter_mask = 1;
    for (uint8_t i = _rx_shifter; i > 0; i--)
        _rx_shifter_mask <<= 1;

#ifdef DEBUG_FlexSPI
    DEBUG_FlexSPI.printf("timer index: %d shifter index: %d mask: %x\n", _timer, _tx_shifter, _tx_shifter_mask);
    // lets try to configure a tranmitter like example
    DEBUG_FlexSPI.println("Before configure flexio");
#endif
    p->SHIFTCFG[_tx_shifter] = 0; // Start/stop disabled;

    p->SHIFTCTL[_tx_shifter] = FLEXIO_SHIFTCTL_MODE_TRANSMIT |
                               FLEXIO_SHIFTCTL_SHIFT_ON_FALLING_EDGE |
                               FLEXIO_SHIFTCTL_PINMODE_OUTPUT |
                               FLEXIO_SHIFTCTL_TIMSEL(_timer) |
                               FLEXIO_SHIFTCTL_PINSEL(_mosi_flex_pin); // 0x0003_0002
    p->SHIFTCFG[_rx_shifter] = 0;                                      // Start/stop disabled;
    p->SHIFTCTL[_rx_shifter] = FLEXIO_SHIFTCTL_MODE_RECEIVE |
                               FLEXIO_SHIFTCTL_SHIFT_ON_RISING_EDGE |
                               FLEXIO_SHIFTCTL_PINMODE_INPUT |
                               FLEXIO_SHIFTCTL_TIMSEL(_timer) |
                               FLEXIO_SHIFTCTL_PINSEL(_miso_flex_pin); // 0x0003_0002
    p->TIMCMP[_timer] = 0x0f01;                                        // (8 bits?)0x3f01; // ???0xf00 | baud_div; //0xF01; //0x0000_0F01;		//
    p->TIMCTL[_timer] = FLEXIO_TIMCTL_MODE_8BIT_BAUD |
                        FLEXIO_TIMCTL_TRIGGER_SHIFTER(0) |
                        FLEXIO_TIMCTL_TRIGGER_ACTIVE_LOW |
                        FLEXIO_TIMCTL_PINMODE_OUTPUT |
                        FLEXIO_TIMCTL_PINSEL(_sck_flex_pin); // 0x01C0_0001
    if (_csPin != -1) {
        p->TIMCFG[_timer] = FLEXIO_TIMCFG_DEC_ON_CLOCK_SHIFT_ON_OUTPUT |
                            FLEXIO_TIMCFG_OUTPUT_LOW_WHEN_ENABLED |
                            FLEXIO_TIMCFG_RESET_NEVER |
                            FLEXIO_TIMCFG_DISABLE_ON_8BIT_MATCH |
                            FLEXIO_TIMCFG_ENABLE_WHEN_TRIGGER_HIGH |
                            FLEXIO_TIMCFG_STOPBIT_ENABLE_ON_TIMER_DISABLE |
                            FLEXIO_TIMCFG_STARTBIT_ENABLED; // 0x0100_2222

        p->TIMCMP[_timer + 1] = 0xffff; // never compare
        p->TIMCFG[_timer + 1] = FLEXIO_TIMCFG_DISABLE_WHEN_PRIOR_TIMER_DISABLES |
                                FLEXIO_TIMCFG_ENABLE_WHEN_PRIOR_TIMER_ENABLES; // 0x0000_1100
        p->TIMCTL[_timer + 1] = FLEXIO_TIMCTL_MODE_16BIT |
                                FLEXIO_TIMCTL_PINMODE_OUTPUT |
                                FLEXIO_TIMCTL_PIN_ACTIVE_LOW |
                                FLEXIO_TIMCTL_PINSEL(_cs_flex_pin); // 0003_0383
    } else {
        p->TIMCFG[_timer] = FLEXIO_TIMCFG_DEC_ON_CLOCK_SHIFT_ON_OUTPUT |
                            FLEXIO_TIMCFG_OUTPUT_LOW_WHEN_ENABLED |
                            FLEXIO_TIMCFG_DISABLE_ON_8BIT_MATCH |
                            FLEXIO_TIMCFG_ENABLE_WHEN_TRIGGER_HIGH;
    }

    // Make sure this flex IO object is enabled
    p->CTRL = FLEXIO_CTRL_FLEXEN;
    // p->SHIFTSTAT = _tx_shifter_mask;   // Clear out the status.

    // Set the IO pins into FLEXIO mode
    _pflex->setIOPinToFlexMode(_mosiPin);
    _pflex->setIOPinToFlexMode(_sckPin);
    _pflex->setIOPinToFlexMode(_misoPin);
    // Wonder if we should cofigure the port config registers like SPI does?
    uint32_t fastio = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
    // uint32_t fastio = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(1);
    // uint32_t fastio = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
    // Serial.printf("SPI MISO: %d MOSI: %d, SCK: %d\n", hardware().miso_pin[miso_pin_index], hardware().mosi_pin[mosi_pin_index], hardware().sck_pin[sck_pin_index]);
    *(portControlRegister(_mosiPin)) = fastio;
    *(portControlRegister(_sckPin)) = fastio;
    *(portControlRegister(_misoPin)) = fastio | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3); // maybe add our own internal PU?

    if (_csPin != -1)
        _pflex->setIOPinToFlexMode(_csPin);

    _pflex->addIOHandlerCallback(this);

    // precompute the shift registers depending on MSB or LSB first...
    _bitOrder = MSBFIRST;
    _shiftBufOutReg = &_pflex->port().SHIFTBUFBBS[_tx_shifter];
    _shiftBufInReg = &_pflex->port().SHIFTBUFBIS[_rx_shifter];
    ;

    // Lets print out some of the settings and the like to get idea of state
#ifdef DEBUG_FlexSPI
    DEBUG_FlexSPI.printf("Mosi map: %d %x %d\n", _mosiPin, (uint32_t)_pflex, _mosi_flex_pin);
    DEBUG_FlexSPI.printf("Miso map: %d %d\n", _misoPin, _miso_flex_pin);
    DEBUG_FlexSPI.printf("Sck map: %d %d\n", _sckPin, _sck_flex_pin);
    DEBUG_FlexSPI.printf("CCM_CDCDR: %x\n", CCM_CDCDR);
    DEBUG_FlexSPI.printf("FlexIO bus speed: %d\n", _pflex->computeClockRate());
    DEBUG_FlexSPI.printf("VERID:%x PARAM:%x CTRL:%x PIN: %x\n", p->PARAM, p->CTRL, p->CTRL, p->PIN);
    DEBUG_FlexSPI.printf("SHIFTSTAT:%x SHIFTERR=%x TIMSTAT=%x\n", p->SHIFTSTAT, p->SHIFTERR, p->TIMSTAT);
    DEBUG_FlexSPI.printf("SHIFTSIEN:%x SHIFTEIEN=%x TIMIEN=%x\n", p->SHIFTSIEN, p->SHIFTEIEN, p->TIMIEN);
    DEBUG_FlexSPI.printf("SHIFTSDEN:%x SHIFTSTATE=%x\n", p->SHIFTSDEN, p->SHIFTSTATE);
    DEBUG_FlexSPI.printf("SHIFTCTL:%x %x %x %x\n", p->SHIFTCTL[0], p->SHIFTCTL[1], p->SHIFTCTL[2], p->SHIFTCTL[3]);
    DEBUG_FlexSPI.printf("SHIFTCFG:%x %x %x %x\n", p->SHIFTCFG[0], p->SHIFTCFG[1], p->SHIFTCFG[2], p->SHIFTCFG[3]);
    DEBUG_FlexSPI.printf("TIMCTL:%x %x %x %x\n", p->TIMCTL[0], p->TIMCTL[1], p->TIMCTL[2], p->TIMCTL[3]);
    DEBUG_FlexSPI.printf("TIMCFG:%x %x %x %x\n", p->TIMCFG[0], p->TIMCFG[1], p->TIMCFG[2], p->TIMCFG[3]);
    DEBUG_FlexSPI.printf("TIMCMP:%x %x %x %x\n", p->TIMCMP[0], p->TIMCMP[1], p->TIMCMP[2], p->TIMCMP[3]);
#endif

    return true;
}

void FlexIOSPI::end(void) {
    // If the transmit was allocated free it now as well as timers and shifters.
    if (_pflex) {
        _pflex->freeTimers(_timer, (_csPin != -1) ? 2 : 1);
        _timer = 0xff;
        _pflex->freeShifter(_tx_shifter);
        _pflex->freeShifter(_rx_shifter);
        _tx_shifter = 0xff;

        _pflex->removeIOHandlerCallback(this);
        _pflex = nullptr;
    }
}

void FlexIOSPI::beginTransaction(FlexIOSPISettings settings) {
#ifdef SPI_TRANSACTION_MISMATCH_LED
    if (inTransactionFlag) {
        pinMode(SPI_TRANSACTION_MISMATCH_LED, OUTPUT);
        digitalWrite(SPI_TRANSACTION_MISMATCH_LED, HIGH);
    }
    _in_transaction_flag = 1;
#endif

    // right now pretty stupid
    if ((settings._clock != _clock) || (settings._dataMode != _dataMode) || (settings._nTransferBits != _nTransferBits)) {
        _clock = settings._clock;
        _dataMode = settings._dataMode;
        _nTransferBits = settings._nTransferBits; // Probaby should have some safety checking to keep this in the 1-32 range for now.
        _nTransferBytes = (_nTransferBits - 1) / 8 + 1;
        if (_nTransferBytes == 3)
            _nTransferBytes = 4;                               // DMA doesn't handle arbitrary pointer shifts so force 32bit alignment even though it would fit into 24bits.
        uint32_t clock_speed = _pflex->computeClockRate() / 2; // get speed divide by
        uint32_t div = clock_speed / _clock;
        if (div) {
            if ((clock_speed / div) > _clock)
                div++; // unless even multiple increment
            div--;     // the actual value stored is the -1...
        }
        if (!(_dataMode & SPI_MODE_TRANSMIT_ONLY)) {
            if (div == 0)
                div = 1; // force to at least one as Reads will fail at 0...
            else if ((div == 1) && (_clock > 30000000u))
                div = 2;
        }

        _pflex->port().TIMCMP[_timer] = div | (_nTransferBits * 2 - 1) << 8; // Set the clk div for shifter and set transfer length
#ifdef DEBUG_FlexSPI
        DEBUG_FlexSPI.printf("FlexIOSPI:beginTransaction TIMCMP: %x\n", _pflex->port().TIMCMP[_timer]);
#endif
    }

    _bitOrder = settings._bitOrder;
}

// After performing a group of transfers and releasing the chip select
// signal, this function allows others to access the SPI bus
void FlexIOSPI::endTransaction(void) {
#ifdef SPI_TRANSACTION_MISMATCH_LED
    if (!inTransactionFlag) {
        pinMode(SPI_TRANSACTION_MISMATCH_LED, OUTPUT);
        digitalWrite(SPI_TRANSACTION_MISMATCH_LED, HIGH);
    }
    _in_transaction_flag = 0;
#endif
#ifdef DEBUG_FlexSPI
    DEBUG_FlexSPI.printf("FlexIOSPI:endTransaction\n");
#endif
}

void FlexIOSPI::setShiftBufferOut(uint32_t val, uint8_t nbits) {
    if (_bitOrder == MSBFIRST) {
        _pflex->port().SHIFTBUFBIS[_tx_shifter] = val << (32 - nbits);
    } else {
        _pflex->port().SHIFTBUF[_tx_shifter] = val;
    }
}

void FlexIOSPI::setShiftBufferOut(const void *buf, uint8_t nbits, size_t dtype_size) {
    uint32_t val = 0;
    switch (dtype_size) {
    case 1:
        val = *(uint8_t *)buf;
        break;
    case 2:
        val = *(uint16_t *)buf;
        break;
    case 3:
        val = (*(uint32_t *)buf) & 0xffffff;
        break;
    case 4:
        val = *(uint32_t *)buf;
        break;
    default:
        break;
    }
    setShiftBufferOut(val, nbits);
}

uint32_t FlexIOSPI::getShiftBufferIn(uint8_t nbits) {
    uint32_t ret_val;
    if (_bitOrder == MSBFIRST) {
        ret_val = _pflex->port().SHIFTBUFBIS[_rx_shifter];
    } else {
        ret_val = _pflex->port().SHIFTBUF[_rx_shifter] >> (32 - nbits);
    }
    return ret_val;
}

void FlexIOSPI::getShiftBufferIn(void *retbuf, uint8_t nbits, size_t dtype_size) {
    uint32_t tmp;
    switch (dtype_size) {
    case 1:
        *(uint8_t *)retbuf = (uint8_t)getShiftBufferIn(nbits);
        break;
    case 2:
        *(uint16_t *)retbuf = (uint16_t)getShiftBufferIn(nbits);
        break;
    case 3: // The weird case that gets some shuffling to handle the last chunk without overflowing
        tmp = getShiftBufferIn(nbits);
        *(uint16_t *)retbuf = (uint16_t)tmp & 0xffff;           // Lower16 bits
        *((uint8_t *)retbuf + 2) = (uint8_t)(tmp >> 16) & 0xff; // Upper8 bits
        break;
    case 4:
        *(uint32_t *)retbuf = (uint32_t)getShiftBufferIn(nbits);
        break;
    default:
        break;
    }
}

uint32_t FlexIOSPI::transferNBits(uint32_t w_out, uint8_t nbits) {
    // Need to do some validation...

    uint32_t return_val;
    uint16_t timcmp_save = _pflex->port().TIMCMP[_timer];                        // remember value coming in
    _pflex->port().TIMCMP[_timer] = (timcmp_save & 0xff) | (nbits * 2 - 1) << 8; // Adjust transmission length to nbits
    // Serial.printf("TCMP bits = %x\n",_pflex->port().TIMCMP[_timer]);
    //  Now lets wait for something to come back.
    uint16_t timeout = 0xffff; // don't completely hang
    // Clear any current pending RX input
    if (_pflex->port().SHIFTSTAT & _rx_shifter_mask) {
        return_val = getShiftBufferIn(nbits);
    }

    setShiftBufferOut(w_out, nbits);

    return_val = 0xff;
    while (!(_pflex->port().SHIFTSTAT & _rx_shifter_mask) && (--timeout))
        ;

    if (_pflex->port().SHIFTSTAT & _rx_shifter_mask) {
        return_val = getShiftBufferIn(nbits);
    }

    _pflex->port().TIMCMP[_timer] = timcmp_save;
    return return_val;
}

void FlexIOSPI::transferBufferNBits(const void *buf, void *retbuf, size_t count, uint8_t nbits) {
    if (!nbits)
        nbits = _nTransferBits;
    uint8_t bytestride = (nbits - 1) / 8 + 1;
    if (bytestride == 3) {
        bytestride = 4;
    }
    uint32_t tx_count = count;
    const uint8_t *tx_buffer = (const uint8_t *)buf;
    uint8_t *rx_buffer = (uint8_t *)retbuf;

    if (count <= 0)
        return; // bail if 0 count passed in.

    // put out the first character.
    _pflex->port().SHIFTERR = _rx_shifter_mask | _tx_shifter_mask; // clear out any previous errors
    while (!(_pflex->port().SHIFTSTAT & _tx_shifter_mask))
        ; // wait for room for the first character

    if (tx_buffer) {
        setShiftBufferOut(tx_buffer, nbits, bytestride);
        tx_buffer += bytestride;
    }
    tx_count--;
    while (tx_count) {
        // wait for room for the next character
        while (!(_pflex->port().SHIFTSTAT & _tx_shifter_mask))
            ;

        if (tx_buffer) {
            setShiftBufferOut(tx_buffer, nbits, bytestride);
            tx_buffer += bytestride;
        }
        tx_count--;

        // Wait for data to come back
        while (!(_pflex->port().SHIFTSTAT & _rx_shifter_mask))
            ;

        if (rx_buffer) {
            getShiftBufferIn(rx_buffer, nbits, bytestride);
            rx_buffer += bytestride;
        }
    }
    // wait for last character to come back...
    while (!(_pflex->port().SHIFTSTAT & _rx_shifter_mask) && !(_pflex->port().SHIFTERR & _rx_shifter_mask))
        ;

    if (rx_buffer) {
        getShiftBufferIn(rx_buffer, nbits, bytestride);
        rx_buffer += bytestride;
    }
}

bool FlexIOSPI::call_back(FlexIOHandler *pflex) {
    //	DEBUG_digitalWriteFast(4, HIGH);
    return false; // right now always return false...
}

//=============================================================================
// ASYNCH Support
//=============================================================================
//=========================================================================
// Try Transfer using DMA.
//=========================================================================
static uint8_t bit_bucket;
#define dontInterruptAtCompletion(dmac) (dmac)->TCD->CSR &= ~DMA_TCD_CSR_INTMAJOR

//=========================================================================
// Init the DMA channels
//=========================================================================
bool FlexIOSPI::initDMAChannels() {
    // Allocate our channels.
    _dmaTX = new DMAChannel();
    if (_dmaTX == nullptr) {
        return false;
    }

    _dmaRX = new DMAChannel();
    if (_dmaRX == nullptr) {
        delete _dmaTX; // release it
        _dmaTX = nullptr;
        return false;
    }

    int iFlexIndex = _pflex->FlexIOIndex();
    // Let's setup the RX chain
    _dmaRX->disable();
    _dmaRX->source((volatile uint8_t &)*_shiftBufInReg);
    _dmaRX->disableOnCompletion();
    _dmaRX->triggerAtHardwareEvent(_pflex->shiftersDMAChannel(_rx_shifter));
    if (iFlexIndex == 0) {
        _dmaRX->attachInterrupt(&_dma_rxISR0);
        _dmaActiveObjects[0] = this;
    } else {
        _dmaRX->attachInterrupt(&_dma_rxISR1);
        _dmaActiveObjects[1] = this;
    }
    _dmaRX->interruptAtCompletion();

    // We may be using settings chain here so lets set it up.
    // Now lets setup TX chain.  Note if trigger TX is not set
    // we need to have the RX do it for us.
    _dmaTX->disable();
    _dmaTX->destination((volatile uint8_t &)*_shiftBufOutReg);
    _dmaTX->disableOnCompletion();

    _dmaTX->triggerAtHardwareEvent(_pflex->shiftersDMAChannel(_tx_shifter));

    _dma_state = DMAState::idle; // Should be first thing set!
    return true;
}

//=========================================================================
// Main Async Transfer function
//=========================================================================
#ifdef DEBUG_DMA_TRANSFERS
void dumpDMA_TCD(DMABaseClass *dmabc) {
    Serial4.printf("%x %x:", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

    Serial4.printf("SA:%x SO:%d AT:%x NB:%x SL:%d DA:%x DO: %d CI:%x DL:%x CS:%x BI:%x\n", (uint32_t)dmabc->TCD->SADDR,
                   dmabc->TCD->SOFF, dmabc->TCD->ATTR, dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR,
                   dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA, dmabc->TCD->CSR, dmabc->TCD->BITER);
}
#endif

bool FlexIOSPI::transfer(const void *buf, void *retbuf, size_t count, EventResponderRef event_responder) {
    if (_dma_state == DMAState::notAllocated) {
        if (!initDMAChannels())
            return false;
    }

    if (_dma_state == DMAState::active)
        return false; // already active

    event_responder.clearEvent(); // Make sure it is not set yet
    if (count < 2) {
        // Use non-async version to simplify cases...
        transfer(buf, retbuf, count);
        event_responder.triggerEvent();
        return true;
    }

    // Now handle the cases where the count > then how many we can output in one DMA request
    if (count > MAX_DMA_COUNT) {
        _dma_count_remaining = count - MAX_DMA_COUNT;
        count = MAX_DMA_COUNT;
    } else {
        _dma_count_remaining = 0;
    }

    // Now See if caller passed in a source buffer.
    uint8_t *write_data = (uint8_t *)buf;
    if (buf) {
        _dmaTX->sourceBuffer((uint8_t *)write_data, count);
        _dmaTX->TCD->SLAST = 0; // Finish with it pointing to next location
        if ((uint32_t)write_data >= 0x20200000u)
            arm_dcache_flush(write_data, count);
    } else {
        _dmaTX->source((uint8_t &)_transferWriteFill); // maybe have setable value
        _dmaTX->transferCount(count);
    }
    _dmaTX->transferSize(_nTransferBytes);

    if (retbuf) {
        // On T3.5 must handle SPI1/2 differently as only one DMA channel
        _dmaRX->destinationBuffer((uint8_t *)retbuf, count);
        _dmaRX->TCD->DLASTSGA = 0; // At end point after our bufffer
        if ((uint32_t)retbuf >= 0x20200000u)
            arm_dcache_delete(retbuf, count);
    } else {
        // Write  only mode
        _dmaRX->destination((uint8_t &)bit_bucket);
        _dmaRX->transferCount(count);
    }
    _dmaRX->transferSize(_nTransferBytes);

    _dma_event_responder = &event_responder;
    // Now try to start it?
    // Setup DMA main object
    yield();

#ifdef DEBUG_DMA_TRANSFERS
    // Lets dump TX, RX
    dumpDMA_TCD(_dmaTX);
    dumpDMA_TCD(_dmaRX);
#endif

    // Lets turn on the DMA handling for this
    _pflex->port().SHIFTSDEN |= _rx_shifter_mask | _tx_shifter_mask;

    _dmaRX->enable();
    _dmaTX->enable();

    _dma_state = DMAState::active;
    return true;
}

void FlexIOSPI::_dma_rxISR0(void) {
    FlexIOSPI::_dmaActiveObjects[0]->dma_rxisr();
}

void FlexIOSPI::_dma_rxISR1(void) {
    FlexIOSPI::_dmaActiveObjects[1]->dma_rxisr();
}

//-------------------------------------------------------------------------
// DMA RX ISR
//-------------------------------------------------------------------------
void FlexIOSPI::dma_rxisr(void) {
    _dmaRX->clearInterrupt();
    _dmaTX->clearComplete();
    _dmaRX->clearComplete();

    if (_dma_count_remaining) {
        // What do I need to do to start it back up again...
        // We will use the BITR/CITR from RX as TX may have prefed some stuff
        if (_dma_count_remaining > MAX_DMA_COUNT) {
            _dma_count_remaining -= MAX_DMA_COUNT;
        } else {
            _dmaTX->transferCount(_dma_count_remaining);
            _dmaRX->transferCount(_dma_count_remaining);

            _dma_count_remaining = 0;
        }
        _dmaRX->enable();
        _dmaTX->enable();
    } else {

        _pflex->port().SHIFTSDEN &= ~(_rx_shifter_mask | _tx_shifter_mask); // turn off DMA on both RX and TX
        _dma_state = DMAState::completed;                                   // set back to 1 in case our call wants to start up dma again
        _dma_event_responder->triggerEvent();
    }
}
