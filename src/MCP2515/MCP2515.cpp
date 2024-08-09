/**
 * CAN MCP2515_nb
 * Copyright 2020 WitchCraftWorks Team, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#include "MCP2515.h"
#include "CANPacket.h"
#include "Utilities.hpp"

#define TX_BUFFER_NUM 0

#define REG_BFPCTRL 0x0C
#define REG_TXRTSCTRL 0x0D

#define REG_CANSTAT 0x0E
#define REG_CANCTRL 0x0F

#define REG_TX_ERROR_COUNTER 0x1C
#define REG_RX_ERROR_COUNTER 0x1D

#define REG_CNF3 0x28
#define REG_CNF2 0x29
#define REG_CNF1 0x2A

#define REG_CANINTE 0x2B
#define REG_CANINTF 0x2C
#define REG_EFLG 0x2D

#define FLAG_RXnIE(n) (0x01 << n)
#define FLAG_RXnIF(n) (0x01 << n)
#define FLAG_TXnIF(n) (0x04 << n)

#define REG_RXFnSIDH(n) ((n) < 3 ? (0x00 + (n) * 4) : (0x10 + ((n) - 3) * 4))
#define REG_RXFnSIDL(n) ((n) < 3 ? (0x01 + (n) * 4) : (0x11 + ((n) - 3) * 4))
#define REG_RXFnEID8(n) ((n) < 3 ? (0x02 + (n) * 4) : (0x12 + ((n) - 3) * 4))
#define REG_RXFnEID0(n) ((n) < 3 ? (0x03 + (n) * 4) : (0x13 + ((n) - 3) * 4))

#define REG_RXMnSIDH(n) (0x20 + (n * 0x04))
#define REG_RXMnSIDL(n) (0x21 + (n * 0x04))
#define REG_RXMnEID8(n) (0x22 + (n * 0x04))
#define REG_RXMnEID0(n) (0x23 + (n * 0x04))

#define REG_TXBnCTRL(n) (0x30 + (n * 0x10))
#define REG_TXBnSIDH(n) (0x31 + (n * 0x10))
#define REG_TXBnSIDL(n) (0x32 + (n * 0x10))
#define REG_TXBnEID8(n) (0x33 + (n * 0x10))
#define REG_TXBnEID0(n) (0x34 + (n * 0x10))
#define REG_TXBnDLC(n) (0x35 + (n * 0x10))
#define REG_TXBnD0(n) (0x36 + (n * 0x10))

#define REG_RXBnCTRL(n) (0x60 + (n * 0x10))
#define REG_RXBnSIDH(n) (0x61 + (n * 0x10))
#define REG_RXBnSIDL(n) (0x62 + (n * 0x10))
#define REG_RXBnEID8(n) (0x63 + (n * 0x10))
#define REG_RXBnEID0(n) (0x64 + (n * 0x10))
#define REG_RXBnDLC(n) (0x65 + (n * 0x10))
#define REG_RXBnD0(n) (0x66 + (n * 0x10))

#define FLAG_IDE 0x08
#define FLAG_SRR 0x10
#define FLAG_RTR 0x40
#define FLAG_EXIDE 0x08

#define FLAG_RXM0 0x20
#define FLAG_RXM1 0x40

// ----------- Interrupt garbage -----------

#if defined(ARDUINO_ARCH_SAMD)
# include <wiring_private.h>

# define internalPreviousInterruptState() EIC->INTENSET.reg
# define internalPinToInterrupt(n, prev) _internalPinToInterrupt(n, prev)
# define _internalAttachInterrupt(pin, func, mode) attachInterrupt(pin, func, mode)

static inline __attribute__((always_inline))
int _internalPinToInterrupt(int pin, uint32_t prev) {
    uint32_t reg = EIC->INTENSET.reg & ~prev;

    for (uint32_t i = EXTERNAL_INT_0; i <= EXTERNAL_INT_15; i++) {
        if ((reg & (1 << i))) {
            return i;
        }
    }

    return -1;
}

static inline __attribute__((always_inline))
int interruptGetCalledPin() {
    for (uint32_t i = EXTERNAL_INT_0; i <= EXTERNAL_INT_15; i++) {
        if ((EIC->INTFLAG.reg & (1 << i))) {
            return i;
        }
    }

    return -1;
}
#else
MCP2515* GLOBAL_MCP = nullptr;
#endif

// --- Generic Interrupts ---

#ifdef __MCP2515_MULTI_INTERRUPTS_ENABLE__
struct mcp_interrupt_info {
    uint8_t pin = -1;
    MCP2515* mcp = nullptr;
};

static mcp_interrupt_info _mcp_interrupts[EXTERNAL_NUM_INTERRUPTS];
static int _mcp_interrupts_counter = 0;

int _mcp_interrupts_find(uint8_t pin, mcp_interrupt_info* info) {
    for (int i = 0; i < _mcp_interrupts_counter; i++) {
        if (_mcp_interrupts[i].pin == pin) {
            *info = _mcp_interrupts[i];
            return i;
        }
    }

    return MCP2515Error::NOENT;
}

void _mcp_interrupts_insert(uint8_t pin, MCP2515* mcp) {
    mcp_interrupt_info info;
    int pos = _mcp_interrupts_find(pin, &info);

    info.pin = pin;
    info.mcp = mcp;

    if (pos < 0) {
        memcpy(&_mcp_interrupts[_mcp_interrupts_counter], &info, sizeof(info));
        _mcp_interrupts_counter++;
    } else {
        memcpy(&_mcp_interrupts[pos], &info, sizeof(info));
    }
}

void _mcp_interrupts_remove(uint8_t pin, MCP2515* mcp) {
    mcp_interrupt_info info;
    int pos = _mcp_interrupts_find(pin, &info);

    if (pos >= 0 && info.mcp != nullptr) {
        info.mcp = nullptr;
        memcpy(&_mcp_interrupts[_mcp_interrupts_counter], &info, sizeof(info));
    }
}
#endif

// ----------- End Interrupt Garbage -----------

MCP2515::MCP2515() :
    _csPin(MCP2515_DEFAULT_CS_PIN),
    _intPin(MCP2515_DEFAULT_INT_PIN),
    _clockFrequency(16e6),
    _spiSettings(10e6, MSBFIRST, SPI_MODE0),
#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    _canpacketTxQueue(std::queue<CANPacket*>())
#endif
{
}

MCP2515::~MCP2515() {
#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    while (!_canpacketTxQueue.empty()) {
        _canpacketTxQueue.pop();
    }
#endif
}

int MCP2515::begin(MCP2515_CAN_SPEED baudRate) {
    pinMode(_csPin, OUTPUT);

    SPI.begin();
    reset();

    writeRegister(REG_CANCTRL, 0x80);
    if (readRegister(REG_CANCTRL) != 0x80) {
        return MCP2515Error::BADF;
    }

    _mcp_cnf_frequency cnf = getCnfForClockFrequency(_clockFrequency, baudRate);

    if (!cnf)
        return MCP2515Error::INVAL;

    writeRegister(REG_CNF1, cnf.one);
    writeRegister(REG_CNF2, cnf.two);
    writeRegister(REG_CNF3, cnf.three);

    writeRegister(REG_CANINTE, (FLAG_RXnIE(1) | FLAG_RXnIE(0)));
    writeRegister(REG_BFPCTRL, 0x00);
    writeRegister(REG_TXRTSCTRL, 0x00);
    writeRegister(REG_RXBnCTRL(0), (FLAG_RXM1 | FLAG_RXM0));
    writeRegister(REG_RXBnCTRL(1), (FLAG_RXM1 | FLAG_RXM0));

    writeRegister(REG_CANCTRL, 0x00);
    if (readRegister(REG_CANCTRL) != 0x00) {
        return MCP2515Error::BADF;
    }

    return MCP2515Error::OK;
}

void MCP2515::end() {
    reset();
    SPI.end();
}

uint8_t MCP2515::getStatus() {
    return readRegister(REG_CANSTAT);
}

uint8_t MCP2515::getErrorFlags() {
    return readRegister(REG_EFLG);
}

void MCP2515::setPins(int cs, int irq) {
    _csPin = cs;
    _intPin = irq;
}

void MCP2515::setSPIFrequency(uint32_t frequency) {
    _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void MCP2515::setClockFrequency(MCP2515_CAN_CLOCK clockFrequency) {
    _clockFrequency = clockFrequency;
}

MCP2515Error MCP2515::setMask(const MCP2515_CAN_MASK num, bool extended, uint32_t mask) {
    writeRegister(REG_CANCTRL, 0x80);
    if (readRegister(REG_CANCTRL) != 0x80) {
        return MCP2515Error::BADF;
    }

    uint8_t eid0 = 0;
    uint8_t eid8 = 0;
    uint8_t sidh = 0;
    uint8_t sidl = 0;

    if (extended) {
        uint16_t canfilter = mask & 0x0FFFF;
        eid0 = canfilter & 0xFF;
        eid8 = canfilter >> 8;

        uint16_t canid = mask >> 16;
        sidh = canid >> 5;
        sidl = ((canid & 0x03) + ((canid & 0x1C) << 3)) | FLAG_EXIDE;
    } else {
        uint16_t canfilter = mask & 0x0FFFF;
        sidh = canfilter >> 3;
        sidl = (canfilter & 0x07) << 5;
    }

    writeRegister(REG_RXMnEID0(num), eid0);
    writeRegister(REG_RXMnEID8(num), eid8);
    writeRegister(REG_RXMnSIDH(num), sidh);
    writeRegister(REG_RXMnSIDL(num), sidl);

    writeRegister(REG_CANCTRL, 0x00);
    if (readRegister(REG_CANCTRL) != 0x00) {
        return MCP2515Error::BADF;
    }

    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setFilter(const MCP2515_CAN_RXF num, bool extended, uint32_t filter) {
    writeRegister(REG_CANCTRL, 0x80);
    if (readRegister(REG_CANCTRL) != 0x80) {
        return MCP2515Error::BADF;
    }

    uint8_t eid0 = 0;
    uint8_t eid8 = 0;
    uint8_t sidh = 0;
    uint8_t sidl = 0;

    if (extended) {
        uint16_t canfilter = filter & 0x0FFFF;
        eid0 = canfilter & 0xFF;
        eid8 = canfilter >> 8;

        uint16_t canid = filter >> 16;
        sidh = canid >> 5;
        sidl = ((canid & 0x03) + ((canid & 0x1C) << 3)) | FLAG_EXIDE;
    } else {
        uint16_t canfilter = filter & 0x0FFFF;
        sidh = canfilter >> 3;
        sidl = (canfilter & 0x07) << 5;
    }

    writeRegister(REG_RXFnEID0(num), eid0);
    writeRegister(REG_RXFnEID8(num), eid8);
    writeRegister(REG_RXFnSIDH(num), sidh);
    writeRegister(REG_RXFnSIDL(num), sidl);

    writeRegister(REG_CANCTRL, 0x00);
    if (readRegister(REG_CANCTRL) != 0x00) {
        return MCP2515Error::BADF;
    }

    return MCP2515Error::OK;
}

int MCP2515::getMode() {
    switch ((readRegister(REG_CANCTRL) & 0xE0)) {
        case 0x00:
            return MCP2515_MODES::NORMAL;
        case 0x40:
            return MCP2515_MODES::LOOPBACK;
        case 0x60:
            return MCP2515_MODES::LISTEN;
        case 0x80:
            return MCP2515_MODES::CONFIG;
        case 0x20:
            return MCP2515_MODES::SLEEP;
    }

    return -1;
}

MCP2515Error MCP2515::setConfigMode() {
    writeRegister(REG_CANCTRL, 0x80);
    if (readRegister(REG_CANCTRL) != 0x80) {
        return MCP2515Error::BADF;
    }

    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setListenMode(bool allowInvalidPackets) {
    writeRegister(REG_CANCTRL, 0x60);
    if (readRegister(REG_CANCTRL) != 0x60) {
        return MCP2515Error::BADF;
    }

    if (allowInvalidPackets) {
        modifyRegister(REG_RXBnCTRL(0), 0x30, 0x30);
        modifyRegister(REG_RXBnCTRL(1), 0x30, 0x30);

        // In listen-only mode, we can receive errornous messages,
        // keep track of the counter to detect an errornous message
        _rxErrorCount = readRegister(REG_RX_ERROR_COUNTER);
    } else if (_allowInvalidRx) {
        modifyRegister(REG_RXBnCTRL(0), 0x30, 0x00);
        modifyRegister(REG_RXBnCTRL(1), 0x30, 0x00);
    }

    _allowInvalidRx = allowInvalidPackets;
    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setLoopbackMode() {
    writeRegister(REG_CANCTRL, 0x40);
    if (readRegister(REG_CANCTRL) != 0x40) {
        return MCP2515Error::BADF;
    }

    if (_allowInvalidRx) {
        _allowInvalidRx = false;
        modifyRegister(REG_RXBnCTRL(0), 0x30, 0x00);
        modifyRegister(REG_RXBnCTRL(1), 0x30, 0x00);
    }

    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setSleepMode() {
    writeRegister(REG_CANCTRL, 0x20);

    // Block until CAN controller goes into sleep.
    // "These bits should be read after sending the SLEEP command to the MCP2515.
    // The MCP2515 is active and has not yet entered Sleep mode until these bits
    // indicate that Sleep mode has been entered."
    while ((readRegister(REG_CANCTRL) & 0xE0) != 0x20) {
        yield();
    }

    if (_allowInvalidRx) {
        _allowInvalidRx = false;
        modifyRegister(REG_RXBnCTRL(0), 0x30, 0x00);
        modifyRegister(REG_RXBnCTRL(1), 0x30, 0x00);
    }

    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setNormalMode() {
    writeRegister(REG_CANCTRL, 0x00);
    if (readRegister(REG_CANCTRL) != 0x00) {
        return MCP2515Error::BADF;
    }

    if (_allowInvalidRx) {
        _allowInvalidRx = false;
        modifyRegister(REG_RXBnCTRL(0), 0x30, 0x00);
        modifyRegister(REG_RXBnCTRL(1), 0x30, 0x00);
    }

    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setWakeupFilter(bool enable) {
    uint8_t envalue = (enable ? 0x40 : 0);
    modifyRegister(REG_CNF3, 0x40, envalue);

    if ((readRegister(REG_CNF3) & 0x40) != envalue) {
        return MCP2515Error::BADF;
    }

    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setOneShotMode(bool enable) {
    uint8_t envalue = (enable ? 0x08 : 0);
    modifyRegister(REG_CANCTRL, 0x08, envalue);

    if ((readRegister(REG_CANCTRL) & 0x08) != envalue) {
        return MCP2515Error::BADF;
    }

    _oneShotMode = enable;
    return MCP2515Error::OK;
}

MCP2515Error MCP2515::receivePacket(CANPacket* packet) {
    int n;
    uint8_t intf = readRegister(REG_CANINTF);

    if (intf & FLAG_RXnIF(0)) {
        n = 0;
    } else if (intf & FLAG_RXnIF(1)) {
        n = 1;
    } else {
        return MCP2515Error::NOENT;
    }

    packet->_started = true;
    packet->_extended = (readRegister(REG_RXBnSIDL(n)) & FLAG_IDE ? true : false);

    uint8_t sidh = readRegister(REG_RXBnSIDH(n));
    uint8_t sidl = readRegister(REG_RXBnSIDL(n));

    uint32_t idA = ((sidh << 3) & 0x07F8) | ((sidl >> 5) & 0x07);

    if (packet->_extended) {
        uint32_t idB = (((uint32_t)(sidl & 0x03) << 16) & 0x30000) | ((readRegister(REG_RXBnEID8(n)) << 8) & 0xFF00) | readRegister(REG_RXBnEID0(n));

        packet->_id = (idA << 18) | idB;
        packet->_rtr = (readRegister(REG_RXBnDLC(n)) & FLAG_RTR ? true : false);
    } else {
        packet->_id = idA;
        packet->_rtr = (sidl & FLAG_SRR ? true : false);
    }

    packet->_dlc = readRegister(REG_RXBnDLC(n)) & 0x0F;

    if (packet->_rtr) {
        modifyRegister(REG_CANINTF, FLAG_RXnIF(n), 0x00);
    } else {
        // READ RX BUFFER
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_csPin, LOW);

        SPI.transfer((0x92 | (n << 2)));

        for (int i = 0; i < packet->_dlc; i++) {
            packet->writeData(SPI.transfer(0x00));
        }

        digitalWrite(_csPin, HIGH);
        SPI.endTransaction();
    }

    packet->end();

    if (_allowInvalidRx) {
        uint8_t newRxErrorCount = readRegister(REG_RX_ERROR_COUNTER);

        if (_rxErrorCount < newRxErrorCount) {
            packet->_status |= CANPacket::STATUS_RX_INVALID_MESSAGE;
        } else {
            packet->_status |= CANPacket::STATUS_RX_OK;
        }

        _rxErrorCount = newRxErrorCount;
    } else {
        packet->_status |= CANPacket::STATUS_RX_OK;
    }

    return MCP2515Error::OK;
}

void MCP2515::onReceivePacket(void(*callback)(CANPacket*)) {
    _onReceivePacket = callback;
    pinMode(_intPin, INPUT);

#ifdef __MCP2515_MULTI_INTERRUPTS_ENABLE__
    uint32_t prev = internalPreviousInterruptState();
#endif

    if (callback) {
        SPI.usingInterrupt(digitalPinToInterrupt(_intPin));

#ifdef __MCP2515_MULTI_INTERRUPTS_ENABLE__
        _internalAttachInterrupt(digitalPinToInterrupt(_intPin), MCP2515::onInterrupt, LOW);
        _mcp_interrupts_insert(internalPinToInterrupt(_intPin, prev), this);
#else
        GLOBAL_MCP = this;
        attachInterrupt(digitalPinToInterrupt(_intPin), MCP2515::onInterrupt, LOW);
#endif
        modifyRegister(REG_CANINTE, 0xDF, 0xDF);
    } else {
#ifdef __MCP2515_MULTI_INTERRUPTS_ENABLE__
        _mcp_interrupts_remove(internalPinToInterrupt(_intPin, prev), this);
#else
        GLOBAL_MCP = nullptr;
#endif

        modifyRegister(REG_CANINTE, 0xDF, 0x00);
        detachInterrupt(digitalPinToInterrupt(_intPin));

#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.notUsingInterrupt(digitalPinToInterrupt(_intPin));
#endif
    }
}

size_t MCP2515::getTxQueueLength() {
#ifdef MCP2515_DISABLE_ASYNC_TX_QUEUE
    return 0;
#else
    return _canpacketTxQueue.size();
#endif
}

void MCP2515::processTxQueue() {
#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    if (_canpacketTxQueue.empty()) {
        return;
    }

    CANPacket* packet = _canpacketTxQueue.front();
    if (packet->_status & CANPacket::STATUS_TX_WRITTEN) {
        if (packet->_status & CANPacket::STATUS_TX_SENT || handleMessageTransmit(packet, TX_BUFFER_NUM, false) != MCP2515Error::AGAIN) {
            _canpacketTxQueue.pop();
        }

        return;
    }

    if (packet->_status & CANPacket::STATUS_TX_ABORT_REQUESTED) {
        packet->_status &= ~CANPacket::STATUS_TX_PENDING;
        packet->_status |= CANPacket::STATUS_TX_ABORTED;

        _canpacketTxQueue.pop();
        return;
    }

    if (readRegister(REG_TXBnCTRL(TX_BUFFER_NUM)) & 0x08) {
        // TX buffer not empty and packet pending
        return;
    }

    writePacket(packet, true);
#endif
}

MCP2515Error MCP2515::writePacket(CANPacket* packet, bool nowait) {
    if (!packet->_ended) {
        return MCP2515Error::INVAL;
    }

    // MCP controller is NOT in normal or loopback mode
    uint8_t canctrl = readRegister(REG_CANCTRL) & 0xE0;
    if (canctrl != 0x00 && canctrl != 0x40) {
        return MCP2515Error::COMM;
    }

    packet->_status &= ~(CANPacket::STATUS_TX_WRITTEN | CANPacket::STATUS_TX_ERROR);
    packet->_status &= ~(CANPacket::STATUS_TX_ABORT_REQUESTED | CANPacket::STATUS_TX_ABORTED);
    packet->_status |= CANPacket::STATUS_TX_PENDING;

    if (nowait) {
#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
        if (getTxQueueLength() >= MCP2515_CANPACKET_TX_QUEUE_SIZE) {
            return MCP2515Error::OVERFLOW;
        }

        _canpacketTxQueue.push(packet);
#endif

        uint8_t txbncntl = readRegister(REG_TXBnCTRL(TX_BUFFER_NUM));
        if (txbncntl & 0x08) {
#ifdef MCP2515_DISABLE_ASYNC_TX_QUEUE
            return MCP2515Error::AGAIN;
#else
            // TX buffer not empty, defer send
            return MCP2515Error::OK;
#endif
        }
    } else {
        uint8_t txbncntl = readRegister(REG_TXBnCTRL(TX_BUFFER_NUM));
        if (txbncntl & 0x08) {
            // TX buffer not empty, cancel write
            return MCP2515Error::BUSY;
        }
    }

    noInterrupts();
    int n = TX_BUFFER_NUM;

    // Clear bits 3-6
    // "The TXREQ bit (TXBnCTRL[3]) must be clear (indicating the transmit buffer is not pending transmission) before writing to the transmit buffer."
    modifyRegister(REG_TXBnCTRL(n), 0x78, 0x00);

    // Clear abort bit (might have been set previously)
    modifyRegister(REG_CANCTRL, 0x10, 0x00);

    if (packet->_extended) {
        writeRegister(REG_TXBnSIDH(n), (packet->_id >> 21));
        writeRegister(REG_TXBnSIDL(n), ((((packet->_id >> 18) & 0x07) << 5) | FLAG_EXIDE | ((packet->_id >> 16) & 0x03)));
        writeRegister(REG_TXBnEID8(n), ((packet->_id >> 8) & 0xFF));
        writeRegister(REG_TXBnEID0(n), (packet->_id & 0xFF));
    } else {
        writeRegister(REG_TXBnSIDH(n), (packet->_id >> 3));
        writeRegister(REG_TXBnSIDL(n), (packet->_id << 5));
        writeRegister(REG_TXBnEID8(n), 0x00);
        writeRegister(REG_TXBnEID0(n), 0x00);
    }

    if (packet->_rtr) {
        writeRegister(REG_TXBnDLC(n), 0x40 | packet->_dlc);
    } else {
        writeRegister(REG_TXBnDLC(n), packet->_dlc);

        // LOAD TX BUFFER
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_csPin, LOW);

        SPI.transfer(0x41);

        for (int i = 0; i < packet->_dlc; i++) {
            SPI.transfer(packet->_data[i]);
        }

        digitalWrite(_csPin, HIGH);
        SPI.endTransaction();
    }

    modifyRegister(REG_TXBnCTRL(n), 0x08, 0x08);

    packet->_status |= CANPacket::STATUS_TX_WRITTEN;
    interrupts();

    if (nowait) {
        return MCP2515Error::OK;
    }

    return handleMessageTransmit(packet, 0, true);
}

MCP2515Error MCP2515::abortPacket(CANPacket* packet, bool nowait) {
    packet->_status |= CANPacket::STATUS_TX_ABORT_REQUESTED;

#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    if (_canpacketTxQueue.empty() || _canpacketTxQueue.front() == packet) {
#endif
        modifyRegister(REG_CANCTRL, 0x10, 0x10);
#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    }
#endif

    if (nowait) {
        return MCP2515Error::OK;
    }

    handleMessageTransmit(packet, TX_BUFFER_NUM, true);
    return (packet->_status & CANPacket::STATUS_TX_ABORTED ? MCP2515Error::OK : MCP2515Error::BADF);
}

int MCP2515::waitForPacketStatus(CANPacket* packet, unsigned long status, bool nowait, unsigned long timeout) {
    if (packet->_status & (CANPacket::STATUS_RX_OK | CANPacket::STATUS_RX_INVALID_MESSAGE)) {
        return MCP2515Error::INVAL;
    }

    unsigned long ms = millis();

    do {
        if (handleMessageTransmit(packet, TX_BUFFER_NUM, false) != MCP2515Error::AGAIN) {
            return (packet->_status & status ? MCP2515Error::OK : MCP2515Error::BADF);
        }

        if (!nowait) {
            delayMicroseconds(10);
        }
    } while (!nowait && (timeout == 0 || (millis() - ms) < timeout));

    return MCP2515Error::AGAIN;
}

MCP2515Error MCP2515::handleMessageTransmit(CANPacket* packet, int n, bool cond) {
    if ((packet->_status & CANPacket::STATUS_TX_PENDING) == 0) {
        if (packet->_status & CANPacket::STATUS_TX_WRITTEN) {
            return MCP2515Error::OK;
        }

        return MCP2515Error::INVAL;
    }

    int status = MCP2515Error::AGAIN;

    do {
        uint8_t txbctrl = readRegister(REG_TXBnCTRL(n));

        if ((packet->_status & CANPacket::STATUS_TX_PENDING) == 0) {
            return determineReturnCodeByPacketStatus(packet);
        }

        if (txbctrl & 0x40) {
            modifyRegister(REG_CANCTRL, 0x10, 0x00);

            packet->_status &= ~CANPacket::STATUS_TX_PENDING;
            packet->_status |= CANPacket::STATUS_TX_ABORTED;

            status = MCP2515Error::INTR;
            break;
        } else if (txbctrl & 0x10) {
            packet->_status &= ~CANPacket::STATUS_TX_PENDING;
            packet->_status |= CANPacket::STATUS_TX_ERROR;

            status = MCP2515Error::BADF;
            break;
        } else if ((txbctrl & 0x08) == 0) {
            packet->_status &= ~CANPacket::STATUS_TX_PENDING;
            packet->_status |= CANPacket::STATUS_TX_SENT;

            status = MCP2515Error::OK;
            break;
        }

        yield();

        if (cond) {
            delayMicroseconds(10);
        }

        if ((packet->_status & CANPacket::STATUS_TX_PENDING) == 0) {
            return determineReturnCodeByPacketStatus(packet);
        }
    } while (cond);

    if (status != MCP2515Error::AGAIN && (_oneShotMode || status != MCP2515Error::BADF)) {
        modifyRegister(REG_TXBnCTRL(n), 0x08, 0x00);
        modifyRegister(REG_CANINTF, FLAG_TXnIF(n), 0x00);
    }

    return status;
}

template<typename T, size_t row, size_t col> 
using array2d = std::array<std::array<T, col> row>;

_mcp_cnf_frequency MCP2515::getCnfForClockFrequency(MCP2515_CAN_CLOCK clock, MCP2515_CAN_SPEED baudRate) {

    const array2d<_mcp_cnf_frequency, 2, MCP2515_CAN_SPEED::_max> configs = {{
        { // 8MHz
            {0x00, 0x80, 0x00},     // CAN_1000KBPS
            {0x00, 0x90, 0x02},     // CAN_500KBPS
            {0x00, 0xB1, 0x05},     // CAN_250KBPS
            {0x00, 0xB4, 0x06},     // CAN_200KBPS
            {0x01, 0xB1, 0x05},     // CAN_125KBPS
            {0x01, 0xB4, 0x06},     // CAN_100KBPS
            {0x01, 0xBF, 0x07},     // CAN_80KBPS
            {0x03, 0xB4, 0x06},     // CAN_50KBPS
            {0x03, 0xBF, 0x07},     // CAN_40KBPS
            {0x07, 0xBF, 0x07},     // CAN_20KBPS
            {0x0F, 0xBF, 0x07},     // CAN_10KBPS
            {0x1F, 0xBF, 0x07},     // CAN_5KBPS
        },
        { // 16 MHz
            {0x00, 0xD0, 0x82},     // CAN_1000KBPS
            {0x00, 0xF0, 0x86},     // CAN_500KBPS
            {0x41, 0xF1, 0x85},     // CAN_250KBPS
            {0x01, 0xFA, 0x87},     // CAN_200KBPS
            {0x03, 0xF0, 0x86},     // CAN_125KBPS
            {0x03, 0xFA, 0x87},     // CAN_100KBPS
            {0x03, 0xFF, 0x87},     // CAN_80KBPS
            {0x07, 0xFA, 0x87},     // CAN_50KBPS
            {0x07, 0xFF, 0x87},     // CAN_40KBPS
            {0x0F, 0xFF, 0x87},     // CAN_20KBPS
            {0x1F, 0xFF, 0x87},     // CAN_10KBPS
            {0x3F, 0xFF, 0x87},     // CAN_5KBPS
        }
    }};

    if (baudRate > MCP2515_CAN_SPEED::MCP_SPEED_max || clock > MCP2515_CAN_CLOCK::MCP_CLOCK_max)
        return {};
    return configs[clock][baudRate];
}

void MCP2515::onInterrupt() {
#ifdef __MCP2515_MULTI_INTERRUPTS_ENABLE__
    int pin = interruptGetCalledPin();
    if (pin < 0) {
        return;
    }

    mcp_interrupt_info info;
    if (_mcp_interrupts_find(pin, &info) >= 0) {
        info.mcp->_handleInterruptPacket();
    }
#else
    if (GLOBAL_MCP != nullptr) {
        GLOBAL_MCP->_handleInterruptPacket();
    }
#endif
}

void MCP2515::_handleInterruptPacket() {
    uint8_t intf = readRegister(REG_CANINTF);

    if (intf == 0) {
        return;
    }

    int n = TX_BUFFER_NUM;

#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    if (intf & FLAG_TXnIF(n)) {
        if (!_canpacketTxQueue.empty()) {
            CANPacket* packet = _canpacketTxQueue.front();
            _canpacketTxQueue.pop();

            packet->_status &= ~(CANPacket::STATUS_TX_PENDING | CANPacket::STATUS_TX_ABORTED);
            packet->_status |= CANPacket::STATUS_TX_SENT;
        }
    } else if (intf & 0x80) {
        if (!_canpacketTxQueue.empty()) {
            CANPacket* packet = _canpacketTxQueue.front();
            _canpacketTxQueue.pop();

            packet->_status &= ~(CANPacket::STATUS_TX_PENDING | CANPacket::STATUS_TX_ABORTED);
            packet->_status |= CANPacket::STATUS_TX_ERROR;
        }
    }
#endif

    // Clear all TXnIF + MERRF bits
    modifyRegister(REG_CANINTF, 0x9C, 0x00);

    if ((intf & FLAG_RXnIF(0)) == 0 && (intf & FLAG_RXnIF(1)) == 0) {
        return;
    }

    while (true) {
        CANPacket packet = CANPacket();

        if (receivePacket(&packet) == 0) {
            _onReceivePacket(&packet);
        } else {
            break;
        }
    }

    processTxQueue();
}

void MCP2515::reset() {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    SPI.transfer(0xC0);

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();

    delayMicroseconds(10);
}

uint8_t MCP2515::readRegister(uint8_t address) {
    uint8_t value;

    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    SPI.transfer(0x03);
    SPI.transfer(address);
    value = SPI.transfer(0x00);

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();

    return value;
}

void MCP2515::modifyRegister(uint8_t address, uint8_t mask, uint8_t value) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    SPI.transfer(0x05);
    SPI.transfer(address);
    SPI.transfer(mask);
    SPI.transfer(value);

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}

void MCP2515::writeRegister(uint8_t address, uint8_t value) {
    SPI.beginTransaction(_spiSettings);

    digitalWrite(_csPin, LOW);
    SPI.transfer(0x02);
    SPI.transfer(address);
    SPI.transfer(value);

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}
