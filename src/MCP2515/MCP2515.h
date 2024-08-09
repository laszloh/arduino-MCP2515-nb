/**
 * CAN MCP2515_nb
 * Copyright 2020 WitchCraftWorks Team, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#ifndef MCP2515_H
#define MCP2515_H

#undef max
#undef min

#ifdef ARDUINO_ARCH_AVR
# include <avr_stl.h>
#endif

#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
# include <queue>
#endif

#include <Arduino.h>
#include <SPI.h>

#include "ErrorCodes.hpp"

// According to VS there is a "OVERFLOW" macro defined in corecrt_math.h
#undef OVERFLOW

#if defined(ARDUINO_ARCH_SAMD)
# define __MCP2515_MULTI_INTERRUPTS_ENABLE__ 1
#else
# undef __MCP2515_MULTI_INTERRUPTS_ENABLE__
#endif

enum MCP2515_CAN_CLOCK {
    MCP_8MHZ = (long)8e6,
    MCP_16MHZ = (long)16e6
};

enum MCP2515_CAN_SPEED {
    CAN_5KBPS = (long)5e3,
    CAN_10KBPS = (long)10e3,
    CAN_20KBPS = (long)20e3,
    CAN_40KBPS = (long)40e3,
    CAN_50KBPS = (long)50e3,
    CAN_80KBPS = (long)80e3,
    CAN_100KBPS = (long)100e3,
    CAN_125KBPS = (long)125e3,
    CAN_200KBPS = (long)200e3,
    CAN_250KBPS = (long)250e3,
    CAN_500KBPS = (long)500e3,
    CAN_1000KBPS = (long)1000e3
};

enum MCP2515_CAN_MASK {
    MASK0 = 0,
    MASK1 = 1
};

enum MCP2515_CAN_RXF {
    RXF0 = 0,
    RXF1 = 1,
    RXF2 = 2,
    RXF3 = 3,
    RXF4 = 4,
    RXF5 = 5
};

enum MCP2515_MODES {
    NORMAL = 0,
    LOOPBACK = 1,
    LISTEN = 2,
    CONFIG = 3,
    SLEEP = 4
};

// defer include to MCP2515_nb.h
class CANPacket;

#ifndef MCP2515_CANPACKET_TX_QUEUE_SIZE
# define MCP2515_CANPACKET_TX_QUEUE_SIZE 16
#endif

#define MCP2515_DEFAULT_CS_PIN  10
#define MCP2515_DEFAULT_INT_PIN 2

struct _mcp_cnf_frequency {
    uint8_t one;
    uint8_t two;
    uint8_t three;
};

class MCP2515 {

public:
    MCP2515();
    ~MCP2515();

    int begin(long baudRate);
    void end();

    uint8_t getStatus();
    uint8_t getErrorFlags();

    void setPins(int cs = MCP2515_DEFAULT_CS_PIN, int irq = MCP2515_DEFAULT_INT_PIN);
    void setSPIFrequency(uint32_t frequency);
    void setClockFrequency(long clockFrequency);

    MCP2515Error setMask(const MCP2515_CAN_MASK num, bool extended, uint32_t mask);
    MCP2515Error setFilter(const MCP2515_CAN_RXF num, bool extended, uint32_t filter);

    int getMode();
    MCP2515Error setConfigMode();
    MCP2515Error setListenMode(bool allowInvalidPackets = false);
    MCP2515Error setLoopbackMode();
    MCP2515Error setSleepMode();
    MCP2515Error setNormalMode();

    MCP2515Error setWakeupFilter(bool enable);
    MCP2515Error setOneShotMode(bool enable);

    MCP2515Error receivePacket(CANPacket* packet);
    void onReceivePacket(void(*callback)(CANPacket*));

    size_t getTxQueueLength();
    void processTxQueue();

    MCP2515Error writePacket(CANPacket* packet, bool nowait = false);
    MCP2515Error abortPacket(CANPacket* packet, bool nowait = false);
    MCP2515Error waitForPacketStatus(CANPacket* packet, unsigned long status, bool nowait = false, unsigned long timeout = 0);

    static void onInterrupt();
    void _handleInterruptPacket();
private:
    void reset();

    uint8_t readRegister(uint8_t address);
    void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
    void writeRegister(uint8_t address, uint8_t value);

    MCP2515Error handleMessageTransmit(CANPacket* packet, int n, bool cond);

    bool getCnfForClockFrequency8e6(long baudRate, _mcp_cnf_frequency* cnf);
    bool getCnfForClockFrequency16e6(long baudRate, _mcp_cnf_frequency* cnf);

private:
    int _csPin;
    int _intPin;
    long _clockFrequency;
    SPISettings _spiSettings;

    void (*_onReceivePacket)(CANPacket*);

    bool _oneShotMode = false;
    bool _allowInvalidRx = false;
    uint8_t _rxErrorCount = 0;

#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    std::queue<CANPacket*> _canpacketTxQueue;
#endif
};

#endif
