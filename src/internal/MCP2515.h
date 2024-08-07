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

#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    #include <etl/queue>
    #include <queue>
#endif

#include <Arduino.h>
#include <SPI.h>

// According to VS there is a "OVERFLOW" macro defined in corecrt_math.h
#undef OVERFLOW

#if defined(ARDUINO_ARCH_SAMD)
    #define __MCP2515_MULTI_INTERRUPTS_ENABLE__ 1
#else
    #undef __MCP2515_MULTI_INTERRUPTS_ENABLE__
#endif

enum class MCP2515_CAN_CLOCK : uint8_t {
    MCP_8MHZ = 0,
    MCP_16MHZ
};

enum class MCP2515_CAN_SPEED : uint8_t {
    CAN_5KBPS = 0,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_40KBPS,
    CAN_50KBPS,
    CAN_80KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_200KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS,

    _max
};

enum class MCP2515_CAN_MASK : uint8_t {
    MASK0 = 0,
    MASK1 = 1
};

enum class MCP2515_CAN_RXF : uint8_t {
    RXF0 = 0,
    RXF1 = 1,
    RXF2 = 2,
    RXF3 = 3,
    RXF4 = 4,
    RXF5 = 5
};

enum class MCP2515_ERRORCODES {
    OK = 0,
    PERM = -1,
    NOENT = -2,
    INTR = -4,
    BADF = -9,
    AGAIN = -11,
    BUSY = -16,
    INVAL = -22,
    COMM = -70,
    OVERFLOW = -75
};

enum class MCP2515_MODES {
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

class MCP2515 {

public:
    MCP2515(int CSPin = MCP2515_DEFAULT_CS_PIN, int intPin = MCP2515_DEFAULT_INT_PIN, MCP2515_CAN_CLOCK clockFrequency = MCP2515_CAN_CLOCK::MCP_16MHZ, SPIClass &spi = SPI);
    ~MCP2515();

    int begin(MCP2515_CAN_SPEED baudRate);
    void end();

    uint8_t getStatus();
    uint8_t getErrorFlags();

    void setPins(int cs = MCP2515_DEFAULT_CS_PIN, int irq = MCP2515_DEFAULT_INT_PIN);
    void setSPIFrequency(uint32_t frequency);
    void setClockFrequency(MCP2515_CAN_CLOCK clockFrequency);

    int setMask(const MCP2515_CAN_MASK num, bool extended, uint32_t mask);
    int setFilter(const MCP2515_CAN_RXF num, bool extended, uint32_t filter);

    int getMode();
    int setConfigMode();
    int setListenMode(bool allowInvalidPackets = false);
    int setLoopbackMode();
    int setSleepMode();
    int setNormalMode();

    int setWakeupFilter(bool enable);
    int setOneShotMode(bool enable);

    int receivePacket(CANPacket* packet);
    void onReceivePacket(void(*callback)(CANPacket*));

    size_t getTxQueueLength();
    void processTxQueue();

    int writePacket(CANPacket* packet, bool nowait = false);
    int abortPacket(CANPacket* packet, bool nowait = false);
    int waitForPacketStatus(CANPacket* packet, unsigned long status, bool nowait = false, unsigned long timeout = 0);

    static void onInterrupt();
    void _handleInterruptPacket();

private:
    void reset();

    uint8_t readRegister(uint8_t address);
    void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
    void writeRegister(uint8_t address, uint8_t value);

    int handleMessageTransmit(CANPacket* packet, int n, bool cond);

private:
    int _csPin;
    int _intPin;
    MCP2515_CAN_CLOCK _clockFrequency;
    SPISettings _spiSettings{10e6, MSBFIRST, SPI_MODE0};
    SPIClass &_spi;

    void (*_onReceivePacket)(CANPacket*);

    bool _oneShotMode{false};
    bool _allowInvalidRx{false};
    uint8_t _rxErrorCount{0};

#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    etl::queue<CANPacket*, MCP2515_CANPACKET_TX_QUEUE_SIZE> _canpacketTxQueue{};
#endif
};

#endif
