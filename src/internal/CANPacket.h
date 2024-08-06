/**
 * CAN MCP2515_nb
 * Copyright 2020 WitchCraftWorks Team, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#ifndef CANPACKET_H
#define CANPACKET_H

#include <Arduino.h>
#include "MCP2515.h"

class CANPacket : public Print {
    friend class MCP2515;

public:
    struct Status{
        bool rxOK : 1;
        bool rxInvalidMessage : 1;
        bool txPending : 1;
        bool txWritten : 1;
        bool txSend : 1;
        bool txAbortRequest : 1;
        bool txAborted : 1;
        bool txError : 1;

        Status() : rxOk(false), rxInvalidMessage(false), txPending(false), 
                    txWritten(false), txSend(false), txAbortRequest(false), txAborted(false), txError(false) { }

    };
    // static constexpr size_t x = sizeof(Status);
    // static const unsigned long STATUS_RX_OK = (1 << 0);
    // static const unsigned long STATUS_RX_INVALID_MESSAGE = (1 << 1);

    // static const unsigned long STATUS_TX_PENDING = (1 << 4);
    // static const unsigned long STATUS_TX_WRITTEN = (1 << 5); // packet written to CAN controller, confirmation pending
    // static const unsigned long STATUS_TX_SENT = (1 << 7);

    // static const unsigned long STATUS_TX_ABORT_REQUESTED = (1 << 10);
    // static const unsigned long STATUS_TX_ABORTED = (1 << 11);

    // static const unsigned long STATUS_TX_ERROR = (1 << 14);

    CANPacket() { memset(data, 0, sizeof(data)); }
    ~CANPacket() = default;

    // copy operators
    CANPacket(const CANPacket&) = default;
    CANPacket &operator =(const CANPacket&) = default;

    // move operators
    CANPacket(CANPacket&&) = default;
    CANPacket &operator=(CANPacket &&) = default;

    bool isValid() const { return _ended; }
    bool isExtended() const { return _extended; }
    uint32_t getStatus() const { return _status; }

    uint32_t getId() const { return _id; }
    int getDlc() const { return _dlc; }
    int getRtr() const { return _rtr; };
    uint8_t* getData() const { return data; }

    int startStandard(int id, int dlc = -1, bool rtr = false) { startCanPacket(id, dlc, rtr, false); }
    int startExtended(long id, int dlc = -1, bool rtr = false) { startCanPacket(id, dlc, rtr, true); }

    virtual size_t write(uint8_t data) override { return write(&data, sizeof(data)); }
    virtual size_t write(const uint8_t *buffer, size_t size) override {
        if(!_started) {
            setWriteError(MCP2515_ERRORCODES::PERM);
            return 0;
        }
        if(_dataLength >= 8 || size > (sizeof(_data) - _dataLength)) {
            setWriteError(MCP2515_ERRORCODES::INVAL);
            return 0;
        }
        memcpy(data + _dataLength, buffer, size);
        _dataLength += size;
        clearWriteError();
        return size;
    }
    virtual int availableForWrite() override { return sizeof(data) - _dataLength; }

    int end();

private:
    int startCanPacket(uint32_t id, int dlc, bool rtr, bool extended) {
        if (_started) 
            return MCP2515_ERRORCODES::PERM;

        if ( (extended && id > 0x3FFFFFFF) || (!extended && id > 0x3FF) ) 
            return MCP2515_ERRORCODES::INVAL;

        if (dlc > 8)
            return MCP2515_ERRORCODES::INVAL;

        _extended = extended;
        _dlc = dlc;
        _rtr = rtr;
    }

    bool _started{false};
    bool _ended{false};
    bool _aborted{false};
    uint32_t _status{0};

    bool _extended{false};
    uint32_t _id{0};
    int _dlc{-1};
    bool _rtr{false};
    uint8_t _data[8];
    uint8_t _dataLength{0};
};

static constexpr size_t x = sizeof(CANPacket);

#endif
