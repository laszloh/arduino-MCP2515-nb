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

#include <etl/vector>
#include <vector>
#include <bitmask.hpp>

enum class CanPacketStatus {
    RX_OK = (1<<0),
    RX_INVALID_MESSAGE = (1 << 1),

    TX_PENDING = (1 << 2),
    TX_WRITTEN = (1 << 3), // packet written to CAN controller, confirmation pending
    TX_SENT = (1 << 4),

    TX_ABORT_REQUESTED = (1 << 5),
    TX_ABORTED = (1 << 6),

    TX_ERROR = (1 << 7),

    _bitmask_max_element = TX_ERROR
};
BITMASK_DEFINE(CanPacketStatus)

class CANPacket : public Print {
    friend class MCP2515;

public:

    CANPacket() = default;
    ~CANPacket() = default;

    // copy operators
    CANPacket(const CANPacket&) = default;
    CANPacket &operator =(const CANPacket&) = default;

    // move operators
    CANPacket(CANPacket&&) = default;
    CANPacket &operator=(CANPacket &&) = default;

    bool isValid() const { return _ended; }
    bool isExtended() const { return _extended; }
    bitmask::bitmask<CanPacketStatus> getStatus() const { return _status; }

    uint32_t getId() const { return _id; }
    int getDlc() const { return _data.size(); }
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
        if(_data.full() || size > _data.available()) {
            setWriteError(MCP2515_ERRORCODES::INVAL);
            return 0;
        }
        std::copy(std::begin(buffer), std::begin(buffer) + size, std::advance(_data.begin(), _dataLength));
        _dataLength += size;
        clearWriteError();
        return size;
    }
    virtual int availableForWrite() override { return _data.available(); }

    int end() {
        if (!_started) {
            return MCP2515_ERRORCODES::PERM;
        }

        _ended = true;

        return MCP2515_ERRORCODES::OK;
    }

private:
    int startCanPacket(uint32_t id, int dlc, bool rtr, bool extended) {
        if (_started) 
            return MCP2515_ERRORCODES::PERM;

        if ( (extended && id > 0x3FFFFFFF) || (!extended && id > 0x3FF) ) 
            return MCP2515_ERRORCODES::INVAL;

        if (dlc > 8)
            return MCP2515_ERRORCODES::INVAL;
        else if(dlc >= 0)
            _data.resize(dlc);

        _extended = extended;
        _rtr = rtr;
        _data.clear();
    }

    bool _started{false};
    bool _ended{false};
    bool _aborted{false};
    bitmask::bitmask<CanPacketStatus> status{};

    bool _extended{false};
    uint32_t _id{0};
    bool _rtr{false};
    etl::vector<uint8_t, 8> _data{};

    return MCP2515_ERRORCODES::OK;
};

static constexpr size_t x = sizeof(CANPacket);

#endif
