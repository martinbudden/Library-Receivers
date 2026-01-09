# pragma once

/*!
Packet definitions of telemetry data useful to any Stabilized Vehicle.
*/
#include "ReceiverBase.h"
#include <array>


#pragma pack(push, 1)
/*!
Packet for the transmission of Receiver telemetry data.
*/
struct TD_RECEIVER {
    enum { TYPE = 10 };
    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_RECEIVER)}; //!< length of whole packet, ie sizeof(TD_RECEIVER)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    uint16_t tickInterval {0}; //!< tick number of ticks since last receiver update
    uint16_t droppedPacketCount {0}; //!< the number of packets dropped by the receiver
    struct data_t {
        ReceiverBase::controls_t controls;
        uint32_t switches;
        std::array<uint16_t, 4> aux; //!< 4 auxiliary channels
    };
    data_t data;
};
#pragma pack(pop)
