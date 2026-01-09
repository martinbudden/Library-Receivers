#pragma once

#include "ReceiverSerial.h"


/*!
CRSF receiver protocol'
*/
class ReceiverCRSF : public ReceiverSerial {
public:
    static constexpr uint32_t CHANNEL_COUNT = 16;
    enum { BAUD_RATE = 416666, BAUD_RATE_UNOFFICIAL = 420000 };
    enum { DATA_BITS = 8, PARITY = SerialPort::PARITY_NONE, STOP_BITS = 1 }; // 8N1
    enum { TIME_NEEDED_PER_FRAME_US = 1750 };

    enum { CRSF_SYNC_BYTE = 0xC8, EDGE_TX_SYNC_BYTE = 0xEE };
    // see https://github.com/crsf-wg/crsf/wiki/Packet-Types
    enum frametype_e { 
        FRAMETYPE_GPS = 0x02,
        FRAMETYPE_VARIO_SENSOR = 0x07,
        FRAMETYPE_BATTERY_SENSOR = 0x08,
        FRAMETYPE_BARO_ALTITUDE = 0x09,
        FRAMETYPE_HEARTBEAT = 0x0B,
        FRAMETYPE_LINK_STATISTICS = 0x14,
        FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
        FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
        FRAMETYPE_LINK_STATISTICS_RX = 0x1C,
        FRAMETYPE_LINK_STATISTICS_TX = 0x1D,
        FRAMETYPE_ATTITUDE = 0x1E,
        FRAMETYPE_FLIGHT_MODE = 0x21,
        // Extended Header Frames, range: 0x28 to 0x96
        FRAMETYPE_DEVICE_PING = 0x28,
        FRAMETYPE_DEVICE_INFO = 0x29,
        FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
        FRAMETYPE_PARAMETER_READ = 0x2C,
        FRAMETYPE_PARAMETER_WRITE = 0x2D,
        FRAMETYPE_COMMAND = 0x32,
        // MSP commands
        FRAMETYPE_MSP_REQ = 0x7A,
        FRAMETYPE_MSP_RESP = 0x7B,
        FRAMETYPE_MSP_WRITE = 0x7C,
        FRAMETYPE_DISPLAYPORT_CMD = 0x7D,
        FRAMETYPE_ARDUPILOT_RESP = 0x80
    };
    enum address_e {
        ADDRESS_BROADCAST = 0x00,
        ADDRESS_USB = 0x10,
        ADDRESS_TBS_CORE_PNP_PRO = 0x80,
        ADDRESS_RESERVED1 = 0x8A,
        ADDRESS_CURRENT_SENSOR = 0xC0,
        ADDRESS_GPS = 0xC2,
        ADDRESS_TBS_BLACKBOX = 0xC4,
        ADDRESS_FLIGHT_CONTROLLER = 0xC8,
        ADDRESS_RESERVED2 = 0xCA,
        ADDRESS_RACE_TAG = 0xCC,
        ADDRESS_RADIO_TRANSMITTER = 0xEA,
        ADDRESS_CRSF_RECEIVER = 0xEC,
        ADDRESS_CRSF_TRANSMITTER = 0xEE
    };
    enum {
        COMMAND_SUBCMD_RX_BIND = 0x01,
        COMMAND_SUBCMD_RX = 0x10,
        COMMAND_SUBCMD_GENERAL = 0x0A,
        COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL = 0x70,
        COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE = 0x71,
    };

    enum { MAX_PACKET_SIZE = 64 };
    union packet_u { 
        std::array<uint8_t, MAX_PACKET_SIZE> data;
        struct value_t {
            uint8_t sync;
            uint8_t length; // length is length of type, payload, and CRC
            uint8_t type;
            // payload includes CRC byte at end. CRC includes all bytes from type to end of payload
            std::array<uint8_t, MAX_PACKET_SIZE - 3> payload;
        } value;
    };
#pragma pack(push, 1)
    struct rc_channels_packed_t {
        // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
        unsigned int chan0 : 11;
        unsigned int chan1 : 11;
        unsigned int chan2 : 11;
        unsigned int chan3 : 11;
        unsigned int chan4 : 11;
        unsigned int chan5 : 11;
        unsigned int chan6 : 11;
        unsigned int chan7 : 11;
        unsigned int chan8 : 11;
        unsigned int chan9 : 11;
        unsigned int chan10 : 11;
        unsigned int chan11 : 11;
        unsigned int chan12 : 11;
        unsigned int chan13 : 11;
        unsigned int chan14 : 11;
        unsigned int chan15 : 11;
    };
#pragma pack(pop)
public:
    explicit ReceiverCRSF(SerialPort& serialPort);
private:
    // Receiver is not copyable or moveable
    ReceiverCRSF(const ReceiverCRSF&) = delete;
    ReceiverCRSF& operator=(const ReceiverCRSF&) = delete;
    ReceiverCRSF(ReceiverCRSF&&) = delete;
    ReceiverCRSF& operator=(ReceiverCRSF&&) = delete;
public:
    virtual bool onDataReceivedFromISR(uint8_t data) override;
    virtual void getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const override;
    virtual uint16_t getChannelPWM(size_t index) const override;
    virtual bool unpackPacket() override;
    static uint8_t calculateCRC(uint8_t crc, uint8_t value);
    uint8_t calculateCRC() const;
    uint8_t getReceivedCRC() const;
// for debug
    uint8_t getPacketSync() const { return _packet.value.sync; }
    uint8_t getPacketLength() const { return _packet.value.length; }
    uint8_t getPacketType() const { return _packet.value.type; }
private:
    enum { MAX_PAYLOAD_SIZE = MAX_PACKET_SIZE - 6 };
    uint32_t _packetSize {};
    uint32_t _packetType {};
    packet_u _packetISR {};
    packet_u _packet {};
    std::array<uint16_t, CHANNEL_COUNT> _channels {};
};
