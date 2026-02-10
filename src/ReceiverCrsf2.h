#pragma once

#include "ReceiverSerial.h"


/*!
CRSF receiver protocol'
*/
class ReceiverCrsf : public ReceiverSerial {
public:
     // 8N1
    static constexpr uint8_t DATA_BITS = 8;
    static constexpr uint8_t PARITY = SerialPort::PARITY_NONE;
    static constexpr uint8_t STOP_BITS = 1;
    static constexpr uint32_t BAUD_RATE = 416666;
    static constexpr uint32_t BAUD_RATE_UNOFFICIAL = 420000;

    static constexpr uint32_t CHANNEL_COUNT = 16;
    static constexpr uint32_t TIME_NEEDED_PER_FRAME_US = 1750;

    static constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;
    static constexpr uint8_t EDGE_TX_SYNC_BYTE = 0xEE;

    // see https://github.com/crsf-wg/crsf/wiki/Packet-Types
    static constexpr uint8_t FRAMETYPE_GPS = 0x02;
    static constexpr uint8_t FRAMETYPE_VARIO_SENSOR = 0x07;
    static constexpr uint8_t FRAMETYPE_BATTERY_SENSOR = 0x08;
    static constexpr uint8_t FRAMETYPE_BARO_ALTITUDE = 0x09;
    static constexpr uint8_t FRAMETYPE_HEARTBEAT = 0x0B;
    static constexpr uint8_t FRAMETYPE_LINK_STATISTICS = 0x14;
    static constexpr uint8_t FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
    static constexpr uint8_t FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17;
    static constexpr uint8_t FRAMETYPE_LINK_STATISTICS_RX = 0x1C;
    static constexpr uint8_t FRAMETYPE_LINK_STATISTICS_TX = 0x1D;
    static constexpr uint8_t FRAMETYPE_ATTITUDE = 0x1E;
    static constexpr uint8_t FRAMETYPE_FLIGHT_MODE = 0x21;
    // Extended Header Frames; range: 0x28 to 0x96
    static constexpr uint8_t FRAMETYPE_DEVICE_PING = 0x28;
    static constexpr uint8_t FRAMETYPE_DEVICE_INFO = 0x29;
    static constexpr uint8_t FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B;
    static constexpr uint8_t FRAMETYPE_PARAMETER_READ = 0x2C;
    static constexpr uint8_t FRAMETYPE_PARAMETER_WRITE = 0x2D;
    static constexpr uint8_t FRAMETYPE_COMMAND = 0x32;
    // MSP commands
    static constexpr uint8_t FRAMETYPE_MSP_REQ = 0x7A;
    static constexpr uint8_t FRAMETYPE_MSP_RESP = 0x7B;
    static constexpr uint8_t FRAMETYPE_MSP_WRITE = 0x7C;
    static constexpr uint8_t FRAMETYPE_DISPLAYPORT_CMD = 0x7D;
    static constexpr uint8_t FRAMETYPE_ARDUPILOT_RESP = 0x80;
    static constexpr uint8_t ADDRESS_BROADCAST = 0x00;
    static constexpr uint8_t ADDRESS_USB = 0x10;
    static constexpr uint8_t ADDRESS_TBS_CORE_PNP_PRO = 0x80;
    static constexpr uint8_t ADDRESS_RESERVED1 = 0x8A;
    static constexpr uint8_t ADDRESS_CURRENT_SENSOR = 0xC0;
    static constexpr uint8_t ADDRESS_GPS = 0xC2;
    static constexpr uint8_t ADDRESS_TBS_BLACKBOX = 0xC4;
    static constexpr uint8_t ADDRESS_FLIGHT_CONTROLLER = 0xC8;
    static constexpr uint8_t ADDRESS_RESERVED2 = 0xCA;
    static constexpr uint8_t ADDRESS_RACE_TAG = 0xCC;
    static constexpr uint8_t ADDRESS_RADIO_TRANSMITTER = 0xEA;
    static constexpr uint8_t ADDRESS_CRSF_RECEIVER = 0xEC;
    static constexpr uint8_t ADDRESS_CRSF_TRANSMITTER = 0xEE;

    static constexpr uint8_t COMMAND_SUBCMD_RX_BIND = 0x01;
    static constexpr uint8_t COMMAND_SUBCMD_RX = 0x10;
    static constexpr uint8_t COMMAND_SUBCMD_GENERAL = 0x0A;
    static constexpr uint8_t COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL = 0x70;
    static constexpr uint8_t COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE = 0x71;

    static constexpr uint8_t MAX_PACKET_SIZE = 64;

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
    explicit ReceiverCrsf(SerialPort& serialPort);
private:
    // Receiver is not copyable or moveable
    ReceiverCrsf(const ReceiverCrsf&) = delete;
    ReceiverCrsf& operator=(const ReceiverCrsf&) = delete;
    ReceiverCrsf(ReceiverCrsf&&) = delete;
    ReceiverCrsf& operator=(ReceiverCrsf&&) = delete;
public:
    virtual bool on_data_received_from_isr(uint8_t data) override;
    virtual uint16_t get_channel_pwm(size_t index) const override;
    virtual bool unpack_packet() override;
    static uint8_t calculate_crc(uint8_t crc, uint8_t value);
    uint8_t calculate_crc() const;
    uint8_t get_received_crc() const;
// for debug
    uint8_t get_packet_sync() const { return _packet.value.sync; }
    uint8_t get_packet_length() const { return _packet.value.length; }
    uint8_t get_packet_type() const { return _packet.value.type; }
private:
    enum { MAX_PAYLOAD_SIZE = MAX_PACKET_SIZE - 6 };
    uint32_t _packetSize {};
    uint32_t _packet_type {};
    packet_u _packet_isr {};
    packet_u _packet {};
    std::array<uint16_t, CHANNEL_COUNT> _channels {};
};
