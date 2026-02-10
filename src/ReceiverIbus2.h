#pragma once

#include "ReceiverSerial.h"


/*!
IBUS receiver protocol, used by Flysky receivers.
*/
class ReceiverIbus : public ReceiverSerial {
public:
     // 8N1
    static constexpr uint8_t DATA_BITS = 8;
    static constexpr uint8_t PARITY = SerialPort::PARITY_NONE;
    static constexpr uint8_t STOP_BITS = 1;
    static constexpr uint32_t BAUD_RATE = 115200;

    static constexpr uint32_t CHANNEL_COUNT = 18;
    static constexpr uint8_t SLOT_COUNT = 14;

    static constexpr uint32_t TIME_NEEDED_PER_FRAME_US = 3000;
    static constexpr uint8_t MODEL_IA6 = 0;
    static constexpr uint8_t MODEL_IA6B = 1;
    static constexpr uint8_t SERIAL_RX_PACKET_LENGTH = 32;
    static constexpr uint8_t TELEMETRY_PACKET_LENGTH = 4;
public:
    explicit ReceiverIbus(SerialPort& serialPort);
private:
    // ReceiverIbus is not copyable or moveable
    ReceiverIbus(const ReceiverIbus&) = delete;
    ReceiverIbus& operator=(const ReceiverIbus&) = delete;
    ReceiverIbus(ReceiverIbus&&) = delete;
    ReceiverIbus& operator=(ReceiverIbus&&) = delete;
public:
    virtual bool on_data_received_from_isr(uint8_t data) override;
    virtual uint16_t get_channel_pwm(size_t index) const override;
    virtual bool unpack_packet() override;
    uint16_t calculate_checksum() const;
    uint16_t get_received_checksum() const { return _packet[_frame_size - 2] + static_cast<uint16_t>(_packet[_frame_size - 1] << 8U); }
// for testing;
    uint8_t getModel() const { return _model; }
    uint8_t get_sync_byte() const { return _sync_byte; }
    uint8_t get_frame_size() const { return _frame_size; }
    uint8_t get_channel_offset() const { return _channel_offset; }
    uint8_t get_packet(size_t index) const { return _packet[index]; }
private:
    enum { PACKET_SIZE = 32 };
    std::array<uint8_t, PACKET_SIZE> _packet_isr {};
    std::array<uint8_t, PACKET_SIZE> _packet {};
    std::array<uint16_t, CHANNEL_COUNT> _channels {};
    uint8_t _model {};
    uint8_t _sync_byte {};
    uint8_t _frame_size {};
    uint8_t _channel_offset {};
};
