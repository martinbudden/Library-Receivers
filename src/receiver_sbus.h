#pragma once

#include "receiver_serial.h"


/*!
SBUS receiver protocol, used primarily by Futaba and FrSky receivers.
*/
class ReceiverSbus : public ReceiverSerial {
public:
    // 8E2
    static constexpr uint8_t DATA_BITS = 8;
    static constexpr uint8_t PARITY = SerialPort::PARITY_EVEN;
    static constexpr uint8_t  STOP_BITS = 2;
    static constexpr uint32_t BAUD_RATE = 100000;
    static constexpr uint32_t FAST_BAUDRATE = 200000;
     // 16 11-bit channels (includes 4 main stick channels) and 2 flag channels
    static constexpr uint8_t CHANNEL_11_BIT_COUNT = 16;
    static constexpr uint32_t CHANNEL_COUNT = 18;
    static constexpr uint8_t SBUS_START_BYTE = 0x0F;
    static constexpr uint8_t SBUS_END_BYTE = 0x00;
    static constexpr uint32_t TIME_NEEDED_PER_FRAME_US = 3000;
public:
    explicit ReceiverSbus(SerialPort& serialPort);
private:
    // ReceiverSbus is not copyable or moveable
    ReceiverSbus(const ReceiverSbus&) = delete;
    ReceiverSbus& operator=(const ReceiverSbus&) = delete;
    ReceiverSbus(ReceiverSbus&&) = delete;
    ReceiverSbus& operator=(ReceiverSbus&&) = delete;
public:
    virtual bool on_data_received_from_isr(uint8_t data) override;
    virtual uint16_t get_channel_pwm(size_t index) const override;
    virtual bool unpack_packet() override;
private:
    enum { PACKET_SIZE = 25 };
    std::array<uint8_t, PACKET_SIZE> _packet_isr {};
    std::array<uint8_t, PACKET_SIZE> _packet {};
    std::array<uint16_t, CHANNEL_COUNT> _channels {};
};
