#include "ReceiverIBUS.h"


ReceiverIbus::ReceiverIbus(SerialPort& serialPort) :
    ReceiverSerial(serialPort)
{
    _auxiliary_channel_count = CHANNEL_COUNT - STICK_COUNT;
}

uint16_t ReceiverIbus::get_channel_pwm(size_t index) const
{
    if (index >= CHANNEL_COUNT) {
        return CHANNEL_LOW;
    }
    return _channels[index];
}

/*!
Called from within ReceiverSerial ISR.
*/
bool ReceiverIbus::on_data_received_from_isr(uint8_t data)
{
    const timeUs32_t timeNowUs = timeUs();
    enum { TIME_ALLOWANCE = 500 };
    if (timeNowUs > _start_time + TIME_NEEDED_PER_FRAME_US) { // cppcheck-suppress unsignedLessThanZero
        _packet_index = 0;
        ++_dropped_packet_count;
    }

    enum { IA6_SYNC_BYTE = 0x55 };
    if (_packet_index == 0) {
        if ((data == SERIAL_RX_PACKET_LENGTH) || (data == TELEMETRY_PACKET_LENGTH)) {
            _model = MODEL_IA6B;
            _sync_byte = data;
            _frame_size = data;
            _channel_offset = 2;
        } else if ((_sync_byte == 0) && (data == IA6_SYNC_BYTE)) {
            _model = MODEL_IA6;
            _sync_byte = IA6_SYNC_BYTE;
            enum { IA6_FRAME_SIZE = 31 };
            _frame_size = IA6_FRAME_SIZE;
            _channel_offset = 1;
        } else if (_sync_byte != data) {
            return false;
        }
        _start_time = timeNowUs;
    }

    _packet_isr[_packet_index] = data;
    ++_packet_index;

    if (_packet_index == PACKET_SIZE) {
        _packet_index = 0;
        _packet = _packet_isr;
        return true;
    }
    return false;
}

uint16_t ReceiverIbus::calculate_checksum() const
{
    uint16_t checksum = (_model == MODEL_IA6) ? 0x0000 : 0xFFFF; // NOLINT(misc-const-correctness,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    size_t offset = _channel_offset;
    for (size_t ii = 0; ii < SLOT_COUNT; ++ii) {
        checksum += _packet[offset];
        checksum += static_cast<uint16_t>(_packet[offset + 1] << 8U);
        offset += 2;
    }
    return checksum;
}

/*!
If the packet is valid then unpack it into the member data and set the packet to empty.

Returns true if a valid packet received, false otherwise.
*/
bool ReceiverIbus::unpack_packet()
{
    if (calculate_checksum() != get_received_checksum()) {
        _packet_is_empty = true;
        return false;
    }

    size_t offset = _channel_offset;
    for (size_t ii = 0; ii < SLOT_COUNT; ++ii) {
        _channels[ii] = _packet[offset] + ((_packet[offset + 1] & 0x0F) << 8U);
        offset += 2;
    }

    // later IBUS receivers increase channel count by using previously unused 4 bits of each channel
    offset = _channel_offset + 1;
    for (size_t ii = SLOT_COUNT; ii < CHANNEL_COUNT; ++ii) {
        _channels[ii] = ((_packet[offset] & 0xF0) >> 4) | (_packet[offset + 2] & 0xF0) | ((_packet[offset + 4] & 0xF0) << 4);
        offset += 6; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    }

    _controls.throttle = (static_cast<float>(_channels[THROTTLE]) - CHANNEL_RANGE_F) / CHANNEL_RANGE_F;
    _controls.roll = (static_cast<float>(_channels[ROLL]) - CHANNEL_MIDDLE_F) / CHANNEL_RANGE_F;
    _controls.pitch = (static_cast<float>(_channels[PITCH]) - CHANNEL_MIDDLE_F) / CHANNEL_RANGE_F;
    _controls.yaw = (static_cast<float>(_channels[YAW]) - CHANNEL_MIDDLE_F) / CHANNEL_RANGE_F;

    _controls_pwm.throttle = _channels[THROTTLE];
    _controls_pwm.roll = _channels[ROLL];
    _controls_pwm.pitch = _channels[PITCH];
    _controls_pwm.yaw = _channels[YAW];

    _packet_is_empty = false;
    return true;
}
