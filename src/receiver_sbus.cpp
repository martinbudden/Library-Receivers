#include "receiver_sbus.h"


ReceiverSbus::ReceiverSbus(SerialPort& serialPort) :
    ReceiverSerial(serialPort)
{
    _auxiliary_channel_count = CHANNEL_COUNT - STICK_COUNT;
}

uint16_t ReceiverSbus::get_channel_pwm(size_t index) const
{
    if (index >= CHANNEL_COUNT) {
        return CHANNEL_LOW;
    }
    return _channels[index];
}

/*!
Called from within ReceiverSerial ISR.
*/
bool ReceiverSbus::on_data_received_from_isr(uint8_t data)
{
    const time_us32_t time_now_us = time_us();
    enum { TIME_ALLOWANCE = 500 };
    if (time_now_us > _start_time + TIME_NEEDED_PER_FRAME_US + TIME_ALLOWANCE) { // cppcheck-suppress unsignedLessThanZero
        _packet_index = 0;
        ++_dropped_packet_count;
    }

    if (_packet_index == 0) {
        if (data != SBUS_START_BYTE) {
            _packet_is_empty = true;
            return false;
        }
        _start_time = time_now_us;
    }

    _packet_isr[_packet_index] = data;
    ++_packet_index;

    if (_packet_index == PACKET_SIZE) {
        _packet_index = 0;
        if (_packet_isr[PACKET_SIZE - 1] != SBUS_END_BYTE) {
            ++_error_packet_count;
            _packet_is_empty = true;
            return false;
        }
        _packet = _packet_isr;
        return true;
    }
    return false;
}

/*!
If the packet is valid then unpack it into the member data and set the packet to empty.

Returns true if a valid packet received, false otherwise.

SBUS packet is
    1 start byte (has value be 0x0F)
    22 bytes of channel data which is 176 bits: 16 channels of 11 bits per channel
    1 flag byte, which gives flags pluse 2 1-bit channels
    1 stop byte (has value 0x00)

SBUS uses range [192,1792] which is mapped to [1000,2000] ([CHANNEL_LOW,CHANNEL_HIGH])

Some transmitters/receivers use range [172,1811] which is  clipped.
*/
bool ReceiverSbus::unpack_packet()
{
    if (_packet[PACKET_SIZE - 1] != SBUS_END_BYTE) {
        _packet_is_empty = true;
        return false;
    }
    // SBUS uses AETR (Ailerons, Elevator, Throttle, Rudder), ie ROLL, PITCH, THROTTLE, YAW
    // This is the default, so no reordering required
    _channels[0]  = _packet[1]     | _packet[2]<<8;
    _channels[1]  = _packet[2]>>3  | _packet[3]<<5;
    _channels[2]  = _packet[3]>>6  | _packet[4]<<2  | _packet[5]<<10;
    _channels[3]  = _packet[5]>>1  | _packet[6]<<7;
    _channels[4]  = _packet[6]>>4  | _packet[7]<<4;
    _channels[5]  = _packet[7]>>7  | _packet[8]<<1  | _packet[9]<<9;
    _channels[6]  = _packet[9]>>2  | _packet[10]<<6;
    _channels[7]  = _packet[10]>>5 | _packet[11]<<3;
    _channels[8]  = _packet[12]    | _packet[13]<<8;
    _channels[9]  = _packet[13]>>3 | _packet[14]<<5;
    _channels[10] = _packet[14]>>6 | _packet[15]<<2 | _packet[16]<<10;
    _channels[11] = _packet[16]>>1 | _packet[17]<<7;
    _channels[12] = _packet[17]>>4 | _packet[18]<<4;
    _channels[13] = _packet[18]>>7 | _packet[19]<<1 | _packet[20]<<9;
    _channels[14] = _packet[20]>>2 | _packet[21]<<6;
    _channels[15] = _packet[21]>>5 | _packet[22]<<3;


    // map range [192,1792] to [1000,2000]
#if true
    for (size_t ii = 0; ii < CHANNEL_11_BIT_COUNT; ++ii) {
        _channels[ii] &= 0x07FF;
        _channels[ii] = static_cast<uint16_t>(5.0F * static_cast<float>(_channels[ii]) / 8.0F) + 880;
    }
#else
    for (size_t ii = 0; ii < 16; ++ii) {
        uint32_t channel =  _channels[ii];
        channel <<= 16;
        channel *=5;
        channel >>= 19;
        channel += 880;
        _channels[ii] = static_cast<uint16_t>(channel);
    }
#endif

    enum { FLAG_CHANNEL_16 = 0x01, FLAG_CHANNEL_17 = 0x02, FLAG_LOST_FRAME = 0x04, FLAG_LOST_SIGNAL = 0x08 };
    const uint8_t flags = _packet[23];
    _channels[16] = (flags & FLAG_CHANNEL_16) ? CHANNEL_HIGH : CHANNEL_LOW;
    _channels[17] = (flags & FLAG_CHANNEL_17) ? CHANNEL_HIGH : CHANNEL_LOW;

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
