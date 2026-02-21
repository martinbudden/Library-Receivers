#include "receiver_virtual.h"


ReceiverVirtual::ReceiverVirtual()
{
    _auxiliary_channel_count = static_cast<uint32_t>(CHANNEL_COUNT) - static_cast<uint32_t>(STICK_COUNT);
}

int32_t ReceiverVirtual::WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait)
{
    (void)ticksToWait;
    return 0;
}

/*!
If a packet was received then unpack it and inform the motor controller there are new stick values.

Returns true if a packet has been received.
*/
bool ReceiverVirtual::update(uint32_t tick_count_delta)
{
    (void)tick_count_delta;

    ++_packet_count;
    _dropped_packet_count = static_cast<int32_t>(_received_packet_count) - _packet_count;
    _dropped_packet_count_delta = _dropped_packet_count - _dropped_packet_count_previous;
    _dropped_packet_count_previous = _dropped_packet_count;

    _new_packet_available = true;
    return true;
}

bool ReceiverVirtual::unpack_packet()
{
    return true;
}

uint16_t ReceiverVirtual::get_channel_pwm(size_t index) const
{
    // map switches to the auxiliary channels
    if (index < STICK_COUNT) {
        return CHANNEL_LOW;
    }
    if (index >= _auxiliary_channel_count + STICK_COUNT) {
        return CHANNEL_LOW;
    }
    const uint16_t pwm_value = _pwm_values[index];
    return (pwm_value == 0) ? get_switch(index - STICK_COUNT) ? CHANNEL_HIGH : CHANNEL_LOW : pwm_value;
}

void ReceiverVirtual::set_channel_pwm(size_t index, uint16_t pwm_value)
{
    if (index < CHANNEL_COUNT) {
        _pwm_values[index] = pwm_value;
        if (index >=  STICK_COUNT) {
            set_switch(index- STICK_COUNT, pwm_value < CHANNEL_MIDDLE ? 0 : 1);
        }
    }
}
