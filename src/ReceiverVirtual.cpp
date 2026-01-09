#include "ReceiverVirtual.h"


ReceiverVirtual::ReceiverVirtual()
{
    _auxiliaryChannelCount = static_cast<uint32_t>(CHANNEL_COUNT) - static_cast<uint32_t>(STICK_COUNT);
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
bool ReceiverVirtual::update(uint32_t tickCountDelta)
{
    (void)tickCountDelta;

    ++_packetCount;
    _droppedPacketCount = static_cast<int32_t>(_receivedPacketCount) - _packetCount;
    _droppedPacketCountDelta = _droppedPacketCount - _droppedPacketCountPrevious;
    _droppedPacketCountPrevious = _droppedPacketCount;

    _newPacketAvailable = true;
    return true;
}

bool ReceiverVirtual::unpackPacket()
{
    return true;
}

void ReceiverVirtual::getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const
{
    throttleStick = _controls.throttle;
    rollStick = _controls.roll;
    pitchStick = _controls.pitch;
    yawStick = _controls.yaw;
}

uint16_t ReceiverVirtual::getChannelPWM(size_t index) const
{
    // map switches to the auxiliary channels
    if (index < STICK_COUNT) {
        return CHANNEL_LOW;
    }
    if (index >= _auxiliaryChannelCount + STICK_COUNT) {
        return CHANNEL_LOW;
    }
    const uint16_t pwmValue = _pwmValues[index];
    return (pwmValue == 0) ? getSwitch(index - STICK_COUNT) ? CHANNEL_HIGH : CHANNEL_LOW : pwmValue;
}

void ReceiverVirtual::setChannelPWM(size_t index, uint16_t pwmValue)
{
    if (index < CHANNEL_COUNT) {
        _pwmValues[index] = pwmValue;
        if (index >=  STICK_COUNT) {
            setSwitch(index- STICK_COUNT, pwmValue < CHANNEL_MIDDLE ? 0 : 1);
        }
    }
}
