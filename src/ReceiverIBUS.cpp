#include "ReceiverIBUS.h"


ReceiverIBUS::ReceiverIBUS(SerialPort& serialPort) :
    ReceiverSerial(serialPort)
{
    _auxiliaryChannelCount = CHANNEL_COUNT - STICK_COUNT;
}

/*!
Maps channels in range [1000,2000] to floats in range [0,1] for throttle, [-1,1] for roll, pitch yaw
*/
void ReceiverIBUS::getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const
{
    throttleStick = (static_cast<float>(_channels[THROTTLE]) - CHANNEL_RANGE_F) / CHANNEL_RANGE_F;
    rollStick = (static_cast<float>(_channels[ROLL]) - CHANNEL_MIDDLE_F) / CHANNEL_RANGE_F;
    pitchStick = (static_cast<float>(_channels[PITCH]) - CHANNEL_MIDDLE_F) / CHANNEL_RANGE_F;
    yawStick = (static_cast<float>(_channels[YAW]) - CHANNEL_MIDDLE_F) / CHANNEL_RANGE_F;
}

uint16_t ReceiverIBUS::getChannelPWM(size_t index) const
{
    if (index >= CHANNEL_COUNT) {
        return CHANNEL_LOW;
    }
    return _channels[index];
}

/*!
Called from within ReceiverSerial ISR.
*/
bool ReceiverIBUS::onDataReceivedFromISR(uint8_t data)
{
    const timeUs32_t timeNowUs = timeUs();
    enum { TIME_ALLOWANCE = 500 };
    if (timeNowUs > _startTime + TIME_NEEDED_PER_FRAME_US) { // cppcheck-suppress unsignedLessThanZero
        _packetIndex = 0;
        ++_droppedPacketCount;
    }

    enum { IA6_SYNC_BYTE = 0x55 };
    if (_packetIndex == 0) {
        if ((data == SERIAL_RX_PACKET_LENGTH) || (data == TELEMETRY_PACKET_LENGTH)) {
            _model = MODEL_IA6B;
            _syncByte = data;
            _frameSize = data;
            _channelOffset = 2;
        } else if ((_syncByte == 0) && (data == IA6_SYNC_BYTE)) {
            _model = MODEL_IA6;
            _syncByte = IA6_SYNC_BYTE;
            enum { IA6_FRAME_SIZE = 31 };
            _frameSize = IA6_FRAME_SIZE;
            _channelOffset = 1;
        } else if (_syncByte != data) {
            return false;
        }
        _startTime = timeNowUs;
    }

    _packetISR[_packetIndex++] = data;

    if (_packetIndex == PACKET_SIZE) {
        _packetIndex = 0;
        _packet = _packetISR;
        return true;
    }
    return false;
}

uint16_t ReceiverIBUS::calculateChecksum() const
{
    uint16_t checksum = (_model == MODEL_IA6) ? 0x0000 : 0xFFFF; // NOLINT(misc-const-correctness,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    size_t offset = _channelOffset;
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
bool ReceiverIBUS::unpackPacket()
{
    if (calculateChecksum() != getReceivedChecksum()) {
        _packetIsEmpty = true;
        return false;
    }

    size_t offset = _channelOffset;
    for (size_t ii = 0; ii < SLOT_COUNT; ++ii) {
        _channels[ii] = _packet[offset] + ((_packet[offset + 1] & 0x0F) << 8U);
        offset += 2;
    }

    // later IBUS receivers increase channel count by using previously unused 4 bits of each channel
    offset = _channelOffset + 1;
    for (size_t ii = SLOT_COUNT; ii < CHANNEL_COUNT; ++ii) {
        _channels[ii] = ((_packet[offset] & 0xF0) >> 4) | (_packet[offset + 2] & 0xF0) | ((_packet[offset + 4] & 0xF0) << 4);
        offset += 6; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    }
    _packetIsEmpty = false;
    return true;
}
