#include "ReceiverCRSF.h"


ReceiverCRSF::ReceiverCRSF(SerialPort& serialPort) :
    ReceiverSerial(serialPort)
{
    _auxiliaryChannelCount = CHANNEL_COUNT - STICK_COUNT;
}

/*!
Maps channels in range [1000,2000] to floats in range [0,1] for throttle, [-1,1] for roll, pitch yaw
*/
void ReceiverCRSF::getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const
{
    throttleStick = (static_cast<float>(_channels[THROTTLE]) - CHANNEL_RANGE_F) / CHANNEL_RANGE_F;
    rollStick = (static_cast<float>(_channels[ROLL]) - CHANNEL_MIDDLE_F) / CHANNEL_RANGE_F;
    pitchStick = (static_cast<float>(_channels[PITCH]) - CHANNEL_MIDDLE_F) / CHANNEL_RANGE_F;
    yawStick = (static_cast<float>(_channels[YAW]) - CHANNEL_MIDDLE_F) / CHANNEL_RANGE_F;
}

uint16_t ReceiverCRSF::getChannelPWM(size_t index) const
{
    if (index >= CHANNEL_COUNT) {
        return CHANNEL_LOW;
    }
    /* 
    conversion from RC value to PWM
    for FRAMETYPE_RC_CHANNELS_PACKED(0x16)
           RC     PWM
    min   172 ->  988us
    mid   992 -> 1500us
    max  1811 -> 2012us
    scale factor = (2012-988) / (1811-172) = 0.62477120195241
    offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
    */
    static constexpr float CHANNEL_SCALE = 0.62477120195241F;
    static constexpr float CHANNEL_OFFSET = 880.53935326418548F;

    return static_cast<uint16_t>(CHANNEL_SCALE * static_cast<float>(_channels[index]) + CHANNEL_OFFSET);
}

/*!
Called from within ReceiverSerial ISR.
*/
bool ReceiverCRSF::onDataReceivedFromISR(uint8_t data)
{
    const timeUs32_t timeNowUs = timeUs();
    if (timeNowUs > _startTime + TIME_NEEDED_PER_FRAME_US) { // cppcheck-suppress unsignedLessThanZero
        _packetIndex = 0;
        ++_droppedPacketCount;
    }

    switch (_packetIndex) {
    case 0:
        if (data != CRSF_SYNC_BYTE && data != EDGE_TX_SYNC_BYTE) {
            _packetIsEmpty = true;
            return false;
        }
        _startTime = timeNowUs;
        break;
    case 1:
        _packetSize = data + 2;
        break;
    case 2:
        _packetType = data;
        break;
    }

    _packetISR.data[_packetIndex++] = data;

    if (_packetSize != 0 && _packetIndex == _packetSize) {
        _packetIndex = 0;
        _packetSize = 0;
        _packet = _packetISR;
        return true;
    }
    return false;
}

uint8_t ReceiverCRSF::calculateCRC(uint8_t crc, uint8_t value)
{
    static constexpr uint8_t POLYNOMIAL = 0xD5;

    crc ^= value;
    for (int ii = 0; ii < 8; ++ii) { // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        const uint8_t topBit = crc & 0x80U;
        crc <<= 1U;
        if (topBit) {
            crc ^= POLYNOMIAL;
        }
    }
    return crc;
}

uint8_t ReceiverCRSF::calculateCRC() const
{
    uint8_t crc = calculateCRC(0, _packet.value.type);
    for (size_t ii = 2; ii < _packet.value.length; ++ii) { // length is length of type, payload, and CRC
        crc = calculateCRC(crc, _packet.value.payload[ii - 2]);
    }
    return crc;
}

uint8_t ReceiverCRSF::getReceivedCRC() const
{
    return _packet.value.payload[_packet.value.length - 2];
}

/*!
If the packet is valid then unpack it into the member data and set the packet to empty.

Returns true if a valid packet received, false otherwise.

*/
bool ReceiverCRSF::unpackPacket()
{
    if (calculateCRC() != getReceivedCRC()) {
        _packetIsEmpty = true;
        return false;
    }

    if (_packet.value.type == FRAMETYPE_RC_CHANNELS_PACKED) {
#if false
        union channels_u { 
            std::array<uint8_t, MAX_PACKET_SIZE - 3> payload;
            rc_channels_packed_t rc;
        };
        channels_u channels { .payload = _packet.value.payload };
        _channels[0] = channels.rc.chan0;
        _channels[1] = channels.rc.chan1;
        _channels[2] = channels.rc.chan2;
        _channels[3] = channels.rc.chan3;
        _channels[4] = channels.rc.chan4;
        _channels[5] = channels.rc.chan5;
        _channels[6] = channels.rc.chan6;
        _channels[7] = channels.rc.chan7;
        _channels[8] = channels.rc.chan8;
        _channels[9] = channels.rc.chan9;
        _channels[10] = channels.rc.chan10;
        _channels[11] = channels.rc.chan11;
        _channels[12] = channels.rc.chan12;
        _channels[13] = channels.rc.chan13;
        _channels[14] = channels.rc.chan14;
        _channels[15] = channels.rc.chan15;
#else
        const rc_channels_packed_t* const rcChannels = reinterpret_cast<rc_channels_packed_t*>(&_packet.value.payload[0]);
        _channels[0] = rcChannels->chan0;
        _channels[1] = rcChannels->chan1;
        _channels[2] = rcChannels->chan2;
        _channels[3] = rcChannels->chan3;
        _channels[4] = rcChannels->chan4;
        _channels[5] = rcChannels->chan5;
        _channels[6] = rcChannels->chan6;
        _channels[7] = rcChannels->chan7;
        _channels[8] = rcChannels->chan8;
        _channels[9] = rcChannels->chan9;
        _channels[10] = rcChannels->chan10;
        _channels[11] = rcChannels->chan11;
        _channels[12] = rcChannels->chan12;
        _channels[13] = rcChannels->chan13;
        _channels[14] = rcChannels->chan14;
        _channels[15] = rcChannels->chan15;
#endif
        _packetIsEmpty = false;
        return true;
    }

    _packetIsEmpty = true;
    return false;
}
