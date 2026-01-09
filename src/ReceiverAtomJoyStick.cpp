#include "ReceiverAtomJoyStick.h"
#include <cstring>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
//#include <HardwareSerial.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else


#endif

ReceiverAtomJoyStick::ReceiverAtomJoyStick(const uint8_t* macAddress, uint8_t channel) :
    _transceiver(macAddress, channel),
    _received_data(&_packet[0], sizeof(_packet))
{
    // switches are mapped to the auxiliary channels
    _auxiliaryChannelCount = SWITCH_COUNT;
}

/*!
Initialize the transceiver.
*/
int32_t ReceiverAtomJoyStick::init()
{
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
    const esp_err_t err = _transceiver.init(_received_data, nullptr);
    return err;
#else
    return 0;
#endif
}

int32_t ReceiverAtomJoyStick::WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait)
{
    return _transceiver.WAIT_FOR_PRIMARY_DATA_RECEIVED(ticksToWait);
}

/*!
If a packet was received from the atomJoyStickReceiver then unpack it and return true.

Returns false if an empty or invalid packet was received.
*/
bool ReceiverAtomJoyStick::update(uint32_t tickCountDelta)
{
    if (isPacketEmpty()) {
        return false;
    }

    _packetReceived = true;
    ++_packetCount;

    // record tickoutDelta for instrumentation
    _tickCountDelta = tickCountDelta;

    // track dropped packets
    _receivedPacketCount = _transceiver.getReceivedPacketCount();
    _droppedPacketCount = static_cast<int32_t>(_receivedPacketCount) - _packetCount;
    _droppedPacketCountDelta = _droppedPacketCount - _droppedPacketCountPrevious;
    _droppedPacketCountPrevious = _droppedPacketCount;

    if (unpackPacket(CHECK_PACKET)) {
        if (_packetCount == 5) { // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
            // set the bias so that the current readings are zero.
            setCurrentReadingsToBias();
        }

        // Save the stick values.
        _controls.throttle = normalizedStick(THROTTLE);
        // Atom Joystick returns throttle in range [-1.0, 1.0]
        // _positiveHalfThrottle discards range [-1.0, 0.0) for use by aerial vehicles
        // since Atom Joystick is sprung to return to center position on all axes
        if (_positiveHalfThrottle) {
            _controls.throttle = std::max(0.0F, _controls.throttle); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        }
        _controls.roll = normalizedStick(ROLL);
        _controls.pitch = normalizedStick(PITCH);
        _controls.yaw = normalizedStick(YAW);

        // Save the button values.
        setSwitch(MOTOR_ON_OFF_SWITCH, _flipButton);
        setSwitch(MODE_SWITCH, _mode);
        setSwitch(ALT_MODE_SWITCH, _altMode == 4 ? 0 : 1); // _altMode has a value of 4 or 5

        // now we have copied all the packet values, set the _newPacketAvailable flag
        // NOTE: there is no mutex around this flag
        _newPacketAvailable = true;
        return true;
    }
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
    static const char *TAG = "ReceiverAtomJoyStick::update";
    ESP_LOGI(TAG, "BadPacket");
    //Serial.printf("BadPacket\r\n");
#endif
    // we've had a packet even though it is a bad one, so we haven't lost contact with the receiver
    return true;
}

/*!
Maps the joystick values as floats in the range [-1, 1].

Called by the receiver task.
*/
void ReceiverAtomJoyStick::getStickValues(float& throttle, float& roll, float& pitch, float& yaw) const
{
    throttle = _controls.throttle;
    roll = _controls.roll;
    pitch = _controls.pitch;
    yaw = _controls.yaw;
}

ReceiverBase::EUI_48_t ReceiverAtomJoyStick::getMyEUI() const
{
    EUI_48_t ret {};
    memcpy(&ret, _transceiver.myMacAddress(), sizeof(EUI_48_t));
    return ret;
}

ReceiverBase::EUI_48_t ReceiverAtomJoyStick::getPrimaryPeerEUI() const
{
    EUI_48_t ret {};
    memcpy(&ret, _transceiver.getPrimaryPeerMacAddress(), sizeof(EUI_48_t));
    return ret;
}

void ReceiverAtomJoyStick::broadcastMyEUI() const
{
    broadcastMyMacAddressForBinding();
}

#if false
uint16_t ReceiverAtomJoyStick::getAuxiliaryChannel(size_t index) const
{
    return (index >= _auxiliaryChannelCount) ? CHANNEL_LOW : getSwitch(index) ? CHANNEL_HIGH : CHANNEL_LOW;
}
#endif

uint16_t ReceiverAtomJoyStick::getChannelPWM(size_t index) const
{
    if (index >= _auxiliaryChannelCount + STICK_COUNT) {
        return CHANNEL_LOW;
    }
    switch (index) {
    case ROLL:
        return static_cast<uint16_t>(_controls.roll*CHANNEL_RANGE_F + CHANNEL_MIDDLE_F);
    case PITCH:
        return static_cast<uint16_t>(_controls.pitch*CHANNEL_RANGE_F + CHANNEL_MIDDLE_F);
    case THROTTLE:
        return _positiveHalfThrottle ? static_cast<uint16_t>(_controls.throttle*CHANNEL_RANGE_F + CHANNEL_LOW_F)
                                     : static_cast<uint16_t>(_controls.throttle*CHANNEL_RANGE_F + CHANNEL_MIDDLE_F);
    case YAW:
        return static_cast<uint16_t>(_controls.yaw*CHANNEL_RANGE_F + CHANNEL_MIDDLE_F);
    default:
        return getSwitch(index - STICK_COUNT) ? CHANNEL_HIGH : CHANNEL_LOW;
    }
}

esp_err_t ReceiverAtomJoyStick::broadcastMyMacAddressForBinding(int broadcastCount, uint32_t broadcastDelayMs) const
{
    // peer command as used by the StampFlyController, see: https://github.com/m5stack/Atom-JoyStick/blob/main/examples/StampFlyController/src/main.cpp#L117
    static const std::array<uint8_t, 4> peerCommand { 0xaa, 0x55, 0x16, 0x88 };
    enum { DATA_SIZE = 16 };
    std::array<uint8_t, DATA_SIZE> data;
    static_assert(sizeof(data) > sizeof(peerCommand) + ESP_NOW_ETH_ALEN + 2);

    data[0] = _transceiver.getBroadcastChannel();
    memcpy(&data[1], _transceiver.myMacAddress(), ESP_NOW_ETH_ALEN);
    memcpy(&data[1 + ESP_NOW_ETH_ALEN], &peerCommand[0], sizeof(peerCommand));

    for (int ii = 0; ii < broadcastCount; ++ii) {
        const esp_err_t err = _transceiver.broadcastData(&data[0], sizeof(data));
        //  cppcheck-suppress knownConditionTrueFalse
        if (err != ESP_OK) {
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
            static const char *TAG = "ReceiverAtomJoyStick::broadcastMyMacAddressForBinding";
            ESP_LOGI(TAG, "failed: %X", err);
            //Serial.printf("broadcastMyMacAddressForBinding failed: %X\r\n", err);
#endif
            return err;
        }
#if defined(LIBRARY_RECEIVER_USE_ESPNOW) && !defined(FRAMEWORK_ESPIDF)
        vTaskDelay(pdMS_TO_TICKS(broadcastDelayMs));
#else
        (void)broadcastDelayMs;
#endif
    }
    return ESP_OK;
}

/*!
Convert the 4 bytes of a floating point number to a fixed point integer in Q12dot4 format, ie in range [-2048,2047].
*/
int32_t ReceiverAtomJoyStick::ubyte4float_to_Q12dot4(const uint8_t f[4]) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
{
    union bi_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        std::array<uint8_t, 4> b;
        uint32_t i;
    };
    const bi_t n = { .b = { f[0], f[1], f[2], f[3] } }; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    const uint8_t exponent = static_cast<uint8_t>((n.i >> 23U) & 0xFFU); // 0x7F80 0000 // NOLINT(cppcoreguidelines-pro-type-union-access,hicpp-use-auto,modernize-use-auto)
    if (exponent == 0) {
        return 0;
    }

    const uint8_t sign     = static_cast<uint8_t>((n.i >> 31U) & 0x1U); // 0x1000 0000 // NOLINT(cppcoreguidelines-pro-type-union-access,hicpp-use-auto,modernize-use-auto)
    const uint32_t mantissa = (n.i & 0x7FFFFFU) | 0x800000U; // 0x007F FFFF, OR in implicit bit NOLINT(cppcoreguidelines-pro-type-union-access)

    const auto i = static_cast<int32_t>(mantissa >> ((22U-11U) - (exponent - 0x80U))); // -Wshift-count-overflow
    return sign ? -i : i;
}

bool ReceiverAtomJoyStick::unpackPacket()
{
    return unpackPacket(CHECK_PACKET);
}

/*!
Check the packet if `checkPacket` set. If the packet is valid then unpack it into the member data and set the packet to empty.

Returns true if a valid packet received, false otherwise.
*/
bool ReceiverAtomJoyStick::unpackPacket(checkPacket_t checkPacket)
{
    // see https://github.com/M5Fly-kanazawa/AtomJoy2024June/blob/main/src/main.cpp#L560 for packet format
    if (isPacketEmpty()) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t ii = 0; ii < PACKET_SIZE - 1; ++ii) {
        checksum += _packet[ii]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    if (checkPacket == CHECK_PACKET) {
        if (checksum != _packet[PACKET_SIZE - 1]) {
            //Serial.printf("checksum:%d, packet[24]:%d, packet[0]:%d, len:%d\r\n", checksum, _packet[24], _packet[0], receivedDataLen());
            setPacketEmpty();
            return false;
        }
//NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        const uint8_t* macAddress = _transceiver.myMacAddress();
        if (_packet[0] != macAddress[3] || _packet[1] != macAddress[4] || _packet[2] != macAddress[5]) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            //Serial.printf("packet: %02X:%02X:%02X\r\n", _packet[0], _packet[1], _packet[2]);
            //Serial.printf("my:     %02X:%02X:%02X\r\n", macAddress[3], macAddress[4], macAddress[5]);
            setPacketEmpty();
            return false;
        }
    } else {
#if !defined(UNIT_TEST_BUILD)
        //Serial.printf("packet: %02X:%02X:%02X\r\n", _packet[0], _packet[1], _packet[2]);
#endif
        //Serial.printf("peer:   %02X:%02X:%02X\r\n", _primaryPeerInfo.peer_addr[3], _primaryPeerInfo.peer_addr[4], _primaryPeerInfo.peer_addr[5]);
        //Serial.printf("my:     %02X:%02X:%02X\r\n", macAddress[3], macAddress[4], macAddress[5]);
    }

    _sticks[YAW].rawQ12dot4 = ubyte4float_to_Q12dot4(&_packet[3]); // cppcheck-suppress invalidPointerCast
    _sticks[THROTTLE].rawQ12dot4 = ubyte4float_to_Q12dot4(&_packet[7]); // cppcheck-suppress invalidPointerCast
    _sticks[ROLL].rawQ12dot4 = ubyte4float_to_Q12dot4(&_packet[11]); // cppcheck-suppress invalidPointerCast
    _sticks[PITCH].rawQ12dot4 = -ubyte4float_to_Q12dot4(&_packet[15]); // cppcheck-suppress invalidPointerCast

    _armButton = _packet[19];
    _flipButton = _packet[20];
    _mode = _packet[21];  // _mode: stable or sport
    _altMode = _packet[22];
    _proactiveFlag = _packet[23];
//NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    setPacketEmpty();
    return true;
}

void ReceiverAtomJoyStick::setCurrentReadingsToBias()
{
    _biasIsSet = static_cast<int>(true);
    for (auto& stick : _sticks) {
        stick.biasQ12dot4 = stick.rawQ12dot4;
    }
}

float ReceiverAtomJoyStick::normalizedStick(size_t stickIndex) const
{
    const stick_t stick = _sticks[stickIndex];
    if (!_biasIsSet) {
       return Q12dot4_to_float(stick.rawQ12dot4);
    }

    const int32_t ret = stick.rawQ12dot4 - stick.biasQ12dot4;
    if (ret < -stick.deadbandQ12dot4) {
        return -Q12dot4_to_float(-stick.deadbandQ12dot4 - ret); // (stick.bias - stick.deadband/2 - min);
    }
    if (ret > stick.deadbandQ12dot4) {
        return Q12dot4_to_float(ret - stick.deadbandQ12dot4); // (max - stick.bias - stick.deadband/2);
    }
    return 0.0F;
}

void ReceiverAtomJoyStick::resetSticks()
{
    _biasIsSet = static_cast<int>(false);
    for (auto& stick : _sticks) {
        stick.biasQ12dot4 = 0;
        stick.deadbandQ12dot4 = 0;
    }
}

void ReceiverAtomJoyStick::setDeadband(int32_t deadbandQ12dot4)
{
    for (auto& stick : _sticks) {
        stick.deadbandQ12dot4 = deadbandQ12dot4;
    }
}
