#include "ReceiverAtomJoyStick.h"
#include <cstring>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
//#include <HardwareSerial.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else


#endif

ReceiverAtomJoyStick::ReceiverAtomJoyStick(const uint8_t* mac_address, uint8_t channel) :
    _transceiver(mac_address, channel),
    _received_data(&_packet[0], sizeof(_packet))
{
    // switches are mapped to the auxiliary channels
    _auxiliary_channel_count = SWITCH_COUNT;
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
bool ReceiverAtomJoyStick::update(uint32_t tick_count_delta)
{
    if (is_packet_empty()) {
        return false;
    }

    _packet_received = true;
    ++_packet_count;

    // record tickoutDelta for instrumentation
    _tick_count_delta = tick_count_delta;

    // track dropped packets
    _received_packet_count = _transceiver.get_received_packet_count();
    _dropped_packet_count = static_cast<int32_t>(_received_packet_count) - _packet_count;
    _dropped_packet_count_delta = _dropped_packet_count - _dropped_packet_count_previous;
    _dropped_packet_count_previous = _dropped_packet_count;

    if (unpack_packet(CHECK_PACKET)) {
        if (_packet_count == 5) { // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
            // set the bias so that the current readings are zero.
            set_current_readings_to_bias();
        }

        // Save the stick values.
        _controls.throttle = normalized_stick(THROTTLE);
        // Atom Joystick returns throttle in range [-1.0, 1.0]
        // _positive_half_throttle discards range [-1.0, 0.0) for use by aerial vehicles
        // since Atom Joystick is sprung to return to center position on all axes
        if (_positive_half_throttle) {
            _controls.throttle = std::max(0.0F, _controls.throttle); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        }
        _controls.roll = normalized_stick(ROLL);
        _controls.pitch = normalized_stick(PITCH);
        _controls.yaw = normalized_stick(YAW);

        // Save the button values.
        set_switch(MOTOR_ON_OFF_SWITCH, _flip_button);
        set_switch(MODE_SWITCH, _mode);
        set_switch(ALT_MODE_SWITCH, _alt_mode == 4 ? 0 : 1); // _alt_mode has a value of 4 or 5

        // now we have copied all the packet values, set the _new_packet_available flag
        // NOTE: there is no mutex around this flag
        _new_packet_available = true;
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

ReceiverBase::EUI_48_t ReceiverAtomJoyStick::get_mu_eui() const
{
    EUI_48_t ret {};
    memcpy(&ret, _transceiver.my_mac_address(), sizeof(EUI_48_t));
    return ret;
}

ReceiverBase::EUI_48_t ReceiverAtomJoyStick::get_primary_peer_eui() const
{
    EUI_48_t ret {};
    memcpy(&ret, _transceiver.get_primary_peer_mac_address(), sizeof(EUI_48_t));
    return ret;
}

void ReceiverAtomJoyStick::broadcast_my_eui() const
{
    broadcast_my_mac_address_for_binding();
}

#if false
uint16_t ReceiverAtomJoyStick::get_auxiliary_channel(size_t index) const
{
    return (index >= _auxiliary_channel_count) ? CHANNEL_LOW : get_switch(index) ? CHANNEL_HIGH : CHANNEL_LOW;
}
#endif

uint16_t ReceiverAtomJoyStick::get_channel_pwm(size_t index) const
{
    if (index >= _auxiliary_channel_count + STICK_COUNT) {
        return CHANNEL_LOW;
    }
    switch (index) {
    case ROLL:
        return _controls_pwm.roll;
    case PITCH:
        return _controls_pwm.pitch;
    case THROTTLE:
        return _controls_pwm.throttle;
    case YAW:
        return _controls_pwm.yaw;
    default:
        return get_switch(index - STICK_COUNT) ? CHANNEL_HIGH : CHANNEL_LOW;
    }
}

esp_err_t ReceiverAtomJoyStick::broadcast_my_mac_address_for_binding(int broadcast_count, uint32_t broadcastDelayMs) const
{
    // peer command as used by the StampFlyController, see: https://github.com/m5stack/Atom-JoyStick/blob/main/examples/StampFlyController/src/main.cpp#L117
    static const std::array<uint8_t, 4> peerCommand { 0xaa, 0x55, 0x16, 0x88 };
    enum { DATA_SIZE = 16 };
    std::array<uint8_t, DATA_SIZE> data;
    static_assert(sizeof(data) > sizeof(peerCommand) + ESP_NOW_ETH_ALEN + 2);

    data[0] = _transceiver.get_broadcast_channel();
    memcpy(&data[1], _transceiver.my_mac_address(), ESP_NOW_ETH_ALEN);
    memcpy(&data[1 + ESP_NOW_ETH_ALEN], &peerCommand[0], sizeof(peerCommand));

    for (int ii = 0; ii < broadcast_count; ++ii) {
        const esp_err_t err = _transceiver.broadcast_data(&data[0], sizeof(data));
        //  cppcheck-suppress knownConditionTrueFalse
        if (err != ESP_OK) {
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
            static const char *TAG = "ReceiverAtomJoyStick::broadcast_my_mac_address_for_binding";
            ESP_LOGI(TAG, "failed: %X", err);
            //Serial.printf("broadcast_my_mac_address_for_binding failed: %X\r\n", err);
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
Convert the 4 bytes of a floating point number to a fixed point integer in _q12dot4 format, ie in range [-2048,2047].
*/
int32_t ReceiverAtomJoyStick::ubyte4float_to_q12dot4(const uint8_t f[4]) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
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

bool ReceiverAtomJoyStick::unpack_packet()
{
    return unpack_packet(CHECK_PACKET);
}

/*!
Check the packet if `checkPacket` set. If the packet is valid then unpack it into the member data and set the packet to empty.

Returns true if a valid packet received, false otherwise.
*/
bool ReceiverAtomJoyStick::unpack_packet(checkPacket_t checkPacket)
{
    // see https://github.com/M5Fly-kanazawa/AtomJoy2024June/blob/main/src/main.cpp#L560 for packet format
    if (is_packet_empty()) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t ii = 0; ii < PACKET_SIZE - 1; ++ii) {
        checksum += _packet[ii]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    if (checkPacket == CHECK_PACKET) {
        if (checksum != _packet[PACKET_SIZE - 1]) {
            //Serial.printf("checksum:%d, packet[24]:%d, packet[0]:%d, len:%d\r\n", checksum, _packet[24], _packet[0], receivedDataLen());
            set_packet_empty();
            return false;
        }
//NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        const uint8_t* mac_address = _transceiver.my_mac_address();
        if (_packet[0] != mac_address[3] || _packet[1] != mac_address[4] || _packet[2] != mac_address[5]) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            //Serial.printf("packet: %02X:%02X:%02X\r\n", _packet[0], _packet[1], _packet[2]);
            //Serial.printf("my:     %02X:%02X:%02X\r\n", mac_address[3], mac_address[4], mac_address[5]);
            set_packet_empty();
            return false;
        }
    } else {
#if !defined(UNIT_TEST_BUILD)
        //Serial.printf("packet: %02X:%02X:%02X\r\n", _packet[0], _packet[1], _packet[2]);
#endif
        //Serial.printf("peer:   %02X:%02X:%02X\r\n", _primaryPeerInfo.peer_addr[3], _primaryPeerInfo.peer_addr[4], _primaryPeerInfo.peer_addr[5]);
        //Serial.printf("my:     %02X:%02X:%02X\r\n", mac_address[3], mac_address[4], mac_address[5]);
    }

    _sticks[YAW].raw_q12dot4 = ubyte4float_to_q12dot4(&_packet[3]); // cppcheck-suppress invalidPointerCast
    _sticks[THROTTLE].raw_q12dot4 = ubyte4float_to_q12dot4(&_packet[7]); // cppcheck-suppress invalidPointerCast
    _sticks[ROLL].raw_q12dot4 = ubyte4float_to_q12dot4(&_packet[11]); // cppcheck-suppress invalidPointerCast
    _sticks[PITCH].raw_q12dot4 = -ubyte4float_to_q12dot4(&_packet[15]); // cppcheck-suppress invalidPointerCast

    _arm_button = _packet[19];
    _flip_button = _packet[20];
    _mode = _packet[21];  // _mode: stable or sport
    _alt_mode = _packet[22];
    _proactive_flag = _packet[23];

    _controls_pwm = receiver_controls_pwm_t {
        .throttle = static_cast<uint16_t>(_positive_half_throttle ? (_controls.throttle*CHANNEL_RANGE_F + CHANNEL_LOW_F) : (_controls.throttle * CHANNEL_RANGE_F / 2.0F) + CHANNEL_MIDDLE_F),
        .roll = static_cast<uint16_t>((_controls.roll * CHANNEL_RANGE_F / 2.0F) + CHANNEL_MIDDLE_F),
        .pitch = static_cast<uint16_t>((_controls.pitch * CHANNEL_RANGE_F / 2.0F) + CHANNEL_MIDDLE_F),
        .yaw = static_cast<uint16_t>((_controls.yaw * CHANNEL_RANGE_F /2.0F) + CHANNEL_MIDDLE_F)
    };
//NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    set_packet_empty();
    return true;
}

void ReceiverAtomJoyStick::set_current_readings_to_bias()
{
    _bias_is_set = static_cast<int>(true);
    for (auto& stick : _sticks) {
        stick.bias_q12dot4 = stick.raw_q12dot4;
    }
}

float ReceiverAtomJoyStick::normalized_stick(size_t stick_index) const
{
    const stick_t stick = _sticks[stick_index];
    if (!_bias_is_set) {
       return q12dot4_to_float(stick.raw_q12dot4);
    }

    const int32_t ret = stick.raw_q12dot4 - stick.bias_q12dot4;
    if (ret < -stick.deadband_q12dot4) {
        return -q12dot4_to_float(-stick.deadband_q12dot4 - ret); // (stick.bias - stick.deadband/2 - min);
    }
    if (ret > stick.deadband_q12dot4) {
        return q12dot4_to_float(ret - stick.deadband_q12dot4); // (max - stick.bias - stick.deadband/2);
    }
    return 0.0F;
}

void ReceiverAtomJoyStick::reset_sticks()
{
    _bias_is_set = static_cast<int>(false);
    for (auto& stick : _sticks) {
        stick.bias_q12dot4 = 0;
        stick.deadband_q12dot4 = 0;
    }
}

void ReceiverAtomJoyStick::set_deadband(int32_t deadband_q12dot4)
{
    for (auto& stick : _sticks) {
        stick.deadband_q12dot4 = deadband_q12dot4;
    }
}
