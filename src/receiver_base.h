#pragma once

#include <cstddef>
#include <cstdint>

//! control values from receiver scaled to the range [-1.0F, 1.0F]
struct receiver_controls_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
};

//! controls mapped to the Pulse Width Modulation (PWM) range [1000, 2000]
struct receiver_controls_pwm_t {
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
};

/*!
Abstract Base Class defining a receiver.
*/
class ReceiverBase {
public:
    static constexpr uint8_t STICK_COUNT = 4;
    static constexpr uint8_t MOTOR_ON_OFF_SWITCH = 0;
    static constexpr uint8_t CONTROL_MODE_SWITCH = 1;
    static constexpr uint8_t ALTITUDE_MODE_SWITCH = 2;

    static constexpr uint16_t CHANNEL_LOW =  1000;
    static constexpr uint16_t CHANNEL_HIGH = 2000;
    static constexpr uint16_t CHANNEL_MIDDLE = 1500;
    static constexpr uint16_t CHANNEL_RANGE = CHANNEL_HIGH - CHANNEL_LOW;
    static constexpr float CHANNEL_LOW_F = 1000.0F;
    static constexpr float CHANNEL_HIGH_F = 2000.0F;
    static constexpr float CHANNEL_MIDDLE_F = 1500.0F;
    static constexpr float CHANNEL_RANGE_F = 1000.0F;

    static constexpr uint16_t CHANNEL_RANGE_MIN = 900;
    static constexpr uint16_t CHANNEL_RANGE_MID = 1500;
    static constexpr uint16_t CHANNEL_RANGE_MAX = 2100;
    static constexpr uint16_t CHANNEL_RANGE_STEP = 25;
    static constexpr uint16_t RANGE_STEP_MIN = 0;
    static constexpr uint16_t RANGE_STEP_MID = ((CHANNEL_RANGE_MID - CHANNEL_RANGE_MIN) / CHANNEL_RANGE_STEP);
    static constexpr uint16_t RANGE_STEP_MAX = ((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / CHANNEL_RANGE_STEP);

    // standardize receivers to use AETR (Ailerons, Elevator, Throttle, Rudder), ie ROLL, PITCH, THROTTLE, YAW
    static constexpr uint8_t ROLL = 0;
    static constexpr uint8_t PITCH = 1;
    static constexpr uint8_t THROTTLE = 2;
    static constexpr uint8_t YAW = 3;
    static constexpr uint8_t AUX1= 4;
    static constexpr uint8_t AUX2= 5;
    static constexpr uint8_t AUX3= 6;
    static constexpr uint8_t AUX4= 7;
    static constexpr uint8_t AUX5= 8;
    static constexpr uint8_t AUX6= 9;
    static constexpr uint8_t AUX7= 10;
    static constexpr uint8_t AUX8= 11;
    static constexpr uint8_t AUX9= 12;
    static constexpr uint8_t AUX10= 13;
    static constexpr uint8_t AUX11= 14;
    static constexpr uint8_t AUX12= 15;
    static constexpr uint8_t AUX13= 16;
    static constexpr uint8_t AUX14= 17;
    static constexpr uint8_t AUX15= 18;
    static constexpr uint8_t AUX16= 19;
public:
     //! 48-bit extended unique identifier (often synonymous with MAC address)
    struct EUI_48_t {
        uint8_t octets[6];
    };
public:
    virtual ~ReceiverBase() = default;

    void set_positive_half_throttle(bool positive_half_throttle) { _positive_half_throttle = positive_half_throttle; }

    // 48-bit Extended Unique Identifiers, usually the MAC address if the receiver has one, but may be an alternative provided by the receiver.
    virtual EUI_48_t get_my_eui() const { const EUI_48_t ret {}; return ret; }
    virtual EUI_48_t get_primary_peer_eui() const  { const EUI_48_t ret {}; return ret; }
    virtual void broadcast_my_eui() const {}

    virtual int32_t WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait) = 0;
    virtual bool on_data_received_from_isr(uint8_t data) { (void)data; return false; }
    virtual bool is_data_available() const { return false; }
    virtual uint8_t read_byte() { return 0; }
    virtual bool update(uint32_t tick_count_delta) = 0;
    virtual bool unpack_packet() = 0;

    //! Controls in range [0,1] for throttle, [-1,1] for roll, pitch, and yaw
    receiver_controls_t get_controls() const { return _controls; }
    receiver_controls_pwm_t get_controls_pwm() const { return _controls_pwm; }//!< channels in range [1000,2000]

    virtual uint16_t get_channel_pwm(size_t index) const = 0;
    uint32_t get_auxiliary_channel_count() const { return _auxiliary_channel_count; }
    uint16_t get_auxiliary_channel(size_t index) const { return get_channel_pwm(index + STICK_COUNT); }
    bool is_range_active(uint8_t auxiliary_channel_index, uint8_t range_start, uint8_t range_end) const {
        if (range_start >= range_end) {
            return false;
        }
        const uint16_t channel_value = get_auxiliary_channel(auxiliary_channel_index);
        return (channel_value >= CHANNEL_RANGE_MIN + (range_start*CHANNEL_RANGE_STEP) && channel_value < CHANNEL_RANGE_MIN + (range_end*CHANNEL_RANGE_STEP));
    }

    uint32_t get_switch(size_t index) const { return static_cast<uint32_t>((_switches & (0b11U << (2*index))) >> (2*index)); }
    void set_switch(size_t index, uint8_t value) { _switches &= static_cast<uint32_t>(~(0b11U << (2*index))); _switches |= static_cast<uint32_t>((value & 0b11U) << (2*index)); }
    uint32_t get_switches() const { return _switches; }

    int32_t get_dropped_packet_count_delta() const { return _dropped_packet_count_delta; }
    uint32_t get_tick_count_delta() const { return _tick_count_delta; }
    static float q12dot4_to_float(int32_t q4dot12) { return static_cast<float>(q4dot12) * (1.0F / 2048.0F); } //<! convert _q12dot4 fixed point number to floating point

    bool isPacket_received() const { return _packet_received; }
    bool isNew_packet_available() const { return _new_packet_available; }
    void clearNew_packet_available() { _new_packet_available = false; }
protected:
    uint8_t _packet_received {false}; // may be invalid packet
    uint8_t _new_packet_available {false};
    uint8_t _positive_half_throttle {false};
    int32_t _packet_count {};
    int32_t _dropped_packet_count_delta {};
    int32_t _dropped_packet_count {};
    int32_t _dropped_packet_count_previous {};
    uint32_t _tick_count_delta {};
    uint32_t _switches {}; // 16 2 or 3 positions switches, each using 2-bits
    receiver_controls_t _controls {}; //!< the main 4 channels
    receiver_controls_pwm_t _controls_pwm {}; //!< the main 4 channels in PWM range
    uint32_t _auxiliary_channel_count {};
};
