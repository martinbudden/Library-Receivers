#pragma once

#include <cstddef>
#include <cstdint>

class ReceiverWatcher {
public:
    virtual ~ReceiverWatcher() = default;
    virtual void newReceiverPacketAvailable() = 0;
};

/*!
Abstract Base Class defining a receiver.
*/
class ReceiverBase {
public:
    enum { STICK_COUNT = 4 };
    enum { MOTOR_ON_OFF_SWITCH = 0, CONTROL_MODE_SWITCH = 1, ALTITUDE_MODE_SWITCH = 2 };

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

    enum { // standardize receivers to use AETR (Ailerons, Elevator, Throttle, Rudder), ie ROLL, PITCH, THROTTLE, YAW
        ROLL,
        PITCH,
        THROTTLE,
        YAW,
        AUX1,
        AUX2,
        AUX3,
        AUX4,
        AUX5,
        AUX6,
        AUX7,
        AUX8,
        AUX9,
        AUX10,
        AUX11,
        AUX12,
        AUX13,
        AUX14,
        AUX15,
        AUX16,
    };
public:
     //! 48-bit extended unique identifier (often synonymous with MAC address)
    struct EUI_48_t {
        uint8_t octets[6];
    };
     //! control values from receiver scaled to the range [-1.0F, 1.0F]
    struct controls_t {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    };
    //! controls mapped to the Pulse Width Modulation (PWM) range [1000, 2000]
    struct controls_pwm_t {
        uint16_t throttle;
        uint16_t roll;
        uint16_t pitch;
        uint16_t yaw;
    };
    /*! 
    Steps are 25 apart
        a value of 0 corresponds to a channel value of 900 or less
        a value of 48 corresponds to a channel value of 2100 or more
    48 steps between 900 and 2100
    */
    struct channel_range_t {
        uint8_t startStep;
        uint8_t endStep;
    };
public:
    virtual ~ReceiverBase() = default;

    ReceiverWatcher* getReceiverWatcher() const { return _receiverWatcher; }
    void setReceiverWatcher(ReceiverWatcher* receiverWatcher) { _receiverWatcher = receiverWatcher; }
    void setPositiveHalfThrottle(bool positiveHalfThrottle) { _positiveHalfThrottle = positiveHalfThrottle; }

    // 48-bit Extended Unique Identifiers, usually the MAC address if the receiver has one, but may be an alternative provided by the receiver.
    virtual EUI_48_t getMyEUI() const { const EUI_48_t ret {}; return ret; }
    virtual EUI_48_t getPrimaryPeerEUI() const  { const EUI_48_t ret {}; return ret; }
    virtual void broadcastMyEUI() const {}

    virtual int32_t WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait) = 0;
    virtual bool onDataReceivedFromISR(uint8_t data) { (void)data; return false; }
    virtual bool isDataAvailable() const { return false; }
    virtual uint8_t readByte() { return 0; }
    virtual bool update(uint32_t tickCountDelta) = 0;
    virtual bool unpackPacket() = 0;
    virtual void getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const = 0;

    inline controls_t getControls() const { return _controls; }
    //! Maps floats in range [0,1] for throttle, [-1,1] for roll, pitch, and yaw to channels in range [1000,2000]
    controls_pwm_t getControlsPWM() const {
        return controls_pwm_t {
            .throttle = static_cast<uint16_t>(_positiveHalfThrottle ? (_controls.throttle*CHANNEL_RANGE_F + CHANNEL_LOW_F): (_controls.throttle * CHANNEL_RANGE_F / 2.0F) + CHANNEL_MIDDLE_F),
            .roll = static_cast<uint16_t>((_controls.roll * CHANNEL_RANGE_F / 2.0F) + CHANNEL_MIDDLE_F),
            .pitch = static_cast<uint16_t>((_controls.pitch * CHANNEL_RANGE_F / 2.0F) + CHANNEL_MIDDLE_F),
            .yaw = static_cast<uint16_t>((_controls.yaw * CHANNEL_RANGE_F /2.0F) + CHANNEL_MIDDLE_F)
        };
    }
    /*! For reversible robots and 3D flying
        Maps floats in the range [-1,1] for throttle, roll, pitch, and yaw to channels in range [1000,2000]
    */
    controls_pwm_t getControlsPWM_NegativeThrottle() const {
        return controls_pwm_t {
            .throttle = static_cast<uint16_t>((_controls.throttle * CHANNEL_RANGE_F / 2.0F) + CHANNEL_MIDDLE_F),
            .roll = static_cast<uint16_t>((_controls.roll * CHANNEL_RANGE_F / 2.0F) + CHANNEL_MIDDLE_F),
            .pitch = static_cast<uint16_t>((_controls.pitch * CHANNEL_RANGE_F / 2.0F) + CHANNEL_MIDDLE_F),
            .yaw = static_cast<uint16_t>((_controls.yaw * CHANNEL_RANGE_F /2.0F) + CHANNEL_MIDDLE_F)
        };
    }

    virtual uint16_t getChannelPWM(size_t index) const = 0;
    uint32_t getAuxiliaryChannelCount() const { return _auxiliaryChannelCount; }
    uint16_t getAuxiliaryChannel(size_t index) const { return getChannelPWM(index + STICK_COUNT); }
    bool isRangeActive(uint8_t auxiliaryChannelIndex, const channel_range_t& range) const {
        if (range.startStep >= range.endStep) {
            return false;
        }
        const uint16_t channelValue = getAuxiliaryChannel(auxiliaryChannelIndex);
        return (channelValue >= CHANNEL_RANGE_MIN + (range.startStep*CHANNEL_RANGE_STEP) && channelValue < CHANNEL_RANGE_MIN + (range.endStep*CHANNEL_RANGE_STEP));
    }

    inline uint32_t getSwitch(size_t index) const { return static_cast<uint32_t>((_switches & (0b11U << (2*index))) >> (2*index)); }
    inline void setSwitch(size_t index, uint8_t value) { _switches &= static_cast<uint32_t>(~(0b11U << (2*index))); _switches |= static_cast<uint32_t>((value & 0b11U) << (2*index)); }
    inline uint32_t getSwitches() const { return _switches; }

    inline int32_t getDroppedPacketCountDelta() const { return _droppedPacketCountDelta; }
    inline uint32_t getTickCountDelta() const { return _tickCountDelta; }
    inline static float Q12dot4_to_float(int32_t q4dot12) { return static_cast<float>(q4dot12) * (1.0F / 2048.0F); } //<! convert Q12dot4 fixed point number to floating point

    inline bool isPacketReceived() const { return _packetReceived; }
    inline bool isNewPacketAvailable() const { return _newPacketAvailable; }
    inline void clearNewPacketAvailable() { _newPacketAvailable = false; }
protected:
    ReceiverWatcher* _receiverWatcher {nullptr};
    uint8_t _packetReceived {false}; // may be invalid packet
    uint8_t _newPacketAvailable {false};
    uint8_t _positiveHalfThrottle {false};
    int32_t _packetCount {};
    int32_t _droppedPacketCountDelta {};
    int32_t _droppedPacketCount {};
    int32_t _droppedPacketCountPrevious {};
    uint32_t _tickCountDelta {};
    uint32_t _switches {}; // 16 2 or 3 positions switches, each using 2-bits
    controls_t _controls {}; //!< the main 4 channels
    uint32_t _auxiliaryChannelCount {};
};
