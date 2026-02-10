#pragma once

#include "EspnowTransceiver.h"
#include "ReceiverBase.h"


class ReceiverAtomJoyStick : public ReceiverBase {
public:
    ReceiverAtomJoyStick(const uint8_t* mac_address, uint8_t channel);
private:
    // ReceiverAtomJoyStick is not copyable or moveable
    ReceiverAtomJoyStick(const ReceiverAtomJoyStick&) = delete;
    ReceiverAtomJoyStick& operator=(const ReceiverAtomJoyStick&) = delete;
    ReceiverAtomJoyStick(ReceiverAtomJoyStick&&) = delete;
    ReceiverAtomJoyStick& operator=(ReceiverAtomJoyStick&&) = delete;
public:
    static constexpr uint8_t MODE_SWITCH = 1;
    static constexpr uint8_t ALT_MODE_SWITCH = 2;
    static constexpr uint8_t SWITCH_COUNT = 3; // switch count includes MOTOR_ON_OFF_SWITCH
    static constexpr uint8_t MODE_STABLE = 0;
    static constexpr uint8_t MODE_SPORT = 1;
    static constexpr uint8_t ALT_MODE_AUTO = 4;
    static constexpr uint8_t ALT_MODE_MANUAL = 5;
public:
    int32_t init();

    virtual int32_t WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait) override;
    virtual bool update(uint32_t tick_count_delta) override;
    virtual EUI_48_t get_mu_eui() const override;
    virtual EUI_48_t get_primary_peer_eui() const override;
    virtual void broadcast_my_eui() const override;
    uint16_t get_channel_pwm(size_t index) const override;

    EspnowTransceiver& get_espnow_transceiver() { return _transceiver; }
    static int32_t ubyte4float_to_q12dot4(const uint8_t f[4]);
private:
    // from AtomJoyStickReceiver
    inline bool is_packet_empty() const { return _received_data.len == 0 ? true : false;  }
    inline void set_packet_empty() { _received_data.len = 0; }
    enum { DEFAULT_BROADCAST_COUNT = 20, DEFAULT_BROADCAST_DELAY_MS = 50 };
    esp_err_t broadcast_my_mac_address_for_binding(int broadcast_count, uint32_t broadcastDelayMs) const;
    esp_err_t broadcast_my_mac_address_for_binding() const { return broadcast_my_mac_address_for_binding(DEFAULT_BROADCAST_COUNT, DEFAULT_BROADCAST_DELAY_MS); }
    virtual bool unpack_packet() override;
    enum checkPacket_t { CHECK_PACKET, DONT_CHECK_PACKET };
    bool unpack_packet(checkPacket_t checkPacket);
    void reset_sticks();
    void set_deadband(int32_t deadband);
    void set_current_readings_to_bias();
    float normalized_stick(size_t stick_index) const;
private:
    struct stick_t {
        int32_t raw_q12dot4 {0};
        int32_t bias_q12dot4 {0};
        int32_t deadband_q12dot4 {16}; // last 4 bits of number
    };
private:
    EspnowTransceiver _transceiver;
    EspnowTransceiver::received_data_t _received_data;
    uint32_t _received_packet_count {0};
    std::array<stick_t, STICK_COUNT> _sticks {};
    enum { PACKET_SIZE = 25 };
    uint8_t _packet[PACKET_SIZE] {};
    uint8_t _bias_is_set {false}; //NOTE: if `bool` type is used here then `getMode()` sometimes returns incorrect value
    uint8_t _mode {0};
    uint8_t _alt_mode {0};
    uint8_t _arm_button {0};
    uint8_t _flip_button {0};
    uint8_t _proactive_flag {0};
};
