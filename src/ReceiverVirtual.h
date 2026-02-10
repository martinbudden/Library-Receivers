#pragma once

#include "ReceiverBase.h"

#include <array>


class ReceiverVirtual : public ReceiverBase {
public:
    enum { CHANNEL_COUNT = 18 };

    virtual ~ReceiverVirtual() = default;
    ReceiverVirtual();
private:
    // ReceiverVirtual is not copyable or moveable
    ReceiverVirtual(const ReceiverVirtual&) = delete;
    ReceiverVirtual& operator=(const ReceiverVirtual&) = delete;
    ReceiverVirtual(ReceiverVirtual&&) = delete;
    ReceiverVirtual& operator=(ReceiverVirtual&&) = delete;
public:
    virtual int32_t WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait) override;
    virtual bool update(uint32_t tick_count_delta) override;
    virtual bool unpack_packet() override;
    virtual uint16_t get_channel_pwm(size_t index) const override;
public: // for testing
    void set_channel_pwm(size_t index, uint16_t pwm_value);
    void set_auxiliary_channel_pwm(size_t index, uint16_t pwm_value) { set_channel_pwm(index + ReceiverBase::STICK_COUNT, pwm_value); }
    void set_controls(const receiver_controls_t& controls) { _controls = controls; }
private:
    uint32_t _received_packet_count {};
    std::array<uint16_t, CHANNEL_COUNT> _pwm_values {};
};
