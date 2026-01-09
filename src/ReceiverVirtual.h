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
    virtual bool update(uint32_t tickCountDelta) override;
    virtual bool unpackPacket() override;
    virtual void getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const override;
    virtual uint16_t getChannelPWM(size_t index) const override;
public: // for testing
    void setChannelPWM(size_t index, uint16_t pwmValue);
    void setAuxiliaryChannelPWM(size_t index, uint16_t pwmValue) { setChannelPWM(index + ReceiverBase::STICK_COUNT, pwmValue); }
    void setControls(const controls_t& controls) { _controls = controls; }
private:
    uint32_t _receivedPacketCount {};
    std::array<uint16_t, CHANNEL_COUNT> _pwmValues {};
};
