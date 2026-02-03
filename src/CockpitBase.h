#pragma once

#include <cstdint> // NOLINT(clang-diagnostic-pragma-pack)

class ReceiverBase;


class CockpitBase {
public:
    struct controls_t {
        uint32_t tickCount;
        float throttleStick;
        float rollStick;
        float pitchStick;
        float yawStick;
    };
public:
    virtual ~CockpitBase() = default;
    CockpitBase() = default;


    uint32_t getTimeoutTicks() const { return _timeoutTicks; }
    void setTimeoutTicks(uint32_t timeoutTicks) { _timeoutTicks = timeoutTicks; }

    virtual void updateControls(uint32_t tickCount, ReceiverBase& receiver) = 0;
    virtual void checkFailsafe(uint32_t tickCount) = 0;
protected:
    uint32_t _timeoutTicks {100};
};
