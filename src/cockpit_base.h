#pragma once

#include <cstdint> // NOLINT(clang-diagnostic-pragma-pack)

class ReceiverBase;
struct receiver_context_t;


struct cockpit_controls_t {
    uint32_t tick_count;
    float throttle_stick;
    float roll_stick;
    float pitch_stick;
    float yaw_stick;
};

class CockpitBase {
public:
    virtual ~CockpitBase() = default;
    CockpitBase() = default;

    uint32_t get_timeout_ticks() const { return _timeout_ticks; }
    void set_timeout_ticks(uint32_t timeout_ticks) { _timeout_ticks = timeout_ticks; }

    virtual void update_controls(uint32_t tick_count, const ReceiverBase& receiver, receiver_context_t& ctx) = 0;
    virtual void check_failsafe(uint32_t tick_count, receiver_context_t& ctx) = 0;
protected:
    uint32_t _timeout_ticks {100};
};
