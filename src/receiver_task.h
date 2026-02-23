#pragma once

#include <task_base.h> // NOLINT(clang-diagnostic-pragma-pack)

class CockpitBase;
class Debug;
class FlightController;
class MotorMixerBase;
class RcModes;
class ReceiverBase;


struct receiver_parameter_group_t {
    ReceiverBase& receiver;
    CockpitBase& cockpit;
    RcModes& rc_modes;
    FlightController& flight_controller;
    MotorMixerBase& motor_mixer;
    Debug& debug;
};


class ReceiverTask : public TaskBase {
public:
    ReceiverTask(uint32_t task_interval_microseconds, receiver_parameter_group_t& parameter_group);
public:
    static ReceiverTask* create_task(task_info_t& task_info, receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static ReceiverTask* create_task(receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static ReceiverTask* create_task(task_info_t& task_info, receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core);
    static ReceiverTask* create_task(receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core);
public:
    [[noreturn]] static void task_static(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    receiver_parameter_group_t& _parameter_group;
};
