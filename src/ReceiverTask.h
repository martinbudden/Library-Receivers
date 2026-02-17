#pragma once

#include <TaskBase.h> // NOLINT(clang-diagnostic-pragma-pack)

class CockpitBase;
class FlightController;
class MotorMixerBase;
class RcModes;
class ReceiverBase;


struct receiver_task_parameters_t {
    ReceiverBase& receiver;
    CockpitBase& cockpit;
    RcModes& rc_modes;
    FlightController& flightController;
    MotorMixerBase& motorMixer;
};

class ReceiverTask : public TaskBase {
public:
    ReceiverTask(uint32_t taskIntervalMicroseconds, const receiver_task_parameters_t& parameters);
public:
    static ReceiverTask* createTask(task_info_t& taskInfo, const receiver_task_parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static ReceiverTask* createTask(const receiver_task_parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static ReceiverTask* createTask(task_info_t& taskInfo, const receiver_task_parameters_t& parameters, uint8_t priority, uint32_t core);
    static ReceiverTask* createTask(const receiver_task_parameters_t& parameters, uint8_t priority, uint32_t core);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    receiver_task_parameters_t _task;
};
