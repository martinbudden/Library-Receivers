#pragma once

#include <TaskBase.h> // NOLINT(clang-diagnostic-pragma-pack)

class CockpitBase;
class RcRates;
class ReceiverBase;


class ReceiverTask : public TaskBase {
public:
    ReceiverTask(uint32_t taskIntervalMicroseconds, ReceiverBase& receiver, CockpitBase& cockpit, RcRates& rc_rates);
public:
    static ReceiverTask* createTask(task_info_t& taskInfo, ReceiverBase& receiver, CockpitBase& cockpit, RcRates& rc_rates, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static ReceiverTask* createTask(ReceiverBase& receiver, CockpitBase& cockpit, RcRates& rc_rates, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static ReceiverTask* createTask(task_info_t& taskInfo, ReceiverBase& receiver, CockpitBase& cockpit, RcRates& rc_rates, uint8_t priority, uint32_t core);
    static ReceiverTask* createTask(ReceiverBase& receiver, CockpitBase& cockpit, RcRates& rc_rates, uint8_t priority, uint32_t core);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    ReceiverBase& _receiver;
    CockpitBase& _cockpit;
    RcRates& _rc_rates;
};
