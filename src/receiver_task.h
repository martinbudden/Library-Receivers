#pragma once

#include <task_base.h> // NOLINT(clang-diagnostic-pragma-pack)

class ReceiverBase;
class CockpitBase;
struct receiver_parameter_group_t;


class ReceiverTask : public TaskBase {
public:
    ReceiverTask(uint32_t task_interval_microseconds, ReceiverBase& receiver, CockpitBase& cockpit, receiver_parameter_group_t& parameter_group);
public:
    static ReceiverTask* create_task(task_info_t& task_info, ReceiverBase& receiver, CockpitBase& cockpit, receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static ReceiverTask* create_task(ReceiverBase& receiver, CockpitBase& cockpit, receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static ReceiverTask* create_task(task_info_t& task_info, ReceiverBase& receiver, CockpitBase& cockpit, receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core);
    static ReceiverTask* create_task(ReceiverBase& receiver, CockpitBase& cockpit, receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core);
public:
    [[noreturn]] static void task_static(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    ReceiverBase& _receiver;
    CockpitBase& _cockpit;
    receiver_parameter_group_t& _parameter_group;
};
