#pragma once

#include <TaskBase.h> // NOLINT(clang-diagnostic-pragma-pack)

class CockpitBase;
class ReceiverBase;
class ReceiverWatcher;

class ReceiverTask : public TaskBase {
public:
    ReceiverTask(uint32_t taskIntervalMicroseconds, ReceiverBase& receiver, CockpitBase& cockpit);
public:
    static ReceiverTask* createTask(task_info_t& taskInfo, ReceiverBase& receiver, CockpitBase& cockpit, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static ReceiverTask* createTask(ReceiverBase& receiver, CockpitBase& cockpit,uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static ReceiverTask* createTask(task_info_t& taskInfo, ReceiverBase& receiver, CockpitBase& cockpit, uint8_t priority, uint32_t core);
    static ReceiverTask* createTask(ReceiverBase& receiver, CockpitBase& cockpit, uint8_t priority, uint32_t core);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    ReceiverBase& _receiver;
    CockpitBase& _cockpit;
    ReceiverWatcher* _receiverWatcher;
};
