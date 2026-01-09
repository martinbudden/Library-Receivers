#include "ReceiverBase.h"
#include "ReceiverTask.h"

#include <TaskBase.h>
#include <array>
#include <cassert>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)

#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif

#endif // FRAMEWORK_USE_FREERTOS


ReceiverTask* ReceiverTask::createTask(ReceiverBase& receiver, CockpitBase& cockpit, uint8_t priority, uint32_t core)
{
    return createTask(receiver, cockpit, priority, core, 0);
}

ReceiverTask* ReceiverTask::createTask(task_info_t& taskInfo, ReceiverBase& receiver, CockpitBase& cockpit, uint8_t priority, uint32_t core)
{
    return createTask(taskInfo, receiver, cockpit, priority, core, 0);
}

ReceiverTask* ReceiverTask::createTask(ReceiverBase& receiver, CockpitBase& cockpit, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds)
{
    task_info_t taskInfo {};
    return createTask(taskInfo, receiver, cockpit, priority, core, taskIntervalMicroseconds);
}

ReceiverTask* ReceiverTask::createTask(task_info_t& taskInfo, ReceiverBase& receiver, CockpitBase& cockpit, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds)
{
    static ReceiverTask receiverTask(taskIntervalMicroseconds, receiver, cockpit);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &receiverTask,
    };
#if !defined(RECEIVER_TASK_STACK_DEPTH_BYTES)
    enum { RECEIVER_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array<uint8_t, RECEIVER_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array<StackType_t, RECEIVER_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    taskInfo = {
        .taskHandle = nullptr,
        .name = "ReceiverTask", // max length 16, including zero terminator
        .stackDepthBytes = RECEIVER_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .taskIntervalMicroseconds = taskIntervalMicroseconds
    };
#if defined(FRAMEWORK_USE_FREERTOS)
    assert(std::strlen(taskInfo.name) < configMAX_TASK_NAME_LEN);
    assert(taskInfo.priority < configMAX_PRIORITIES);

    static StaticTask_t taskBuffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        ReceiverTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create ReceiverTask");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    taskInfo.taskHandle = xTaskCreateStaticAffinitySet(
        ReceiverTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create ReceiverTask");
#else
    taskInfo.taskHandle = xTaskCreateStatic(
        ReceiverTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create ReceiverTask");
    // vTaskCoreAffinitySet(taskInfo.taskHandle, taskInfo.core);
#endif
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &receiverTask;
}
