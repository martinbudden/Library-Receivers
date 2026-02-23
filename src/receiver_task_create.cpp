#include "receiver_base.h"
#include "receiver_task.h"

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
#include <task_base.h>

#endif // FRAMEWORK_USE_FREERTOS


ReceiverTask* ReceiverTask::create_task(receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core)
{
    return create_task(parameter_group, priority, core, 0);
}

ReceiverTask* ReceiverTask::create_task(task_info_t& task_info, receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core)
{
    return create_task(task_info, parameter_group, priority, core, 0);
}

ReceiverTask* ReceiverTask::create_task(receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds)
{
    task_info_t task_info {};
    return create_task(task_info, parameter_group, priority, core, task_interval_microseconds);
}

ReceiverTask* ReceiverTask::create_task(task_info_t& task_info, receiver_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds)
{
    static ReceiverTask receiver_task(task_interval_microseconds, parameter_group);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &receiver_task,
    };
#if !defined(RECEIVER_TASK_STACK_DEPTH_BYTES)
    enum { RECEIVER_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array<uint8_t, RECEIVER_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array<StackType_t, RECEIVER_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    task_info = {
        .task_handle = nullptr,
        .name = "ReceiverTask", // max length 16, including zero terminator
        .stack_depth_bytes = RECEIVER_TASK_STACK_DEPTH_BYTES,
        .stack_buffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .task_interval_microseconds = task_interval_microseconds
    };
#if defined(FRAMEWORK_USE_FREERTOS)
    assert(std::strlen(task_info.name) < configMAX_TASK_NAME_LEN);
    assert(task_info.priority < configMAX_PRIORITIES);

    static StaticTask_t taskBuffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    task_info.task_handle = xTaskCreateStaticPinnedToCore(
        ReceiverTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &taskParameters,
        task_info.priority,
        &stack[0],
        &taskBuffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create ReceiverTask");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    task_info.task_handle = xTaskCreateStaticAffinitySet(
        ReceiverTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &taskParameters,
        task_info.priority,
        &stack[0],
        &taskBuffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create ReceiverTask");
#else
    task_info.task_handle = xTaskCreateStatic(
        ReceiverTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &taskParameters,
        task_info.priority,
        &stack[0],
        &taskBuffer
    );
    assert(task_info.task_handle != nullptr && "Unable to create ReceiverTask");
    // vTaskCoreAffinitySet(task_info.task_handle, task_info.core);
#endif
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &receiver_task;
}
