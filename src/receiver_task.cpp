#include "cockpit_base.h"
#include "receiver_base.h"
#include "receiver_task.h"

#include <time_microseconds.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <task.h>
#endif
#endif


ReceiverTask::ReceiverTask(uint32_t task_interval_microseconds, receiver_parameter_group_t& parameter_group) :
    TaskBase(task_interval_microseconds),
    _parameter_group(parameter_group)
{
}

/*!
loop() function for when not using FREERTOS
*/
void ReceiverTask::loop()
{
    // calculate _tick_count_delta to get actual deltaT value, since we may have been delayed for more than task_interval_ticks
#if defined(FRAMEWORK_USE_FREERTOS)
    const TickType_t tick_count = xTaskGetTickCount();
#else
    const uint32_t tick_count = time_ms();
#endif

    _tick_count_delta = tick_count - _tick_count_previous;
    _tick_count_previous = tick_count;

    if (_parameter_group.receiver.update(_tick_count_delta)) {
        _parameter_group.cockpit.update_controls(tick_count, _parameter_group);
    } else {
        _parameter_group.cockpit.check_failsafe(tick_count, _parameter_group);
    }
}

/*!
Task function for the ReceiverTask. Sets up and runs the task loop() function.
*/
[[noreturn]] void ReceiverTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)

    // BaseType_t is int, TickType_t is uint32_t
    if (_task_interval_microseconds == 0) {
        // event driven scheduling
        const uint32_t ticksToWait = _parameter_group.cockpit.get_timeout_ticks();
        while (true) {
            if (_parameter_group.receiver.WAIT_FOR_DATA_RECEIVED(ticksToWait) == pdPASS) {
                loop();
            } else {
                // WAIT timed out, so check failsafe
                _parameter_group.cockpit.check_failsafe(xTaskGetTickCount(), _parameter_group);
            }
        }
    } else {
        // time based scheduling
        const uint32_t task_interval_ticks = _task_interval_microseconds < 1000 ? 1 : pdMS_TO_TICKS(_task_interval_microseconds / 1000);
        _previous_wake_time_ticks = xTaskGetTickCount();

        while (true) {
            // delay until the end of the next task_interval_ticks
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const BaseType_t was_delayed = xTaskDelayUntil(&_previous_wake_time_ticks, task_interval_ticks);
            if (was_delayed) {
                _was_delayed = true;
            }
#else
            vTaskDelayUntil(&_previous_wake_time_ticks, task_interval_ticks);
#endif
            while (_parameter_group.receiver.is_data_available()) {
                // Read 1 byte from UART buffer and give it to the RX protocol parser
                if (_parameter_group.receiver.on_data_received_from_isr(_parameter_group.receiver.read_byte())) {
                    // on_data_received returns true once packet is complete
                    break;
                }
            }
            loop();
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for ReceiverTask::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void ReceiverTask::task_static(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<ReceiverTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast}
}
