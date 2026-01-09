#include "CockpitBase.h"
#include "ReceiverBase.h"
#include "ReceiverTask.h"

#include <TimeMicroseconds.h>

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


ReceiverTask::ReceiverTask(uint32_t taskIntervalMicroseconds, ReceiverBase& receiver, CockpitBase& cockpit) :
    TaskBase(taskIntervalMicroseconds),
    _receiver(receiver),
    _cockpit(cockpit),
    _receiverWatcher(receiver.getReceiverWatcher())
{
}

/*!
loop() function for when not using FREERTOS
*/
void ReceiverTask::loop()
{
    // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than taskIntervalTicks
#if defined(FRAMEWORK_USE_FREERTOS)
    const TickType_t tickCount = xTaskGetTickCount();
#else
    const uint32_t tickCount = timeMs();
#endif

    _tickCountDelta = tickCount - _tickCountPrevious;
    _tickCountPrevious = tickCount;

    if (_receiver.update(_tickCountDelta)) {
        CockpitBase::controls_t controls; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        controls.tickCount = tickCount;
        _receiver.getStickValues(controls.throttleStick, controls.rollStick, controls.pitchStick, controls.yawStick);
        _cockpit.updateControls(controls);
        // if there a watcher, then let it know there is a new packet
        if (_receiverWatcher) {
            _receiverWatcher->newReceiverPacketAvailable();
        }
    } else {
        _cockpit.checkFailsafe(tickCount);
    }
}

/*!
Task function for the ReceiverTask. Sets up and runs the task loop() function.
*/
[[noreturn]] void ReceiverTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)

    // BaseType_t is int, TickType_t is uint32_t
    if (_taskIntervalMicroseconds == 0) {
        // event driven scheduling
        const uint32_t ticksToWait = _cockpit.getTimeoutTicks();
        while (true) {
            if (_receiver.WAIT_FOR_DATA_RECEIVED(ticksToWait) == pdPASS) {
                loop();
            } else {
                // WAIT timed out, so check failsafe
                _cockpit.checkFailsafe(xTaskGetTickCount());
            }
        }
    } else {
        // time based scheduling
        const uint32_t taskIntervalTicks = _taskIntervalMicroseconds < 1000 ? 1 : pdMS_TO_TICKS(_taskIntervalMicroseconds / 1000);
        _previousWakeTimeTicks = xTaskGetTickCount();

        while (true) {
            // delay until the end of the next taskIntervalTicks
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const BaseType_t wasDelayed = xTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);
            if (wasDelayed) {
                _wasDelayed = true;
            }
#else
            vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);
#endif
            while (_receiver.isDataAvailable()) {
                // Read 1 byte from UART buffer and give it to the RX protocol parser
                if (_receiver.onDataReceivedFromISR(_receiver.readByte())) {
                    // onDataReceived returns true once packet is complete
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
[[noreturn]] void ReceiverTask::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<ReceiverTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast}
}
