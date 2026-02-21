#pragma once

#include <array>
#include <time_microseconds.h>


#if !defined(FAST_CODE)
#if defined(FRAMEWORK_ESPIDF)
#define FAST_CODE IRAM_ATTR
#else
#define FAST_CODE
#endif
#endif

#if defined(FRAMEWORK_USE_FREERTOS)

#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#else
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <queue.h>
#endif

#endif // FRAMEWORK_USE_FREERTOS

#if defined(FRAMEWORK_RPI_PICO)
#include <hardware/uart.h>
#include <pico/mutex.h>
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)

#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_uart.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_gpio.h>
#include <stm32f3xx_hal_uart.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_uart.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_gpio.h>
#include <stm32f7xx_hal_uart.h>
#endif
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <HardwareSerial.h>
#endif
#endif


class SerialPortWatcherBase {
public:
    virtual ~SerialPortWatcherBase() = default;
    virtual bool on_data_received_from_isr(uint8_t data) = 0;
};


class SerialPort {
public:
    static constexpr uint8_t UART_INDEX_0 = 0;
    static constexpr uint8_t UART_INDEX_1 = 1;
    static constexpr uint8_t UART_INDEX_2 = 2;
    static constexpr uint8_t UART_INDEX_3 =3;
    static constexpr uint8_t UART_INDEX_4 = 4;
    static constexpr uint8_t UART_INDEX_5 = 5;
    static constexpr uint8_t UART_INDEX_6 = 6;
    static constexpr uint8_t UART_INDEX_7 =7;

    static constexpr uint8_t PARITY_NONE = 0;
    static constexpr uint8_t PARITY_EVEN = 1;
    static constexpr uint8_t PARITY_ODD = 2;

    static constexpr uint8_t STOP_BITS_1 = 1;
    static constexpr uint8_t STOP_BITS_2 = 2;

    static constexpr uint8_t DATA_BITS_5 = 5;
    static constexpr uint8_t DATA_BITS_6 = 6;
    static constexpr uint8_t DATA_BITS_7 = 7; 
    static constexpr uint8_t DATA_BITS_8 = 8;
    static constexpr uint8_t DATA_BITS_9 = 9;

    static constexpr uint8_t BAUDRATE_AUTO = 0;
    static constexpr uint8_t BAUDRATE_9600 = 1;
    static constexpr uint8_t BAUDRATE_19200 = 2;
    static constexpr uint8_t BAUDRATE_38400 = 3;
    static constexpr uint8_t BAUDRATE_57600 = 4;
    static constexpr uint8_t BAUDRATE_115200 = 5;
    static constexpr uint8_t BAUDRATE_230400 = 6;
    static constexpr uint8_t BAUDRATE_250000 = 7;
    static constexpr uint8_t BAUDRATE_400000 = 8;
    static constexpr uint8_t BAUDRATE_460800 = 9;
    static constexpr uint8_t BAUDRATE_500000 = 10;
    static constexpr uint8_t BAUDRATE_921600 = 11;
    static constexpr uint8_t BAUDRATE_1000000 = 12;
    static constexpr uint8_t BAUDRATE_1500000 = 13;
    static constexpr uint8_t BAUDRATE_2000000 = 14;
    static constexpr uint8_t BAUDRATE_2470000 = 15;
    static constexpr uint8_t BAUDRATE_COUNT = 16;
public:
    // negative pin means it is inverted
    struct port_pin_t {
        int8_t port;
        int8_t pin;
    };
    struct serial_pin_t {
        int8_t port;
        int8_t pin;
        bool inverted;
    };
    struct serial_pins_t {
        serial_pin_t rx;
        serial_pin_t tx;
    };
    struct uart_pins_t {
        int8_t rx;
        int8_t tx;
    };
    struct stm32_uart_pins_t {
        port_pin_t rx;
        port_pin_t tx;
    };
public:
    SerialPort(const stm32_uart_pins_t& pins, uint8_t uart_index, uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity);
    SerialPort(const uart_pins_t& pins, uint8_t uart_index, uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity);
    SerialPort(SerialPortWatcherBase* watcher, const serial_pins_t& pins, uint8_t uart_index, uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity);
    void init();
    void uartInit();
private:
    // SerialPort is not copyable or moveable
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&&) = delete;
    SerialPort& operator=(SerialPort&&) = delete;
public:
    int32_t WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait);
    bool on_data_received_from_isr(uint8_t data);
    bool is_data_available() const;
    uint8_t read_byte();
    size_t available_for_write();
    void write_byte(uint8_t data);
    size_t write(const uint8_t* buf, size_t len);
    uint32_t set_baudrate(uint32_t baudrate);
public:
    static void data_ready_isr();
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    static void data_ready_isr(const UART_HandleTypeDef *huart);
#endif
private:
    static SerialPort* self; //!< alias of `this` to be used in Interrupt Service Routine
    SerialPortWatcherBase* _watcher {nullptr};
    const serial_pins_t _pins {};
    const uint8_t _uart_index;
    const uint8_t _data_bits;
    const uint8_t _stop_bits;
    const uint8_t _parity;
    uint32_t _baudrate;
#if defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    uart_inst_t* _uart {};
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    UART_HandleTypeDef _uart {};
    uint8_t _rx_byte {};
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    HardwareSerial _uart;
#endif
#endif

#if defined(FRAMEWORK_USE_FREERTOS)

    uint32_t _data_ready_queue_item {};
    BaseType_t _data_ready_queue_higher_priority_task_woken = pdFALSE;
    enum { DATA_READY_QUEUE_LENGTH = 1 };
    std::array<uint8_t, DATA_READY_QUEUE_LENGTH * sizeof(_data_ready_queue_item)> _data_ready_queue_storage_area {};
    StaticQueue_t _data_ready_queueStatic {};
    QueueHandle_t _data_ready_queue {};
public:
    inline int32_t WAIT_DATA_READY(uint32_t ticksToWait) { return xQueueReceive(_data_ready_queue, &_data_ready_queue_item, ticksToWait); } // returns pdPASS(1) if queue read, pdFAIL(0) if timeout
    inline void SIGNAL_DATA_READY_FROM_ISR() {
        _data_ready_queue_higher_priority_task_woken = pdFALSE;
        xQueueOverwriteFromISR(_data_ready_queue, &_data_ready_queue_item, &_data_ready_queue_higher_priority_task_woken); 
        portYIELD_FROM_ISR(_data_ready_queue_higher_priority_task_woken); // cppcheck-suppress cstyleCast
    }
#else

public:
    inline int32_t WAIT_DATA_READY(uint32_t ticksToWait) { (void)ticksToWait; return 0; }
    inline void SIGNAL_DATA_READY_FROM_ISR() {}

#endif // FRAMEWORK_USE_FREERTOS
public:
    static constexpr std::array <uint32_t, BAUDRATE_COUNT> baudrates {
        0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000
    };
};
