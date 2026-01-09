#include "SerialPort.h"

#include "ReceiverBase.h"

#if defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
#include <hardware/gpio.h>
#include <hardware/uart.h>
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
static inline GPIO_TypeDef* gpioPort(uint8_t port) { return reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + port*(GPIOB_BASE - GPIOA_BASE)); }
static inline uint16_t gpioPin(uint8_t pin) { return static_cast<uint16_t>(1U << pin); }
#else // defaults to FRAMEWORK_ARDUINO
#endif // FRAMEWORK


/*!
Pointer to the receiver used by the ISR.
*/
SerialPort* SerialPort::self;


#if defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
void __not_in_flash_func(SerialPort::dataReadyISR)() // NOLINT(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
{
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
    while (uart_is_readable(self->_uart)) {
        // Read 1 byte from UART buffer and give it to the RX protocol parser
        const uint8_t data = uart_getc(self->_uart);
        if (self->onDataReceivedFromISR(data)) {
            // onDataReceived returns true once packet is complete
            self->SIGNAL_DATA_READY_FROM_ISR();
        }
    }
}
#elif defined(FRAMEWORK_STM32_CUBE)
// ISR called back when byte received on uart
#if false
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // cppcheck-suppress constParameterPointer
{
    SerialPort::dataReadyISR(huart);
}
#endif

FAST_CODE void SerialPort::dataReadyISR(const UART_HandleTypeDef *huart) // NOLINT(readability-convert-member-functions-to-static)
{
    if (huart->Instance == self->_uart.Instance) {
        if (self->onDataReceivedFromISR(self->_rxByte)) {
            // onDataReceived returns true once packet is complete
            self->SIGNAL_DATA_READY_FROM_ISR();
        }
        // Re-enable the interrupt for the next byte
        HAL_UART_Receive_IT(&self->_uart, &self->_rxByte, 1);
    }
}
#else
FAST_CODE void SerialPort::dataReadyISR()
{
    self->SIGNAL_DATA_READY_FROM_ISR();
}
#endif

/*!
Negative pin means it is inverted.
*/
SerialPort::SerialPort(SerialPortWatcherBase* watcher, const serial_pins_t& pins, uint8_t uartIndex, uint32_t baudrate, uint8_t dataBits, uint8_t stopBits, uint8_t parity) : // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    _watcher(watcher),
    _pins({
        .rx = { .port = pins.rx.port, .pin = static_cast<int8_t>(pins.rx.pin < 0 ? -pins.rx.pin : pins.rx.pin), .inverted = pins.rx.pin < 0 },
        .tx = { .port = pins.tx.port, .pin = static_cast<int8_t>(pins.tx.pin < 0 ? -pins.tx.pin : pins.tx.pin), .inverted = pins.tx.pin < 0 }
    }),
    _uartIndex(uartIndex),
    _dataBits(dataBits),
    _stopBits(stopBits),
    _parity(parity),
    _baudrate(baudrate)
#if defined(FRAMEWORK_ARDUINO_ESP32)
    ,_uart(uartIndex)
#endif
{
}

SerialPort::SerialPort(const stm32_uart_pins_t& pins, uint8_t uartIndex, uint32_t baudrate, uint8_t dataBits, uint8_t stopBits, uint8_t parity) :
    SerialPort(nullptr, serial_pins_t{{pins.tx.port,pins.tx.pin,false},{pins.rx.port,pins.rx.pin,false}}, uartIndex, baudrate, dataBits, stopBits, parity)
{
}

SerialPort::SerialPort(const uart_pins_t& pins, uint8_t uartIndex, uint32_t baudrate, uint8_t dataBits, uint8_t stopBits, uint8_t parity) :
    SerialPort(nullptr, serial_pins_t{{0,pins.tx,false},{0,pins.rx,false}}, uartIndex, baudrate, dataBits, stopBits, parity)
{
}

void SerialPort::init() // NOLINT(readability-make-member-function-const)
{
#if defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    // see https://github.com/victorhook/asac-fc/blob/main/src/receiver.c
    _uart = uart_get_instance(_uartIndex);

    uart_init(_uart, _baudrate);
    gpio_set_function(_pins.rx.pin, GPIO_FUNC_UART);
    gpio_set_function(_pins.tx.pin, GPIO_FUNC_UART);

    enum { NO_CTS = false, NO_RTS = false };
    uart_set_hw_flow(_uart, NO_CTS, NO_RTS);

    const uart_parity_t parity =
        (_parity == PARITY_NONE) ? UART_PARITY_NONE :
        (_parity == PARITY_EVEN) ? UART_PARITY_EVEN : UART_PARITY_ODD;
    uart_set_format(_uart, _dataBits, _stopBits, parity);

    uart_set_fifo_enabled(_uart, true);

    // Enable UART interrupt
    const irq_num_t irqNum = _uartIndex == 0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(irqNum, dataReadyISR);
    irq_set_enabled(irqNum, true);
    enum { RX_NEEDS_DATA = true, RX_DOES_NOT_NEED_DATA = false };
    enum { TX_NEEDS_DATA = true, TX_DOES_NOT_NEED_DATA = false };
    uart_set_irq_enables(_uart, RX_NEEDS_DATA, TX_DOES_NOT_NEED_DATA);

#elif defined(FRAMEWORK_ESPIDF)

#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)

    //enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};
    static constexpr int8_t PA = 0;
    static constexpr int8_t PB = 1;
    static constexpr int8_t PC = 2;
    static constexpr int8_t PD = 3;
    static constexpr int8_t PE = 4;
    static constexpr int8_t PF = 5;
    static constexpr int8_t PG = 6;
    static constexpr int8_t PH = 7;
    if (_pins.rx.port == PA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (_pins.rx.port == PB) {
#if defined(GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
#endif
    } else if (_pins.rx.port == PC) {
#if defined(GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
#endif
    } else if (_pins.rx.port == PD) {
#if defined(GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
#endif
    } else if (_pins.rx.port == PE) {
#if defined(GPIOE)
        __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
    } else if (_pins.rx.port == PF) {
#if defined(GPIOF)
        __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
    } else if (_pins.rx.port == PG) {
#if defined(GPIOG)
        __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
    } else if (_pins.rx.port == PH) {
#if defined(GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
    }

#if !defined(FRAMEWORK_STM32_CUBE_F1)
    uint32_t alternate {}; // NOLINT(misc-const-correctness)
#endif
    // STM32 indices are 1-based
    if (_uartIndex == 0) {
        __HAL_RCC_USART1_CLK_ENABLE();
        _uart.Instance = USART1;
#if !defined(FRAMEWORK_STM32_CUBE_F1)
        alternate = GPIO_AF7_USART1;
#endif
    } else if (_uartIndex == 1) {
#if defined(USART2)
        __HAL_RCC_USART2_CLK_ENABLE();
        _uart.Instance = USART2;
#if !defined(FRAMEWORK_STM32_CUBE_F1)
        alternate = GPIO_AF7_USART2;
#endif
#endif
    } else if (_uartIndex == 2) {
#if defined(USART3)
        __HAL_RCC_USART3_CLK_ENABLE();
        _uart.Instance = USART3;
#if !defined(FRAMEWORK_STM32_CUBE_F1)
        alternate = GPIO_AF7_USART3;
#endif
#endif
    } else if (_uartIndex == 3) {
#if defined(UART4)
        __HAL_RCC_UART4_CLK_ENABLE();
        _uart.Instance = UART4;
#if defined(FRAMEWORK_STM32_CUBE_F3)
        alternate = GPIO_AF5_UART4;
#elif defined(FRAMEWORK_STM32_CUBE_F4)
        alternate = GPIO_AF8_UART4;
#endif
#endif
    } else if (_uartIndex == 4) {
#if defined(UART5)
        __HAL_RCC_UART5_CLK_ENABLE();
        _uart.Instance = UART5;
#if defined(FRAMEWORK_STM32_CUBE_F3)
        alternate = GPIO_AF5_UART5;
#elif defined(FRAMEWORK_STM32_CUBE_F4)
        alternate = GPIO_AF8_UART5;
#endif
#endif
    } else if (_uartIndex == 5) { // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
#if defined(USART6)
        __HAL_RCC_USART6_CLK_ENABLE();
        _uart.Instance = USART6;
#if !defined(FRAMEWORK_STM32_CUBE_F1)
        alternate = GPIO_AF8_USART6;
#endif
#endif
    }

    // Initialize RX/TX pins
    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = gpioPin(_pins.rx.pin) | gpioPin(_pins.tx.pin);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Set as Alternate Function
    GPIO_InitStruct.Pull = GPIO_NOPULL; // or GPIO_PULLUP for open-drain lines
#if defined(FRAMEWORK_STM32_CUBE_F3)
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = alternate;
#elif defined(FRAMEWORK_STM32_CUBE_F4)
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = alternate;
#endif
    HAL_GPIO_Init(gpioPort(_pins.rx.port), &GPIO_InitStruct);

    uartInit();

    // Enable UART interrupt, calls back HAL_UART_RxCpltCallback when data received
    HAL_UART_Receive_IT(&_uart, &_rxByte, 1);


#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    const uint32_t config = (_parity == PARITY_NONE) ? SERIAL_8N1 : (_parity == PARITY_EVEN) ? SERIAL_8E1 : SERIAL_8O1;
    _uart.begin(_baudrate, config, _pins.rx.pin, _pins.tx.pin);
#endif
#endif
}

void SerialPort::uartInit() // NOLINT(readability-make-member-function-const)
{
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    _uart.Init.BaudRate = _baudrate;
    _uart.Init.WordLength = UART_WORDLENGTH_8B;
    _uart.Init.StopBits = UART_STOPBITS_1;
    _uart.Init.Parity = (_parity == PARITY_NONE) ? UART_PARITY_NONE : (_parity == PARITY_EVEN) ? UART_PARITY_EVEN : UART_PARITY_ODD;
    _uart.Init.Mode = UART_MODE_TX_RX;
    _uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    _uart.Init.OverSampling = UART_OVERSAMPLING_16;
#if defined(FRAMEWORK_STM32_CUBE_F3)
    // pin inversion on STM32_F3
    if (_pins.rx.inverted) {
        _uart.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
        _uart.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
    }
    if (_pins.tx.inverted) {
        _uart.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_TXINVERT_INIT;
        _uart.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
    }
#endif
    HAL_UART_Init(&_uart);
#endif
}

/*!
This waits for data from the serial UART
*/
int32_t SerialPort::WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait) // NOLINT(readability-make-member-function-const)
{
    return WAIT_DATA_READY(ticksToWait);
}

bool SerialPort::isDataAvailable() const
{
#if defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    return uart_is_readable(_uart);
#elif defined(FRAMEWORK_ESPIDF)
    return false;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    return (__HAL_UART_GET_FLAG(&_uart, UART_FLAG_RXNE)) ? true : false;
#elif defined(FRAMEWORK_TEST)
    return false;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    return const_cast<HardwareSerial&>(_uart).available() > 0;
#else
    return Serial.available() > 0;
#endif
#endif
}

/*!
Used to get received byte when using time-based scheduling.
*/
uint8_t SerialPort::readByte()
{
#if defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    return uart_getc(_uart);
#elif defined(FRAMEWORK_ESPIDF)
    return 0;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#if defined(FRAMEWORK_STM32_CUBE_F4)
    const uint8_t data = static_cast<uint8_t>(_uart.Instance->DR & 0xFF);
#else
    uint8_t data {}; // NOLINT(misc-const-correctness)
    HAL_UART_Receive(&_uart, &data, sizeof(data), HAL_MAX_DELAY);
#endif
    return data;
#elif defined(FRAMEWORK_TEST)
    return 0;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    return static_cast<uint8_t>(_uart.read());
#else
    return Serial.read();
#endif
#endif
}

size_t SerialPort::availableForWrite()
{
#if defined(FRAMEWORK_RPI_PICO)
    return uart_is_writable(_uart);
#elif defined(FRAMEWORK_ESPIDF)
    return 0;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    return (__HAL_UART_GET_FLAG(&_uart, UART_FLAG_TXE)) ? true : false;
#elif defined(FRAMEWORK_TEST)
    return 0;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    return _uart.availableForWrite();
#else
    return Serial.availableForWrite();
#endif
#endif
}

void SerialPort::writeByte(uint8_t data)
{
#if defined(FRAMEWORK_RPI_PICO)
    uart_putc_raw(_uart, data);
#elif defined(FRAMEWORK_ESPIDF)
    (void)data;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    HAL_UART_Transmit(&_uart, &data, 1, HAL_MAX_DELAY);
#elif defined(FRAMEWORK_TEST)
    (void)data;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    _uart.write(data);
#else
    Serial.write(data);
#endif
#endif
}

size_t SerialPort::write(const uint8_t* buf, size_t len)
{
#if defined(FRAMEWORK_RPI_PICO)
    uart_write_blocking (_uart, buf, len);
    return len;
#elif defined(FRAMEWORK_ESPIDF)
    (void)buf;
    (void)len;
    return 0;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    HAL_UART_Transmit(&_uart, buf, len, HAL_MAX_DELAY);
    return len;
#elif defined(FRAMEWORK_TEST)
    (void)buf;
    (void)len;
    return 0;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    return _uart.write(buf, len);
#else
    return Serial.write(buf, len);
#endif
#endif
}

bool SerialPort::onDataReceivedFromISR(uint8_t data)
{
    return _watcher ? _watcher->onDataReceivedFromISR(data) : true;
}

uint32_t SerialPort::setBaudrate(uint32_t baudrate)
{
    _baudrate = baudrate;
#if defined(FRAMEWORK_RPI_PICO)
    return uart_set_baudrate (_uart, baudrate);
#elif defined(FRAMEWORK_ESPIDF)
    return baudrate;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    HAL_UART_DeInit(&_uart);
    uartInit();
    return baudrate;
#elif defined(FRAMEWORK_TEST)
    return baudrate;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    _uart.updateBaudRate(baudrate);
    return baudrate;
#else
    return baudrate;
#endif
#endif
}
