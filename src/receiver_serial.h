#pragma once

#include "serial_port.h"
#include "receiver_base.h"


class ReceiverSerialPortWatcher : public SerialPortWatcherBase {
public:
    virtual ~ReceiverSerialPortWatcher() = default;
    explicit ReceiverSerialPortWatcher(ReceiverBase& receiver);
    bool on_data_received_from_isr(uint8_t data) override;
private:
    ReceiverBase& _receiver;
};


class ReceiverSerial : public ReceiverBase {
public:
    explicit ReceiverSerial(SerialPort& serialPort);
    void init();
private:
    // ReceiverSerial is not copyable or moveable
    ReceiverSerial(const ReceiverSerial&) = delete;
    ReceiverSerial& operator=(const ReceiverSerial&) = delete;
    ReceiverSerial(ReceiverSerial&&) = delete;
    ReceiverSerial& operator=(ReceiverSerial&&) = delete;
public:
    virtual int32_t WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait) override;
    virtual bool is_data_available() const override;
    virtual uint8_t read_byte() override;
    virtual bool update(uint32_t tick_count_delta) override;
    bool is_packet_empty() const { return _packet_is_empty; }
    void set_packet_empty() { _packet_is_empty = true; }
    size_t get_packet_index() const { return _packet_index; } // for testing
protected:
    SerialPort& _serial_port;
    ReceiverSerialPortWatcher _serial_port_watcher;
    bool _packet_is_empty {true};
    uint32_t _received_packet_count {};
    int32_t _error_packet_count {};
    size_t _packet_index {};
    time_us32_t _start_time {};
};
