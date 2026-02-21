#include "receiver_serial.h"


ReceiverSerialPortWatcher::ReceiverSerialPortWatcher(ReceiverBase& receiver) :
    _receiver(receiver)
{
}

bool ReceiverSerialPortWatcher::on_data_received_from_isr(uint8_t data)
{
    return _receiver.on_data_received_from_isr(data);
}


ReceiverSerial::ReceiverSerial(SerialPort& serialPort) :
    _serial_port(serialPort),
    _serial_port_watcher(*this)
{
}

void ReceiverSerial::init()
{
    _serial_port.init();
    _packet_count = 0;
}

/*!
This waits for data from the serial UART
*/
int32_t ReceiverSerial::WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait)
{
    return _serial_port.WAIT_DATA_READY(ticksToWait);
}

bool ReceiverSerial::is_data_available() const
{
    return _serial_port.is_data_available();
}

/*!
Used to get received byte when using time-based scheduling.
*/
uint8_t ReceiverSerial::read_byte()
{
    return _serial_port.read_byte();
}

/*!
If a packet was received then unpack it and return true.

Returns false if an empty or invalid packet was received.
*/
bool ReceiverSerial::update(uint32_t tick_count_delta)
{
    if (is_packet_empty()) {
        return false;
    }

    if (!unpack_packet()) {
        return false;
    }

    _packet_received = true;
    ++_packet_count;

    // record tickoutDelta for instrumentation
    _tick_count_delta = tick_count_delta;

    // track dropped packets
    _dropped_packet_count_delta = _dropped_packet_count - _dropped_packet_count_previous;
    _dropped_packet_count_previous = _dropped_packet_count;

    // NOTE: there is no mutex around this flag
    _new_packet_available = true;
    return true;
}
