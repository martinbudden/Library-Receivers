#include "ReceiverSerial.h"


ReceiverSerialPortWatcher::ReceiverSerialPortWatcher(ReceiverBase& receiver) :
    _receiver(receiver)
{
}

bool ReceiverSerialPortWatcher::onDataReceivedFromISR(uint8_t data)
{
    return _receiver.onDataReceivedFromISR(data);
}


ReceiverSerial::ReceiverSerial(SerialPort& serialPort) :
    _serialPort(serialPort),
    _serialPortWatcher(*this)
{
}

void ReceiverSerial::init()
{
    _serialPort.init();
    _packetCount = 0;
}

/*!
This waits for data from the serial UART
*/
int32_t ReceiverSerial::WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait)
{
    return _serialPort.WAIT_DATA_READY(ticksToWait);
}

bool ReceiverSerial::isDataAvailable() const
{
    return _serialPort.isDataAvailable();
}

/*!
Used to get received byte when using time-based scheduling.
*/
uint8_t ReceiverSerial::readByte()
{
    return _serialPort.readByte();
}

/*!
If a packet was received then unpack it and return true.

Returns false if an empty or invalid packet was received.
*/
bool ReceiverSerial::update(uint32_t tickCountDelta)
{
    if (isPacketEmpty()) {
        return false;
    }

    if (!unpackPacket()) {
        return false;
    }

    _packetReceived = true;
    ++_packetCount;

    // record tickoutDelta for instrumentation
    _tickCountDelta = tickCountDelta;

    // track dropped packets
    _droppedPacketCountDelta = _droppedPacketCount - _droppedPacketCountPrevious;
    _droppedPacketCountPrevious = _droppedPacketCount;

    // NOTE: there is no mutex around this flag
    _newPacketAvailable = true;
    return true;
}
