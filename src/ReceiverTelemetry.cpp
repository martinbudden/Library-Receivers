#include "ReceiverTelemetry.h"
#include "ReceiverTelemetryData.h"


/*!
Packs the Receiver telemetry data into a TD_RECEIVER packet. Returns the length of the packet.
*/
size_t packTelemetryData_Receiver(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const ReceiverBase& receiver)
{
    TD_RECEIVER* td = reinterpret_cast<TD_RECEIVER*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_RECEIVER::TYPE;
    td->len = sizeof(TD_RECEIVER);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    td->tickInterval = static_cast<uint16_t>(receiver.getTickCountDelta());
    td->droppedPacketCount = static_cast<uint16_t>(receiver.getDroppedPacketCountDelta());

    td->data.controls = receiver.getControls(),
    td->data.switches = receiver.getSwitches(),
    td->data.aux[0] = static_cast<uint16_t>(receiver.getAuxiliaryChannel(0));
    td->data.aux[1] = static_cast<uint16_t>(receiver.getAuxiliaryChannel(1));
    td->data.aux[2] = static_cast<uint16_t>(receiver.getAuxiliaryChannel(2));
    td->data.aux[3] = static_cast<uint16_t>(receiver.getAuxiliaryChannel(3));

    return td->len;
};
