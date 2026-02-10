#include "ReceiverTelemetry.h"
#include "ReceiverTelemetryData.h"


/*!
Packs the Receiver telemetry data into a TD_RECEIVER packet. Returns the length of the packet.
*/
size_t pack_telemetry_data_receiver(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, const ReceiverBase& receiver)
{
    TD_RECEIVER* td = reinterpret_cast<TD_RECEIVER*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_RECEIVER::TYPE;
    td->len = sizeof(TD_RECEIVER);
    td->subType = 0;
    td->sequence_number = static_cast<uint8_t>(sequence_number);

    td->tickInterval = static_cast<uint16_t>(receiver.get_tick_count_delta());
    td->dropped_packet_count = static_cast<uint16_t>(receiver.get_dropped_packet_count_delta());

    td->data.controls = receiver.get_controls(),
    td->data.switches = receiver.get_switches(),
    td->data.aux[0] = static_cast<uint16_t>(receiver.get_auxiliary_channel(0));
    td->data.aux[1] = static_cast<uint16_t>(receiver.get_auxiliary_channel(1));
    td->data.aux[2] = static_cast<uint16_t>(receiver.get_auxiliary_channel(2));
    td->data.aux[3] = static_cast<uint16_t>(receiver.get_auxiliary_channel(3));

    return td->len;
};
