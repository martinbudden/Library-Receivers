#pragma once

#include <cstddef>
#include <cstdint>

class ReceiverBase;

size_t pack_telemetry_data_receiver(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, const ReceiverBase& receiver); // NOLINT(readability-avoid-const-params-in-decls) false positive
