#pragma once

#include <cstddef>
#include <cstdint>

class ReceiverBase;

size_t packTelemetryData_Receiver(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const ReceiverBase& receiver); // NOLINT(readability-avoid-const-params-in-decls) false positive
