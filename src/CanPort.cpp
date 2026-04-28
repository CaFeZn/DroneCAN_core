#include "dronecan_core/CanPort.hpp"

#include <array>

#include "dronecan_esc_raw_command/EscCodec.hpp"

using LibXR::ErrorCode;

namespace DroneCANCoreSupport
{

CanPort::CanPort(DroneCANNode& node, LibXR::CAN& can, CanPoller* poller)
    : node_(node), can_(can), poller_(poller)
{
}

void CanPort::Poll()
{
  if (poller_ != nullptr)
  {
    poller_->Poll();
  }
  node_.Poll();
}

LibXR::ErrorCode CanPort::PublishEscStatus(const EscStatusTelemetry& telemetry)
{
  std::array<std::uint8_t, EscCodec::kStatusMaxPayloadSize> payload{};
  const std::size_t payload_size = EscCodec::EncodeEscStatusPayload(telemetry, payload);
  return node_.Broadcast(EscCodec::kStatusDataTypeId, EscCodec::kStatusSignature,
                         CANARD_TRANSFER_PRIORITY_MEDIUM,
                         LibXR::ConstRawData(payload.data(), payload_size));
}

}  // namespace DroneCANCoreSupport

