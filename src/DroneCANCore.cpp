#include "dronecan_core/DroneCANCore.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "adc.hpp"
#include "dac.hpp"
#include "dronecan_esc_raw_command/EscCodec.hpp"
#include "libxr.hpp"


using LibXR::ErrorCode;

const char* DroneCANCore::NormalizeCString(const char* value, const char* fallback) noexcept
{
  if ((value != nullptr) && (value[0] != '\0'))
  {
    return value;
  }
  return fallback;
}

std::uint32_t DroneCANCore::NormalizePeriodMs(std::uint32_t period_ms) noexcept
{
  return (period_ms == 0U) ? 1U : period_ms;
}

std::uint8_t DroneCANCore::NormalizeEscCount(std::uint8_t esc_count) noexcept
{
  if (esc_count == 0U)
  {
    return 1U;
  }

  const std::size_t bounded_count = std::min<std::size_t>(esc_count, kMaxEscChannels);
  return static_cast<std::uint8_t>(bounded_count);
}

std::int32_t DroneCANCore::ClampRawCommand(std::int16_t raw_command) noexcept
{
  return std::clamp<std::int32_t>(raw_command, -8191, 8191);
}

std::int32_t DroneCANCore::AbsI32(std::int32_t value) noexcept
{
  return (value >= 0) ? value : -value;
}

float DroneCANCore::RawCommandToSignedVoltage(std::int16_t raw_command) noexcept
{
  const std::int32_t clamped_raw = ClampRawCommand(raw_command);
  const std::uint32_t shifted = static_cast<std::uint32_t>(clamped_raw + 8191);
  return (static_cast<float>(shifted) * kAnalogReferenceVoltage) / 16382.0F;
}

float DroneCANCore::RawMagnitudeToVoltage(std::int32_t raw_magnitude) noexcept
{
  const std::int32_t clamped_raw = std::clamp<std::int32_t>(raw_magnitude, 0, 8191);
  return (static_cast<float>(clamped_raw) * kAnalogReferenceVoltage) / 8191.0F;
}

LibXR::DroneCAN::Config DroneCANCore::MakeNodeConfig(std::uint32_t node_status_period_ms) noexcept
{
  LibXR::DroneCAN::Config config{};
  config.node_status_period_us =
      static_cast<std::uint64_t>(NormalizePeriodMs(node_status_period_ms)) * 1000ULL;
  config.cleanup_interval_us = 1000000ULL;
  config.rx_queue_size = 16U;
  return config;
}

LibXR::DroneCAN::NodeInfo DroneCANCore::MakeNodeInfo(const char* node_name)
{
  LibXR::DroneCAN::NodeInfo info{};
  const char* normalized_name = NormalizeCString(node_name, "org.libxr.dronecan_core");
  std::strncpy(info.name, normalized_name, sizeof(info.name) - 1U);
  info.name[sizeof(info.name) - 1U] = '\0';
  info.hardware_version_major = 1U;
  info.hardware_version_minor = 0U;
  return info;
}

DroneCANCoreSupport::EscStatusTelemetry DroneCANCore::MakeEscStatusTelemetry(
    std::uint8_t esc_index, std::int16_t raw_command, std::uint32_t now_ms) const noexcept
{
  DroneCANCoreSupport::EscStatusTelemetry telemetry{};
  const std::int32_t raw_abs = AbsI32(ClampRawCommand(raw_command));
  const std::int32_t signed_rpm =
      (raw_abs * kSyntheticEscMaxRpm / 8191) *
      ((raw_command < 0) ? -1 : 1);

  telemetry.error_count = 0U;
  telemetry.voltage = 12.0F;
  telemetry.current = static_cast<float>(raw_abs) / 2048.0F;
  telemetry.temperature = 25.0F + static_cast<float>(esc_index) + (telemetry.current * 2.0F);
  telemetry.rpm = signed_rpm;
  telemetry.power_rating_pct =
      static_cast<std::uint8_t>((static_cast<std::uint32_t>(raw_abs) * 100U) /
                                8191U);
  telemetry.esc_index = esc_index;

  if (feedback_adc_ != nullptr)
  {
    const float magnitude = static_cast<float>(raw_abs) / 8191.0F;
    telemetry.voltage = 9.0F + (last_feedback_voltage_ * 2.0F);
    telemetry.current = last_feedback_voltage_ * magnitude;
    telemetry.temperature =
        25.0F + (last_feedback_voltage_ * 6.0F) + (magnitude * 20.0F);
  }

  return telemetry;
}

DroneCANCore::DroneCANCore(LibXR::HardwareContainer& hw,
                         LibXR::ApplicationManager& appmgr,
                         std::uint8_t node_id,
                         std::uint32_t heartbeat_period_ms,
                         std::uint32_t node_status_period_ms,
                         const char* can_alias,
                         const char* timebase_alias,
                         const char* node_name,
                         std::uint8_t esc_count,
                         const char* can_poller_alias,
                         bool publish_idle_esc_status,
                         bool enable_dynamic_node_id,
                         std::uint8_t preferred_node_id)
    : timebase_(*hw.FindOrExit<LibXR::Timebase>(
          {NormalizeCString(timebase_alias, "timebase")})),
      can_(*hw.FindOrExit<LibXR::CAN>({NormalizeCString(can_alias, "can0")})),
      node_(can_, timebase_, node_arena_.data(), node_arena_.size(),
            MakeNodeConfig(node_status_period_ms)),
      can_port_(node_, can_,
                hw.Find<DroneCANCoreSupport::CanPoller>(
                    {NormalizeCString(can_poller_alias, "can0_poller")})),
      heartbeat_feature_(node_, NormalizePeriodMs(heartbeat_period_ms), this,
                         MakeHeartbeatStatusCodeStatic),
      dynamic_node_id_feature_(node_, enable_dynamic_node_id, preferred_node_id,
                               MakeNodeInfo(node_name)),
      esc_raw_command_feature_(NormalizeEscCount(esc_count)),
      esc_status_feature_(can_port_, esc_raw_command_feature_, NormalizeEscCount(esc_count),
                          publish_idle_esc_status, this,
                          +[](void* context, std::uint8_t esc_index, std::int16_t raw_command,
                              std::uint32_t now_ms) -> DroneCANCoreSupport::EscStatusTelemetry {
                            return static_cast<DroneCANCore*>(context)->MakeEscStatusTelemetry(
                                esc_index, raw_command, now_ms);
                          }),
      heartbeat_period_ms_(NormalizePeriodMs(heartbeat_period_ms)),
      esc_count_(NormalizeEscCount(esc_count)),
      publish_idle_esc_status_(publish_idle_esc_status)
{
  InitializeHardwareBindings(hw);
  InitializeNode(enable_dynamic_node_id ? 0U : node_id, node_name);
  ASSERT(node_.RegisterTransferHandler(LibXR::DroneCAN::TransferKind::Message,
                                       DroneCANCoreSupport::EscCodec::kRawCommandDataTypeId,
                                       DroneCANCoreSupport::EscCodec::kRawCommandSignature,
                                       esc_raw_command_feature_.GetTransferHandler()) == ErrorCode::OK);
  ASSERT(node_.RegisterTransferHandler(
             LibXR::DroneCAN::TransferKind::Message,
             LibXR::DroneCAN::DYNAMIC_NODE_ID_ALLOCATION_DATA_TYPE_ID,
             LibXR::DroneCAN::DYNAMIC_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE,
             dynamic_node_id_feature_.GetTransferHandler()) == ErrorCode::OK);

  const std::uint32_t now_ms =
      static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds());
  next_due_ms_ = now_ms + heartbeat_period_ms_;
  dynamic_node_id_feature_.OnStart();

  appmgr.Register(*this);

  LibXR::STDIO::Printf("[DroneCANCore] ready node=%u hb=%lu status=%lu esc=%u can=%s\r\n",
                       static_cast<unsigned>(node_.GetNodeID()),
                       static_cast<unsigned long>(heartbeat_period_ms_),
                       static_cast<unsigned long>(NormalizePeriodMs(node_status_period_ms)),
                       static_cast<unsigned>(esc_count_),
                       NormalizeCString(can_alias, "can0"));
}

void DroneCANCore::OnMonitor()
{
  can_port_.Poll();
  const std::uint64_t now_us =
      static_cast<std::uint64_t>(LibXR::Timebase::GetMicroseconds());
  esc_raw_command_feature_.OnPoll(0U, now_us);
  const std::uint32_t now_ms =
      static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds());
  dynamic_node_id_feature_.OnPoll(now_ms, now_us);
  esc_status_feature_.OnPoll(now_ms, now_us);
  heartbeat_feature_.OnPoll(now_ms, now_us);
}

void DroneCANCore::InitializeNode(std::uint8_t node_id, const char* node_name)
{
  ASSERT(node_.SetNodeID(node_id) == ErrorCode::OK);
  node_.SetNodeInfo(MakeNodeInfo(node_name));
  node_.SetNodeStatusHealth(LibXR::DroneCAN::NodeHealth::OK);
  node_.SetNodeStatusMode((node_id == 0U) ? LibXR::DroneCAN::NodeMode::INITIALIZATION
                                          : LibXR::DroneCAN::NodeMode::OPERATIONAL);
  node_.SetVendorSpecificStatusCode(0U);
  node_.SetSubMode(0U);
}


void DroneCANCore::InitializeHardwareBindings(LibXR::HardwareContainer& hw) noexcept
{
  feedback_sources_[0] =
      hw.Find<DroneCANCoreSupport::EscFeedbackSource>({"esc_feedback0", "pwm_in0"});
  esc_status_feature_.BindFeedbackSource(0U, feedback_sources_[0]);
  feedback_sources_[1] =
      hw.Find<DroneCANCoreSupport::EscFeedbackSource>({"esc_feedback1", "pwm_in1"});
  esc_status_feature_.BindFeedbackSource(1U, feedback_sources_[1]);
  feedback_sources_[2] =
      hw.Find<DroneCANCoreSupport::EscFeedbackSource>({"esc_feedback2", "pwm_in2"});
  esc_status_feature_.BindFeedbackSource(2U, feedback_sources_[2]);
  feedback_sources_[3] =
      hw.Find<DroneCANCoreSupport::EscFeedbackSource>({"esc_feedback3", "pwm_in3"});
  esc_status_feature_.BindFeedbackSource(3U, feedback_sources_[3]);
}

void DroneCANCore::UpdateAnalogFeedback() noexcept
{
}

std::uint16_t DroneCANCore::MakeHeartbeatStatusCodeStatic(void* self) noexcept
{
  return (self != nullptr) ? static_cast<DroneCANCore*>(self)->MakeHeartbeatStatusCode() : 0U;
}

std::uint16_t DroneCANCore::MakeHeartbeatStatusCode() const noexcept
{
  return static_cast<std::uint16_t>(((node_.GetRxDropCount() & 0x0FU) << 12U) |
                                    ((node_.GetRxTransferCount() & 0x3FU) << 6U) |
                                    (node_.GetRxFrameCount() & 0x3FU));
}

