#pragma once

#include <array>
#include <cstdint>

#include "app_framework.hpp"
#include "can.hpp"
#include "dronecan_core/CanPort.hpp"
#include "dronecan_dynamic_node_id/dynamic_node_id_feature.hpp"
#include "dronecan_core/DroneCANHost.hpp"
#include "dronecan_esc_raw_command/esc_raw_command_feature.hpp"
#include "dronecan_esc_status/esc_status_feature.hpp"
#include "dronecan_heartbeat/heartbeat_feature.hpp"
#include "dronecan_core/DroneCANNode.hpp"
#include "dronecan_esc_status/EscFeedbackSource.hpp"
#include "timebase.hpp"

/**
 * @file DroneCANCore.hpp
 * @brief 四合一电调演示节点的 DroneCAN 应用声明。
 *
 * @details
 * 该应用负责：
 * - 初始化并维护一个 DroneCAN 节点
 * - 接收 `uavcan.equipment.esc.RawCommand`
 * - 采集四路 PWM 输入测量值
 * - 周期性广播 `uavcan.equipment.esc.Status`
 * - 通过双 LED 输出简单心跳
 *
 * 当前运行形态重点保留 `DroneCAN + PWM 捕获` 并行工作，
 * 模拟反馈链路的成员仍保留在接口中，但不会出现在主轮询路径里。
 */

namespace LibXR
{
class ADC;
class DAC;
}  // namespace LibXR

/**
 * @brief 基于 XRobot/LibXR 的 DroneCAN ESC 应用模块。
 *
 * @details
 * 该类作为一个 `LibXR::Application` 注册到应用管理器中，
 * 由主循环周期性调用 `OnMonitor()`。
 * 它内部组合了：
 * - `DroneCANCoreSupport::DroneCANNode` 协议栈
 * - `DroneCANCoreSupport::CanPort` 发送/轮询门面
 * - 板级 PWM 捕获设备
 * - 状态 LED 对象
 */
class DroneCANCore final : public LibXR::Application, public DroneCANCoreSupport::DroneCANHost
{
 public:
  /**
   * @brief 构造 DroneCAN 应用模块。
   * @param hw 硬件容器，用于按别名查找底层设备。
   * @param appmgr 应用管理器，构造函数会把自己注册进去。
   * @param node_id 本节点使用的静态 DroneCAN 节点 ID。
   * @param heartbeat_period_ms 应用心跳打印与 LED 翻转周期，单位毫秒。
   * @param node_status_period_ms DroneCAN NodeStatus 发布周期，单位毫秒。
   * @param can_alias CAN 设备别名。
   * @param timebase_alias 系统时基设备别名。
   * @param node_name 节点名称字符串。
   * @param esc_count 逻辑 ESC 通道数量。
   *
   * @details
   * 构造阶段会完成：
   * - 设备别名解析
   * - DroneCAN 节点初始化
   * - RawCommand 消息处理器注册
   * - 应用自身注册到 `appmgr`
   */
  DroneCANCore(LibXR::HardwareContainer& hw,
              LibXR::ApplicationManager& appmgr,
              std::uint8_t node_id = 10U,
              std::uint32_t heartbeat_period_ms = 500U,
              std::uint32_t node_status_period_ms = 1000U,
              const char* can_alias = "can0",
              const char* timebase_alias = "timebase",
              const char* node_name = "org.libxr.dronecan_core",
              std::uint8_t esc_count = 4U,
              const char* can_poller_alias = "can0_poller",
              bool publish_idle_esc_status = false,
              bool enable_dynamic_node_id = false,
              std::uint8_t preferred_node_id = 0U);

  /**
   * @brief 应用管理器周期调用的主监控入口。
   *
   * @details
   * 当前执行顺序为：
   * 1. 推进 CAN 硬件与 DroneCAN 协议栈
   * 2. 应用 RawCommand 超时保护
   * 3. 更新 PWM 捕获反馈
   * 4. 按需发布 ESC 状态
   * 5. 更新应用心跳与 LED
   */
  void OnMonitor() override;
  DroneCANCoreSupport::DroneCANNode& Node() noexcept override { return node_; }

 private:
  /** @brief DroneCAN 节点使用的静态内存池大小，单位字节。 */
  static constexpr std::size_t kNodeArenaSize = 4096U;
  /** @brief 协议层允许的最大 ESC 逻辑通道数。 */
  static constexpr std::size_t kMaxEscChannels = 20U;
  /** @brief 当前硬件平台实际接入的 ESC/PWM 通道数。 */
  static constexpr std::size_t kHardwareEscChannels = 4U;
  /** @brief 当没有真实测速反馈时，用于状态合成的最大转速。 */
  static constexpr std::int32_t kSyntheticEscMaxRpm = 6000;
  /** @brief 历史模拟链路使用的参考电压，单位 V。 */
  static constexpr float kAnalogReferenceVoltage = 3.3F;
  /** @brief RawCommand 超时保护窗口，超时后自动清零输出，单位毫秒。 */
  static constexpr std::uint32_t kEscCommandTimeoutMs = 200U;
  /** @brief ESC 状态广播周期，单位毫秒。 */
  static constexpr std::uint32_t kEscStatusPeriodMs = 50U;
  /** @brief PWM 测量值的新鲜度窗口，超出后认为反馈过期，单位毫秒。 */
  static constexpr std::uint32_t kPwmFeedbackStaleMs = 200U;
  /** @brief 动态分配请求重发周期，单位毫秒。 */
  static constexpr std::uint32_t kDynamicIdRequestPeriodMs = 1000U;

  /**
   * @brief 单路 PWM 捕获反馈缓存。
   *
   * @details
   * 用于保存最近一次有效 PWM 测量，
   * 并在生成 ESC 状态时决定是否用真实频率覆盖合成转速。
   */
  struct PwmCaptureFeedback
  {
    /** @brief 当前缓存是否包含有效测量结果。 */
    bool valid = false;
    /** @brief 最近一次测得的 PWM 频率，单位 Hz。 */
    std::uint32_t frequency_hz = 0U;
    /** @brief 最近一次测得的占空比，单位为万分比。 */
    std::uint16_t duty_u10000 = 0U;
    /** @brief 最近一次更新该缓存的系统时间戳，单位毫秒。 */
    std::uint32_t last_update_ms = 0U;
  };

  /**
   * @brief 规范化可选 C 字符串参数。
   * @param value 待检查字符串。
   * @param fallback 备用字符串。
   * @return 若 `value` 非空则返回 `value`，否则返回 `fallback`。
   */
  static const char* NormalizeCString(const char* value, const char* fallback) noexcept;

  /**
   * @brief 规范化周期参数。
   * @param period_ms 输入周期，单位毫秒。
   * @return 至少为 1 的有效周期值。
   */
  static std::uint32_t NormalizePeriodMs(std::uint32_t period_ms) noexcept;

  /**
   * @brief 规范化 ESC 通道数量。
   * @param esc_count 输入通道数。
   * @return 经过上下界裁剪后的通道数。
   */
  static std::uint8_t NormalizeEscCount(std::uint8_t esc_count) noexcept;

  /**
   * @brief 把原始 ESC 指令裁剪到协议支持范围。
   * @param raw_command 待裁剪原始指令。
   * @return 裁剪后的指令值。
   */
  static std::int32_t ClampRawCommand(std::int16_t raw_command) noexcept;

  /**
   * @brief 计算 32 位整数绝对值。
   * @param value 输入值。
   * @return 绝对值结果。
   */
  static std::int32_t AbsI32(std::int32_t value) noexcept;

  /**
   * @brief 把原始 ESC 指令映射为模拟链路使用的有符号电压。
   * @param raw_command 原始 ESC 指令。
   * @return 对应电压值，单位 V。
   *
   * @note 当前主运行路径未使用该函数，保留是为了兼容后续重新接回模拟链路。
   */
  static float RawCommandToSignedVoltage(std::int16_t raw_command) noexcept;

  /**
   * @brief 把原始指令幅值映射为模拟幅值电压。
   * @param raw_magnitude 原始指令幅值。
   * @return 对应电压值，单位 V。
   *
   * @note 当前主运行路径未使用该函数，保留是为了兼容后续重新接回模拟链路。
   */
  static float RawMagnitudeToVoltage(std::int32_t raw_magnitude) noexcept;

  /**
   * @brief 构造 DroneCAN 节点配置。
   * @param node_status_period_ms NodeStatus 发布周期，单位毫秒。
   * @return 供模块内 `DroneCANNode` 使用的配置对象。
   */
  static LibXR::DroneCAN::Config MakeNodeConfig(std::uint32_t node_status_period_ms) noexcept;

  /**
   * @brief 构造节点信息对象。
   * @param node_name 节点名称。
   * @return 已填好名称和硬件版本字段的 `NodeInfo`。
   */
  static LibXR::DroneCAN::NodeInfo MakeNodeInfo(const char* node_name);

  /**
   * @brief RawCommand 回调的静态桥接函数。
   * @param in_isr 当前是否在中断上下文。
   * @param self 当前应用对象指针。
   * @param meta 传输元数据。
   * @param payload 原始负载。
   */

  /**
   * @brief 生成一条 ESC 状态广播所需的遥测值。
   * @param esc_index 当前 ESC 索引。
   * @param raw_command 当前应用到该路的 RawCommand。
   * @param now_ms 当前系统时间，单位毫秒。
   * @return 已组织好的 ESC 状态抽象值。
   */
  DroneCANCoreSupport::EscStatusTelemetry MakeEscStatusTelemetry(std::uint8_t esc_index,
                                                                std::int16_t raw_command,
                                                                std::uint32_t now_ms) const
      noexcept;

  /**
   * @brief 初始化 DroneCAN 节点的静态身份与运行状态。
   * @param node_id 节点 ID。
   * @param node_name 节点名称。
   */
  void InitializeNode(std::uint8_t node_id, const char* node_name);

  /**
   * @brief 从硬件容器中解析并缓存应用要用到的板级设备。
   * @param hw 硬件容器。
   *
   * @details
   * 当前会主动绑定四路 PWM 捕获设备。
   * 模拟链路相关成员仍保留在类中，但当前不会再从容器中查找和驱动它们。
   */
  void InitializeHardwareBindings(LibXR::HardwareContainer& hw) noexcept;

  /**
   * @brief 处理一帧 RawCommand 消息。
   * @param meta 传输元数据。
   * @param payload 原始负载。
   *
   * @details
   * 解码成功后会覆盖内部 `last_raw_commands_` 与 `applied_raw_commands_`，
   * 并更新最后一次命令时间戳。
   */
  /**
   * @brief 推进历史模拟反馈链路。
   *
   * @details
   * 该函数保留了 DAC/ADC/PGA 方案的兼容实现，
   * 但当前 `OnMonitor()` 已不再调用它。
   */
  void UpdateAnalogFeedback() noexcept;

  /**
   * @brief 更新四路 PWM 捕获缓存。
   * @param now_ms 当前系统时间，单位毫秒。
   */
  /**
   * @brief 执行应用层心跳逻辑。
   * @param now_ms 当前系统时间，单位毫秒。
   *
   * @details
   * 该函数会翻转 LED、刷新 `vendor_specific_status_code`，并输出一条调试日志。
   */
  static std::uint16_t MakeHeartbeatStatusCodeStatic(void* self) noexcept;
  std::uint16_t MakeHeartbeatStatusCode() const noexcept;

  /** @brief 系统时基引用。 */
  LibXR::Timebase& timebase_;
  /** @brief 当前应用绑定的底层 CAN 驱动引用。 */
  LibXR::CAN& can_;
  /** @brief 第一颗状态灯 GPIO 引用。 */
  /** @brief DroneCAN 节点静态内存池。 */
  std::array<std::uint8_t, kNodeArenaSize> node_arena_{};
  /** @brief 当前应用内部持有的 DroneCAN 节点实例。 */
  DroneCANCoreSupport::DroneCANNode node_;
  /** @brief 应用使用的 CAN/DroneCAN 门面。 */
  DroneCANCoreSupport::CanPort can_port_;
  /** @brief 默认启用的心跳功能。 */
  DroneCANCoreSupport::HeartbeatFeature heartbeat_feature_;
  /** @brief 动态节点 ID 功能。 */
  DroneCANCoreSupport::DynamicNodeIdFeature dynamic_node_id_feature_;
  /** @brief ESC RawCommand 功能。 */
  DroneCANCoreSupport::EscRawCommandFeature esc_raw_command_feature_;
  /** @brief ESC Status 功能。 */
  DroneCANCoreSupport::EscStatusFeature esc_status_feature_;

  /**
   * @brief 历史模拟反馈 ADC 设备指针。
   *
   * @note 当前主运行路径不会主动驱动该设备，通常保持为空。
   */
  LibXR::ADC* feedback_adc_ = nullptr;

  /**
   * @brief 历史模拟反馈监测 DAC 设备指针。
   *
   * @note 当前主运行路径不会主动驱动该设备，通常保持为空。
   */
  LibXR::DAC* feedback_monitor_dac_ = nullptr;

  /**
   * @brief 历史模拟闭环源 DAC 设备指针。
   *
   * @note 当前主运行路径不会主动驱动该设备，通常保持为空。
   */
  LibXR::DAC* loop_source_dac_ = nullptr;

  /**
   * @brief 历史 ESC 模拟输出 DAC 数组。
   *
   * @note 当前主运行路径不会主动驱动这些设备。
   */
  std::array<LibXR::DAC*, kHardwareEscChannels> esc_command_dacs_{};

  /** @brief ESC 反馈源设备指针数组。 */
  std::array<DroneCANCoreSupport::EscFeedbackSource*, kHardwareEscChannels> feedback_sources_{};
  /** @brief 应用心跳周期，单位毫秒。 */
  std::uint32_t heartbeat_period_ms_;
  /** @brief 当前逻辑 ESC 通道数。 */
  std::uint8_t esc_count_ = 1U;
  bool publish_idle_esc_status_ = false;
  /** @brief 下一次执行心跳逻辑的到期时间，单位毫秒。 */
  std::uint32_t next_due_ms_ = 0U;
  /**
   * @brief 历史模拟反馈链路最近一次电压采样值。
   *
   * @details
   * 即使当前模拟链路未运行，该成员仍保留，用于兼容现有状态打印格式。
   */
  float last_feedback_voltage_ = 0.0F;

};

