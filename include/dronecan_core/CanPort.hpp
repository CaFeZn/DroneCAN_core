#pragma once

#include <cstdint>

#include "can.hpp"
#include "dronecan_core/CanPoller.hpp"
#include "dronecan_core/DroneCANNode.hpp"

/**
 * @file CanPort.hpp
 * @brief DroneCAN ESC 状态发布辅助接口。
 *
 * @details
 * 该头文件定义了应用层用于发布 ESC 状态和推进底层 CAN/DroneCAN 轮询的轻量包装。
 * 这样 `DroneCANCore` 可以只依赖一个小型门面，而不必直接操作底层 `CAN` 驱动和
 * `DroneCANNode` 的细节。
 */

namespace DroneCANCoreSupport
{

/**
 * @brief 单路 ESC 状态报文的抽象遥测值。
 *
 * @details
 * 该结构与 `uavcan.equipment.esc.Status` 的编码字段一一对应，
 * 用于在应用内部组织状态数据，再交给 `EscCodec` 完成 DroneCAN 负载编码。
 */
struct EscStatusTelemetry
{
  /** @brief 错误计数，直接映射到 ESC 状态报文的 `error_count` 字段。 */
  std::uint32_t error_count = 0U;
  /** @brief 母线电压估计值，单位 V。 */
  float voltage = 12.0F;
  /** @brief 相电流或输出电流估计值，单位 A。 */
  float current = 0.0F;
  /** @brief 温度估计值，单位摄氏度。 */
  float temperature = 25.0F;
  /** @brief 转速估计值，单位 RPM，可为负值表示反转。 */
  std::int32_t rpm = 0;
  /** @brief 功率利用率百分比，范围 0~100，编码时会再裁剪到协议上限。 */
  std::uint8_t power_rating_pct = 0U;
  /** @brief ESC 在节点内部的逻辑索引。 */
  std::uint8_t esc_index = 0U;
};

/**
 * @brief DroneCAN 应用使用的 CAN 端口门面。
 *
 * @details
 * 该类同时持有：
 * - 一个底层 `LibXR::CAN` 引用，用于标识当前总线
 * - 一个可选的 `CanPoller` 指针，用于推进平台 CAN 驱动轮询
 * - 一个模块内 `DroneCANNode` 引用，用于广播 DroneCAN 消息
 *
 * 其职责是把“平台 CAN 驱动轮询”和“DroneCAN 协议栈轮询”封装成一个统一入口，
 * 并提供 ESC 状态广播的便捷接口。
 */
class CanPort
{
 public:
  /**
   * @brief 构造应用层 CAN 门面。
   * @param node 与当前节点绑定的 DroneCAN 协议栈实例。
   * @param can 与节点共用的底层 CAN 驱动实例。
   * @param poller 可选的平台 CAN 轮询接口。
   *
   * @details
   * 调用方需要保证 `node` 与 `can` 生命周期长于本对象，
   * 且二者属于同一条物理总线。
   * 若 `poller` 为空，`Poll()` 将退化为仅推进 DroneCAN 协议栈。
   */
  CanPort(DroneCANNode& node, LibXR::CAN& can, CanPoller* poller = nullptr);

  /**
   * @brief 推进一次底层 CAN 驱动与 DroneCAN 协议栈。
   *
   * @details
   * 调用顺序固定为：
   * 1. `poller_->Poll()`，若存在则先服务平台 CAN 驱动。
   * 2. `node_.Poll()`，再处理接收到的 DroneCAN 帧以及待发送的协议帧。
   *
   * 这样可以确保主循环模式下，硬件接收队列先被搬运到协议层，再由协议层执行分发。
   */
  void Poll();

  /**
   * @brief 广播一条 ESC 状态消息。
   * @param telemetry 已组织好的单路 ESC 遥测值。
   * @return `ErrorCode::OK` 表示已成功提交到 DroneCAN 节点发送队列。
   *
   * @details
   * 该函数会先把 `telemetry` 编码成 `uavcan.equipment.esc.Status` 负载，
   * 再以中等优先级广播到当前 DroneCAN 总线。
   */
  LibXR::ErrorCode PublishEscStatus(const EscStatusTelemetry& telemetry);

 private:
  /** @brief 当前应用绑定的 DroneCAN 节点实例。 */
  DroneCANNode& node_;
  /** @brief 当前应用绑定的底层 CAN 驱动实例。 */
  LibXR::CAN& can_;
  /** @brief 当前平台可用的 CAN 轮询接口。 */
  CanPoller* poller_ = nullptr;
};

}  // namespace DroneCANCoreSupport

