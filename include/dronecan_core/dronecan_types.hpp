#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "libxr_cb.hpp"
#include "libxr_type.hpp"

/**
 * @file dronecan_types.hpp
 * @brief `DroneCANCore` 模块内聚使用的 DroneCAN 基础类型定义。
 *
 * @details
 * 主线 `libxr` 已不再导出这些 DroneCAN 基础类型；当前模块为了保持
 * `DroneCANCore` 及其节点实现的可移植性，将所需的最小类型集合收拢到本模块。
 */

namespace LibXR::DroneCAN
{

/** @brief DroneCAN 传输类别。 */
enum class TransferKind : std::uint8_t
{
  Message = 0,
  Request = 1,
  Response = 2,
};

/** @brief DroneCAN 标准节点健康状态。 */
enum class NodeHealth : std::uint8_t
{
  OK = 0,
  WARNING = 1,
  ERROR = 2,
  CRITICAL = 3,
};

/** @brief DroneCAN 标准节点模式。 */
enum class NodeMode : std::uint8_t
{
  OPERATIONAL = 0,
  INITIALIZATION = 1,
  MAINTENANCE = 2,
  SOFTWARE_UPDATE = 3,
  OFFLINE = 7,
};

/**
 * @brief 节点身份与版本信息。
 *
 * @details
 * 用于构造 `GetNodeInfo` 响应负载。
 */
struct NodeInfo
{
  /** @brief 软件主版本号。 */
  std::uint8_t software_version_major = 1U;
  /** @brief 软件次版本号。 */
  std::uint8_t software_version_minor = 0U;
  /** @brief 软件可选标志位。 */
  std::uint8_t software_optional_flags = 0U;
  /** @brief 软件版本控制提交号。 */
  std::uint32_t software_vcs_commit = 0U;
  /** @brief 软件镜像 CRC。 */
  std::uint64_t software_image_crc = 0U;
  /** @brief 硬件主版本号。 */
  std::uint8_t hardware_version_major = 0U;
  /** @brief 硬件次版本号。 */
  std::uint8_t hardware_version_minor = 0U;
  /** @brief 唯一硬件 ID。 */
  std::array<std::uint8_t, 16> unique_id{};
  /** @brief DroneCAN 节点名称，最长 80 字节。 */
  char name[81] = "org.libxr.dronecan";
};

/**
 * @brief 一条已完成 DroneCAN 传输的元数据。
 */
struct TransferMetadata
{
  /** @brief 该传输的接收时间戳，单位微秒。 */
  std::uint64_t timestamp_us = 0U;
  /** @brief 数据类型 ID。 */
  std::uint16_t data_type_id = 0U;
  /** @brief 传输类别。 */
  TransferKind transfer_kind = TransferKind::Message;
  /** @brief 传输 ID。 */
  std::uint8_t transfer_id = 0U;
  /** @brief 优先级。 */
  std::uint8_t priority = 0U;
  /** @brief 源节点 ID。 */
  std::uint8_t source_node_id = 0U;
  /** @brief 有效负载大小，单位字节。 */
  std::size_t payload_size = 0U;
};

/** @brief DroneCAN 传输处理器签名。 */
using TransferHandler = Callback<const TransferMetadata&, ConstRawData>;

/**
 * @brief libcanard 内存池使用统计。
 */
struct PoolStatistics
{
  /** @brief 总块数。 */
  std::uint16_t capacity_blocks = 0U;
  /** @brief 当前占用块数。 */
  std::uint16_t current_usage_blocks = 0U;
  /** @brief 峰值占用块数。 */
  std::uint16_t peak_usage_blocks = 0U;
};

/**
 * @brief 节点轮询与协议调度配置。
 */
struct Config
{
  /** @brief 接收队列深度。 */
  std::size_t rx_queue_size = 16U;
  /** @brief `NodeStatus` 发布周期，单位微秒。 */
  std::uint64_t node_status_period_us = 1000000ULL;
  /** @brief stale transfer 清理周期，单位微秒。 */
  std::uint64_t cleanup_interval_us = 1000000ULL;
};

/** @brief `uavcan.protocol.NodeStatus` 的数据类型 ID。 */
inline constexpr std::uint16_t NODE_STATUS_DATA_TYPE_ID = 341U;
/** @brief `uavcan.protocol.NodeStatus` 的数据类型签名。 */
inline constexpr std::uint64_t NODE_STATUS_DATA_TYPE_SIGNATURE = 0x0F0868D0C1A7C6F1ULL;
/** @brief `uavcan.protocol.NodeStatus` 编码后的固定字节数。 */
inline constexpr std::size_t NODE_STATUS_MESSAGE_SIZE = 7U;

/** @brief `uavcan.protocol.GetNodeInfo` 的服务 ID。 */
inline constexpr std::uint16_t GET_NODE_INFO_DATA_TYPE_ID = 1U;
/** @brief `uavcan.protocol.GetNodeInfo` 的数据类型签名。 */
inline constexpr std::uint64_t GET_NODE_INFO_DATA_TYPE_SIGNATURE = 0xEE468A8121C46A9EULL;
/** @brief `GetNodeInfo` 响应允许的最大负载字节数。 */
inline constexpr std::size_t GET_NODE_INFO_RESPONSE_MAX_SIZE = ((3015U + 7U) / 8U);

/** @brief `uavcan.protocol.dynamic_node_id.Allocation` 的数据类型 ID。 */
inline constexpr std::uint16_t DYNAMIC_NODE_ID_ALLOCATION_DATA_TYPE_ID = 1U;
/** @brief `uavcan.protocol.dynamic_node_id.Allocation` 的数据类型签名。 */
inline constexpr std::uint64_t DYNAMIC_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE =
    0x0B2A812620A11D40ULL;

/** @brief 软件版本字段中的 VCS 提交标志位。 */
inline constexpr std::uint8_t SOFTWARE_VERSION_FLAG_VCS_COMMIT = 0x01U;
/** @brief 软件版本字段中的镜像 CRC 标志位。 */
inline constexpr std::uint8_t SOFTWARE_VERSION_FLAG_IMAGE_CRC = 0x02U;

/** @brief 节点名称最大长度，不含结尾零。 */
inline constexpr std::size_t MAX_NODE_NAME_LENGTH = 80U;
/** @brief 允许注册的传输处理器数量上限。 */
inline constexpr std::size_t MAX_TRANSFER_HANDLERS = 8U;
/** @brief 传输 ID 槽位数量上限。 */
inline constexpr std::size_t MAX_TRANSFER_ID_SLOTS = 16U;

}  // namespace LibXR::DroneCAN

