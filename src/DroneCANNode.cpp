#include "dronecan_core/DroneCANNode.hpp"

#include <array>
#include <cstring>

#include "dronecan_core/dronecan_frame.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "lockfree_queue.hpp"

extern "C"
{
#include "canard.h"
}

using LibXR::ErrorCode;

namespace
{
using LibXR::DroneCAN::Config;
using LibXR::DroneCAN::GET_NODE_INFO_DATA_TYPE_ID;
using LibXR::DroneCAN::GET_NODE_INFO_DATA_TYPE_SIGNATURE;
using LibXR::DroneCAN::GET_NODE_INFO_RESPONSE_MAX_SIZE;
using LibXR::DroneCAN::MAX_NODE_NAME_LENGTH;
using LibXR::DroneCAN::MAX_TRANSFER_HANDLERS;
using LibXR::DroneCAN::MAX_TRANSFER_ID_SLOTS;
using LibXR::DroneCAN::NODE_STATUS_DATA_TYPE_ID;
using LibXR::DroneCAN::NODE_STATUS_DATA_TYPE_SIGNATURE;
using LibXR::DroneCAN::NODE_STATUS_MESSAGE_SIZE;
using LibXR::DroneCAN::NodeHealth;
using LibXR::DroneCAN::NodeInfo;
using LibXR::DroneCAN::NodeMode;
using LibXR::DroneCAN::PoolStatistics;
using LibXR::DroneCAN::TransferHandler;
using LibXR::DroneCAN::TransferKind;
using LibXR::DroneCAN::TransferMetadata;

/**
 * @brief 把 XRobot 抽象层传输类型映射为 libcanard 类型。
 */
static CanardTransferType ToCanardTransferType(TransferKind kind)
{
  switch (kind)
  {
    case TransferKind::Request:
      return CanardTransferTypeRequest;
    case TransferKind::Response:
      return CanardTransferTypeResponse;
    case TransferKind::Message:
    default:
      return CanardTransferTypeBroadcast;
  }
}

/**
 * @brief 把 libcanard 传输类型映射为 XRobot 抽象层类型。
 */
static TransferKind FromCanardTransferType(CanardTransferType kind)
{
  switch (kind)
  {
    case CanardTransferTypeRequest:
      return TransferKind::Request;
    case CanardTransferTypeResponse:
      return TransferKind::Response;
    case CanardTransferTypeBroadcast:
    default:
      return TransferKind::Message;
  }
}

/**
 * @brief 把 libcanard 返回码转换为 LibXR 错误码。
 */
static LibXR::ErrorCode ToErrorCode(const int16_t result)
{
  if (result > 0)
  {
    return ErrorCode::OK;
  }

  switch (-result)
  {
    case CANARD_ERROR_INVALID_ARGUMENT:
      return ErrorCode::ARG_ERR;
    case CANARD_ERROR_OUT_OF_MEMORY:
      return ErrorCode::NO_MEM;
    case CANARD_ERROR_NODE_ID_NOT_SET:
      return ErrorCode::STATE_ERR;
    case CANARD_ERROR_RX_NOT_WANTED:
      return ErrorCode::NOT_FOUND;
    default:
      return ErrorCode::FAILED;
  }
}

}  // namespace

/**
 * @brief 本地 DroneCAN 节点的实际实现体。
 */
struct DroneCANCoreSupport::DroneCANNode::Impl
{
  /**
   * @brief 带时间戳的待处理 CAN 帧。
   */
  struct StampedFrame
  {
    /** @brief 原始经典 CAN 帧。 */
    LibXR::CAN::ClassicPack frame{};
    /** @brief 对应接收时间戳，单位微秒。 */
    std::uint64_t timestamp_us = 0U;
  };

  /**
   * @brief 传输 ID 分配槽位。
   */
  struct TransferIdSlot
  {
    /** @brief 当前槽位是否已经被占用。 */
    bool used = false;
    /** @brief 槽位对应的传输类型。 */
    TransferKind kind = TransferKind::Message;
    /** @brief 槽位绑定的数据类型 ID。 */
    std::uint16_t data_type_id = 0U;
    /** @brief 槽位绑定的远端节点 ID，请求类传输使用。 */
    std::uint8_t remote_node_id = 0U;
    /** @brief 当前传输 ID 计数值。 */
    std::uint8_t transfer_id = 0U;
  };

  /**
   * @brief 应用层处理器注册表项。
   */
  struct HandlerEntry
  {
    /** @brief 当前表项是否有效。 */
    bool used = false;
    /** @brief 处理器匹配的传输类型。 */
    TransferKind kind = TransferKind::Message;
    /** @brief 处理器匹配的数据类型 ID。 */
    std::uint16_t data_type_id = 0U;
    /** @brief 处理器匹配的数据类型签名。 */
    std::uint64_t data_type_signature = 0U;
    /** @brief 应用层回调对象。 */
    TransferHandler handler;
  };

  /**
   * @brief 构造协议实现体并完成底层回调注册。
   */
  Impl(LibXR::CAN& can, LibXR::Timebase& timebase, void* arena_ptr, std::size_t arena_len,
       const Config& cfg)
      : can(can),
        timebase(timebase),
        arena(arena_ptr),
        arena_size(arena_len),
        config(cfg),
        rx_queue(cfg.rx_queue_size)
  {
    ASSERT(arena != nullptr);
    ASSERT(arena_size >= CANARD_MEM_BLOCK_SIZE);

    rx_callback = LibXR::CAN::Callback::Create(OnCanFrameStatic, this);
    can.Register(rx_callback, LibXR::CAN::Type::EXTENDED);

    canardInit(&instance, arena, arena_size, OnTransferReceptionStatic,
               ShouldAcceptTransferStatic, this);

    started_at_us = NowUs();
    next_node_status_us = started_at_us + config.node_status_period_us;
    next_cleanup_us = started_at_us + config.cleanup_interval_us;
    std::strncpy(node_info.name, "org.libxr.dronecan", sizeof(node_info.name) - 1U);
    node_status_mode = NodeMode::INITIALIZATION;
    node_status_health = NodeHealth::OK;
  }

  /**
   * @brief 获取当前系统时间，单位微秒。
   */
  std::uint64_t NowUs() const
  {
    (void)timebase;
    return static_cast<std::uint64_t>(LibXR::Timebase::GetMicroseconds());
  }

  /**
   * @brief 推进一次协议栈接收、定时任务与发送冲刷。
   */
  void Poll()
  {
    StampedFrame entry{};
    while (rx_queue.Pop(entry) == ErrorCode::OK)
    {
      ProcessFrame(entry.frame, entry.timestamp_us);
    }

    const std::uint64_t now_us = NowUs();
    const std::uint64_t status_period_us =
        (config.node_status_period_us == 0U) ? 1U : config.node_status_period_us;
    const std::uint64_t cleanup_period_us =
        (config.cleanup_interval_us == 0U) ? 1U : config.cleanup_interval_us;

    if (next_node_status_us > (now_us + (status_period_us * 2U)))
    {
      next_node_status_us = now_us + status_period_us;
    }
    if (next_cleanup_us > (now_us + (cleanup_period_us * 2U)))
    {
      next_cleanup_us = now_us + cleanup_period_us;
    }

    if ((GetNodeID() != CANARD_BROADCAST_NODE_ID) && (now_us >= next_node_status_us))
    {
      PublishNodeStatus(now_us);
      do
      {
        next_node_status_us += status_period_us;
      } while (now_us >= next_node_status_us);
    }

    if (now_us >= next_cleanup_us)
    {
      canardCleanupStaleTransfers(&instance, now_us);
      do
      {
        next_cleanup_us += cleanup_period_us;
      } while (now_us >= next_cleanup_us);
    }

    FlushTx();
  }

  /**
   * @brief 处理一帧注入的底层 CAN 数据。
   */
  void HandleFrame(const LibXR::CAN::ClassicPack& frame, std::uint64_t timestamp_us)
  {
    if (frame.type != LibXR::CAN::Type::EXTENDED)
    {
      return;
    }

    ProcessFrame(frame, timestamp_us);
  }

  /**
   * @brief 将底层 CAN 帧转换并喂给 libcanard。
   */
  void ProcessFrame(const LibXR::CAN::ClassicPack& frame, std::uint64_t timestamp_us)
  {
    ++rx_frame_count;
    CanardCANFrame canard_frame{};
    if (LibXR::DroneCANDetail::ToCanardFrame(frame, canard_frame) != ErrorCode::OK)
    {
      return;
    }

    const std::uint16_t data_type_id =
        static_cast<std::uint16_t>((canard_frame.id >> (((canard_frame.id >> 7U) & 0x1U) ? 16U : 8U)) &
                                   (((canard_frame.id >> 7U) & 0x1U) ? 0xFFU : 0xFFFFU));
    if (data_type_id == GET_NODE_INFO_DATA_TYPE_ID)
    {
      LibXR::STDIO::Printf("[DroneCANNode] rx can_id=0x%08lX dtid=%u ts=%llu\r\n",
                           static_cast<unsigned long>(canard_frame.id),
                           static_cast<unsigned>(data_type_id),
                           static_cast<unsigned long long>(timestamp_us));
    }

    (void)canardHandleRxFrame(&instance, &canard_frame, timestamp_us);
  }

  /**
   * @brief 冲刷 libcanard 待发送队列到下层 CAN 驱动。
   */
  void FlushTx()
  {
    for (const CanardCANFrame* txf = canardPeekTxQueue(&instance); txf != nullptr;
         txf = canardPeekTxQueue(&instance))
    {
      LibXR::CAN::ClassicPack pack{};
      if (LibXR::DroneCANDetail::ToClassicPack(*txf, pack) != ErrorCode::OK)
      {
        break;
      }

      const ErrorCode ec = can.AddMessage(pack);
      if (ec == ErrorCode::OK)
      {
        canardPopTxQueue(&instance);
        continue;
      }

      if ((ec == ErrorCode::FULL) || (ec == ErrorCode::BUSY) || (ec == ErrorCode::PENDING))
      {
        break;
      }

      break;
    }
  }

  /**
   * @brief 广播消息。
   */
  LibXR::ErrorCode Broadcast(std::uint16_t data_type_id, std::uint64_t signature,
                             std::uint8_t priority, LibXR::ConstRawData payload)
  {
    if (payload.size_ > CANARD_MAX_TRANSFER_PAYLOAD_LEN)
    {
      return ErrorCode::SIZE_ERR;
    }

    std::uint8_t* transfer_id = AcquireTransferId(TransferKind::Message, data_type_id, 0U);
    if (transfer_id == nullptr)
    {
      return ErrorCode::FULL;
    }

    const int16_t rc = canardBroadcast(&instance, signature, data_type_id, transfer_id, priority,
                                       payload.addr_, static_cast<std::uint16_t>(payload.size_));
    return ToErrorCode(rc);
  }

  /**
   * @brief 发起请求。
   */
  LibXR::ErrorCode Request(std::uint8_t destination_node_id, std::uint16_t data_type_id,
                           std::uint64_t signature, std::uint8_t priority,
                           LibXR::ConstRawData payload)
  {
    if (payload.size_ > CANARD_MAX_TRANSFER_PAYLOAD_LEN)
    {
      return ErrorCode::SIZE_ERR;
    }

    std::uint8_t* transfer_id =
        AcquireTransferId(TransferKind::Request, data_type_id, destination_node_id);
    if (transfer_id == nullptr)
    {
      return ErrorCode::FULL;
    }

    const int16_t rc = canardRequestOrRespond(
        &instance, destination_node_id, signature, data_type_id, transfer_id, priority,
        CanardRequest, payload.addr_, static_cast<std::uint16_t>(payload.size_));
    return ToErrorCode(rc);
  }

  /**
   * @brief 发送响应。
   */
  LibXR::ErrorCode Respond(std::uint8_t destination_node_id, std::uint16_t data_type_id,
                           std::uint64_t signature, std::uint8_t transfer_id,
                           std::uint8_t priority, LibXR::ConstRawData payload)
  {
    if (payload.size_ > CANARD_MAX_TRANSFER_PAYLOAD_LEN)
    {
      return ErrorCode::SIZE_ERR;
    }

    std::uint8_t response_transfer_id = transfer_id;
    const int16_t rc = canardRequestOrRespond(
        &instance, destination_node_id, signature, data_type_id, &response_transfer_id,
        priority, CanardResponse, payload.addr_, static_cast<std::uint16_t>(payload.size_));
    return ToErrorCode(rc);
  }

  /**
   * @brief 设置本地节点 ID。
   */
  LibXR::ErrorCode SetNodeID(std::uint8_t node_id)
  {
    if (node_id == CANARD_BROADCAST_NODE_ID)
    {
      canardForgetLocalNodeID(&instance);
      return ErrorCode::OK;
    }

    if ((node_id < CANARD_MIN_NODE_ID) || (node_id > CANARD_MAX_NODE_ID))
    {
      return ErrorCode::ARG_ERR;
    }

    canardSetLocalNodeID(&instance, node_id);
    return ErrorCode::OK;
  }

  /**
   * @brief 查询本地节点 ID。
   */
  std::uint8_t GetNodeID() const
  {
    return canardGetLocalNodeID(&instance);
  }

  /**
   * @brief 更新节点信息。
   */
  void SetNodeInfo(const NodeInfo& info)
  {
    node_info = info;
    node_info.name[MAX_NODE_NAME_LENGTH] = '\0';
  }

  /**
   * @brief 更新节点运行模式。
   */
  void SetNodeStatusMode(NodeMode mode)
  {
    node_status_mode = mode;
  }

  /**
   * @brief 更新节点健康状态。
   */
  void SetNodeStatusHealth(NodeHealth health)
  {
    node_status_health = health;
  }

  /**
   * @brief 更新厂商状态码。
   */
  void SetVendorSpecificStatusCode(std::uint16_t code)
  {
    vendor_specific_status_code = code;
  }

  /**
   * @brief 更新 3-bit 子模式。
   */
  void SetSubMode(std::uint8_t value)
  {
    node_sub_mode = static_cast<std::uint8_t>(value & 0x07U);
  }

  /**
   * @brief 注册或更新一个传输处理器。
   */
  LibXR::ErrorCode RegisterTransferHandler(TransferKind kind, std::uint16_t data_type_id,
                                           std::uint64_t signature,
                                           const TransferHandler& handler)
  {
    for (auto& entry : handlers)
    {
      if (entry.used && (entry.kind == kind) && (entry.data_type_id == data_type_id))
      {
        entry.data_type_signature = signature;
        entry.handler = handler;
        return ErrorCode::OK;
      }
    }

    for (auto& entry : handlers)
    {
      if (!entry.used)
      {
        entry.used = true;
        entry.kind = kind;
        entry.data_type_id = data_type_id;
        entry.data_type_signature = signature;
        entry.handler = handler;
        return ErrorCode::OK;
      }
    }

    return ErrorCode::FULL;
  }

  /**
   * @brief 读取 libcanard 内存池统计信息。
   */
  PoolStatistics GetPoolStatistics() const
  {
    const CanardPoolAllocatorStatistics stats =
        canardGetPoolAllocatorStatistics(const_cast<CanardInstance*>(&instance));
    return PoolStatistics{stats.capacity_blocks, stats.current_usage_blocks,
                          stats.peak_usage_blocks};
  }

  /**
   * @brief 底层 CAN 回调的静态桥接函数。
   */
  static void OnCanFrameStatic(bool, Impl* self, const LibXR::CAN::ClassicPack& frame)
  {
    const StampedFrame stamped{frame, self->NowUs()};
    if (self->rx_queue.Push(stamped) != ErrorCode::OK)
    {
      ++self->rx_drop_count;
    }
  }

  /**
   * @brief 告知 libcanard 当前是否接受某类传输。
   */
  static bool ShouldAcceptTransferStatic(const CanardInstance* ins,
                                         std::uint64_t* out_data_type_signature,
                                         std::uint16_t data_type_id,
                                         CanardTransferType transfer_type,
                                         std::uint8_t)
  {
    auto* self = static_cast<Impl*>(canardGetUserReference(ins));
    if (self == nullptr)
    {
      return false;
    }

    if ((canardGetLocalNodeID(ins) != CANARD_BROADCAST_NODE_ID) &&
        (transfer_type == CanardTransferTypeRequest) &&
        (data_type_id == GET_NODE_INFO_DATA_TYPE_ID))
    {
      LibXR::STDIO::Printf("[DroneCANNode] accept GetNodeInfo request\r\n");
      *out_data_type_signature = GET_NODE_INFO_DATA_TYPE_SIGNATURE;
      return true;
    }

    for (const auto& entry : self->handlers)
    {
      if (entry.used && (entry.data_type_id == data_type_id) &&
          (ToCanardTransferType(entry.kind) == transfer_type))
      {
        *out_data_type_signature = entry.data_type_signature;
        return true;
      }
    }

    return false;
  }

  /**
   * @brief libcanard 接收完成回调桥接。
   */
  static void OnTransferReceptionStatic(CanardInstance* ins, CanardRxTransfer* transfer)
  {
    auto* self = static_cast<Impl*>(canardGetUserReference(ins));
    if (self == nullptr)
    {
      return;
    }

    self->OnTransferReception(transfer);
  }

  /**
   * @brief 处理一条已经完成重组的 DroneCAN 传输。
   */
  void OnTransferReception(CanardRxTransfer* transfer)
  {
    ++rx_transfer_count;
    if (transfer->data_type_id == GET_NODE_INFO_DATA_TYPE_ID)
    {
      LibXR::STDIO::Printf("[DroneCANNode] transfer dtid=%u type=%u src=%u len=%u tid=%u\r\n",
                           static_cast<unsigned>(transfer->data_type_id),
                           static_cast<unsigned>(transfer->transfer_type),
                           static_cast<unsigned>(transfer->source_node_id),
                           static_cast<unsigned>(transfer->payload_len),
                           static_cast<unsigned>(transfer->transfer_id));
    }

    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == GET_NODE_INFO_DATA_TYPE_ID))
    {
      if (transfer->payload_len == 0U)
      {
        HandleGetNodeInfoRequest(*transfer);
      }
      canardReleaseRxTransferPayload(&instance, transfer);
      return;
    }

    for (const auto& entry : handlers)
    {
      if (entry.used && (entry.data_type_id == transfer->data_type_id) &&
          (entry.kind == FromCanardTransferType(
                             static_cast<CanardTransferType>(transfer->transfer_type))))
      {
        LinearizePayload(*transfer, rx_linearized_payload.data());
        const TransferMetadata meta{transfer->timestamp_usec,
                                    transfer->data_type_id,
                                    FromCanardTransferType(
                                        static_cast<CanardTransferType>(transfer->transfer_type)),
                                    transfer->transfer_id,
                                    transfer->priority,
                                    transfer->source_node_id,
                                    transfer->payload_len};
        canardReleaseRxTransferPayload(&instance, transfer);
        entry.handler.Run(false, meta,
                          LibXR::ConstRawData(rx_linearized_payload.data(), meta.payload_size));
        return;
      }
    }

    canardReleaseRxTransferPayload(&instance, transfer);
  }

  /**
   * @brief 处理标准 `GetNodeInfo` 请求。
   */
  void HandleGetNodeInfoRequest(const CanardRxTransfer& transfer)
  {
    std::array<std::uint8_t, GET_NODE_INFO_RESPONSE_MAX_SIZE> buffer{};
    const std::size_t total_size = EncodeGetNodeInfoResponse(buffer.data(), buffer.size(), NowUs());
    (void)Respond(transfer.source_node_id, GET_NODE_INFO_DATA_TYPE_ID,
                  GET_NODE_INFO_DATA_TYPE_SIGNATURE, transfer.transfer_id, transfer.priority,
                  LibXR::ConstRawData(buffer.data(), total_size));
  }

  /**
   * @brief 发布一次 `NodeStatus` 广播。
   */
  void PublishNodeStatus(std::uint64_t now_us)
  {
    std::uint8_t buffer[NODE_STATUS_MESSAGE_SIZE] = {};
    EncodeNodeStatus(buffer, now_us);
    (void)Broadcast(NODE_STATUS_DATA_TYPE_ID, NODE_STATUS_DATA_TYPE_SIGNATURE,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    LibXR::ConstRawData(buffer, NODE_STATUS_MESSAGE_SIZE));
  }

  /**
   * @brief 编码 `NodeStatus` 负载。
   */
  void EncodeNodeStatus(std::uint8_t buffer[NODE_STATUS_MESSAGE_SIZE], std::uint64_t now_us) const
  {
    std::memset(buffer, 0, NODE_STATUS_MESSAGE_SIZE);

    const std::uint32_t uptime_sec =
        static_cast<std::uint32_t>((now_us - started_at_us) / 1000000ULL);
    std::uint8_t health = static_cast<std::uint8_t>(node_status_health);
    std::uint8_t mode = static_cast<std::uint8_t>(node_status_mode);
    std::uint8_t sub_mode = static_cast<std::uint8_t>(node_sub_mode & 0x07U);
    std::uint16_t vendor = vendor_specific_status_code;

    canardEncodeScalar(buffer, 0U, 32U, &uptime_sec);
    canardEncodeScalar(buffer, 32U, 2U, &health);
    canardEncodeScalar(buffer, 34U, 3U, &mode);
    canardEncodeScalar(buffer, 37U, 3U, &sub_mode);
    canardEncodeScalar(buffer, 40U, 16U, &vendor);
  }

  /**
   * @brief 编码 `GetNodeInfo` 响应负载。
   *
   * @details
   * 这里沿用当前项目已经验证通过的响应布局：
   * 节点名直接放在硬件版本证书长度字段之后，起始字节偏移为 41。
   * 该布局已被当前使用的 DroneCAN GUI / 前门脚本正确识别。
   */
  std::size_t EncodeGetNodeInfoResponse(std::uint8_t* buffer, std::size_t buffer_size,
                                        std::uint64_t now_us) const
  {
    ASSERT(buffer_size >= GET_NODE_INFO_RESPONSE_MAX_SIZE);
    std::memset(buffer, 0, buffer_size);

    EncodeNodeStatus(buffer, now_us);

    buffer[7] = node_info.software_version_major;
    buffer[8] = node_info.software_version_minor;
    buffer[9] = node_info.software_optional_flags;

    std::uint32_t vcs = node_info.software_vcs_commit;
    std::uint64_t image_crc = node_info.software_image_crc;
    canardEncodeScalar(buffer, 80U, 32U, &vcs);
    canardEncodeScalar(buffer, 112U, 64U, &image_crc);

    buffer[22] = node_info.hardware_version_major;
    buffer[23] = node_info.hardware_version_minor;
    std::memcpy(&buffer[24], node_info.unique_id.data(), node_info.unique_id.size());

    const std::size_t name_len = strnlen(node_info.name, MAX_NODE_NAME_LENGTH);
    static constexpr std::size_t kNameOffset = 41U;
    std::memcpy(&buffer[kNameOffset], node_info.name, name_len);
    return kNameOffset + name_len;
  }

  /**
   * @brief 把链式负载线性化到连续字节缓冲。
   */
  void LinearizePayload(const CanardRxTransfer& transfer, std::uint8_t* out) const
  {
    for (std::uint16_t i = 0U; i < transfer.payload_len; ++i)
    {
      std::uint8_t value = 0U;
      (void)canardDecodeScalar(&transfer, static_cast<std::uint32_t>(i) * 8U, 8U, false,
                               &value);
      out[i] = value;
    }
  }

  /**
   * @brief 为某类传输分配或复用传输 ID 槽位。
   */
  std::uint8_t* AcquireTransferId(TransferKind kind, std::uint16_t data_type_id,
                                  std::uint8_t remote_node_id)
  {
    for (auto& slot : transfer_ids)
    {
      if (slot.used && (slot.kind == kind) && (slot.data_type_id == data_type_id) &&
          (slot.remote_node_id == remote_node_id))
      {
        return &slot.transfer_id;
      }
    }

    for (auto& slot : transfer_ids)
    {
      if (!slot.used)
      {
        slot.used = true;
        slot.kind = kind;
        slot.data_type_id = data_type_id;
        slot.remote_node_id = remote_node_id;
        slot.transfer_id = 0U;
        return &slot.transfer_id;
      }
    }

    return nullptr;
  }

  /** @brief 节点所依附的底层 CAN 驱动。 */
  LibXR::CAN& can;
  /** @brief 节点共用的系统时基对象。 */
  LibXR::Timebase& timebase;
  /** @brief libcanard 静态内存池首地址。 */
  void* arena = nullptr;
  /** @brief libcanard 静态内存池大小，单位字节。 */
  std::size_t arena_size = 0U;
  /** @brief 节点运行配置快照。 */
  Config config{};
  /** @brief libcanard 协议实例。 */
  CanardInstance instance{};
  /** @brief 挂到 CAN 扩展帧订阅上的回调对象。 */
  LibXR::CAN::Callback rx_callback;
  /** @brief 中断/回调到主循环之间的接收缓冲队列。 */
  LibXR::LockFreeQueue<StampedFrame> rx_queue;
  /** @brief 不同传输键值的传输 ID 分配表。 */
  std::array<TransferIdSlot, MAX_TRANSFER_ID_SLOTS> transfer_ids{};
  /** @brief 应用层注册的传输处理器表。 */
  std::array<HandlerEntry, MAX_TRANSFER_HANDLERS> handlers{};
  /** @brief 线性化接收负载时复用的临时缓冲。 */
  std::array<std::uint8_t, CANARD_MAX_TRANSFER_PAYLOAD_LEN> rx_linearized_payload{};
  /** @brief 当前节点信息。 */
  NodeInfo node_info{};
  /** @brief 当前节点运行模式。 */
  NodeMode node_status_mode = NodeMode::INITIALIZATION;
  /** @brief 当前节点健康状态。 */
  NodeHealth node_status_health = NodeHealth::OK;
  /** @brief 当前节点 3-bit 子模式。 */
  std::uint8_t node_sub_mode = 0U;
  /** @brief 厂商自定义状态码。 */
  std::uint16_t vendor_specific_status_code = 0U;
  /** @brief 节点启动时间戳，单位微秒。 */
  std::uint64_t started_at_us = 0U;
  /** @brief 下一次 `NodeStatus` 发布时间戳，单位微秒。 */
  std::uint64_t next_node_status_us = 0U;
  /** @brief 下一次 stale transfer 清理时间戳，单位微秒。 */
  std::uint64_t next_cleanup_us = 0U;
  /** @brief 因接收队列满而被丢弃的 CAN 帧计数。 */
  std::uint32_t rx_drop_count = 0U;
  /** @brief 已送入 libcanard 的 CAN 帧计数。 */
  std::uint32_t rx_frame_count = 0U;
  /** @brief 已完成重组的 DroneCAN 传输计数。 */
  std::uint32_t rx_transfer_count = 0U;
};

namespace DroneCANCoreSupport
{

DroneCANNode::DroneCANNode(LibXR::CAN& can, LibXR::Timebase& timebase, void* arena,
                           std::size_t arena_size, const LibXR::DroneCAN::Config& config)
    : impl_(new Impl(can, timebase, arena, arena_size, config))
{
}

DroneCANNode::~DroneCANNode()
{
  delete impl_;
}

LibXR::ErrorCode DroneCANNode::SetNodeID(std::uint8_t node_id)
{
  return impl_->SetNodeID(node_id);
}

std::uint8_t DroneCANNode::GetNodeID() const
{
  return impl_->GetNodeID();
}

void DroneCANNode::Poll()
{
  impl_->Poll();
}

void DroneCANNode::HandleFrame(const LibXR::CAN::ClassicPack& frame, std::uint64_t timestamp_us)
{
  impl_->HandleFrame(frame, timestamp_us);
}

LibXR::ErrorCode DroneCANNode::Broadcast(std::uint16_t data_type_id,
                                         std::uint64_t data_type_signature,
                                         std::uint8_t priority, LibXR::ConstRawData payload)
{
  return impl_->Broadcast(data_type_id, data_type_signature, priority, payload);
}

LibXR::ErrorCode DroneCANNode::Request(std::uint8_t destination_node_id,
                                       std::uint16_t data_type_id,
                                       std::uint64_t data_type_signature,
                                       std::uint8_t priority,
                                       LibXR::ConstRawData payload)
{
  return impl_->Request(destination_node_id, data_type_id, data_type_signature, priority,
                        payload);
}

LibXR::ErrorCode DroneCANNode::Respond(std::uint8_t destination_node_id,
                                       std::uint16_t data_type_id,
                                       std::uint64_t data_type_signature,
                                       std::uint8_t transfer_id,
                                       std::uint8_t priority,
                                       LibXR::ConstRawData payload)
{
  return impl_->Respond(destination_node_id, data_type_id, data_type_signature, transfer_id,
                        priority, payload);
}

void DroneCANNode::SetNodeInfo(const LibXR::DroneCAN::NodeInfo& info)
{
  impl_->SetNodeInfo(info);
}

void DroneCANNode::SetNodeStatusMode(LibXR::DroneCAN::NodeMode mode)
{
  impl_->SetNodeStatusMode(mode);
}

void DroneCANNode::SetNodeStatusHealth(LibXR::DroneCAN::NodeHealth health)
{
  impl_->SetNodeStatusHealth(health);
}

void DroneCANNode::SetVendorSpecificStatusCode(std::uint16_t code)
{
  impl_->SetVendorSpecificStatusCode(code);
}

void DroneCANNode::SetSubMode(std::uint8_t sub_mode)
{
  impl_->SetSubMode(sub_mode);
}

LibXR::ErrorCode DroneCANNode::RegisterTransferHandler(
    LibXR::DroneCAN::TransferKind kind, std::uint16_t data_type_id,
    std::uint64_t data_type_signature,
    const LibXR::DroneCAN::TransferHandler& handler)
{
  return impl_->RegisterTransferHandler(kind, data_type_id, data_type_signature, handler);
}

LibXR::DroneCAN::PoolStatistics DroneCANNode::GetPoolStatistics() const
{
  return impl_->GetPoolStatistics();
}

std::uint32_t DroneCANNode::GetRxFrameCount() const
{
  return impl_->rx_frame_count;
}

std::uint32_t DroneCANNode::GetRxDropCount() const
{
  return impl_->rx_drop_count;
}

std::uint32_t DroneCANNode::GetRxTransferCount() const
{
  return impl_->rx_transfer_count;
}

}  // namespace DroneCANCoreSupport

