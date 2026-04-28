#pragma once

#include <cstddef>
#include <cstdint>

#include "can.hpp"
#include "dronecan_core/dronecan_types.hpp"
#include "timebase.hpp"

/**
 * @file DroneCANNode.hpp
 * @brief XRobot `DroneCANCore` 模块内聚的 DroneCAN 节点协议栈声明。
 *
 * @details
 * 该实现从 `libxr` 中抽离到当前模块，目的是把 XRobot 项目需要的
 * DroneCAN 行为固定在模块内部，避免继续依赖 `libxr` 子仓上的项目级补丁。
 *
 * 与 `libxr` 基础实现相比，这个模块内版本额外固化了两类行为：
 * - `GetNodeInfo` 响应按当前联调验证通过的布局序列化
 * - 轮询调度对异常周期值和超期时间戳做了更稳健的收敛处理
 */

namespace DroneCANCoreSupport
{

/**
 * @brief `DroneCANCore` 使用的本地 DroneCAN 节点封装。
 *
 * @details
 * 该类负责：
 * - 订阅底层扩展帧 CAN 数据
 * - 管理 libcanard 实例、接收队列和发送队列
 * - 周期发布 `NodeStatus`
 * - 自动响应 `GetNodeInfo`
 * - 向应用层暴露广播、请求、响应和传输处理器注册接口
 *
 * 它保留了与原 `LibXR::DroneCANNode` 基本一致的调用方式，
 * 以便 `DroneCANCore` 迁移后仍能保持原有业务代码结构。
 */
class DroneCANNode
{
 public:
  /**
   * @brief 构造一个本地 DroneCAN 节点实例。
   * @param can 与节点绑定的底层 CAN 驱动。
   * @param timebase 系统时基对象。
   * @param arena 供 libcanard 使用的静态内存池首地址。
   * @param arena_size `arena` 可用字节数。
   * @param config DroneCAN 运行配置。
   *
   * @details
   * 构造后对象会立即：
   * - 在 `can` 上注册扩展帧回调
   * - 初始化 libcanard 实例
   * - 设定默认节点信息、状态与周期调度起点
   *
   * 调用方必须保证：
   * - `can`、`timebase` 生命周期覆盖整个节点对象
   * - `arena` 指向长期有效且可写的静态缓冲
   */
  explicit DroneCANNode(
      LibXR::CAN& can,
      LibXR::Timebase& timebase,
      void* arena,
      std::size_t arena_size,
      const LibXR::DroneCAN::Config& config = LibXR::DroneCAN::Config{});

  /**
   * @brief 析构节点并释放内部实现对象。
   */
  ~DroneCANNode();

  /** @brief 禁止拷贝构造，避免重复持有底层回调和协议状态。 */
  DroneCANNode(const DroneCANNode&) = delete;
  /** @brief 禁止拷贝赋值，避免重复持有底层回调和协议状态。 */
  DroneCANNode& operator=(const DroneCANNode&) = delete;
  /** @brief 禁止移动构造，避免打断已注册到 CAN 的回调绑定。 */
  DroneCANNode(DroneCANNode&&) = delete;
  /** @brief 禁止移动赋值，避免打断已注册到 CAN 的回调绑定。 */
  DroneCANNode& operator=(DroneCANNode&&) = delete;

  /**
   * @brief 设置本地 DroneCAN 节点 ID。
   * @param node_id 待设置的静态节点 ID。
   * @return `ErrorCode::OK` 表示设置成功；若超出 DroneCAN 允许范围则返回参数错误。
   */
  LibXR::ErrorCode SetNodeID(std::uint8_t node_id);

  /**
   * @brief 读取当前本地节点 ID。
   * @return 当前 libcanard 实例持有的本地节点 ID。
   */
  std::uint8_t GetNodeID() const;

  /**
   * @brief 推进一次节点内部轮询。
   *
   * @details
   * 本函数会依次完成：
   * - 消费接收队列中的 CAN 帧并交给 libcanard
   * - 按周期发送 `NodeStatus`
   * - 定期清理过期传输状态
   * - 冲刷 libcanard 待发送队列到下层 CAN
   *
   * 该接口需要由应用主循环周期调用。
   */
  void Poll();

  /**
   * @brief 直接向节点注入一帧底层 CAN 数据。
   * @param frame 待处理的经典 CAN 帧。
   * @param timestamp_us 接收时间戳，单位微秒。
   *
   * @details
   * 常规路径下节点通过已注册的 CAN 回调接收帧；
   * 该接口保留为显式注入入口，便于测试或特殊桥接场景复用。
   */
  void HandleFrame(const LibXR::CAN::ClassicPack& frame, std::uint64_t timestamp_us);

  /**
   * @brief 广播一条 DroneCAN 消息传输。
   * @param data_type_id 数据类型 ID。
   * @param data_type_signature 数据类型签名。
   * @param priority 传输优先级。
   * @param payload 原始负载。
   * @return 发送入队结果。
   */
  LibXR::ErrorCode Broadcast(std::uint16_t data_type_id,
                             std::uint64_t data_type_signature,
                             std::uint8_t priority,
                             LibXR::ConstRawData payload);

  /**
   * @brief 发起一条 DroneCAN 请求传输。
   * @param destination_node_id 目标节点 ID。
   * @param data_type_id 数据类型 ID。
   * @param data_type_signature 数据类型签名。
   * @param priority 传输优先级。
   * @param payload 原始负载。
   * @return 发送入队结果。
   */
  LibXR::ErrorCode Request(std::uint8_t destination_node_id,
                           std::uint16_t data_type_id,
                           std::uint64_t data_type_signature,
                           std::uint8_t priority,
                           LibXR::ConstRawData payload);

  /**
   * @brief 发送一条 DroneCAN 响应传输。
   * @param destination_node_id 目标节点 ID。
   * @param data_type_id 数据类型 ID。
   * @param data_type_signature 数据类型签名。
   * @param transfer_id 需要复用的传输 ID。
   * @param priority 传输优先级。
   * @param payload 原始负载。
   * @return 发送入队结果。
   */
  LibXR::ErrorCode Respond(std::uint8_t destination_node_id,
                           std::uint16_t data_type_id,
                           std::uint64_t data_type_signature,
                           std::uint8_t transfer_id,
                           std::uint8_t priority,
                           LibXR::ConstRawData payload);

  /**
   * @brief 设置节点信息对象。
   * @param info 新的节点信息。
   *
   * @details
   * 设置后会被后续 `GetNodeInfo` 请求响应使用。
   */
  void SetNodeInfo(const LibXR::DroneCAN::NodeInfo& info);

  /**
   * @brief 设置节点运行模式。
   * @param mode 新的节点模式。
   */
  void SetNodeStatusMode(LibXR::DroneCAN::NodeMode mode);

  /**
   * @brief 设置节点健康状态。
   * @param health 新的健康状态。
   */
  void SetNodeStatusHealth(LibXR::DroneCAN::NodeHealth health);

  /**
   * @brief 设置厂商自定义状态码。
   * @param code 新的 16 位状态码。
   */
  void SetVendorSpecificStatusCode(std::uint16_t code);

  /**
   * @brief 设置节点子模式。
   * @param sub_mode 新的 3-bit 子模式值。
   */
  void SetSubMode(std::uint8_t sub_mode);

  /**
   * @brief 注册某个数据类型的接收处理器。
   * @param kind 传输种类。
   * @param data_type_id 数据类型 ID。
   * @param data_type_signature 数据类型签名。
   * @param handler 接收回调处理器。
   * @return 注册结果；若处理器表已满则返回 `ErrorCode::FULL`。
   */
  LibXR::ErrorCode RegisterTransferHandler(LibXR::DroneCAN::TransferKind kind,
                                           std::uint16_t data_type_id,
                                           std::uint64_t data_type_signature,
                                           const LibXR::DroneCAN::TransferHandler& handler);

  /**
   * @brief 查询当前协议栈内存池统计信息。
   * @return libcanard 分配器容量、当前占用与峰值占用。
   */
  LibXR::DroneCAN::PoolStatistics GetPoolStatistics() const;

  /**
   * @brief 查询已进入协议栈的底层 CAN 帧数量。
   */
  std::uint32_t GetRxFrameCount() const;

  /**
   * @brief 查询接收队列溢出丢帧数量。
   */
  std::uint32_t GetRxDropCount() const;

  /**
   * @brief 查询完成重组的 DroneCAN 传输数量。
   */
  std::uint32_t GetRxTransferCount() const;

 private:
  /**
   * @brief 隐藏具体协议实现和 libcanard 细节。
   */
  struct Impl;

  /** @brief 指向实际协议实现对象的裸指针。 */
  Impl* impl_ = nullptr;
};

}  // namespace DroneCANCoreSupport

