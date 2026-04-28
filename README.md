# dronecan_core

## 中文

`dronecan_core` 是 XRobot 中 DroneCAN 的宿主/运行时入口模块。

- 负责节点生命周期、CAN 收发、transfer 解析、handler 注册与共享节点状态
- 负责自动挂接其余 `_` 风格 DroneCAN 能力模块
- 是 XRobot 配置层唯一需要实例化的 DroneCAN 模块

默认行为：

- 默认启用 `dronecan_heartbeat`
- 可选启用 `dronecan_dynamic_node_id`
- 默认启用 `dronecan_esc_raw_command`
- `dronecan_esc_status` 存在，但 idle status 默认不主动发布

拆仓边界：

- 该模块应作为 DroneCAN 主宿主仓保留
- 其他能力模块应只依赖它的公开宿主/节点接口

### 在 XRobot 中怎么用

`dronecan_core` 是唯一需要在 `User/xrobot.yaml` 里显式实例化的 DroneCAN 模块。
其他能力模块不会单独实例化，而是由宿主自动挂接。

为兼容 XRobot 官方 `xrobot_gen_main` 的原生生成规则，模块根目录额外提供：

- `Modules/dronecan_core/dronecan_core.hpp`

该入口头会把官方生成器期望的 `dronecan_core` 类型映射到当前实现类 `DroneCANCore`，因此可以直接使用官方生成流程，而不依赖仓库私有命名修正。

示例：

```yaml
modules:
  - id: dronecan_core
    name: dronecan_core
    constructor_args:
      node_id:
        constexpr: DroneCANNodeId
      heartbeat_period_ms:
        constexpr: DroneCANHeartbeatPeriodMs
      node_status_period_ms:
        constexpr: DroneCANNodeStatusPeriodMs
      can_alias: can0
      timebase_alias: timebase
      node_name: org.libxr.spc1185.dronecan
      esc_count:
        constexpr: DroneCANEscCount
      can_poller_alias: can0_poller
      publish_idle_esc_status:
        constexpr: DroneCANPublishIdleEscStatus
      enable_dynamic_node_id:
        constexpr: DroneCANEnableDynamicNodeId
      preferred_node_id:
        constexpr: DroneCANPreferredNodeId
```

### 参数说明

- `node_id`
  - 静态节点 ID
  - 当 `enable_dynamic_node_id=true` 时，启动阶段可为 `0`
- `heartbeat_period_ms`
  - 心跳调度周期，单位毫秒
- `node_status_period_ms`
  - 节点状态发布周期，单位毫秒
- `can_alias`
  - 绑定的 `LibXR::CAN` 设备别名
- `timebase_alias`
  - 绑定的 `LibXR::Timebase` 设备别名
- `node_name`
  - DroneCAN 节点名，用于 `GetNodeInfo`
- `esc_count`
  - ESC 通道数
- `can_poller_alias`
  - 可选的 CAN 轮询器别名；在需要主动轮询收包的平台上使用
- `publish_idle_esc_status`
  - 是否在空闲时也持续发布 ESC status
- `enable_dynamic_node_id`
  - 是否启用动态节点 ID 分配客户端
- `preferred_node_id`
  - 动态分配开启时的首选节点 ID；`0` 表示无偏好

### 自动挂接关系

- `dronecan_heartbeat`
  - 默认启用
- `dronecan_dynamic_node_id`
  - 由 `enable_dynamic_node_id` 决定是否挂接
- `dronecan_esc_raw_command`
  - 默认启用
- `dronecan_esc_status`
  - 默认挂接，但是否发布 idle status 由 `publish_idle_esc_status` 控制

## English

`dronecan_core` is the host/runtime entry module for DroneCAN in XRobot.

- Owns node lifecycle, CAN transport, transfer parsing, handler registration, and shared node state
- Auto-attaches the other `_`-style DroneCAN capability modules
- Is the only DroneCAN module that should be instantiated from XRobot configuration

Default behavior:

- `dronecan_heartbeat` enabled by default
- `dronecan_dynamic_node_id` optional
- `dronecan_esc_raw_command` enabled by default
- `dronecan_esc_status` is present, but idle-status publishing stays disabled unless configured

Repository split boundary:

- This module should remain the primary DroneCAN host repository
- Other capability modules should depend only on its public host/node interfaces

### How To Use In XRobot

`dronecan_core` is the only DroneCAN module that should be instantiated explicitly in `User/xrobot.yaml`.
The other capability modules are not instantiated separately; they are auto-attached by the host.

To stay compatible with the native XRobot `xrobot_gen_main` generation rule, the module root also provides:

- `Modules/dronecan_core/dronecan_core.hpp`

This compatibility entry maps the generator-expected `dronecan_core` type to the current implementation class `DroneCANCore`, allowing the official generation flow to work directly without repository-specific naming rewrites.

Example:

```yaml
modules:
  - id: dronecan_core
    name: dronecan_core
    constructor_args:
      node_id:
        constexpr: DroneCANNodeId
      heartbeat_period_ms:
        constexpr: DroneCANHeartbeatPeriodMs
      node_status_period_ms:
        constexpr: DroneCANNodeStatusPeriodMs
      can_alias: can0
      timebase_alias: timebase
      node_name: org.libxr.spc1185.dronecan
      esc_count:
        constexpr: DroneCANEscCount
      can_poller_alias: can0_poller
      publish_idle_esc_status:
        constexpr: DroneCANPublishIdleEscStatus
      enable_dynamic_node_id:
        constexpr: DroneCANEnableDynamicNodeId
      preferred_node_id:
        constexpr: DroneCANPreferredNodeId
```

### Constructor Arguments

- `node_id`
  - Static node ID
  - May be `0` during startup when `enable_dynamic_node_id=true`
- `heartbeat_period_ms`
  - Heartbeat scheduling period in milliseconds
- `node_status_period_ms`
  - Node-status publish period in milliseconds
- `can_alias`
  - Alias of the bound `LibXR::CAN` device
- `timebase_alias`
  - Alias of the bound `LibXR::Timebase` device
- `node_name`
  - DroneCAN node name used by `GetNodeInfo`
- `esc_count`
  - Number of ESC channels
- `can_poller_alias`
  - Optional CAN poller alias for platforms that need explicit RX polling
- `publish_idle_esc_status`
  - Whether idle ESC status should still be published continuously
- `enable_dynamic_node_id`
  - Enables the dynamic node-ID allocation client
- `preferred_node_id`
  - Preferred node ID for dynamic allocation; `0` means no preference

### Auto-Attached Capabilities

- `dronecan_heartbeat`
  - Enabled by default
- `dronecan_dynamic_node_id`
  - Attached only when `enable_dynamic_node_id` is enabled
- `dronecan_esc_raw_command`
  - Enabled by default
- `dronecan_esc_status`
  - Attached by default, while idle-status publishing is controlled by `publish_idle_esc_status`
