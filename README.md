# dronecan_core

`dronecan_core` 是生成式 DroneCAN DSDL 模块共用的运行时模块。

`dronecan_core` is the shared DroneCAN runtime used by generated DSDL modules.

它提供：

It provides:

- `DroneCANCoreSupport::DroneCANNode`
- `LibXR::DroneCAN` base types and transfer metadata
- CAN frame conversion helpers
- bundled libcanard sources
- optional `CanPoller` interface for platform code

它不再持有 XRobot application entry 或旧 feature modules。当前主线的 application facade 由 `dronecan_dsdl` 模块生成并实例化。

It no longer owns the XRobot application entry or feature modules on the mainline.
The active application facade is generated and instantiated by `dronecan_dsdl`.

根级 `dronecan_core.hpp` 只提供 XRobot manifest 和聚合 include，用于模块同步器识别依赖；该 runtime 模块不需要写入 `User/xrobot.yaml`。

The root `dronecan_core.hpp` provides only the XRobot manifest and aggregate
includes so the module synchronizer can resolve dependencies. This runtime
module does not need a `User/xrobot.yaml` instance.
