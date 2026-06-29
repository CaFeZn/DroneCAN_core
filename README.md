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

## 仓库文件约定 / Repository File Policy

### 必需文件 / Required

- `CMakeLists.txt`: 把 runtime 源码和 libcanard 加入 `xr` 目标。
- `dronecan_core.hpp`: 模块入口，包含 `MODULE MANIFEST V2` 和聚合 include。
- `include/dronecan_core/`: 对外头文件。
- `src/`: runtime 源码。
- `third_party/canard.*`: 内置 libcanard 依赖。

### 可选文件 / Optional

- `README.md`: 模块说明。
- `info.cmake`: 给外部同步/索引工具看的简短说明；当前 CMake 构建不依赖它。

### 不应放入模块仓库 / Not Stored In This Module Repo

- `module.yaml`: 项目实例配置不属于 runtime 模块仓库；依赖和构造信息以
  `dronecan_core.hpp` 的 `MODULE MANIFEST V2` 为准。
- `User/xrobot.yaml`: 这是主工程配置文件，只应存在于使用该模块的项目里。
