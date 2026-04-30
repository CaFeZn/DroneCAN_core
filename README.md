# dronecan_core

`dronecan_core` is the shared DroneCAN runtime used by generated DSDL modules.

It provides:

- `DroneCANCoreSupport::DroneCANNode`
- `LibXR::DroneCAN` base types and transfer metadata
- CAN frame conversion helpers
- bundled libcanard sources
- optional `CanPoller` interface for platform code

It no longer owns the XRobot application entry or feature modules on the mainline.
The active application facade is generated in `Modules/dronecan_dsdl`.
