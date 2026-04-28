#pragma once

#include "dronecan_core/DroneCANNode.hpp"

namespace DroneCANCoreSupport
{

class DroneCANHost
{
 public:
  virtual ~DroneCANHost() = default;

  virtual DroneCANNode& Node() noexcept = 0;
};

}  // namespace DroneCANCoreSupport

