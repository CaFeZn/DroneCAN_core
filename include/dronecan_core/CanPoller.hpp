#pragma once

namespace DroneCANCoreSupport
{

class CanPoller
{
 public:
  virtual ~CanPoller() = default;
  virtual void Poll() = 0;
};

}  // namespace DroneCANCoreSupport

