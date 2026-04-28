#pragma once

#include <cstdint>

namespace DroneCANCoreSupport
{

class DroneCANFeature
{
 public:
  virtual ~DroneCANFeature() = default;
  virtual void OnStart() {}
  virtual void OnPoll(std::uint32_t now_ms, std::uint64_t now_us) = 0;
};

}  // namespace DroneCANCoreSupport

