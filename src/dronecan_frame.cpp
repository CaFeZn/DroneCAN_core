#include "dronecan_core/dronecan_frame.hpp"

#include <cstring>

extern "C"
{
#include "canard.h"
}

using LibXR::ErrorCode;

namespace LibXR::DroneCANDetail
{

LibXR::ErrorCode ToCanardFrame(const LibXR::CAN::ClassicPack& source, CanardCANFrame& destination)
{
  if (source.type != LibXR::CAN::Type::EXTENDED)
  {
    return ErrorCode::ARG_ERR;
  }

  if (source.id > CANARD_CAN_EXT_ID_MASK)
  {
    return ErrorCode::ARG_ERR;
  }

  destination.id = (source.id & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
  if (source.dlc > CANARD_CAN_FRAME_MAX_DATA_LEN)
  {
    return ErrorCode::ARG_ERR;
  }
  destination.data_len = source.dlc;
  destination.iface_id = 0U;
  std::memcpy(destination.data, source.data, source.dlc);
  return ErrorCode::OK;
}

LibXR::ErrorCode ToClassicPack(const CanardCANFrame& source, LibXR::CAN::ClassicPack& destination)
{
  if ((source.id & CANARD_CAN_FRAME_EFF) == 0U)
  {
    return ErrorCode::ARG_ERR;
  }

  if (((source.id & CANARD_CAN_FRAME_ERR) != 0U) || ((source.id & CANARD_CAN_FRAME_RTR) != 0U))
  {
    return ErrorCode::ARG_ERR;
  }

  destination.id = source.id & CANARD_CAN_EXT_ID_MASK;
  destination.type = LibXR::CAN::Type::EXTENDED;
  if (source.data_len > CANARD_CAN_FRAME_MAX_DATA_LEN)
  {
    return ErrorCode::ARG_ERR;
  }
  destination.dlc = source.data_len;
  std::memcpy(destination.data, source.data, source.data_len);
  return ErrorCode::OK;
}

}  // namespace LibXR::DroneCANDetail
