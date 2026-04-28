#pragma once

#include "can.hpp"

extern "C"
{
#include "canard.h"
}

/**
 * @file dronecan_frame.hpp
 * @brief 经典 CAN 帧与 libcanard 帧之间的转换接口。
 */

namespace LibXR::DroneCANDetail
{

/**
 * @brief 把 LibXR 经典 CAN 扩展帧转换为 libcanard 帧。
 * @param source 输入经典 CAN 帧。
 * @param destination 输出 libcanard 帧。
 * @return 转换结果。
 */
LibXR::ErrorCode ToCanardFrame(const LibXR::CAN::ClassicPack& source, CanardCANFrame& destination);

/**
 * @brief 把 libcanard 帧转换为 LibXR 经典 CAN 扩展帧。
 * @param source 输入 libcanard 帧。
 * @param destination 输出经典 CAN 帧。
 * @return 转换结果。
 */
LibXR::ErrorCode ToClassicPack(const CanardCANFrame& source, LibXR::CAN::ClassicPack& destination);

}  // namespace LibXR::DroneCANDetail
