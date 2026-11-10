/**
 * @file vehicle.hpp
 * @brief 车辆矩形包络几何尺寸与运动学模型参数定义及校验。
 */

#pragma once

#include <string>

#include "planner/math/kinematics/bicycle_model.hpp"

namespace planner::common {

/**
 * @brief 矩形包络车辆的几何尺寸与运动学模型参数。
 * @details 最小转弯半径由轴距与最大前轮转角推导得出。
 */
struct VehicleModel {
  std::string id;               ///< 车辆模型唯一标识。
  double wheelbase_m{0.0};       ///< 轴距（前后轴中心距，米）。
  double width_m{0.0};           ///< 车身总宽度（米）。
  double front_overhang_m{0.0};  ///< 前悬长度（前轴至前保险杠外缘距离，米）。
  double rear_overhang_m{0.0};   ///< 后悬长度（后轴至后保险杠外缘距离，米）。
  double max_steer_rad{0.0};     ///< 最大前轮转向角（弧度）。
  double safety_margin_m{0.0};   ///< 车体外围额外扩充的安全裕量（米）。

  /**
   * @brief 计算车辆后轴中心处的最小转弯半径。
   * @return 后轴中心最小转弯半径（米）：wheelbase / tan(max_steer)。
   */
  double min_turn_radius_m() const {
    return math::kinematics::min_turn_radius_m(wheelbase_m, max_steer_rad);
  }
};

/**
 * @brief 校验车辆模型几何尺寸与运动学参数（轴距、车宽、前后悬、安全裕量、最大前轮转角）的合法性。
 * @details 轴距与车宽须为有限正数；前后悬与安全裕量须为有限非负数；
 *          最大前轮转角须为有限值且位于开区间 (0, pi/2) 内。
 * @param[in] vehicle 车辆模型参数。
 * @param[out] reason 失败时的诊断说明文本指针，可为 nullptr。
 * @return 若所有尺寸与转角参数均为有限合法值则返回 true。
 */
bool vehicle_valid(const VehicleModel& vehicle, std::string* reason = nullptr);

/**
 * @name 精确全等比较操作符
 * @brief 规划核心数据结构的值相等性判断（用于缓存校验、回放对比与结果校验）。
 * @{
 */
bool operator==(const VehicleModel& lhs, const VehicleModel& rhs);
/// @}

}  // namespace planner::common
