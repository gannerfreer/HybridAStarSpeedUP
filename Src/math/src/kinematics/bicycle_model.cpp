/**
 * @file bicycle_model.cpp
 * @brief 后轴中心自行车模型运动学换算实现
 */

#include "planner/math/kinematics/bicycle_model.hpp"

#include <cmath>

namespace planner::math::kinematics {

double curvature_from_steer(double steer_rad, double wheelbase_m) noexcept {
  return std::tan(steer_rad) / wheelbase_m;
}

double steer_from_curvature(double curvature_per_m, double wheelbase_m) noexcept {
  return std::atan(curvature_per_m * wheelbase_m);
}

double min_turn_radius_m(double wheelbase_m, double max_steer_rad) noexcept {
  return wheelbase_m / std::tan(max_steer_rad);
}

}  // namespace planner::math::kinematics
