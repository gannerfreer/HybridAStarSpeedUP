/**
 * @file vehicle.cpp
 * @brief 车辆模型合法性校验与相等性比较操作符的具体实现。
 */

#include "planner/common/vehicle.hpp"

#include <cmath>
#include <string>

#include "planner/math/angle.hpp"

namespace planner::common {

namespace {

bool finite_nonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

bool vehicle_valid(const VehicleModel& vehicle, std::string* reason) {
  if (!std::isfinite(vehicle.wheelbase_m) || !(vehicle.wheelbase_m > 0.0)) {
    if (reason) *reason = "vehicle wheelbase must be finite and positive";
    return false;
  }
  if (!std::isfinite(vehicle.width_m) || !(vehicle.width_m > 0.0)) {
    if (reason) *reason = "vehicle width must be finite and positive";
    return false;
  }
  if (!finite_nonnegative(vehicle.front_overhang_m)) {
    if (reason) *reason = "vehicle front overhang must be finite and >= 0";
    return false;
  }
  if (!finite_nonnegative(vehicle.rear_overhang_m)) {
    if (reason) *reason = "vehicle rear overhang must be finite and >= 0";
    return false;
  }
  if (!finite_nonnegative(vehicle.safety_margin_m)) {
    if (reason) *reason = "vehicle safety margin must be finite and >= 0";
    return false;
  }
  if (!std::isfinite(vehicle.max_steer_rad) || !(vehicle.max_steer_rad > 0.0) ||
      !(vehicle.max_steer_rad < 0.5 * math::kPi)) {
    if (reason) {
      *reason = "vehicle max steer must be finite and in (0, pi/2)";
    }
    return false;
  }
  return true;
}

bool operator==(const VehicleModel& lhs, const VehicleModel& rhs) {
  return lhs.id == rhs.id && lhs.wheelbase_m == rhs.wheelbase_m && lhs.width_m == rhs.width_m &&
         lhs.front_overhang_m == rhs.front_overhang_m && lhs.rear_overhang_m == rhs.rear_overhang_m &&
         lhs.max_steer_rad == rhs.max_steer_rad && lhs.safety_margin_m == rhs.safety_margin_m;
}

}  // namespace planner::common
