#pragma once
#include "types.hpp"
namespace planner::math {
// 有限输入归一化到 [-pi, pi)，非有限输入返回 NaN。
double normalize_angle_rad(double angle_rad);
}
