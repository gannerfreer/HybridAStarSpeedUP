/**
 * @file path.cpp
 * @brief 路径方向字符串转换及路径数据结构相等性比较操作符的具体实现。
 */

#include "planner/common/path.hpp"

#include <string_view>

namespace planner::common {

std::string_view to_string(Direction direction) noexcept {
  switch (direction) {
    case Direction::kForward:
      return "forward";
    case Direction::kReverse:
      return "reverse";
  }
  return "unknown";
}

bool operator==(const PathPoint& lhs, const PathPoint& rhs) {
  return lhs.pose == rhs.pose && lhs.s_m == rhs.s_m && lhs.curvature_per_m == rhs.curvature_per_m &&
         lhs.direction == rhs.direction;
}

bool operator==(const PathSegment& lhs, const PathSegment& rhs) {
  return lhs.points == rhs.points;
}

}  // namespace planner::common
