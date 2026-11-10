/**
 * @file path.cpp
 * @brief 路径方向字符串转换及路径数据结构相等性比较操作符的具体实现。
 */

#include "planner/common/path.hpp"

#include <stdexcept>
#include <string>

namespace planner::common {

std::string to_string(Direction direction) {
  switch (direction) {
    case Direction::kForward:
      return "forward";
    case Direction::kReverse:
      return "reverse";
  }
  throw std::invalid_argument("to_string(Direction): invalid enumerator");
}

bool operator==(const PathPoint& lhs, const PathPoint& rhs) {
  return lhs.pose == rhs.pose && lhs.s_m == rhs.s_m && lhs.curvature_inv_m == rhs.curvature_inv_m &&
         lhs.direction == rhs.direction;
}

bool operator==(const PathSegment& lhs, const PathSegment& rhs) {
  return lhs.points == rhs.points;
}

}  // namespace planner::common
