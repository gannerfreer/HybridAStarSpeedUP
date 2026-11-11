/**
 * @file status.cpp
 * @brief 执行状态码枚举字符串转换的具体实现
 */

#include "planner/common/status.hpp"

#include <string_view>

namespace planner::common {

std::string_view to_string(Status status) noexcept {
  switch (status) {
    case Status::kSuccess:
      return "success";
    case Status::kInvalidInput:
      return "invalid_input";
    case Status::kNoPath:
      return "no_path";
    case Status::kTimeout:
      return "timeout";
    case Status::kCancelled:
      return "cancelled";
    case Status::kOptimizationFailed:
      return "optimization_failed";
    case Status::kValidationFailed:
      return "validation_failed";
  }
  return "unknown";
}

}  // namespace planner::common
