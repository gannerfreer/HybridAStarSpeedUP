/**
 * @file status.cpp
 * @brief 执行状态码枚举字符串转换的具体实现。
 */

#include "planner/common/status.hpp"

#include <stdexcept>
#include <string>

namespace planner::common {

std::string to_string(Status status) {
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
  throw std::invalid_argument("to_string(Status): invalid enumerator");
}

}  // namespace planner::common
