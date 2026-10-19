// 适配器内部工具：控制（取消/预算）检查与计时。不对外安装。

#pragma once

#include <chrono>

#include "planner/graph_search/types.hpp"

namespace planner {
namespace graph_search {
namespace detail {

using Clock = std::chrono::steady_clock;

inline bool cancel_requested(const Control& control) {
  return control.cancel != nullptr &&
         control.cancel->load(std::memory_order_relaxed);
}

inline bool budget_exceeded(const Control& control, Clock::time_point start) {
  if (!(control.budget_s > 0.0)) {
    return false;
  }
  return std::chrono::duration<double>(Clock::now() - start).count() >
         control.budget_s;
}

inline double elapsed_ms(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start)
      .count();
}

// 统一入口的公共预检查：先取消后超时（与各适配器内部检查顺序一致）。
inline Status precheck_control(const Control& control, Clock::time_point start) {
  if (cancel_requested(control)) {
    return Status::kCancel;
  }
  if (budget_exceeded(control, start)) {
    return Status::kTimeout;
  }
  return Status::kSuccess;  // 继续搜索
}

}  // namespace detail
}  // namespace graph_search
}  // namespace planner
