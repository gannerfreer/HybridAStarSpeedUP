#pragma once

// 夹具侧轻量二维有效性检查助手（X-S4）。
//
// 统一契约要求"沿边检查一致，非仅端点"。本助手把点有效性回调扩展为
// 固定步长的沿边检查：沿线段采样（含两端点），全部采样点有效才有效。
// 夹具、示例与测试用它构造 Problem::point_valid / edge_valid；
// 库自身不内置任何障碍模型。

#include <cmath>
#include <cstdint>

#include <planner/sampling_search/types.hpp>

namespace planner::sampling_search {

class SampledEdgeValidity {
 public:
  // point_fn：点有效性回调；step：沿边采样步长（米），必须为正有限值。
  SampledEdgeValidity(PointValidityFn point_fn, double step)
      : point_fn_(std::move(point_fn)), step_(step) {}

  bool is_point_valid(const Point2d& p) const {
    return point_fn_ && point_fn_(p);
  }

  // 沿边采样：n = max(1, ceil(|b-a| / step))，检查 t = i/n（含端点）。
  bool is_edge_valid(const Point2d& a, const Point2d& b) const {
    if (!point_fn_) {
      return false;
    }
    const double dx = b.x_m - a.x_m;
    const double dy = b.y_m - a.y_m;
    const double distance = std::hypot(dx, dy);
    const double raw_n = distance / step_;
    std::uint64_t n = 1;
    if (raw_n > 1.0) {
      n = static_cast<std::uint64_t>(std::ceil(raw_n));
    }
    for (std::uint64_t i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      if (!point_fn_(Point2d{a.x_m + t * dx, a.y_m + t * dy})) {
        return false;
      }
    }
    return true;
  }

  const PointValidityFn& point_fn() const { return point_fn_; }
  double step() const { return step_; }

 private:
  PointValidityFn point_fn_;
  double step_;
};

}  // namespace planner::sampling_search
