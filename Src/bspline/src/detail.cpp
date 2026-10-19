// planner_bspline 的共享元数据辅助实现（内部）。
//
// 这里只保留与后端无关的部分：参数缩放换算与节点查询。
// 具体几何操作由 Backend 实现（tinyspline_backend.cpp / eigen_backend.cpp /
// 对照库的 ego_backend.cpp）。

#include "impl.hpp"

#include <algorithm>
#include <cmath>

#include "backend.hpp"

namespace planner::bspline::detail {

// ---------------------------------------------------------------------------
// 对照后端（EGO UniformBspline）工厂注册
// ---------------------------------------------------------------------------

namespace {

EgoBackendFactory g_ego_backend_factory = nullptr;

}  // namespace

void set_ego_backend_factory(EgoBackendFactory factory) noexcept {
  g_ego_backend_factory = factory;
}

EgoBackendFactory ego_backend_factory() noexcept {
  return g_ego_backend_factory;
}

// ---------------------------------------------------------------------------
// 参数缩放与节点查询（后端/缩放单位）
// ---------------------------------------------------------------------------

double to_scaled_parameter(const SplineImpl& impl, double u) noexcept {
  return (u - impl.knot_min) * impl.knot_scale;
}

double to_original_parameter(const SplineImpl& impl, double scaled) noexcept {
  return impl.knot_min + scaled * impl.knot_scale_inverse;
}

int knot_multiplicity(const std::vector<double>& knots, double value) noexcept {
  int count = 0;
  for (const double knot : knots) {
    if (knot == value) {
      ++count;
    }
  }
  return count;
}

bool interior_knot_multiplicity(const SplineImpl& impl, double scaled,
                                int* multiplicity) noexcept {
  if (!(scaled > impl.scaled_min && scaled < impl.scaled_max)) {
    return false;
  }
  const int count = knot_multiplicity(impl.scaled_knots, scaled);
  if (count <= 0) {
    return false;
  }
  if (multiplicity != nullptr) {
    *multiplicity = count;
  }
  return true;
}

double span_left(const SplineImpl& impl, double scaled) noexcept {
  double left = impl.scaled_min;
  for (const double knot : impl.scaled_knots) {
    if (knot <= scaled && knot < impl.scaled_max && knot > left) {
      left = knot;
    }
  }
  return left;
}

bool merge_possible(const SplineImpl& impl, int order) noexcept {
  if (order <= 0) {
    return false;
  }
  // 在第 k 次求导期间，内部节点的重数必须达到
  // degree - k + 2，TinySpline 才会合并该间断。
  const int threshold = impl.degree - order + 2;
  for (std::size_t i = 0; i < impl.scaled_knots.size();) {
    std::size_t j = i;
    while (j < impl.scaled_knots.size() &&
           impl.scaled_knots[j] == impl.scaled_knots[i]) {
      ++j;
    }
    const double value = impl.scaled_knots[i];
    if (value > impl.scaled_min && value < impl.scaled_max &&
        static_cast<int>(j - i) >= threshold) {
      return true;
    }
    i = j;
  }
  return false;
}

}  // namespace planner::bspline::detail
