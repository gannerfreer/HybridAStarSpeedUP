#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "planner/rs/reeds_shepp.hpp"

namespace planner::rs {
namespace {

// 防止病态输入造成意外的无界分配。
constexpr std::size_t kMaxSampleCount = 20'000'000;

struct PathView {
  std::vector<double> bounds;             // n + 1 个累计边界
  std::vector<common::Pose2d> seg_start;  // n 个分段起始位姿
  double length_m{0.0};
};

double tolerance_for(double length_m) {
  return 1e-12 * std::max(1.0, length_m);
}

// 校验分段结构，并预计算累计边界和分段起始位姿。
// 路径畸形时抛出 std::invalid_argument。
PathView make_view(const RsPath& path) {
  PathView view;
  view.length_m = path.length_m;
  const std::size_t n = path.segments.size();
  view.bounds.reserve(n + 1);
  view.seg_start.reserve(n);
  view.bounds.push_back(0.0);
  double cumulative = 0.0;
  common::Pose2d cursor = path.start;
  for (const RsSegment& segment : path.segments) {
    if (!std::isfinite(segment.length_m) || segment.length_m < 0.0) {
      throw std::invalid_argument(
          "rs::sample: segment length_m must be finite and >= 0");
    }
    if (!std::isfinite(segment.curvature_inv_m)) {
      throw std::invalid_argument(
          "rs::sample: segment curvature_inv_m must be finite");
    }
    view.seg_start.push_back(cursor);
    cumulative += segment.length_m;
    view.bounds.push_back(cumulative);
    cursor = integrate_segment(cursor, segment, segment.length_m);
  }
  if (!std::isfinite(path.length_m)) {
    throw std::invalid_argument("rs::sample: path.length_m must be finite");
  }
  const double tol = tolerance_for(std::max(std::fabs(path.length_m), cumulative));
  if (path.segments.empty()) {
    if (std::fabs(path.length_m) > tol) {
      throw std::invalid_argument(
          "rs::sample: an empty path must have length_m == 0");
    }
    view.length_m = 0.0;
  } else if (std::fabs(cumulative - path.length_m) > tol) {
    throw std::invalid_argument(
        "rs::sample: path.length_m does not match the segment sum");
  }
  // `path.length_m` 仍是权威总长，使采样出的 s_m 与调用方的路径精确一致；
  // 各分段的局部偏移无论如何都会被夹紧。
  return view;
}

// 弧长 `s` 所属分段的索引；尖点边界属于从该处开始的分段
// （最后一个分段拥有 s == length）。
std::size_t locate_segment(const PathView& view, double s) {
  const std::size_t n = view.seg_start.size();
  const double tol = tolerance_for(view.length_m);
  std::size_t index = 0;
  while (index + 1 < n && s >= view.bounds[index + 1] - tol) {
    ++index;
  }
  return index;
}

common::PathPoint point_at(const RsPath& path, const PathView& view, double s) {
  common::PathPoint point;
  point.s_m = s;
  if (view.seg_start.empty()) {
    point.pose = path.start;
    point.curvature_inv_m = 0.0;
    point.direction = common::Direction::kForward;
    return point;
  }
  const std::size_t index = locate_segment(view, s);
  const RsSegment& segment = path.segments[index];
  double local = s - view.bounds[index];
  local = std::min(std::max(local, 0.0), segment.length_m);
  point.pose = integrate_segment(view.seg_start[index], segment, local);
  point.curvature_inv_m = segment.curvature_inv_m;
  point.direction = segment.direction;
  return point;
}

}  // namespace

std::vector<common::PathPoint> sample(const RsPath& path, double ds_m) {
  if (!std::isfinite(ds_m) || !(ds_m > 0.0)) {
    throw std::invalid_argument("rs::sample: ds_m must be finite and > 0");
  }
  const PathView view = make_view(path);
  std::vector<common::PathPoint> points;
  const double length = view.length_m;
  if (length <= 0.0) {
    points.push_back(point_at(path, view, 0.0));
    return points;
  }

  const double expected = length / ds_m;
  if (expected > static_cast<double>(kMaxSampleCount)) {
    throw std::invalid_argument(
        "rs::sample: ds_m is too small for this path (sample limit exceeded)");
  }
  const std::size_t grid_count = static_cast<std::size_t>(std::floor(expected));

  // 将规则 ds 网格与每个分段边界合并（尖点必须被精确采样）。
  // 去重时容差内的边界优先，以保证尖点精确。
  struct Target {
    double s;
    bool boundary;
  };
  std::vector<Target> targets;
  targets.reserve(grid_count + view.seg_start.size() + 2);
  targets.push_back({0.0, false});
  for (std::size_t k = 1; k <= grid_count; ++k) {
    const double s = static_cast<double>(k) * ds_m;
    if (s < length) {
      targets.push_back({s, false});
    }
  }
  for (std::size_t i = 1; i < view.bounds.size(); ++i) {
    targets.push_back({view.bounds[i], true});
  }
  std::sort(targets.begin(), targets.end(),
            [](const Target& a, const Target& b) { return a.s < b.s; });

  const double tol = tolerance_for(length);
  std::vector<Target> merged;
  merged.reserve(targets.size());
  for (const Target& target : targets) {
    if (!merged.empty() && target.s - merged.back().s <= tol) {
      if (target.boundary && !merged.back().boundary) {
        merged.back() = target;
      }
      continue;
    }
    merged.push_back(target);
  }
  if (merged.empty()) {
    merged.push_back({0.0, false});
  }
  // 首个采样点总是恰为 s = 0（比合并容差更近的分段边界
  // 不得替换起点）。
  merged.front().s = 0.0;
  if (length - merged.back().s > tol) {
    merged.push_back({length, true});
  } else {
    merged.back() = {length, true};
  }

  points.reserve(merged.size());
  for (const Target& target : merged) {
    const double s = std::min(std::max(target.s, 0.0), length);
    points.push_back(point_at(path, view, s));
  }
  // 末个采样点恰为路径终点（空分段已在上方处理）。
  points.back().s_m = length;
  return points;
}

common::PathPoint sample_at_s(const RsPath& path, double s_m) {
  if (!std::isfinite(s_m)) {
    throw std::invalid_argument("rs::sample_at_s: s_m must be finite");
  }
  const PathView view = make_view(path);
  if (view.seg_start.empty()) {
    if (s_m == 0.0) {
      return point_at(path, view, 0.0);
    }
    throw std::out_of_range("rs::sample_at_s: s_m outside [0, 0]");
  }
  const double tol = tolerance_for(view.length_m);
  if (s_m < -tol || s_m > view.length_m + tol) {
    throw std::out_of_range("rs::sample_at_s: s_m outside [0, path.length_m]");
  }
  const double s = std::min(std::max(s_m, 0.0), view.length_m);
  return point_at(path, view, s);
}

}  // namespace planner::rs
