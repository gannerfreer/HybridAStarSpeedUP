#pragma once

// 独立参考实现，用于 Docs/component_development_plan.md §7.1 规定的随机
// 差分对比：
//
//   * 它不重用任何生产加速结构（没有分块索引、没有空间哈希、没有包围体
//     层次结构；生产检查器是 planner/collision/collision_checker.hpp）；
//   * 位姿检查使用直接编码的多边形重叠测试（顶点/边相交），而不是生产
//     的分离轴例程；
//   * 运动检查使用保守的区间细分：不断细分区间，直到车辆任一点的最大
//     位移不超过 1e-3 m；每个区间的残余运动由保守包络覆盖。包络在精度
//     下限以下无法证明无碰撞的区间给出 `inconclusive`（绝不给出真值）。
//
// 接触计为碰撞，未知单元与地图之外视为占据：语义与生产检查器完全一致。
// `inconclusive` 用于在精度下限处无法判定的样本，因此它们可从漏检/误报
// 统计中排除并替换。

#include <string>
#include <vector>

#include "planner/collision/collision_checker.hpp"
#include "planner/collision/scene.hpp"

namespace planner::collision {

using planner::common::PathSegment;

enum class ReferenceVerdict {
  kCollision,
  kClear,
  kInconclusive,
  kInvalidInput
};

struct ReferenceResult {
  ReferenceVerdict verdict{ReferenceVerdict::kClear};
  Pose2d first_pose;
  double first_s_m{0.0};
  std::string detail;
};

ReferenceResult reference_check_pose(const Pose2d& pose,
                                     const VehicleModel& vehicle,
                                     const Scene& scene);

ReferenceResult reference_check_motion(const MotionSegment& segment,
                                       const VehicleModel& vehicle,
                                       const Scene& scene);

ReferenceResult reference_check_path(const std::vector<PathSegment>& path,
                                     const VehicleModel& vehicle,
                                     const Scene& scene);

}  // namespace planner::collision
