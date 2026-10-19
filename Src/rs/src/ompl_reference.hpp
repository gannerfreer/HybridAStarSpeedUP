#pragma once

// 私有 OMPL 1.6.0 参照适配器（仅在 PLANNER_RS_WITH_OMPL=ON
// 且锁定的 OMPL 源码构建成功时才编译）。
//
// 此头文件绝不能出现在安装/公开的 include 树中：planner::rs 从不暴露
// OMPL 类型。该适配器用于让测试和基准测试将交付的原生求解器与锁定的
// OMPL `ReedsSheppStateSpace` 作比较。两者实现同一 48 曲线族，因此该
// 比较是*封装一致性测试*，而不是独立的双重验证
// （Docs/component_development_plan.md 3.7）。

#include "planner/rs/reeds_shepp.hpp"

namespace planner::rs::ompl_reference {

// 来自锁定的 OMPL ReedsSheppStateSpace 的距离。被 rs::is_valid_query
// 拒绝的查询会抛出 std::invalid_argument。
double distance(const common::Pose2d& start, const common::Pose2d& goal,
                const common::VehicleModel& vehicle);

// 将 OMPL 的最短词转换为 planner::rs 路径表示。
// 无效查询会抛出 std::invalid_argument。
RsPath path(const common::Pose2d& start, const common::Pose2d& goal,
            const common::VehicleModel& vehicle);

}  // namespace planner::rs::ompl_reference
