#pragma once

// 每个 Hybrid A* 适配器使用的 Reeds-Shepp 提供者抽象。
//
// §6.3 要求所有后端的解析连接与 RS 距离启发式函数都走同一契约。`native`
// 适配器可在此接口后接入自己的上游 RS 实现；归一化 `ros_free` 后端使用
// PlannerRsProvider，它封装正式的 planner::rs 组件。本组件的公开头文件只
// 暴露 planner::rs 类型（项目组件），绝不暴露第三方上游类型。

#include <string>

#include "planner/common/types.hpp"
#include "planner/rs/reeds_shepp.hpp"

namespace planner::hybrid_astar {

class RsProvider {
 public:
  virtual ~RsProvider() = default;

  // 提供者构建时使用的车辆模型。plan() 必须以与请求/碰撞检测器相同的
  // 车辆调用。
  virtual const common::VehicleModel& vehicle() const = 0;

  // 不抛异常的最短 RS 距离。成功时返回 kSuccess 并写入 `*distance_m`；
  // 否则返回 kInvalidInput / kOptimizationFailed（此时输出保持不变）。
  // `reason` 接收失败信息。
  virtual common::Status distance(const common::Pose2d& from,
                                  const common::Pose2d& to, double* distance_m,
                                  std::string* reason = nullptr) const = 0;

  // 不抛异常的解析 RS 连接（调用正式的 planner::rs 求解）。
  virtual rs::RsPlan plan(const common::Pose2d& start,
                          const common::Pose2d& goal) const = 0;
};

// 默认提供者：为单一固定车辆封装 planner::rs。
class PlannerRsProvider final : public RsProvider {
 public:
  explicit PlannerRsProvider(const common::VehicleModel& vehicle);

  const common::VehicleModel& vehicle() const override;

  common::Status distance(const common::Pose2d& from, const common::Pose2d& to,
                          double* distance_m,
                          std::string* reason = nullptr) const override;

  rs::RsPlan plan(const common::Pose2d& start,
                  const common::Pose2d& goal) const override;

 private:
  common::VehicleModel vehicle_;
};

}  // namespace planner::hybrid_astar
