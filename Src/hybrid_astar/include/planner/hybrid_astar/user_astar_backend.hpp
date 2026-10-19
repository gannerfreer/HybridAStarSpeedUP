#pragma once

// 用户自研 HybridAstar 适配后端（vendored 上游见
// upstream/user_astar/PROVENANCE.md；公开头文件不暴露上游类型）。
//
// 两个实例：
//   * user_astar_raw        (kNative)     - 上游原样行为：内部栅格碰撞、
//     原始倒车运动学、内置目标容差、无 RS 解析连接；参考项，不参与选型。
//   * user_astar_normalized (kNormalized) - 修正倒车运动学（标准自行车模型）+
//     公共 planner::collision 位姿/运动检查 + 公共 planner::rs 目标连接 +
//     预算/取消；参与选型（与共享核心的归一化候选同口径）。

#include <functional>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "planner/hybrid_astar/backend.hpp"

namespace planner::hybrid_astar {

class UserAstarBackend final : public HybridAStarBackend {
 public:
  explicit UserAstarBackend(BackendKind kind);
  ~UserAstarBackend() override;

  UserAstarBackend(const UserAstarBackend&) = delete;
  UserAstarBackend& operator=(const UserAstarBackend&) = delete;

  std::string id() const override;
  BackendKind kind() const noexcept override;
  BackendInfo info() const override;

  ConfigReport configure(const nlohmann::json& config) override;
  const nlohmann::json& effective_config() const override;

  common::PlanResult plan(const common::PlanRequest& request,
                          const common::GridView& grid,
                          const collision::CollisionChecker& collision_checker,
                          RsProvider& rs_provider) override;

  void reset_cache() override;

  void set_cancel_callback(std::function<bool()> callback) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace planner::hybrid_astar
