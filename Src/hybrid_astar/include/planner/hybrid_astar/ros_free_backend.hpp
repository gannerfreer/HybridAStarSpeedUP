#pragma once

// 供所有 Wave-5 后端共用的离散状态 Hybrid A* 搜索核心。
//
// `RosFreeBackend` 实现该共享核心（运动基元 + 离散状态 A* + RS 解析连接），
// 每个后端配置实例化一次：
//   * `ros_free`               - 第一个归一化后端（默认配置）；
//   * `smac_native` / `smac_normalized`
//   * `kth_native`  / `kth_normalized`
//   * `zm0612_native` / `zm0612_normalized`
//
// Native 配置复现锁定的上游搜索结构/参数（ROS-free 重写，见
// docs/upstream_notes.md），仅供参考；normalized 配置遵循统一契约并参与
// 选择。每个后端内部都使用正式项目组件：
//   * planner::rs       - RS 距离启发式函数与解析连接；
//   * planner::collision - 对每个运动基元和每条 RS 连接分段做整车位姿
//     与扫掠运动检查；
//   * planner::common   - PlanRequest / PlanResult 契约。
//
// 保证（在后处理中会再次检查，Docs §6.3）：
//   * 每个加入的基元和每条 RS 连接分段都用整车碰撞检测器检查；后处理
//     时间计入报告的总时间，最终路径会被再次验证（端点容差、曲率上限、
//     碰撞复检）；
//   * 方向变化（尖点）保留为 PathSegment 边界；
//   * 不施加额外平滑（上游平滑参数属于不支持项，会被拒绝而非忽略）；
//   * 扩展循环检查时间预算、节点预算与取消。

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/hybrid_astar/backend.hpp"

namespace planner::hybrid_astar {

// 共享搜索核心的配置。每个字段与 configure() 中同名的 JSON 键一一对应；
// 默认值为该键缺省时使用的值。后端配置提供各后端默认值，并可将额外的
// 上游参数名暴露为别名。
struct RosFreeConfig {
  // 搜索离散化。
  double xy_resolution_m{0.5};   // 用于闭集去重的状态栅格单元 [m]
  int heading_bins{72};          // 状态栅格的航向离散份数

  // 运动基元。
  double motion_step_m{2.0};           // 前进基元长度 [m]
  double reverse_motion_step_m{1.5};   // 倒车基元长度 [m]
  std::vector<double> steer_fractions{-1.0, -0.5, 0.0, 0.5, 1.0};
  bool allow_reverse{true};
  // > 0 时覆盖基元曲率使用的最小转弯半径（不得小于请求车辆推导出的
  // 半径）。0 表示由请求车辆推导。
  double min_turn_radius_m{0.0};

  // 行驶代价（g）。
  double forward_cost{1.0};      // 每前进 1 m 的代价
  double reverse_cost{2.0};      // 每倒车 1 m 的代价
  double steer_cost{0.1};        // 每米每 |转向比例| 的额外代价
  double gear_change_cost{2.0};  // 每次方向变化时的额外代价
  double turn_penalty{1.0};      // 转向 != 0 时步进代价的倍率
  double steer_change_cost{0.0}; // 转向档位变化时的额外代价

  // 启发式函数。
  double heuristic_weight{1.0};  // 加权 A* 因子，必须 > 0
  double grid_weight{1.0};       // 启发式函数 h 中栅格势场项的权重

  // 局部环路防护（Wave 8 实验）：运动基元在几何上穿过某个先前链路分段
  // （不含相邻段）时的额外代价。仅为软偏好：碰撞检测与完备性不变。默认
  // 禁用：惩罚取 15 时 ros_free 的绕行环路消失，但 zm0612_normalized 的
  // obstacle_109 路径长度比从 1.105 退化到 1.245，因此改为登记残留的
  // 自相交（见 README "Wave 8 跨组件复核"）。
  double loop_penalty{0.0};      // 每次穿越的额外代价；0 禁用
  int loop_guard_segments{16};   // 要检测的先前分段数量

  // 解析连接。
  int analytic_expansion_ratio{1};              // 每 N 次扩展尝试一次
  double analytic_expansion_max_length_m{50.0}; // 0 = 无长度限制 [m]
  double analytic_shot_distance_m{0.0};         // > 0：仅在该距离内尝试 [m]

  // 未实现上游的额外平滑：只接受 `smooth: false`（该参数按此说明列为
  // 受支持）。
  bool smooth{false};

  // 请求未设置时使用的预算（Docs §7.2 默认值）。
  double default_time_budget_s{5.0};
  std::int64_t default_node_budget{1000000};
};

// 一个后端配置：标识、类别、默认值、完整参数支持表（核心键加上游别名键）
// 以及映射到规范核心键的上游名称别名。
struct BackendProfile {
  std::string id;
  BackendKind kind{BackendKind::kNormalized};
  std::string description;
  RosFreeConfig defaults;
  std::vector<ParameterSupport> parameters;
  std::vector<std::pair<std::string, std::string>> aliases;
  std::vector<std::string> notes;  // 已记录的偏差（仅供参考）
};

// 规范核心键的支持表（全部受支持）。
std::vector<ParameterSupport> core_parameter_support();

// 默认配置：`ros_free` 归一化。
BackendProfile ros_free_profile();

// 使用 `profile` 的默认值、别名和支持表把 `config` 解析到 *out。不支持的
// 键记录在报告中（并使 plan() 拒绝运行）；类型错误或取值越界会拒绝整个
// 配置。
ConfigReport configure_ros_free(const nlohmann::json& config,
                                const BackendProfile& profile,
                                RosFreeConfig* out);

class RosFreeBackend final : public HybridAStarBackend {
 public:
  RosFreeBackend();
  explicit RosFreeBackend(BackendProfile profile);
  ~RosFreeBackend() override;

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

  const BackendProfile& profile() const noexcept { return profile_; }

 private:
  struct Cache;

  common::PlanResult run_plan(const common::PlanRequest& request,
                              const common::GridView& grid,
                              const collision::CollisionChecker& collision_checker,
                              RsProvider& rs_provider);

  BackendProfile profile_;
  RosFreeConfig config_;
  ConfigReport report_;
  std::string unsupported_reason_;  // 设置了不支持的键时非空
  bool configured_{true};
  std::function<bool()> cancel_callback_;
  std::unique_ptr<Cache> cache_;
};

}  // namespace planner::hybrid_astar
