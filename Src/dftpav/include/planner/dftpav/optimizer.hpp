#pragma once

// planner_dftpav：独立的实验性 Dftpav 轨迹优化器后端。
//
// 范围（Docs/component_development_plan.md 第 2、3.5、6.2 节）：
//   * 消费预制初始路径（有序的 planner::common::PathSegment，带前进/后退
//     方向，例如 Hybrid A* ros_free 输出）、planner::common::Corridor、
//     车辆动力学限制和地图场景；产出按时间参数化的
//     planner::common::TrajectoryPoint 序列，或显式的失败状态/原因；
//   * 优化器核心移植自 ZJU-FAST-Lab/Dftpav（GPL-3.0，pinned commit
//     87f63ad2f67506e090b36eac38f5d247c51a330e，见 PROVENANCE.md），
//     移除了 ROS、protobuf 参数加载与可视化；
//   * 公开 API 不依赖 ROS，不暴露任何上游或 Eigen 类型。
//
// 验证门槛（计划第 6.2 节）：只有当
// planner::validation::validate_trajectory 针对同一车辆 / 限制 / 目标 /
// 场景 / 走廊接受采样轨迹时，结果才标记为 `kSuccess`。validation 是使用
// 固定硬容差的独立组件（第 7.1 节）；适配器从不对其放宽。当优化器的
// 平滑惩罚解未通过验证时，适配器以更紧的内部几何预算重试（有界的确定性
// 阶梯），否则返回 `kValidationFailed` 并附验证原因。
//
// 返回轨迹（planner::common::TrajectoryPoint）的语义：
//   * t_s 严格递增且有限；
//   * velocity_mps 带符号；每次换向（换挡）处速度为零
//     （验收界限 |v| <= 1e-3 m/s，见第 7.1 节）；
//   * acceleration_mps2 为纵向加速度 d(velocity)/dt；
//   * curvature_inv_m 为带符号曲率 tan(steer)/wheelbase，满足
//     yaw_rate = velocity_mps * curvature_inv_m；在速度奇异尺度以下为 0
//     （静止时不从位置导数重建航向）。
//
// 验证之后，方向感知的临时性检查会核对 validation 的单限制接口无法表达的
// 限制（分方向的速度/加速度/曲率、横向加速度 |v^2*kappa| 及换向停车）。
// 它只是诊断性质的检查，不能替代验证门槛。

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"
#include "planner/validation/validation.hpp"

namespace planner::dftpav {

// 车辆动力学限制。所有值为正；前进/后退值以幅值给出
// （在优化器内部按行驶方向取符号施加）。
struct OptimizeLimits {
  double max_forward_velocity_mps{5.0};
  double max_backward_velocity_mps{2.0};
  double max_forward_acceleration_mps2{4.0};
  double max_backward_acceleration_mps2{4.0};
  double max_forward_curvature_inv_m{1.0};
  double max_backward_curvature_inv_m{1.0};
  // |velocity|^2 * |curvature| 上界。
  double max_lateral_acceleration_mps2{5.0};
  // 上游 `max_phidot`：曲率项使用的转向速率上界。
  double max_steering_rate_radps{10000.0};
};

// 优化器权重与容差。除特别说明外，默认值遵循上游
// config/minco_config.pb.txt。
struct OptimizeOptions {
  int traj_resolution{16};     // 每个内部分段的走廊采样点
  int destraj_resolution{32};  // 首/末分段的走廊采样点
  double static_obstacle_weight{1000.0};  // 上游 wei_sta_obs
  double feasibility_weight{2500.0};      // 上游 wei_feas
  double time_weight{500.0};              // 上游 wei_time
  // 优化器所用整车外壳各边额外增加的余量。空 = vehicle.safety_margin_m。
  std::optional<double> hull_margin_m{};
  double traj_piece_duration_s{1.0};  // 初值分段时长
  double min_piece_duration_s{0.1};   // 上游 mini_t
  int lbfgs_memsize{256};
  int lbfgs_past{3};
  double lbfgs_delta{1.0e-4};
  int lbfgs_max_iterations{12000};
  double cost_fail_threshold{50000.0};
  // 将优化器的内部路径点与连接点位置二次锚定到其预制初值（每米^2 代价）。
  // 走廊区间与初值弧长绑定，因此让解靠近初值可保持站点结构；0 表示禁用。
  double anchor_weight{2000.0};
  // 相对上游（使用硬编码的 0.24 m/s）的移植偏差：边界与换挡处的速度
  // 幅值。0.0 使轨迹恰好在换向处停车，符合验收标准的要求。
  double direction_switch_velocity_mps{0.0};
  // 返回的时间参数化轨迹的采样步长。
  double sample_dt_s{0.05};
  // 临时性可行性检查的相对容差：
  // tol = constraint_tolerance * max(1, limit)（计划第 7.1 节取 1e-6）。
  double constraint_tolerance{1.0e-6};
  // 低于此速度时报告的 curvature_inv_m 置为 0。MINCO 表示在静止边界处约束
  // v=0 与 a=0，但那里的路径曲率无界（极限 kappa = cross(sigma', sigma'') /
  // |sigma'|^3 会随亚毫米级的路径弯钩发散），因此在速度奇异尺度以下，
  // 转向相关的值没有物理意义。默认值与上游 Dftpav 的连接点速度常数
  // （non_sinv = 0.24 m/s）一致；上游用该尺度排除同一奇异区域。
  // 高于阈值时保留解析值。
  double curvature_report_min_speed_mps{0.24};
  // 航向（yaw）报告尺度。只要 |v| 超过此（极小）尺度，存储的 yaw 就是
  // 解析运动方向；只有在真正静止时才回退到沿用的/初值航向。尺度取大会在
  // 轨迹加速穿越它时产生航向间断：planner::validation 依据相邻存储位姿的
  // yaw 差重建圆弧，于是虚假的 yaw 跳变会被读成极小半径圆弧，其插值位姿
  // 偏离真实路径（即使每个存储位姿都在走廊内，也可能触发走廊包含检查
  // 失败）。曲率报告使用它自己更大的奇异尺度。
  double heading_report_min_speed_mps{0.02};
  // 仅在诊断运行时禁用；该值为 true 时，未通过检查的结果绝不会报告为
  // kSuccess。
  bool provisional_checks{true};
  // 对交给优化器的走廊半平面做向内收紧（单位米；0 表示禁用）。validation
  // 始终检查原走廊，因此这会把平滑惩罚残差转化为验证余量；它是一种
  // 内近似，从不放宽验证门槛。
  double corridor_tightening_m{0.0};
  // 确定性的求解后走廊包含修复（Wave 7）：在软约束求解之后，将违规采样
  // 向预制路径拉回，把保守足迹投影到覆盖的走廊多边形内，并重新生成
  // MINCO 轨迹。仅在 A/B 诊断时禁用；验证门槛始终运行。
  bool corridor_repair{true};
  // 内部重试阶梯施加的曲率收紧比例（0.01 = 1%）。上游用平滑惩罚施加曲率
  // 约束，活动约束上会留下残差；默认 1% 与既有行为逐位一致。该值只影响
  // 优化器内部软约束预算（收紧方向），从不对 planner::validation 的硬限制
  // 做任何放宽。
  double internal_curvature_tightening{0.01};
  // 碰撞驱动的定向重试（hybrid-006）：当首次 V 判定以 `validation_collision`
  // 失败时，按有界、确定性的阶梯尝试更细分段、更强的初值锚定、更紧的内部
  // 曲率预算（必要时还有更高的可行性权重与约束采样分辨率）；每次尝试仍走
  // 完整 V 判定。全部失败时返回原失败原因与统计，绝不返回部分结果。
  // 仅在 A/B 诊断时禁用。
  bool collision_retry{true};
  // 协作式取消（移植扩展，见 PROVENANCE.md 与 README §1）。可选谓词：
  // 返回 true 表示调用方请求取消。优化器在求解前、L-BFGS 迭代边界
  // （移植核心的代价回调与进度回调）、求解后修复边界和验证前轮询它；
  // 取消一旦被观察到，optimize_trajectory 立即返回
  // status = common::Status::kCancelled、reason = "cancelled"，
  // `trajectory` 为空：**绝不返回部分轨迹**，也绝不放宽 V/碰撞门禁
  // （尚未被 V 判定的中间迭代结果一律丢弃）。
  // 谓词由调用线程（优化器是单线程的）调用，必须非阻塞、不抛异常；
  // 由于取消通常来自其他线程（如作业管理线程），其实现需自行保证
  // 线程安全（典型做法是读取 std::atomic<bool> 或共享的标志位）。
  // 空函数对象表示“永不取消”（默认），此时行为与既有版本逐位一致。
  std::function<bool()> cancel_check{};
};

struct OptimizeRequest {
  std::string request_id;  // 信息性字段，会在统计信息中原样返回
  std::string map_id;      // 两者均非空时必须与走廊一致
  std::string map_version; // 同上
  common::VehicleModel vehicle;
  // 从起点到终点的有序初始路径。点的顺序沿行驶方向；`direction ==
  // kReverse` 表示车辆航向与路径点推进方向相反（标准 RS/路径约定）。
  // Hybrid A* ros_free 后端输出的正是此类型。
  std::vector<common::PathSegment> seed;
  // 沿同一初值生成的安全走廊（planner::corridor 的输出）。
  common::Corridor corridor;
  // validation 检查的终点。空 = 初值最后一个点的位姿。
  std::optional<common::Pose2d> goal;
  // 供 validation 碰撞检测使用的地图快照（栅格 + 凸障碍物）。
  // 空 = 无栅格自由空间（validation 文档规定的空场景）。
  std::optional<collision::Scene> scene;
  OptimizeLimits limits;
  OptimizeOptions options;
};

// 稳定的失败原因编码（成功时为空）：
//   输入      : "invalid_vehicle", "invalid_limits", "invalid_options",
//               "invalid_seed", "empty_corridor", "invalid_corridor",
//               "map_mismatch", "corridor_coverage"
//   优化器    : "solver_failed", "cost_too_high"
//   取消      : "cancelled"（status = kCancelled；总是伴随空 `trajectory`，
//               不返回任何部分迭代结果，见 OptimizeOptions::cancel_check）
//   临时性检查（方向感知诊断）：
//               "provisional_non_finite", "provisional_time_order",
//               "provisional_velocity_limit", "provisional_acceleration_limit",
//               "provisional_curvature_limit",
//               "provisional_lateral_acceleration_limit",
//               "provisional_direction_change_stop"
//   验证门槛（权威；planner::validation 的约束名称）：
//               "validation_speed_limit", "validation_acceleration_limit",
//               "validation_deceleration_limit", "validation_curvature_limit",
//               "validation_corridor_containment", "validation_collision",
//               "validation_endpoint_position", "validation_endpoint_heading",
//               "validation_time_monotonicity", "validation_finite_values",
//               "validation_direction_change_stop"，以及其余以
//               "validation_" 为前缀的 validation 输入编码
struct OptimizeStats {
  int direction_changes{0};
  int total_pieces{0};
  int lbfgs_iterations{0};
  int lbfgs_result{0};
  double final_cost{0.0};
  double solve_time_s{0.0};
  double trajectory_duration_s{0.0};
  std::size_t trajectory_points{0};
  std::size_t corridor_constraint_points{0};
  double max_speed_mps{0.0};
  double max_acceleration_mps2{0.0};
  // 报告采样点中的最大 |curvature|（低于 curvature_report_min_speed_mps 的
  // 采样点报告 0）。
  double max_curvature_inv_m{0.0};
  double max_lateral_acceleration_mps2{0.0};
  // 优化器尝试次数（1 = 基础收紧；限制/走廊残差类原因最多 4 次，最后一次
  // 还会把初值分段时长减半；首次即碰撞失败时另有最多 3 次碰撞驱动的定向
  // 尝试，见 OptimizeOptions::collision_retry）。
  int tightening_attempts{1};
  // 确定性的走廊包含恢复（Wave 9，dftpav-002）：施加的外层迭代次数，
  // 以及恢复后最严重的保守足迹违规量，按 validation 的关联方式测量
  // （存储采样点加上重建的中间位姿）。<= 0 表示位于走廊内。
  int corridor_repair_iterations{0};
  double corridor_repair_worst_violation_m{0.0};
  // 按顺序记录每次尝试的原因（通过的尝试为 "success"）；
  // 返回的结果是严重度最优的那次尝试。
  std::vector<std::string> attempt_reasons;
  // 换向处的全局轨迹时间。
  std::vector<double> direction_switch_times_s;
};

struct OptimizeResult {
  common::Status status{common::Status::kInvalidInput};
  std::string reason;
  std::string detail;
  std::vector<common::TrajectoryPoint> trajectory;
  // planner::validation 依据请求的车辆 / 限制 / 目标 / 场景 / 走廊对
  // `trajectory` 作出的权威判定。只要产出了轨迹就填充（包括未通过的），
  // 便于调用方看到首个被违反的约束及其测量值。
  validation::ValidationResult validation;
  OptimizeStats stats;

  bool ok() const noexcept { return status == common::Status::kSuccess; }
};

// 在预制路径 + 走廊上运行抽取出的 Dftpav 优化器。
//
// 优化器核心（移植自上游，见 PROVENANCE.md）对每个恒定方向的子路径求解
// 时间最优的分段 Bezier（MINCO）问题，在约束速度、纵向/横向加速度和曲率的
// 同时，使整车外壳保持在走廊半平面内。对相同输入，返回的轨迹是确定性的。
OptimizeResult optimize_trajectory(const OptimizeRequest& request);

}  // namespace planner::dftpav
