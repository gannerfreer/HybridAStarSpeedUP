#pragma once

// 安全走廊生成（第一版）与几何验证。
//
// 输入：路径种子（planner::common::PathSegment 的有序序列）、静态凸障碍物
// （矩形或凸多边形）与车辆模型。输出：planner::common::Corridor（同一坐标系
// 下与路径弧长区间绑定的有序凸多边形），或显式失败状态加上稳定的原因码
// 和人类可读的 detail 字符串。狭窄或断裂的走廊显式失败；生成器绝不返回
// 违反自身不变量的走廊。
//
// 第一版算法（Docs/component_development_plan.md 第 6.2 节）：对种子重采样；
// 每相邻两个采样点生成一个凸多边形，先取该区间保守整车足迹的凸包，再做
// 局部扩张。每条边按精确的多边形-障碍物分离量推进到极限，其余各边保持
// 当前偏移（逐坐标最大化，受 `max_expansion_m` 限制）；因此远处障碍物只
// 限制它实际能触及的边，而不会沿法向收紧整条走廊（corridor-001）。每个
// 构造凸包还会按位姿在配置界限（`perturbation_yaw_rad`、
// `perturbation_position_m`，corridor-002）内受扰时最坏情况的足迹角点位移
// 进行膨胀，使狭窄走廊对种子的小幅航向或位置偏差仍保持保守。每个多边形
// 均为逆时针、至少三个顶点、按弧长排序、与相邻多边形重叠、在其区间的
// 每个路径点上包含整个保守足迹（车辆轮廓各边按 `safety_margin_m` 外扩），
// 并使每个障碍物保持在最小间隙之外。可通行性按几何检查（整个足迹位于
// 走廊内，且走廊最小宽度对车辆宽度加安全裕度），绝不只检查中心线。
//
// 调用方不得重新实现 JSON 编码：用 planner/common/serialization.hpp
// （走廊文档是第一版公开类型）持久化结果。

#include <cstddef>
#include <string>
#include <vector>

#include "planner/common/types.hpp"

namespace planner::corridor {

// 静态障碍物表示。顶点位于世界/地图坐标系；多边形应为凸多边形。
// `kind` 仅作说明，不改变顶点的解释方式。
enum class ObstacleKind {
  kRectangle,
  kConvexPolygon,
};

struct Obstacle {
  std::string id;
  ObstacleKind kind{ObstacleKind::kConvexPolygon};
  std::vector<common::Point2d> vertices;
};

// 轴对齐矩形障碍物，要求 x_min < x_max 且 y_min < y_max。
// 顶点按逆时针存储。
Obstacle make_rectangle_obstacle(std::string id, double x_min_m, double y_min_m,
                                 double x_max_m, double y_max_m);

// 由顶点给出的凸多边形障碍物，顶点顺序任意；生成器/验证器会将顺序
// 规范化为逆时针。
Obstacle make_convex_obstacle(std::string id, std::vector<common::Point2d> vertices);

// 走廊生成器的稳定失败分类。CorridorResult 中的 `reason` 字符串为下列
// 规范名称。
enum class CorridorFailure {
  kNone,
  kInvalidInput,
  kBlocked,
  kNarrow,
  kNoCorridor,
};

// 失败值的规范字符串："none"、"invalid_input"、"blocked"、
// "narrow"、"no_corridor"。
const char* to_string(CorridorFailure failure) noexcept;

// 生成选项。所有长度单位为米，角度单位为弧度。
struct CorridorOptions {
  // 种子路径的重采样步长；每相邻采样点区间生成一个凸多边形（实际区间
  // 不会超过该值，零长度种子区间除外）。
  double sample_step_m{0.25};
  // 构建单个多边形内保守足迹扫掠时使用的子采样步长；更小的值会让多边形
  // 在弯曲段更贴合。
  double construction_step_m{0.05};
  // 返回生成的走廊之前，独立包含性检查使用的稠密步长。
  double verification_step_m{0.02};
  // 走廊多边形与每个障碍物之间保持的最小间隙。若种子本身更近，则将间隙
  // 减小为可用间距的一半（多边形绝不允许接触障碍物）。
  double clearance_m{0.05};
  // 开阔空间中单条多边形边扩张的上界。
  double max_expansion_m{4.0};
  // 位姿扰动鲁棒性界限（corridor-002）。每个构造凸包按扰动位姿（航向至多
  // `perturbation_yaw_rad`，位置至多 `perturbation_position_m`）最坏情况的
  // 角点位移进行膨胀，因此生成的走廊仍包含这些界限内任意受扰种子位姿的
  // 保守整车足迹。已有的安全裕度与间隙不变；将两个界限都置零可禁用该包络。
  double perturbation_yaw_rad{0.017453292519943295};  // 1 度
  double perturbation_position_m{0.02};               // 2 厘米
};

struct CorridorRequest {
  std::string request_id;
  std::string map_id;
  std::string map_version;
  std::string frame_id{"map"};
  common::VehicleModel vehicle;
  std::vector<common::PathSegment> seed;
  std::vector<Obstacle> obstacles;
  CorridorOptions options;
};

struct CorridorStats {
  std::size_t seed_points{0};
  std::size_t construction_poses{0};
  std::size_t polygon_count{0};
  double path_length_m{0.0};
  // 所有多边形的最小卡尺宽度；无多边形时为 +infinity。
  double min_polygon_width_m{0.0};
  // 所有多边形的最小障碍物间隙；无障碍物时为 +infinity。
  double min_obstacle_clearance_m{0.0};
  // 因障碍物导致本可达到 `max_expansion_m` 的扩张受限的多边形边数
  // （统计所有多边形）。
  int obstacle_limited_edges{0};
  // 施加到构造凸包上的最坏情况角点位移包络（corridor-002）；
  // 两个扰动界限均为 0 时为 0。
  double robustness_margin_m{0.0};
};

struct CorridorResult {
  common::Status status{common::Status::kInvalidInput};
  CorridorFailure failure{CorridorFailure::kInvalidInput};
  // 稳定原因码："invalid_input"、"blocked"、"narrow"、"no_corridor"；
  // 成功时为空。
  std::string reason;
  // 人类可读说明，成功时为空。
  std::string detail;
  // 仅在 `status == common::Status::kSuccess` 时有效。
  common::Corridor corridor;
  CorridorStats stats;

  bool ok() const noexcept { return status == common::Status::kSuccess; }
};

// 生成走廊。成功时的保证：
//   * 多边形严格凸、逆时针且顶点数 >= 3；
//   * 多边形 s 区间为正、有序、连续，并覆盖整个种子弧长；
//   * 相邻多边形以正面积重叠；
//   * 对每个路径点，保守整车足迹都包含在 s 区间覆盖该路径点的多边形内；
//   * 任何障碍物都不接触多边形（最小间隙见 `clearance_m`）；
//   * 多边形最小宽度至少为车辆宽度 + 2 * 安全裕度。
// 失败时走廊为空，并设置 `failure`/`reason`/`detail`。
CorridorResult generate_corridor(const CorridorRequest& request);

struct CorridorVerifyOptions {
  // 包含性检查的稠密路径重采样步长。
  double sample_step_m{0.02};
  // 包含/分离谓词允许的数值亏空。这只是浮点保护量，比安全裕度小几个
  // 数量级。
  double containment_tolerance_m{1e-6};
};

struct CorridorVerification {
  bool valid{false};
  // 稳定原因码，有效时为空。取值："invalid_input"、"invalid_options"、
  // "structural"、"polygon_not_convex_ccw"、"empty_s_interval"、
  // "negative_s_start"、"unordered_polygons"、"coverage"、"coverage_gap"、
  // "no_overlap"、"obstacle_intersection"、"obstacle_vertex_inside"、
  // "narrow"、"containment"。
  std::string reason;
  std::string detail;
  std::size_t failed_polygon{0};
  double s_m{0.0};
};

// 用产生走廊的同一组输入对其进行独立几何验证。只使用公开的走廊几何
// （不依赖生成器内部状态），因此可应用于任意来源的走廊。
CorridorVerification verify_corridor(const common::Corridor& corridor,
                                     const std::vector<common::PathSegment>& seed,
                                     const std::vector<Obstacle>& obstacles,
                                     const common::VehicleModel& vehicle,
                                     const CorridorVerifyOptions& options = {});

}  // namespace planner::corridor
