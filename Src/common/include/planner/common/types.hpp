#pragma once

// 所有规划组件共享的第一版公开类型。
//
// 这些类型的冻结 JSON 契约（字段名、单位、枚举值和校验规则）位于
// tests/fixtures/v1/serialization/README.md。公开头文件保持不依赖 ROS、
// 不依赖 Lanelet2、不依赖浏览器。序列化本身位于
// planner/common/serialization.hpp。

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace planner::common {

// 每个文档顶层 "schema_version" 的整数值。
inline constexpr int kSchemaVersion = 1;

// pi 的双精度字面量，用于角度归一化。
inline constexpr double kPi = 3.14159265358979323846;

// GridView 栅格状态（GridView::cells 中按行主序的整数值）。
inline constexpr int kCellFree = 0;
inline constexpr int kCellOccupied = 1;
inline constexpr int kCellUnknown = 2;

// 将弧度制角度归一化到 [-pi, pi)。输入必须有限；
// JSON 编解码器在调用本函数前检查有限性。
double normalize_angle_rad(double angle_rad);

// 路径点/路径分段的行驶方向。
enum class Direction {
  kForward,
  kReverse,
};

// 各规划组件共享的结果状态。资源终止由任务运行器记录，
// 不得报告为 kNoPath。
enum class Status {
  kSuccess,
  kInvalidInput,
  kNoPath,
  kTimeout,
  kCancelled,
  kOptimizationFailed,
  kValidationFailed,
};

// JSON 编解码器使用的规范字符串值（"forward"、"success" 等）。
std::string to_string(Direction direction);
std::string to_string(Status status);

// 局部右手坐标系中的普通二维点（x 向前，y 向左）。
struct Point2d {
  double x_m{0.0};
  double y_m{0.0};
};

// 局部右手坐标系中的后轴中心位姿：x 向前，y 向左，偏航角逆时针为正，
// 由编解码器归一化到 [-pi, pi)。
struct Pose2d {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

// 矩形轮廓车辆模型。最小转弯半径由轴距和最大转向角推导。
struct VehicleModel {
  std::string id;
  double wheelbase_m{0.0};
  double width_m{0.0};
  double front_overhang_m{0.0};
  double rear_overhang_m{0.0};
  double max_steer_rad{0.0};
  double safety_margin_m{0.0};

  double min_turn_radius_m() const { return wheelbase_m / std::tan(max_steer_rad); }
};

// 只读占据栅格。`cells` 按行主序存储，取值 0=空闲、1=占据、2=未知；
// 消费方将未知和地图外的栅格视为占据。`origin` 是栅格原点在地图局部
// 坐标系中的位姿，即栅格 (row=0, col=0) 的左下角；栅格 (r, c) 的中心为
// origin 沿其 +x 方向平移 (c + 0.5)*resolution_m、沿 +y 方向平移
// (r + 0.5)*resolution_m 的位置，与 v1 测试夹具使用的 ROS
// OccupancyGrid 约定一致。
struct GridView {
  std::string map_id;
  std::string map_version;  // 不透明版本字符串，按相等性比较
  int width{0};
  int height{0};
  double resolution_m{0.0};
  Pose2d origin;
  std::vector<int> cells;

  bool in_bounds(int row, int col) const {
    return row >= 0 && row < height && col >= 0 && col < width;
  }

  // 返回 cells[row * width + col]；索引超出栅格时抛出 std::out_of_range。
  int cell(int row, int col) const;
};

// 单个路径点。`s_m` 是非负的累积弧长；带符号曲率满足
// yaw_rate = signed_speed * curvature，定义为 tan(steer) / wheelbase。
struct PathPoint {
  Pose2d pose;
  double s_m{0.0};
  double curvature_inv_m{0.0};  // tan(steer) / wheelbase，单位 1/m
  Direction direction{Direction::kForward};
};

// 方向一致的路径片段；方向变化总是开启新分段。`s_m` 在同一分段内严格递增。
struct PathSegment {
  std::vector<PathPoint> points;
};

// 单个采样的轨迹点。轨迹内时间严格递增；速度为带符号值，
// 在方向变化处为零。
struct TrajectoryPoint {
  double t_s{0.0};
  Pose2d pose;
  double velocity_mps{0.0};
  double acceleration_mps2{0.0};
  double curvature_inv_m{0.0};  // tan(steer) / wheelbase，单位 1/m
};

// 规划请求。map_id/map_version 标识地图快照；预算为可选（未设置 = 无限制）；
// `config` 是调用方定义的 JSON 对象，编解码器原样保留。
struct PlanRequest {
  std::string request_id;
  std::string map_id;
  std::string map_version;
  VehicleModel vehicle;
  Pose2d start;
  Pose2d goal;
  nlohmann::json config = nlohmann::json::object();
  std::optional<double> time_budget_s;
  std::optional<std::int64_t> node_budget;
  bool cancelled{false};
};

// 规划结果。`path`/`trajectory` 可能为空，取决于产出组件；
// `statistics` 是产出方定义的 JSON 对象，原样保留。
// 空数组等价于字段缺失。
struct PlanResult {
  std::string request_id;
  Status status{Status::kInvalidInput};
  std::string reason;
  std::vector<PathSegment> path;
  std::vector<TrajectoryPoint> trajectory;
  nlohmann::json statistics = nlohmann::json::object();
};

// 与路径弧长区间关联的有序凸多边形（>= 3 个顶点，在 `frame_id` 中逆时针）。
// 编解码器校验结构；凸性和整车包含性由走廊消费方检查。
struct CorridorPolygon {
  double s_start_m{0.0};
  double s_end_m{0.0};
  std::vector<Point2d> vertices;
};

// 安全走廊：同一坐标系中的有序凸自由空间多边形集合，
// 与地图快照和路径弧长参数化绑定。
struct Corridor {
  std::string id;
  std::string map_id;
  std::string map_version;
  std::string frame_id;
  std::vector<CorridorPolygon> polygons;
};

// 值相等比较，供往返测试和缓存使用。这里有意采用精确的 double 比较：
// 编解码器必须保留 double 的取值。
bool operator==(const Point2d& lhs, const Point2d& rhs);
bool operator==(const Pose2d& lhs, const Pose2d& rhs);
bool operator==(const VehicleModel& lhs, const VehicleModel& rhs);
bool operator==(const GridView& lhs, const GridView& rhs);
bool operator==(const PathPoint& lhs, const PathPoint& rhs);
bool operator==(const PathSegment& lhs, const PathSegment& rhs);
bool operator==(const TrajectoryPoint& lhs, const TrajectoryPoint& rhs);
bool operator==(const PlanRequest& lhs, const PlanRequest& rhs);
bool operator==(const PlanResult& lhs, const PlanResult& rhs);
bool operator==(const CorridorPolygon& lhs, const CorridorPolygon& rhs);
bool operator==(const Corridor& lhs, const Corridor& rhs);

}  // namespace planner::common
