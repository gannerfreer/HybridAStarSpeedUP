#pragma once

// planner map 组件的公开值类型。
//
// 硬边界（方案 3.6 节）：这些头文件中绝不出现 Lanelet2、ROS 或浏览器类型。
// 对象用原始 OSM id 的十进制字符串标识，使 id 在 JSON 往返后不丢失整数精度
// （通用契约，tests/fixtures/v1/serialization）。
//
// 坐标：本地米制地图坐标系，右手系，单位米，x 向东、y 向北（由配置的投影器
// 生成的坐标系）。裁剪输出窗口局部坐标系以及显式变换回地图坐标的转换；
// 当点来自地图时，裁剪后的几何保留原始点 id。由边裁剪产生的点带有空 id，
// 文档中记为合成点。几何裁剪从不改变拓扑：裁剪边会被标记，不应被解释为
// 道路端点。

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <planner/common/types.hpp>

namespace planner::map {

// 原始对象 id（OSM id），以十进制字符串原样保留。
using ObjectId = std::string;

// WGS84 大地坐标；高程是可选的上游数据。
struct GeoPoint {
  double lat_deg{0.0};
  double lon_deg{0.0};
  double ele_m{0.0};
};

// 支持的投影模型（在导入时显式配置）。
enum class ProjectionKind {
  kUtm,                // GeographicLib UTM 投影器（推荐的默认值）
  kMercator,           // 椭球 Mercator（lanelet2_projection）
  kSphericalMercator,  // 无依赖的球面 Mercator（lanelet2_io）
};

// 规范的小写名称（"utm"、"mercator"、"spherical_mercator"）。
std::string to_string(ProjectionKind kind);
std::optional<ProjectionKind> projection_kind_from_string(const std::string& name);

// 本地米制地图坐标系中的轴对齐矩形。
struct Bounds2d {
  double min_x_m{0.0};
  double min_y_m{0.0};
  double max_x_m{0.0};
  double max_y_m{0.0};

  bool contains(double x_m, double y_m) const noexcept;
  bool intersects(const Bounds2d& other) const noexcept;
  bool empty() const noexcept;
};

// 导入地图的对象计数（元数据 / 验证统计）。
struct MapObjectCounts {
  std::size_t points{0};
  std::size_t line_strings{0};
  std::size_t polygons{0};
  std::size_t lanelets{0};
  std::size_t areas{0};
  std::size_t regulatory_elements{0};

  std::size_t total() const noexcept;
};

// 导入时验证运行的摘要（lanelet2_validation 检查）。
struct ValidationStats {
  std::size_t checks_run{0};
  std::size_t errors{0};
  std::size_t warnings{0};
};

// 导入配置。投影原点为必填项：特意不接受 Lanelet2 的默认原点，
// 以保证每个坐标框架都显式且可复现。
struct MapLoadOptions {
  ProjectionKind projection{ProjectionKind::kUtm};
  std::optional<GeoPoint> origin;
  bool validate_on_import{true};
  // 路由/邻接查询使用的交通规则；取值必须已在 lanelet2 中注册
  // （固定版本的上游提供 location "de" 与 participant "vehicle"）。
  std::string traffic_rules_location{"de"};
  std::string traffic_rules_participant{"vehicle"};
};

// 导入成功后生成的地图元数据。
struct MapInfo {
  std::string map_id;        // 源文件主干名
  std::string map_version;   // 源文件字节的 SHA-256 十六进制值
  std::string source_path;
  std::size_t source_bytes{0};
  ProjectionKind projection{ProjectionKind::kUtm};
  GeoPoint origin;
  std::string traffic_rules_location;
  std::string traffic_rules_participant;
  MapObjectCounts counts;
  Bounds2d bounds_m;  // 点层在本地地图坐标系中的边界
  ValidationStats validation;
};

// 本地地图坐标系中的一个点。由几何裁剪合成的点 `id` 为空。
struct MapPoint {
  ObjectId id;
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
};

// 保留对象 id 的 lanelet 几何。`attributes` 对应 OSM 标签
// （值保持字符串，数字不会被隐式转换）。
struct LaneletGeometry {
  ObjectId id;
  std::vector<MapPoint> left;
  std::vector<MapPoint> right;
  std::vector<MapPoint> centerline;
  nlohmann::json attributes = nlohmann::json::object();
};

// 区域几何；外环由其线串拼接而成，每个内环条目是一个孔洞环。
struct AreaGeometry {
  ObjectId id;
  std::vector<MapPoint> outer;
  std::vector<std::vector<MapPoint>> inner;
  nlohmann::json attributes = nlohmann::json::object();
};

enum class NeighborSide {
  kLeft,
  kRight,
};

// 拓扑查询的结果。status 为 kSuccess 时 ids 可能为空（没有邻居）；
// kInvalidInput 表示未知 id 或路由图不可用，原因见 `reason`。
//
// 方向性：`following`/`preceding` 沿 lanelet 存储的方向。Lanelet2 将双向
// lanelet 的反向通行建模为独立的（反向）路由顶点，因此从 A 找到的后继
// 不一定把 A 列为其前驱（该连接属于反向顶点）。`route` 跨越两个顶点
// 解析连通性。
struct IdListResult {
  common::Status status{common::Status::kInvalidInput};
  std::string reason;
  std::vector<ObjectId> ids;
};

// 路由查询的结果。`lanelet_ids` 是有序的最短 lanelet 序列；
// `length_m` 是 Lanelet2 路由计算的二维路径长度。
struct RouteResult {
  common::Status status{common::Status::kInvalidInput};
  std::string reason;
  std::vector<ObjectId> lanelet_ids;
  double length_m{0.0};
};

// lanelet 边界或中心线的一段连续裁剪片段。
// * 原始地图点保留其 id；裁剪点 id 为空。
// * 当该端被窗口切断时，`start_clip_edge` / `end_clip_edge` 为 true
//   （这是几何边，不是道路端点）。拓扑刻意不从裁剪后的几何推导；
//   请在完整地图上查询拓扑。
struct ClippedPath {
  ObjectId lanelet_id;
  std::string role;  // "left"、"right" 或 "centerline"
  std::vector<MapPoint> points;
  bool start_clip_edge{false};
  bool end_clip_edge{false};
};

// 局部窗口裁剪结果。仅含几何：`lanelet_ids` 列出边界框与窗口相交的
// lanelet（窗口的语义引用），`paths` 与 `points` 携带裁剪/截断后的几何。
// 窗口局部坐标系与地图坐标系轴对齐，原点位于窗口最小角，比例 1 m
// （仅平移）；用 local_to_map() / map_to_local() 转换。
struct CropResult {
  Bounds2d window_m;
  common::Pose2d window_frame;  // 窗口局部坐标系原点的地图坐标
  std::vector<ClippedPath> paths;
  std::vector<MapPoint> points;
  std::vector<ObjectId> lanelet_ids;
  // 非致命提示（例如某 lanelet 的中心线无法计算）。
  std::vector<std::string> diagnostics;

  common::Point2d local_to_map(const common::Point2d& p) const noexcept;
  common::Point2d map_to_local(const common::Point2d& p) const noexcept;
  bool empty() const noexcept;
};

// 可行驶域栅格导出的单元数上限（Map::export_driveable_grid）。
// 16M 个 int 单元 = 64 MiB；更大的窗口必须提高分辨率后重试。
inline constexpr std::size_t kMaxDriveableGridCells = 16u * 1024u * 1024u;

// 可行驶域栅格导出结果（见 Map::export_driveable_grid）。
// `status == common::Status::kSuccess` 时 `grid` 有效；否则 `reason` 为稳定
// 失败码："invalid_window"、"empty_window"、"invalid_resolution" 或
// "grid_too_large"。
struct DriveableGridResult {
  common::Status status{common::Status::kInvalidInput};
  std::string reason;
  common::GridView grid;
  bool ok() const noexcept { return status == common::Status::kSuccess; }
};

// 公开 JSON 地图快照的内容选择（M4）。该快照是组件文档类型
// （"map_snapshot"，schema_version 1），遵循通用封装规则：对象 id 为字符串、
// 枚举值为字符串、数字有限、不含 Lanelet2 类型。UI 消费它时无需解析
// Lanelet2 对象。
struct SnapshotOptions {
  bool include_centerline{true};
  bool include_areas{true};
  bool include_regulatory_elements{true};
};

// 用于命中测试和窗口摘要的紧凑对象描述（W2 服务）：
// 原始 id、OSM 属性（字符串值）和对象的边界框。
struct ObjectSummary {
  ObjectId id;
  nlohmann::json attributes = nlohmann::json::object();
  Bounds2d bounds_m;
  bool has_bounds{false};
};

// 点命中测试：二维多边形覆盖该点的 lanelet/区域
// （边界算作内部，与 lanelet2 geometry::inside 一致）。
struct PointQueryResult {
  std::vector<ObjectSummary> lanelets;
  std::vector<ObjectSummary> areas;
};

// 边界框查询：边界框与窗口相交的 lanelet/区域
// （语义与 lanelets_in_bounds 相同）。
struct BoundsQueryResult {
  std::vector<ObjectSummary> lanelets;
  std::vector<ObjectSummary> areas;
};

}  // namespace planner::map
