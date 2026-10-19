#pragma once

// planner_map：基于 Lanelet2 的地图导入、坐标、查询与裁剪
// （Wave 4：M1-M3）。
//
// 该类持有导入的地图快照。Lanelet2 对象只存在于实现中；公开接口不含
// ROS、Lanelet2 与浏览器类型。错误通过显式结果（common::Status）报告，
// 不使用 Lanelet2 异常类型。
//
// 线程安全：单个 Map 实例在并发修改其惰性构建的路由图时不安全。
// 本首版不支持多个线程并发执行 const 查询；调用方需自行串行化访问。

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <planner/common/types.hpp>

#include <planner/map/types.hpp>

namespace planner::map {

struct ImportResult;

// 导入的地图快照。仅可移动；由 import_osm_file() 获得。
class Map {
 public:
  // 以显式指定的投影和原点加载 Lanelet2 OSM 地图。
  //
  // 失败情形（status 为 kInvalidInput、reason 已填充、map == nullptr）：
  //   * 未知或不支持的文件扩展名（只接受 .osm/.xml）；
  //   * 文件缺失或无法读取；
  //   * 缺失或无效的投影原点；
  //   * 转换地图时投影失败；
  //   * OSM 解析失败（XML 格式错误、引用无法解析）；
  //   * 加载的地图不含任何点/lanelet/区域对象。
  // Lanelet2 可恢复的解析问题与验证发现会记录在 ImportResult::diagnostics，
  // 不算作加载失败。
  static ImportResult import_osm_file(const std::string& path,
                                      const MapLoadOptions& options);

  Map(const Map&) = delete;
  Map& operator=(const Map&) = delete;
  Map(Map&&) noexcept;
  Map& operator=(Map&&) noexcept;
  ~Map();

  const MapInfo& info() const noexcept;

  // ---------------------------------------------------------------------
  // 坐标（本地地图坐标系 <-> WGS84）。
  // 当值的投影失败或输入非有限时返回 std::nullopt。
  // ---------------------------------------------------------------------
  std::optional<GeoPoint> to_gps(double x_m, double y_m, double z_m = 0.0) const;
  std::optional<common::Point2d> to_local(const GeoPoint& gps) const;

  // 独立坐标辅助函数，投影模型与原点由参数显式给出；无需已加载的地图
  // （例如转换用户选择的 GPS 位姿或生成合成地图）。有效性与失败规则
  // 同 to_gps/to_local。
  static std::optional<common::Point2d> project_to_local(const GeoPoint& gps,
                                                         ProjectionKind projection,
                                                         const GeoPoint& origin);
  static std::optional<GeoPoint> project_to_gps(const common::Point2d& local,
                                                ProjectionKind projection,
                                                const GeoPoint& origin);

  // ---------------------------------------------------------------------
  // 对象查询。
  // ---------------------------------------------------------------------
  std::vector<ObjectId> lanelet_ids() const;  // 按数字 id 排序
  std::optional<LaneletGeometry> lanelet_geometry(const ObjectId& id) const;
  std::vector<ObjectId> area_ids() const;
  std::optional<AreaGeometry> area_geometry(const ObjectId& id) const;

  // 点层查询。结果按数字 id 排序并保留 id。
  std::vector<MapPoint> points_in_bounds(const Bounds2d& window) const;
  // 边界框与 `window` 相交的 lanelet（按 id 排序）。
  std::vector<ObjectId> lanelets_in_bounds(const Bounds2d& window) const;

  // 对象摘要与命中测试（供本地 HTTP 服务 W2 使用）。
  // query_point 返回覆盖该点的每个 lanelet/区域（含边界）；
  // query_bounds 返回边界框与窗口相交的每个 lanelet/区域。
  // 两者均按数字 id 排序。
  PointQueryResult query_point(double x_m, double y_m) const;
  BoundsQueryResult query_bounds(const Bounds2d& window) const;

  // ---------------------------------------------------------------------
  // 拓扑与路由。路由图在首次使用时按配置的交通规则惰性构建；
  // 构建失败时每个查询都会以 kInvalidInput 报告并附带原因。
  //
  // following/preceding/adjacent 沿 lanelet 存储的方向；双向 lanelet 的
  // 反向通行在 Lanelet2 中是独立的反向路由顶点（见 IdListResult）。
  // route() 跨越两个顶点解析连通性。
  // ---------------------------------------------------------------------
  IdListResult following(const ObjectId& lanelet_id) const;
  IdListResult preceding(const ObjectId& lanelet_id) const;
  IdListResult adjacent(const ObjectId& lanelet_id, NeighborSide side) const;
  RouteResult route(const ObjectId& from_lanelet, const ObjectId& to_lanelet) const;

  // ---------------------------------------------------------------------
  // 可行驶域栅格导出（W5 服务接入；不暴露 Lanelet2 类型）。
  //
  // 在局部地图坐标系的轴对齐窗口上导出一个只读占据栅格：单元矩形与任一
  // lanelet 的可行驶多边形（leftBound + 反向 rightBound）相交（闭集，接触
  // 也算）时为 kCellFree，其余为 kCellOccupied；从不产生 kCellUnknown。
  // 区域（area）对象不参与首版可行驶域，只有 lanelet 车道。
  //
  // 栅格遵循 common::GridView 的冻结约定（与 ROS OccupancyGrid 一致）：
  // origin 是单元 (0,0) 的左下角（窗口最小角，yaw = 0），单元 (r,c) 覆盖
  // [min + c*res, min + (c+1)*res] x [min + r*res, min + (r+1)*res]，
  // width = ceil((max_x-min_x)/res)、height = ceil((max_y-min_y)/res)，
  // 因此栅格覆盖 [min, min + 维度*res]，可能略超出窗口上界（< 1 个单元）。
  //
  // 确定性：同一地图快照、窗口与分辨率总是产生逐单元相同的栅格（单元标记
  // 是与 lanelet 遍历顺序无关的并集）。窗口与所有车道不相交（例如窗口完全
  // 在地图外）不是错误：返回全占据的合法栅格。
  //
  // 失败（status 为 kInvalidInput，reason 已填充，grid 保持无效）：
  //   * 窗口坐标非有限                       → "invalid_window"；
  //   * 窗口为空（min_x >= max_x 或 min_y >= max_y） → "empty_window"；
  //   * 分辨率非有限或 <= 0                  → "invalid_resolution"；
  //   * 单元数超过 kMaxDriveableGridCells    → "grid_too_large"。
  // ---------------------------------------------------------------------
  DriveableGridResult export_driveable_grid(const Bounds2d& window,
                                            double resolution_m) const;

  // ---------------------------------------------------------------------
  // 几何裁剪（局部窗口）。地图快照不被修改，拓扑查询仍在完整地图上进行。
  // 见 CropResult。
  // ---------------------------------------------------------------------
  CropResult crop_to_bounds(const Bounds2d& window) const;

  // ---------------------------------------------------------------------
  // 公开 JSON 地图快照（M4）。返回带通用封装的独立文档
  // （schema_version 1，type "map_snapshot"）：元数据、坐标系与投影原点、
  // 边界、对象计数，以及带字符串对象 id 与属性的 lanelet/区域几何，
  // 还有 regulatory element 引用。只输出有限数字；不暴露任何 Lanelet2
  // 类型。字段文档见 examples/data/README.md。
  // ---------------------------------------------------------------------
  nlohmann::json to_snapshot_json(const SnapshotOptions& options = {}) const;

 private:
  struct Impl;
  explicit Map(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

struct ImportResult {
  common::Status status{common::Status::kInvalidInput};
  std::string reason;
  // 非致命提示：可恢复的解析问题、验证发现与导入诊断，
  // 每条一行人类可读文本。
  std::vector<std::string> diagnostics;
  std::unique_ptr<Map> map;  // 当且仅当 status == kSuccess 时非空

  bool ok() const noexcept { return map != nullptr; }
};

}  // namespace planner::map
