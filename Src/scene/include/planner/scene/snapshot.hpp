#pragma once

// grid_snapshot 文档组装与 base64_u8 单元编码（栅格快照协议 v1 §2，X-M1）。
//
// 地图服务栅格/场景预览与规划服务输入快照共用本组装函数，保证字段集合、
// 单元编码与数值语义一致（协议 §2.1/§2.3）。`window`、分辨率由调用方传入
// 有效（量化后）值；`degraded` 由 effective/requested 派生，避免调用方漂移。
//
// 快照 JSON 的文本布局不是契约（协议 §2.4）；本函数使用 nlohmann::json
// 对象（键排序）保证确定性。

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/common/types.hpp"
#include "planner/map/types.hpp"

namespace planner::scene {

// 协议版本（协议 §7；/health 的 grid_protocol_version 同值）。
inline constexpr const char* kGridProtocolVersion = "1.0";

// 三态单元流的 base64 编码（RFC 4648 §4：标准字母表 + '=' 填充，无空白）。
// 单元值必须 ∈ {0,1,2}，否则抛出 std::invalid_argument（调用方先校验栅格）。
std::string encode_cells_base64(const std::vector<int>& cells);
std::string encode_cells_base64(const common::GridView& grid);

// grid_snapshot 组装的输入选项。
struct GridSnapshotOptions {
  // "driveable" | "planning_scene"（服务端不产生 "fixture"）。
  std::string source;
  // planning_scene 必填；driveable 留空（不输出字段）。
  std::string scene_id;
  std::string scene_version;
  // 有效窗口（量化后；driveable = 请求窗口，planning_scene = 裁剪后窗口）。
  map::Bounds2d window;
  // 请求分辨率（量化后）。
  double requested_resolution_m{0.0};
  // 本次生效的单元预算（默认与地图组件 kMaxDriveableGridCells 同值）。
  std::size_t budget_cells{16u * 1024u * 1024u};
  // 可选请求序号回显（协议 §6；规划服务快照不携带）。
  std::optional<long long> request_seq;
};

// 组装协议 §2.1 的 grid_snapshot 文档（含公共包络 schema_version/type）。
nlohmann::json make_grid_snapshot_json(const common::GridView& grid,
                                       const GridSnapshotOptions& options);

}  // namespace planner::scene
