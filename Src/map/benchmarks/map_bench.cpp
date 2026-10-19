// planner_map_bench --suite v1 --output-dir <dir> [--generate-only]
//
// 正式的地图微基准测试（Docs/component_development_plan.md 7.2/7.3；
// spec 优先 tests/fixtures/v5/benchmarks/map/specs.json，v4/v3/v2/v1 为回退；
// 输出契约见 tools/README.md 第 7.2 节 "基准输出契约"）。
//
// 按 spec 的协议：确定性的合成 lanelet 网格（默认生成 spec 中所有非
// pending_generation 的规模：v4 及更早为 small/medium，v5 起包含 large），
// 每个操作 5 个预热批次和 30 个测量批次，单线程，查询/裁剪/路由/加载分别
// 计时。生成、证据收集和 CSV/JSON/SVG 输出不属于测量批次。
//
// --generate-only：只做确定性生成并写出 synthetic_maps/<scale>.osm /
// .sha256 / .meta.json 后退出，不执行任何测量批次（用于物化 large 输入并
// 按 §9 建立 SHA256 寻址归档）。若 spec 中该规模的 archive.sha256 非空，
// 生成后校验生成文件 SHA256 与之一致，不一致时报错并以非零退出码结束，
// 不静默继续。显式 `--scale <name> --generate-only` 允许物化 spec 中仍为
// pending_generation 的规模（物化正是 pending 状态的解除动作）；测量模式
// 仍拒绝 pending_generation。
//
// spec 解读（记录在摘要和报告中）：v4 起 spec 自洽（lanelet_length_m =
// 25.0 = x_pitch_m）；v1/v2 旧 spec 的 lanelet_length_m = 20.0 与
// x_pitch_m = 25.0 矛盾。无论 spec 版本，生成的 lanelet 长度取 x pitch
// （25 m）并共享边界节点——Lanelet2 路由的后继检测要求相邻 lanelet 共享
// 端点对象（geometry::follows）——从而保持范围、行列数和所有窗口/路由
// 公式不变；其他所有参数（宽度、y pitch、属性、拓扑和 id 规则）保持不变。
// 解析到的 spec 实际值写入 .meta.json 与 bench_summary.json 的解读字段。
//
// --output-dir 下的输出（绝不写到目录之外）：
//   benchmark_metrics.csv        每个测量批次一行（7.2 固定列
//                                + map 组件列）
//   bench_summary.json / .md     聚合结果：P50/P95、吞吐量、内存、
//                                正确性检查、spec 来源
//   plots/p50_p95_<scale>.svg/.csv  批次时延图表（SVG 中为文本数值）
//   synthetic_maps/<scale>.osm   确定性生成的输入
//   synthetic_maps/<scale>.sha256 / .meta.json  输入哈希 + 生成规则
//   crop_evidence.json           第一个测量批次的逐窗口记录
//   route_evidence.json          第一个测量批次的逐对记录
//   query_evidence.json          第一个测量批次的逐查询计数
//
// 退出码：0 成功；1 正确性/一致性检查失败；2 用法错误。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/map/map.hpp"

#include "../src/sha256.hpp"

namespace {

using planner::common::Point2d;
using planner::map::Bounds2d;
using planner::map::GeoPoint;
using planner::map::Map;
using planner::map::MapLoadOptions;
using planner::map::MapObjectCounts;
using planner::map::ObjectId;
using planner::map::ProjectionKind;

// 生成器的确定性 OSM id 方案（记录在 .meta.json 中）：
//   nodes:    1 + 2*(row*(columns+1) + column)  （北/左），+1（南/右）
//   ways:     10'000'000 + 2*k（左），+1（右）
//   relation: 20'000'000 + k + 1
constexpr std::uint64_t kWayIdBase = 10000000ULL;
constexpr std::uint64_t kRelationIdBase = 20000000ULL;
constexpr double kLaneletWidthM = 3.5;
constexpr double kYPitchM = 25.0;
constexpr double kXPitchM = 25.0;  // 生成的 lanelet 长度（见文件头说明）
constexpr double kCropWindowSizeM = 50.0;
constexpr double kQueryExpansionM = 5.0;
constexpr std::uint64_t kSeed = 0;

GeoPoint bench_origin() { return GeoPoint{49.0, 8.4, 0.0}; }

std::string id_for_index(std::uint64_t index) {
  return std::to_string(kRelationIdBase + index + 1);
}

std::optional<std::uint64_t> index_for_id(const ObjectId& id) {
  try {
    std::size_t pos = 0;
    const std::uint64_t value = std::stoull(id, &pos);
    if (pos != id.size() || value <= kRelationIdBase) {
      return std::nullopt;
    }
    return value - kRelationIdBase - 1;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::string logical_id(std::uint64_t index) { return "ll_" + std::to_string(index); }

MapLoadOptions load_options(bool validate) {
  MapLoadOptions options;
  options.projection = ProjectionKind::kUtm;
  options.origin = bench_origin();
  options.validate_on_import = validate;
  return options;
}

struct ScaleSpec {
  std::string name;
  std::uint64_t lanelets{0};
  std::uint64_t columns{0};
  std::uint64_t rows{0};
  double crop_center_x_base{0.0};
  double crop_center_y_base{0.0};
  double crop_center_x_mod{0.0};
  double crop_center_y_mod{0.0};
  std::string status;
  // spec 的 archive 块（§9）：sha256 / relative_path 非空时在生成后校验并
  // 记录到 .meta.json 与 bench_summary.json。
  std::string archive_sha256;
  std::string archive_relative_path;
  std::string archive_note;
};

// 裁剪窗口公式是 spec 文本中的字面常量（并非由范围推导）；
// 必须原样保留。
ScaleSpec scale_from_spec(const nlohmann::json& map_entry) {
  ScaleSpec scale;
  scale.name = map_entry.at("scale").get<std::string>();
  scale.lanelets = map_entry.at("lanelet_count").get<std::uint64_t>();
  scale.columns = map_entry.at("columns").get<std::uint64_t>();
  scale.rows = map_entry.at("rows").get<std::uint64_t>();
  scale.status = map_entry.value("status", "rule_defined");
  if (map_entry.contains("archive") && map_entry.at("archive").is_object()) {
    const auto& archive = map_entry.at("archive");
    if (archive.contains("sha256") && archive.at("sha256").is_string()) {
      scale.archive_sha256 = archive.at("sha256").get<std::string>();
    }
    if (archive.contains("relative_path") && archive.at("relative_path").is_string()) {
      scale.archive_relative_path = archive.at("relative_path").get<std::string>();
    }
    if (archive.contains("note") && archive.at("note").is_string()) {
      scale.archive_note = archive.at("note").get<std::string>();
    }
  }
  if (scale.name == "small") {
    scale.crop_center_x_base = 75.0;
    scale.crop_center_y_base = 75.0;
    scale.crop_center_x_mod = 850.0;
    scale.crop_center_y_mod = 475.0;
  } else if (scale.name == "medium") {
    scale.crop_center_x_base = 375.0;
    scale.crop_center_y_base = 75.0;
    scale.crop_center_x_mod = 4250.0;
    scale.crop_center_y_mod = 1100.0;
  } else if (scale.name == "large") {
    scale.crop_center_x_base = 375.0;
    scale.crop_center_y_base = 75.0;
    scale.crop_center_x_mod = 11750.0;
    scale.crop_center_y_mod = 4850.0;
  }
  return scale;
}

// ---------------------------------------------------------------------------
// 确定性合成 OSM 生成。
// ---------------------------------------------------------------------------

std::string generate_osm_text(const ScaleSpec& scale, std::string* error) {
  std::ostringstream out;
  out << std::setprecision(17);
  out << "<?xml version='1.0' encoding='UTF-8'?>\n"
      << "<osm version='0.6' generator='planner_map_bench'>\n";
  const std::uint64_t cols = scale.columns;
  const std::uint64_t rows = scale.rows;
  for (std::uint64_t row = 0; row < rows; ++row) {
    const double y_south = row * kYPitchM;
    const double y_north = y_south + kLaneletWidthM;
    for (std::uint64_t column = 0; column <= cols; ++column) {
      const double x = column * kXPitchM;
      const auto north = Map::project_to_gps(Point2d{x, y_north}, ProjectionKind::kUtm,
                                             bench_origin());
      const auto south = Map::project_to_gps(Point2d{x, y_south}, ProjectionKind::kUtm,
                                             bench_origin());
      if (!north || !south) {
        *error = "projection failed while generating synthetic nodes";
        return {};
      }
      const std::uint64_t point_index = row * (cols + 1) + column;
      out << "  <node id='" << (1 + 2 * point_index) << "' lat='" << north->lat_deg
          << "' lon='" << north->lon_deg << "'/>\n";
      out << "  <node id='" << (2 + 2 * point_index) << "' lat='" << south->lat_deg
          << "' lon='" << south->lon_deg << "'/>\n";
    }
  }
  for (std::uint64_t k = 0; k < scale.lanelets; ++k) {
    const std::uint64_t row = k / cols;
    const std::uint64_t column = k % cols;
    const std::uint64_t p0 = row * (cols + 1) + column;
    const std::uint64_t p1 = p0 + 1;
    const std::uint64_t left_way = kWayIdBase + 2 * k;
    const std::uint64_t right_way = left_way + 1;
    out << "  <way id='" << left_way << "'>"
        << "<nd ref='" << (1 + 2 * p0) << "'/><nd ref='" << (1 + 2 * p1) << "'/>"
        << "<tag k='type' v='line_thin'/><tag k='subtype' v='solid'/></way>\n";
    out << "  <way id='" << right_way << "'>"
        << "<nd ref='" << (2 + 2 * p0) << "'/><nd ref='" << (2 + 2 * p1) << "'/>"
        << "<tag k='type' v='line_thin'/><tag k='subtype' v='solid'/></way>\n";
    const std::uint64_t speed_limit = 30 + 10 * (k % 3);
    out << "  <relation id='" << (kRelationIdBase + k + 1) << "'>\n"
        << "    <member type='way' ref='" << left_way << "' role='left'/>\n"
        << "    <member type='way' ref='" << right_way << "' role='right'/>\n"
        << "    <tag k='type' v='lanelet'/>\n"
        << "    <tag k='subtype' v='road'/>\n"
        << "    <tag k='one_way' v='yes'/>\n"
        << "    <tag k='location' v='urban'/>\n"
        << "    <tag k='region' v='de'/>\n"
        << "    <tag k='speed_limit_kph' v='" << speed_limit << "'/>\n"
        << "  </relation>\n";
  }
  out << "</osm>\n";
  return out.str();
}

// ---------------------------------------------------------------------------
// 计时与统计。
// ---------------------------------------------------------------------------

using Clock = std::chrono::steady_clock;

struct Trial {
  double wall_ms{0.0};
  double cpu_ms{0.0};
};

template <typename Fn>
Trial run_trial(Fn&& fn) {
  const Clock::time_point start = Clock::now();
  const std::clock_t cpu_start = std::clock();
  fn();
  const std::clock_t cpu_end = std::clock();
  const Clock::time_point end = Clock::now();
  Trial trial;
  trial.wall_ms = std::chrono::duration<double, std::milli>(end - start).count();
  trial.cpu_ms = 1000.0 * static_cast<double>(cpu_end - cpu_start) / CLOCKS_PER_SEC;
  return trial;
}

// 最近秩百分位数（7.2）。
double percentile(const std::vector<double>& sorted, double percent) {
  if (sorted.empty()) {
    return 0.0;
  }
  const std::size_t rank = std::max<std::size_t>(
      1, std::min<std::size_t>(sorted.size(),
                               static_cast<std::size_t>(
                                   std::ceil(percent / 100.0 * sorted.size()))));
  return sorted[rank - 1];
}

struct Stats {
  double p50_ms{0.0};
  double p95_ms{0.0};
  double mean_ms{0.0};
  double min_ms{0.0};
  double max_ms{0.0};
};

Stats summarize(std::vector<double> values_ms) {
  Stats stats;
  if (values_ms.empty()) {
    return stats;
  }
  std::sort(values_ms.begin(), values_ms.end());
  stats.min_ms = values_ms.front();
  stats.max_ms = values_ms.back();
  stats.p50_ms = percentile(values_ms, 50.0);
  stats.p95_ms = percentile(values_ms, 95.0);
  double sum = 0.0;
  for (const double value : values_ms) {
    sum += value;
  }
  stats.mean_ms = sum / static_cast<double>(values_ms.size());
  return stats;
}

std::uint64_t read_vm_hwm_bytes() {
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    if (line.rfind("VmHWM:", 0) == 0) {
      std::istringstream in(line.substr(6));
      std::uint64_t kb = 0;
      in >> kb;
      return kb * 1024ULL;
    }
  }
  return 0;
}

std::string format_number(double value) {
  std::ostringstream out;
  out << std::setprecision(12) << value;
  return out.str();
}

// ---------------------------------------------------------------------------
// 测量记录。
// ---------------------------------------------------------------------------

struct Row {
  std::string case_id;
  std::string cache_mode{"warm"};
  int repeat_id{0};
  std::string status{"success"};
  std::string failure_reason;
  double wall_ms{0.0};
  double cpu_ms{0.0};
  std::uint64_t peak_rss_bytes{0};
  bool has_path_length{false};
  double path_length_m{0.0};
  bool valid{true};
  std::string scale;
  std::string operation;
  int batch_size{1};
  double avg_op_ms{0.0};
  double ops_per_second{0.0};
  bool has_counts{false};
  std::uint64_t count_sum{0};
  std::uint64_t count_min{0};
  std::uint64_t count_max{0};
  std::uint64_t route_mismatch{0};
  std::string notes;
};

void write_benchmark_metrics(const std::filesystem::path& path, const std::string& run_id,
                             const std::vector<Row>& rows) {
  std::ofstream out(path);
  out << "run_id,case_id,repeat_id,cache_mode,seed,status,failure_reason,"
         "preprocess_ms,wall_ms,cpu_ms,peak_rss_bytes,expanded_nodes,"
         "collision_checks,path_length_m,gear_changes,endpoint_position_error_m,"
         "endpoint_yaw_error_rad,valid,scale,operation,batch_size,avg_op_ms,"
         "ops_per_second,result_count_sum,result_count_min,result_count_max,"
         "route_lanelets_mismatch,notes\n";
  for (const Row& row : rows) {
    out << run_id << ',' << row.case_id << ',' << row.repeat_id << ',' << row.cache_mode << ','
        << kSeed << ',' << row.status << ',' << row.failure_reason << ',' << ','
        << format_number(row.wall_ms) << ',' << format_number(row.cpu_ms) << ','
        << row.peak_rss_bytes
        << ",,,"
        << (row.has_path_length ? format_number(row.path_length_m) : std::string())
        << ",,,,"
        << (row.valid ? "true" : "false") << ',' << row.scale << ',' << row.operation << ','
        << row.batch_size << ',' << format_number(row.avg_op_ms) << ','
        << format_number(row.ops_per_second) << ','
        << (row.has_counts ? std::to_string(row.count_sum) : std::string()) << ','
        << (row.has_counts ? std::to_string(row.count_min) : std::string()) << ','
        << (row.has_counts ? std::to_string(row.count_max) : std::string()) << ','
        << row.route_mismatch << ',' << row.notes << '\n';
  }
}

struct BatchResult {
  std::uint64_t count_sum{0};
  std::uint64_t count_min{0};
  std::uint64_t count_max{0};
  std::uint64_t mismatch{0};
  bool has_counts{false};
  double aux_sum{0.0};  // 例如路由长度之和
};

void account(BatchResult* result, std::uint64_t count) {
  if (!result->has_counts) {
    result->has_counts = true;
    result->count_min = count;
    result->count_max = count;
  }
  result->count_sum += count;
  result->count_min = std::min(result->count_min, count);
  result->count_max = std::max(result->count_max, count);
}

// ---------------------------------------------------------------------------
// 操作。
// ---------------------------------------------------------------------------

BatchResult run_query_batch(const Map& map, const ScaleSpec& scale, std::uint64_t queries,
                            nlohmann::json* evidence) {
  BatchResult result;
  const std::uint64_t n = scale.lanelets;
  std::uint64_t attribute_mismatch = 0;
  for (std::uint64_t j = 0; j < queries; ++j) {
    const std::uint64_t kind = j % 5;
    std::uint64_t index = 0;
    std::uint64_t count = 0;
    switch (kind) {
      case 0: {  // 边界查询
        index = (7 * j + 3) % n;
        const auto geometry = map.lanelet_geometry(id_for_index(index));
        count = geometry ? geometry->left.size() + geometry->right.size() : 0;
        break;
      }
      case 1: {  // 中心线查询
        index = (13 * j + 5) % n;
        const auto geometry = map.lanelet_geometry(id_for_index(index));
        count = geometry ? geometry->centerline.size() : 0;
        break;
      }
      case 2: {  // 区域查询：lanelet 矩形按 5 m 扩展
        index = (17 * j + 7) % n;
        const std::uint64_t row = index / scale.columns;
        const std::uint64_t column = index % scale.columns;
        const Bounds2d window{column * kXPitchM - kQueryExpansionM,
                              row * kYPitchM - kQueryExpansionM,
                              (column + 1) * kXPitchM + kQueryExpansionM,
                              row * kYPitchM + kLaneletWidthM + kQueryExpansionM};
        count = map.lanelets_in_bounds(window).size();
        break;
      }
      case 3: {  // 属性查询
        index = (23 * j + 11) % n;
        const auto geometry = map.lanelet_geometry(id_for_index(index));
        if (geometry && geometry->attributes.is_object() &&
            geometry->attributes.contains("speed_limit_kph")) {
          count = 1;
          const std::uint64_t expected = 30 + 10 * (index % 3);
          if (geometry->attributes["speed_limit_kph"].get<std::string>() !=
              std::to_string(expected)) {
            ++attribute_mismatch;
          }
        }
        break;
      }
      default: {  // 邻接查询
        index = (29 * j + 13) % n;
        const ObjectId id = id_for_index(index);
        const auto preceding = map.preceding(id);
        const auto following = map.following(id);
        count = (preceding.status == planner::common::Status::kSuccess ? preceding.ids.size() : 0) +
                (following.status == planner::common::Status::kSuccess ? following.ids.size() : 0);
        break;
      }
    }
    account(&result, count);
    if (evidence != nullptr) {
      evidence->push_back(nlohmann::json{{"j", j},
                                         {"kind", kind},
                                         {"index", index},
                                         {"logical_id", logical_id(index)},
                                         {"count", count}});
    }
  }
  result.mismatch = attribute_mismatch;
  return result;
}

BatchResult run_crop_batch(const Map& map, const ScaleSpec& scale, std::uint64_t windows,
                           nlohmann::json* evidence) {
  BatchResult result;
  double max_outside = 0.0;
  double max_transform_error = 0.0;
  for (std::uint64_t w = 0; w < windows; ++w) {
    const double cx = scale.crop_center_x_base +
                      std::fmod(137.0 * static_cast<double>(w), scale.crop_center_x_mod);
    const double cy = scale.crop_center_y_base +
                      std::fmod(211.0 * static_cast<double>(w), scale.crop_center_y_mod);
    const double half = kCropWindowSizeM / 2.0;
    const Bounds2d window{cx - half, cy - half, cx + half, cy + half};
    const auto crop = map.crop_to_bounds(window);
    account(&result, crop.lanelet_ids.size());
    for (const auto& path : crop.paths) {
      for (const auto& point : path.points) {
        max_outside = std::max(max_outside, std::max(0.0, window.min_x_m - point.x_m));
        max_outside = std::max(max_outside, std::max(0.0, point.x_m - window.max_x_m));
        max_outside = std::max(max_outside, std::max(0.0, window.min_y_m - point.y_m));
        max_outside = std::max(max_outside, std::max(0.0, point.y_m - window.max_y_m));
      }
    }
    const auto corner_map = crop.local_to_map(Point2d{0.0, 0.0});
    const auto corner_back = crop.map_to_local(corner_map);
    max_transform_error =
        std::max(max_transform_error, std::abs(corner_back.x_m) + std::abs(corner_back.y_m));
    max_transform_error = std::max(
        max_transform_error, std::abs(corner_map.x_m - window.min_x_m) +
                                 std::abs(corner_map.y_m - window.min_y_m));
    if (evidence != nullptr) {
      nlohmann::json ids = nlohmann::json::array();
      for (const auto& id : crop.lanelet_ids) {
        const auto index = index_for_id(id);
        ids.push_back(index ? logical_id(*index) : id);
      }
      evidence->push_back(nlohmann::json{{"w", w},
                                         {"window_m", nlohmann::json::array({window.min_x_m,
                                                                             window.min_y_m,
                                                                             window.max_x_m,
                                                                             window.max_y_m})},
                                         {"lanelet_ids", std::move(ids)},
                                         {"paths", crop.paths.size()},
                                         {"points", crop.points.size()}});
    }
  }
  if (max_outside > 1e-9 || max_transform_error > 1e-9) {
    ++result.mismatch;
  }
  result.aux_sum = max_outside + max_transform_error;
  return result;
}

BatchResult run_route_batch(const Map& map, const ScaleSpec& scale, std::uint64_t pairs,
                            nlohmann::json* evidence) {
  BatchResult result;
  double length_sum = 0.0;
  for (std::uint64_t r = 0; r < pairs; ++r) {
    const std::uint64_t row = (7 * r) % scale.rows;
    const std::uint64_t half_columns = scale.columns / 2;
    const std::uint64_t col_start = (3 * r) % half_columns;
    const std::uint64_t quarter = scale.columns / 4;
    const std::uint64_t delta = quarter + (r % quarter);
    const std::uint64_t col_end = col_start + delta;
    const std::uint64_t start_index = row * scale.columns + col_start;
    const std::uint64_t end_index = row * scale.columns + col_end;
    const ObjectId start_id = id_for_index(start_index);
    const ObjectId end_id = id_for_index(end_index);
    const auto route = map.route(start_id, end_id);
    const std::uint64_t expected = delta + 1;
    const std::uint64_t actual =
        route.status == planner::common::Status::kSuccess ? route.lanelet_ids.size() : 0;
    if (actual != expected || (actual > 0 && (route.lanelet_ids.front() != start_id ||
                                              route.lanelet_ids.back() != end_id))) {
      ++result.mismatch;
    }
    account(&result, actual);
    length_sum += route.length_m;
    if (evidence != nullptr) {
      evidence->push_back(nlohmann::json{{"r", r},
                                         {"start", logical_id(start_index)},
                                         {"end", logical_id(end_index)},
                                         {"expected_lanelets", expected},
                                         {"actual_lanelets", actual},
                                         {"length_m", route.length_m},
                                         {"status", planner::common::to_string(route.status)}});
    }
  }
  result.aux_sum = length_sum;
  return result;
}

struct LoadResult {
  Trial trial;
  std::uint64_t lanelets{0};
  bool ok{false};
  std::string error;
};

LoadResult run_load(const std::string& path, const ScaleSpec& scale) {
  LoadResult result;
  result.trial = run_trial([&]() {
    auto import = Map::import_osm_file(path, load_options(false));
    if (!import.ok()) {
      result.error = import.reason;
      return;
    }
    result.lanelets = import.map->info().counts.lanelets;
  });
  result.ok = result.error.empty() && result.lanelets == scale.lanelets;
  if (result.ok == false && result.error.empty()) {
    result.error = "lanelet count mismatch: expected " + std::to_string(scale.lanelets) +
                   ", got " + std::to_string(result.lanelets);
  }
  return result;
}

struct GeometryCheck {
  double max_error_m{0.0};
  std::uint64_t checked{0};
};

GeometryCheck verify_geometry(const Map& map, const ScaleSpec& scale,
                              std::uint64_t sample_count) {
  GeometryCheck check;
  const std::uint64_t n = std::min<std::uint64_t>(sample_count, scale.lanelets);
  for (std::uint64_t i = 0; i < n; ++i) {
    const std::uint64_t k = (i * 7919ULL) % scale.lanelets;
    const auto geometry = map.lanelet_geometry(id_for_index(k));
    if (!geometry) {
      check.max_error_m = 1e9;
      continue;
    }
    const std::uint64_t row = k / scale.columns;
    const std::uint64_t column = k % scale.columns;
    const double x0 = column * kXPitchM;
    const double x1 = (column + 1) * kXPitchM;
    const double y_south = row * kYPitchM;
    const double y_north = y_south + kLaneletWidthM;
    const double expected_left[2][2] = {{x0, y_north}, {x1, y_north}};
    const double expected_right[2][2] = {{x0, y_south}, {x1, y_south}};
    const auto compare = [&](const std::vector<planner::map::MapPoint>& points,
                             const double expected[2][2]) {
      if (points.size() != 2) {
        check.max_error_m = std::max(check.max_error_m, 1e9);
        return;
      }
      for (std::size_t p = 0; p < 2; ++p) {
        check.max_error_m = std::max(
            check.max_error_m,
            std::hypot(points[p].x_m - expected[p][0], points[p].y_m - expected[p][1]));
      }
    };
    compare(geometry->left, expected_left);
    compare(geometry->right, expected_right);
    ++check.checked;
  }
  return check;
}

// ---------------------------------------------------------------------------
// 输出。
// ---------------------------------------------------------------------------

struct ScaleSummary {
  ScaleSpec scale;
  std::string map_path;
  std::string map_sha256;
  std::uint64_t map_bytes{0};
  std::string archive_sha256;
  std::string archive_relative_path;
  bool archive_sha256_matches{false};
  GeometryCheck geometry;
  Stats load;
  Stats query;
  Stats crop;
  Stats route;
  double query_avg_op_ms{0.0};
  double query_ops_per_second{0.0};
  double crop_avg_op_ms{0.0};
  double route_avg_op_ms{0.0};
  double route_length_mean_m{0.0};
  std::uint64_t route_mismatches{0};
  std::uint64_t crop_mismatches{0};
  std::uint64_t query_mismatches{0};
  double aux_validation_load_ms{0.0};
  std::uint64_t aux_validation_errors{0};
  std::uint64_t aux_validation_warnings{0};
  std::uint64_t peak_rss_bytes{0};
  bool ok{true};
  std::string error;
};

void write_scale_plots(const std::filesystem::path& plots_dir, const ScaleSummary& summary) {
  std::filesystem::create_directories(plots_dir);
  const std::vector<std::pair<std::string, Stats>> series = {{"load", summary.load},
                                                             {"query", summary.query},
                                                             {"crop", summary.crop},
                                                             {"route", summary.route}};
  const std::filesystem::path csv_path = plots_dir / ("p50_p95_" + summary.scale.name + ".csv");
  std::ofstream csv(csv_path);
  csv << "operation,batch_size,p50_ms,p95_ms,mean_ms,min_ms,max_ms\n";
  double max_p95 = 1e-9;
  for (const auto& entry : series) {
    max_p95 = std::max(max_p95, entry.second.p95_ms);
    csv << entry.first << ',' << 1 << ',' << format_number(entry.second.p50_ms) << ','
        << format_number(entry.second.p95_ms) << ',' << format_number(entry.second.mean_ms)
        << ',' << format_number(entry.second.min_ms) << ','
        << format_number(entry.second.max_ms) << '\n';
  }

  const std::filesystem::path svg_path = plots_dir / ("p50_p95_" + summary.scale.name + ".svg");
  std::ofstream svg(svg_path);
  const int width = 760;
  const int height = 320;
  const int left = 90;
  const int bottom = 250;
  const int plot_width = 620;
  const int bar_group = 130;
  const int bar_width = 34;
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\""
      << height << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
  svg << "<title>planner_map suite v1: batch latency P50/P95 (" << summary.scale.name
      << ", ms)</title>\n";
  svg << "<text x=\"12\" y=\"22\" font-family=\"sans-serif\" font-size=\"14\" fill=\"#111\">"
      << "planner_map v1 batch latency, scale=" << summary.scale.name
      << " (P50 blue / P95 orange, ms); values printed as text</text>\n";
  svg << "<line x1=\"" << left << "\" y1=\"" << bottom << "\" x2=\"" << (left + plot_width)
      << "\" y2=\"" << bottom << "\" stroke=\"#444\"/>\n";
  svg << "<line x1=\"" << left << "\" y1=\"" << bottom << "\" x2=\"" << left << "\" y2=\"40\" "
      << "stroke=\"#444\"/>\n";
  svg << "<text x=\"12\" y=\"46\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#111\">"
      << "max P95 = " << format_number(max_p95) << " ms</text>\n";
  int x = left + 30;
  for (const auto& entry : series) {
    const double p50_h = entry.second.p50_ms / max_p95 * (bottom - 60);
    const double p95_h = entry.second.p95_ms / max_p95 * (bottom - 60);
    svg << "<rect x=\"" << x << "\" y=\"" << (bottom - p50_h) << "\" width=\"" << bar_width
        << "\" height=\"" << p50_h << "\" fill=\"#4477aa\"/>\n";
    svg << "<rect x=\"" << (x + bar_width + 6) << "\" y=\"" << (bottom - p95_h)
        << "\" width=\"" << bar_width << "\" height=\"" << p95_h
        << "\" fill=\"#ee7733\"/>\n";
    svg << "<text x=\"" << (x - 4) << "\" y=\"" << (bottom + 16)
        << "\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#111\">" << entry.first
        << "</text>\n";
    svg << "<text x=\"" << (x - 4) << "\" y=\"" << (bottom + 30)
        << "\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#111\">P50="
        << format_number(entry.second.p50_ms) << "</text>\n";
    svg << "<text x=\"" << (x - 4) << "\" y=\"" << (bottom + 42)
        << "\" font-family=\"sans-serif\" font-size=\"10\" fill=\"#111\">P95="
        << format_number(entry.second.p95_ms) << "</text>\n";
    x += bar_group;
  }
  svg << "</svg>\n";
}

void write_summary_markdown(const std::filesystem::path& path, const std::string& suite,
                            const std::vector<ScaleSummary>& scales,
                            const std::vector<std::string>& pending_scales) {
  std::ofstream out(path);
  out << "# planner_map benchmark summary (suite " << suite << ")\n\n";
  out << "Batch latencies in ms (nearest-rank P50/P95, 30 measured batches, 5 warmups).\n\n";
  for (const auto& scale : scales) {
    out << "## " << scale.scale.name << " (" << scale.scale.lanelets << " lanelets)\n\n";
    out << "| operation | batch size | P50 ms | P95 ms | mean ms | min ms | max ms |\n";
    out << "| --- | --- | --- | --- | --- | --- | --- |\n";
    const std::vector<std::pair<std::string, Stats>> series = {{"load", scale.load},
                                                               {"query", scale.query},
                                                               {"crop", scale.crop},
                                                               {"route", scale.route}};
    for (const auto& entry : series) {
      out << "| " << entry.first << " | 1 | " << format_number(entry.second.p50_ms) << " | "
          << format_number(entry.second.p95_ms) << " | " << format_number(entry.second.mean_ms)
          << " | " << format_number(entry.second.min_ms) << " | "
          << format_number(entry.second.max_ms) << " |\n";
    }
    out << "\nquery avg/op: " << format_number(scale.query_avg_op_ms)
        << " ms, crop avg/window: " << format_number(scale.crop_avg_op_ms)
        << " ms, route avg/route: " << format_number(scale.route_avg_op_ms)
        << " ms, route length mean: " << format_number(scale.route_length_mean_m) << " m.\n";
    out << "map sha256: `" << scale.map_sha256 << "`, geometry max error: "
        << format_number(scale.geometry.max_error_m) << " m, peak RSS: "
        << scale.peak_rss_bytes << " bytes (VmHWM estimate).\n\n";
  }
  if (pending_scales.empty()) {
    out << "all non-pending spec scales were measured; none was substituted by a shrunken "
           "input under the same label.\n";
  } else {
    out << "pending_generation scales (not measured, no shrunken substitute under the same "
           "label):";
    for (const std::string& name : pending_scales) {
      out << " " << name;
    }
    out << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string suite = "v1";
  std::string output_dir;
  std::string spec_path;
  std::vector<std::string> scale_names;
  bool generate_only = false;
  int warmup_batches = -1;
  int measured_batches = -1;
  int queries_per_batch = -1;
  int windows_per_batch = -1;
  int routes_per_batch = -1;
  int load_batches = -1;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&](const char* flag) -> std::optional<std::string> {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << flag << "\n";
        return std::nullopt;
      }
      return std::string(argv[++i]);
    };
    if (arg == "--suite") {
      const auto value = next("--suite");
      if (!value) return 2;
      suite = *value;
    } else if (arg == "--output-dir") {
      const auto value = next("--output-dir");
      if (!value) return 2;
      output_dir = *value;
    } else if (arg == "--spec") {
      const auto value = next("--spec");
      if (!value) return 2;
      spec_path = *value;
    } else if (arg == "--generate-only") {
      generate_only = true;
    } else if (arg == "--scales" || arg == "--scale") {
      const auto value = next("--scales");
      if (!value) return 2;
      scale_names.clear();
      std::stringstream stream(*value);
      std::string item;
      while (std::getline(stream, item, ',')) {
        if (!item.empty()) scale_names.push_back(item);
      }
    } else if (arg == "--warmup-batches") {
      const auto value = next("--warmup-batches");
      if (!value) return 2;
      warmup_batches = std::stoi(*value);
    } else if (arg == "--measured-batches") {
      const auto value = next("--measured-batches");
      if (!value) return 2;
      measured_batches = std::stoi(*value);
    } else if (arg == "--queries-per-batch") {
      const auto value = next("--queries-per-batch");
      if (!value) return 2;
      queries_per_batch = std::stoi(*value);
    } else if (arg == "--windows-per-batch") {
      const auto value = next("--windows-per-batch");
      if (!value) return 2;
      windows_per_batch = std::stoi(*value);
    } else if (arg == "--routes-per-batch") {
      const auto value = next("--routes-per-batch");
      if (!value) return 2;
      routes_per_batch = std::stoi(*value);
    } else if (arg == "--load-batches") {
      const auto value = next("--load-batches");
      if (!value) return 2;
      load_batches = std::stoi(*value);
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: planner_map_bench --suite v1 --output-dir <dir> [--spec <path>]\n"
                   "       [--scales small,medium[,large]] [--generate-only]\n"
                   "       [--warmup-batches 5] [--measured-batches 30]\n"
                   "       [--queries-per-batch 1000] [--windows-per-batch 100]\n"
                   "       [--routes-per-batch 100] [--load-batches 30]\n"
                   "  --generate-only: write synthetic_maps/<scale>.{osm,sha256,meta.json}\n"
                   "  only (no measurement); --scale is an alias of --scales.\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 2;
    }
  }

  if (output_dir.empty()) {
    std::cerr << "--output-dir is required\n";
    return 2;
  }

  // ---------------------------------------------------------------------
  // spec 解析：显式 --spec，其次冻结的测试夹具（v5 优先，v4/v3/v2/v1 回退），
  // 最后是内置默认值。
  // ---------------------------------------------------------------------
  nlohmann::json spec;
  bool spec_loaded = false;
  if (spec_path.empty()) {
    for (const char* candidate : {"tests/fixtures/v5/benchmarks/map/specs.json",
                                  "tests/fixtures/v4/benchmarks/map/specs.json",
                                  "tests/fixtures/v3/benchmarks/map/specs.json",
                                  "tests/fixtures/v2/benchmarks/map/specs.json",
                                  "tests/fixtures/v1/benchmarks/map/specs.json"}) {
      if (std::filesystem::exists(candidate)) {
        spec_path = candidate;
        break;
      }
    }
  }
  std::string spec_sha256;
  if (!spec_path.empty() && std::filesystem::exists(spec_path)) {
    std::ifstream in(spec_path);
    try {
      spec = nlohmann::json::parse(in);
      spec_loaded = true;
      spec_sha256 = planner::map::detail::sha256_file_hex(spec_path, nullptr);
    } catch (const std::exception& error) {
      std::cerr << "warning: cannot parse spec " << spec_path << ": " << error.what() << "\n";
    }
  }

  // spec 声明的 lanelet_length_m（synthetic_map_family.parameters）；缺省取
  // kXPitchM，与生成器的实际长度一致。生成逻辑本身不随 spec 版本改变。
  double spec_lanelet_length_m = kXPitchM;
  if (spec_loaded) {
    const nlohmann::json::json_pointer lanelet_length_ptr(
        "/synthetic_map_family/parameters/lanelet_length_m");
    if (spec.contains(lanelet_length_ptr)) {
      spec_lanelet_length_m = spec.at(lanelet_length_ptr).get<double>();
    }
  }

  std::map<std::string, ScaleSpec> scale_specs;
  std::vector<std::string> spec_scale_order;
  if (spec_loaded && spec.contains("maps")) {
    for (const auto& entry : spec.at("maps")) {
      const auto scale = scale_from_spec(entry);
      spec_scale_order.push_back(scale.name);
      scale_specs[scale.name] = scale;
    }
    if (queries_per_batch < 0) queries_per_batch = spec.at("query_batch").at("queries_per_batch");
    if (windows_per_batch < 0) windows_per_batch = spec.at("crop_batch").at("windows_per_batch");
    if (routes_per_batch < 0) routes_per_batch = spec.at("route_batch").at("pairs_per_batch");
    if (load_batches < 0) load_batches = spec.at("load_batch").at("batches");
    if (warmup_batches < 0) warmup_batches = spec.at("timing_boundaries").at("warmup_batches");
    if (measured_batches < 0) measured_batches = spec.at("timing_boundaries").at("measured_batches");
  }
  if (scale_specs.empty()) {
    // 与 v1/v2 spec 规则一致的内置回退。
    ScaleSpec small;
    small.name = "small";
    small.lanelets = 1000;
    small.columns = 40;
    small.rows = 25;
    small.crop_center_x_base = 75.0;
    small.crop_center_y_base = 75.0;
    small.crop_center_x_mod = 850.0;
    small.crop_center_y_mod = 475.0;
    small.status = "rule_defined";
    ScaleSpec medium;
    medium.name = "medium";
    medium.lanelets = 10000;
    medium.columns = 200;
    medium.rows = 50;
    medium.crop_center_x_base = 375.0;
    medium.crop_center_y_base = 75.0;
    medium.crop_center_x_mod = 4250.0;
    medium.crop_center_y_mod = 1100.0;
    medium.status = "rule_defined";
    scale_specs[small.name] = small;
    scale_specs[medium.name] = medium;
    spec_scale_order = {"small", "medium"};
  }
  if (scale_names.empty()) {
    // 默认规模 = spec 中所有非 pending_generation 的规模（保持 spec 顺序）：
    // v4 及更早 = small/medium；v5 起 large 物化后自动纳入。
    for (const std::string& name : spec_scale_order) {
      const auto it = scale_specs.find(name);
      if (it != scale_specs.end() && it->second.status != "pending_generation") {
        scale_names.push_back(name);
      }
    }
  }
  if (warmup_batches < 0) warmup_batches = 5;
  if (measured_batches < 0) measured_batches = 30;
  if (queries_per_batch < 0) queries_per_batch = 1000;
  if (windows_per_batch < 0) windows_per_batch = 100;
  if (routes_per_batch < 0) routes_per_batch = 100;
  if (load_batches < 0) load_batches = 30;

  const std::filesystem::path out_dir(output_dir);
  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (ec) {
    std::cerr << "cannot create output dir: " << output_dir << "\n";
    return 2;
  }
  const std::filesystem::path maps_dir = out_dir / "synthetic_maps";
  std::filesystem::create_directories(maps_dir, ec);

  const char* run_id_env = std::getenv("PLANNER_RUN_ID");
  const std::string run_id = run_id_env != nullptr ? run_id_env : "manual";

  std::vector<Row> rows;
  std::vector<ScaleSummary> summaries;
  int exit_code = 0;

  for (const std::string& scale_name : scale_names) {
    const auto spec_it = scale_specs.find(scale_name);
    if (spec_it == scale_specs.end()) {
      std::cerr << "scale '" << scale_name << "' is not defined by the spec\n";
      exit_code = 2;
      continue;
    }
    ScaleSpec scale = spec_it->second;
    if (scale.status == "pending_generation" && !generate_only) {
      std::cerr << "scale '" << scale_name
                << "' is pending_generation in the spec; not generated (7.3/9)\n";
      continue;
    }
    if (scale.status == "pending_generation") {
      std::cerr << "note: scale '" << scale_name
                << "' is pending_generation in the spec; --generate-only materializes it "
                   "(fixture_version must be bumped afterwards)\n";
    }

    ScaleSummary summary;
    summary.scale = scale;

    // 1. 确定性生成（不属于任何测量批次）。
    std::string generation_error;
    const std::string osm_text = generate_osm_text(scale, &generation_error);
    if (!generation_error.empty()) {
      summary.ok = false;
      summary.error = generation_error;
      summaries.push_back(summary);
      exit_code = 1;
      continue;
    }
    const std::filesystem::path map_path = maps_dir / (scale.name + ".osm");
    {
      std::ofstream out(map_path, std::ios::binary);
      out << osm_text;
    }
    summary.map_path = map_path.string();
    summary.map_bytes = osm_text.size();
    summary.map_sha256 = planner::map::detail::sha256_hex(osm_text);
    summary.archive_sha256 = scale.archive_sha256;
    summary.archive_relative_path = scale.archive_relative_path;

    // spec archive.sha256 非空时强制校验（§9）：不一致明确报错，不静默继续。
    if (!scale.archive_sha256.empty()) {
      if (summary.map_sha256 != scale.archive_sha256) {
        summary.ok = false;
        summary.error = "generated sha256 " + summary.map_sha256 +
                        " does not match spec archive.sha256 " + scale.archive_sha256;
        std::cerr << "scale '" << scale.name << "': " << summary.error << "\n";
        summaries.push_back(summary);
        exit_code = 1;
        continue;
      }
      summary.archive_sha256_matches = true;
    }

    {
      std::ofstream out(maps_dir / (scale.name + ".sha256"));
      out << summary.map_sha256 << "  " << (scale.name + ".osm") << '\n';
    }
    {
      nlohmann::json meta{
          {"schema_version", 1},
          {"type", "map_benchmark_generated_map"},
          {"scale", scale.name},
          {"lanelet_count", scale.lanelets},
          {"columns", scale.columns},
          {"rows", scale.rows},
          {"lanelet_width_m", kLaneletWidthM},
          {"y_pitch_m", kYPitchM},
          {"x_pitch_m", kXPitchM},
          {"generated_lanelet_length_m", kXPitchM},
          {"spec_lanelet_length_m", spec_lanelet_length_m},
          {"interpretation",
           "lanelet2 successor detection requires shared endpoint node objects; "
           "generated length equals x_pitch so consecutive lanelets share nodes while "
           "keeping extent, columns/rows and window/route formulas"},
          {"id_scheme",
           nlohmann::json{{"nodes", "1 + 2*(row*(columns+1)+column) north, +1 south"},
                          {"ways", "10000000 + 2*k left, +1 right"},
                          {"relations", "20000000 + k + 1"}}},
          {"origin", nlohmann::json{{"lat_deg", bench_origin().lat_deg},
                                    {"lon_deg", bench_origin().lon_deg},
                                    {"ele_m", bench_origin().ele_m}}},
          {"projection", "utm"}};
      if (!scale.archive_sha256.empty() || !scale.archive_relative_path.empty()) {
        nlohmann::json archive_json{{"sha256", scale.archive_sha256},
                                    {"relative_path", scale.archive_relative_path},
                                    {"note", scale.archive_note}};
        if (!scale.archive_sha256.empty()) {
          archive_json["sha256_matches_generated"] = summary.archive_sha256_matches;
        }
        meta["spec_archive"] = std::move(archive_json);
      }
      std::ofstream out(maps_dir / (scale.name + ".meta.json"));
      out << meta.dump(2) << '\n';
    }

    if (generate_only) {
      std::cout << "generated scale=" << scale.name << " path=" << summary.map_path
                << " bytes=" << summary.map_bytes << " sha256=" << summary.map_sha256;
      if (!scale.archive_sha256.empty()) {
        std::cout << " spec_archive_sha256_match=true";
      }
      std::cout << "\n";
      summaries.push_back(summary);
      continue;
    }

    // 2. 加载批次（每个批次使用新的地图对象；不清理 OS 文件缓存）。
    const std::uint64_t hwm_start = read_vm_hwm_bytes();
    for (int warmup = 0; warmup < warmup_batches; ++warmup) {
      const auto load = run_load(summary.map_path, scale);
      if (!load.ok) {
        summary.ok = false;
        summary.error = "warmup load failed: " + load.error;
      }
    }
    std::vector<double> load_ms;
    for (int batch = 0; batch < load_batches; ++batch) {
      const auto load = run_load(summary.map_path, scale);
      Row row;
      row.case_id = scale.name + "/load";
      row.cache_mode = "cold";
      row.repeat_id = batch;
      row.scale = scale.name;
      row.operation = "load";
      row.wall_ms = load.trial.wall_ms;
      row.cpu_ms = load.trial.cpu_ms;
      row.peak_rss_bytes = read_vm_hwm_bytes();
      row.has_counts = true;
      row.count_sum = load.lanelets;
      row.count_min = load.lanelets;
      row.count_max = load.lanelets;
      row.notes = "os_file_cache_uncleared";
      if (!load.ok) {
        row.status = "failed";
        row.failure_reason = load.error;
        row.valid = false;
        summary.ok = false;
        summary.error = load.error;
      }
      rows.push_back(row);
      if (load.ok) {
        load_ms.push_back(load.trial.wall_ms);
      }
    }
    summary.load = summarize(load_ms);

    if (!summary.ok) {
      summary.peak_rss_bytes = read_vm_hwm_bytes();
      summaries.push_back(summary);
      exit_code = 1;
      continue;
    }

    // 3. 查询/裁剪/路由批次共用一个地图对象。
    auto import = Map::import_osm_file(summary.map_path, load_options(false));
    if (!import.ok()) {
      summary.ok = false;
      summary.error = "query map import failed: " + import.reason;
      summaries.push_back(summary);
      exit_code = 1;
      continue;
    }
    const Map& map = *import.map;
    summary.geometry = verify_geometry(map, scale, 64);
    if (summary.geometry.max_error_m > 1e-2) {
      summary.ok = false;
      summary.error = "generated geometry deviates from the rule: " +
                      format_number(summary.geometry.max_error_m) + " m";
    }

    // 合理性检查：确定性链必须可路由。
    const auto sanity_route = map.route(id_for_index(0), id_for_index(1));
    if (summary.ok && (sanity_route.status != planner::common::Status::kSuccess ||
                       sanity_route.lanelet_ids.size() != 2)) {
      summary.ok = false;
      summary.error = "sanity route ll_0 -> ll_1 failed: " + sanity_route.reason;
    }

    if (!summary.ok) {
      summary.peak_rss_bytes = read_vm_hwm_bytes();
      summaries.push_back(summary);
      exit_code = 1;
      continue;
    }

    // 4. 查询批次。
    std::vector<double> query_ms;
    double query_op_sum_ms = 0.0;
    std::uint64_t query_op_count = 0;
    for (int warmup = 0; warmup < warmup_batches; ++warmup) {
      run_query_batch(map, scale, static_cast<std::uint64_t>(queries_per_batch), nullptr);
    }
    for (int batch = 0; batch < measured_batches; ++batch) {
      nlohmann::json evidence = nlohmann::json::array();
      BatchResult result;
      const Trial trial = run_trial([&]() {
        result = run_query_batch(map, scale, static_cast<std::uint64_t>(queries_per_batch),
                                 batch == 0 ? &evidence : nullptr);
      });
      Row row;
      row.case_id = scale.name + "/query";
      row.repeat_id = batch;
      row.scale = scale.name;
      row.operation = "query";
      row.batch_size = queries_per_batch;
      row.wall_ms = trial.wall_ms;
      row.cpu_ms = trial.cpu_ms;
      row.peak_rss_bytes = read_vm_hwm_bytes();
      row.avg_op_ms = trial.wall_ms / queries_per_batch;
      row.ops_per_second = 1000.0 * queries_per_batch / std::max(1e-9, trial.wall_ms);
      row.has_counts = result.has_counts;
      row.count_sum = result.count_sum;
      row.count_min = result.count_min;
      row.count_max = result.count_max;
      row.route_mismatch = result.mismatch;
      row.notes = "fixed_sequence_j_mod_5";
      query_op_sum_ms += trial.wall_ms;
      query_op_count += static_cast<std::uint64_t>(queries_per_batch);
      if (result.mismatch != 0) {
        row.status = "failed";
        row.failure_reason = "attribute mismatches: " + std::to_string(result.mismatch);
        row.valid = false;
        summary.ok = false;
        summary.query_mismatches += result.mismatch;
      }
      rows.push_back(row);
      query_ms.push_back(trial.wall_ms);
      if (batch == 0 && !evidence.empty()) {
        std::ofstream out(out_dir / "query_evidence.json");
        out << nlohmann::json{{"schema_version", 1},
                              {"type", "map_benchmark_query_evidence"},
                              {"scale", scale.name},
                              {"queries", std::move(evidence)}}
                   .dump(2)
            << '\n';
      }
    }
    summary.query = summarize(query_ms);
    if (query_op_count > 0) {
      summary.query_avg_op_ms = query_op_sum_ms / static_cast<double>(query_op_count);
      summary.query_ops_per_second = 1000.0 * static_cast<double>(query_op_count) /
                                     std::max(1e-9, query_op_sum_ms);
    }

    // 5. 裁剪批次。
    std::vector<double> crop_ms;
    double crop_op_sum_ms = 0.0;
    std::uint64_t crop_op_count = 0;
    for (int warmup = 0; warmup < warmup_batches; ++warmup) {
      run_crop_batch(map, scale, static_cast<std::uint64_t>(windows_per_batch), nullptr);
    }
    for (int batch = 0; batch < measured_batches; ++batch) {
      nlohmann::json evidence = nlohmann::json::array();
      BatchResult result;
      const Trial trial = run_trial([&]() {
        result = run_crop_batch(map, scale, static_cast<std::uint64_t>(windows_per_batch),
                                batch == 0 ? &evidence : nullptr);
      });
      Row row;
      row.case_id = scale.name + "/crop";
      row.repeat_id = batch;
      row.scale = scale.name;
      row.operation = "crop";
      row.batch_size = windows_per_batch;
      row.wall_ms = trial.wall_ms;
      row.cpu_ms = trial.cpu_ms;
      row.peak_rss_bytes = read_vm_hwm_bytes();
      row.avg_op_ms = trial.wall_ms / windows_per_batch;
      row.ops_per_second = 1000.0 * windows_per_batch / std::max(1e-9, trial.wall_ms);
      row.has_counts = result.has_counts;
      row.count_sum = result.count_sum;
      row.count_min = result.count_min;
      row.count_max = result.count_max;
      row.route_mismatch = result.mismatch;
      row.notes = "clip_inside_window_and_transform_checked";
      crop_op_sum_ms += trial.wall_ms;
      crop_op_count += static_cast<std::uint64_t>(windows_per_batch);
      if (result.mismatch != 0) {
        row.status = "failed";
        row.failure_reason = "crop geometry/transform check failed";
        row.valid = false;
        summary.ok = false;
        summary.crop_mismatches += result.mismatch;
      }
      rows.push_back(row);
      crop_ms.push_back(trial.wall_ms);
      if (batch == 0 && !evidence.empty()) {
        std::ofstream out(out_dir / "crop_evidence.json");
        out << nlohmann::json{{"schema_version", 1},
                              {"type", "map_benchmark_crop_evidence"},
                              {"scale", scale.name},
                              {"windows", std::move(evidence)}}
                   .dump(2)
            << '\n';
      }
    }
    summary.crop = summarize(crop_ms);
    if (crop_op_count > 0) {
      summary.crop_avg_op_ms = crop_op_sum_ms / static_cast<double>(crop_op_count);
    }

    // 6. 路由批次。
    std::vector<double> route_ms;
    double route_op_sum_ms = 0.0;
    double route_length_sum = 0.0;
    std::uint64_t route_op_count = 0;
    std::uint64_t route_length_count = 0;
    for (int warmup = 0; warmup < warmup_batches; ++warmup) {
      run_route_batch(map, scale, static_cast<std::uint64_t>(routes_per_batch), nullptr);
    }
    for (int batch = 0; batch < measured_batches; ++batch) {
      nlohmann::json evidence = nlohmann::json::array();
      BatchResult result;
      const Trial trial = run_trial([&]() {
        result = run_route_batch(map, scale, static_cast<std::uint64_t>(routes_per_batch),
                                 batch == 0 ? &evidence : nullptr);
      });
      Row row;
      row.case_id = scale.name + "/route";
      row.repeat_id = batch;
      row.scale = scale.name;
      row.operation = "route";
      row.batch_size = routes_per_batch;
      row.wall_ms = trial.wall_ms;
      row.cpu_ms = trial.cpu_ms;
      row.peak_rss_bytes = read_vm_hwm_bytes();
      row.avg_op_ms = trial.wall_ms / routes_per_batch;
      row.ops_per_second = 1000.0 * routes_per_batch / std::max(1e-9, trial.wall_ms);
      row.has_counts = result.has_counts;
      row.count_sum = result.count_sum;
      row.count_min = result.count_min;
      row.count_max = result.count_max;
      row.has_path_length = true;
      row.path_length_m = result.aux_sum;
      row.route_mismatch = result.mismatch;
      row.notes = "expected_delta_plus_1_lanelets";
      route_op_sum_ms += trial.wall_ms;
      route_op_count += static_cast<std::uint64_t>(routes_per_batch);
      route_length_sum += result.aux_sum;
      route_length_count += static_cast<std::uint64_t>(routes_per_batch);
      if (result.mismatch != 0) {
        row.status = "failed";
        row.failure_reason = "route expectation mismatch: " + std::to_string(result.mismatch);
        row.valid = false;
        summary.ok = false;
        summary.route_mismatches += result.mismatch;
      }
      rows.push_back(row);
      route_ms.push_back(trial.wall_ms);
      if (batch == 0 && !evidence.empty()) {
        std::ofstream out(out_dir / "route_evidence.json");
        out << nlohmann::json{{"schema_version", 1},
                              {"type", "map_benchmark_route_evidence"},
                              {"scale", scale.name},
                              {"pairs", std::move(evidence)}}
                   .dump(2)
            << '\n';
      }
    }
    summary.route = summarize(route_ms);
    if (route_op_count > 0) {
      summary.route_avg_op_ms = route_op_sum_ms / static_cast<double>(route_op_count);
    }
    if (route_length_count > 0) {
      summary.route_length_mean_m = route_length_sum / static_cast<double>(route_length_count);
    }

    // 7. 辅助：一次启用验证的导入（记录在案，不属于 spec 批次）。
    {
      const Trial trial = run_trial([&]() {
        auto validated = Map::import_osm_file(summary.map_path, load_options(true));
        if (validated.ok()) {
          summary.aux_validation_errors = validated.map->info().validation.errors;
          summary.aux_validation_warnings = validated.map->info().validation.warnings;
        }
      });
      summary.aux_validation_load_ms = trial.wall_ms;
    }

    summary.peak_rss_bytes = read_vm_hwm_bytes();
    (void)hwm_start;
    if (!summary.ok) {
      exit_code = 1;
    }
    summaries.push_back(summary);
  }

  if (generate_only) {
    // 只物化输入：不执行测量批次，也不写任何测量输出。
    std::cout << "planner_map_bench --generate-only scales=" << scale_names.size()
              << " exit=" << exit_code << "\n";
    return exit_code;
  }

  // ---------------------------------------------------------------------
  // 输出。
  // ---------------------------------------------------------------------
  write_benchmark_metrics(out_dir / "benchmark_metrics.csv", run_id, rows);

  nlohmann::json summary_json;
  summary_json["schema_version"] = 1;
  summary_json["type"] = "map_benchmark_summary";
  summary_json["suite"] = suite;
  summary_json["run_id"] = run_id;
  summary_json["spec"] = nlohmann::json{{"path", spec_path},
                                        {"sha256", spec_sha256},
                                        {"loaded", spec_loaded}};
  const std::string interpretation_reason =
      std::abs(spec_lanelet_length_m - kXPitchM) < 1e-9
          ? "spec lanelet_length_m equals x_pitch; consecutive lanelets share "
            "endpoint node objects (Lanelet2 successor detection); extent, "
            "columns/rows and window/route formulas unchanged"
          : "lanelet2 successor detection requires shared endpoint node "
            "objects; generated length equals x_pitch while extent, "
            "columns/rows and window/route formulas are unchanged";
  summary_json["interpretation"] =
      nlohmann::json{{"generated_lanelet_length_m", kXPitchM},
                     {"spec_lanelet_length_m", spec_lanelet_length_m},
                     {"reason", interpretation_reason}};
  summary_json["protocol"] = nlohmann::json{{"warmup_batches", warmup_batches},
                                            {"measured_batches", measured_batches},
                                            {"queries_per_batch", queries_per_batch},
                                            {"windows_per_batch", windows_per_batch},
                                            {"routes_per_batch", routes_per_batch},
                                            {"load_batches", load_batches},
                                            {"seed", kSeed},
                                            {"single_threaded", true},
                                            {"os_file_cache_uncleared", true}};
  nlohmann::json pending_scales_json = nlohmann::json::array();
  std::vector<std::string> pending_scale_names;
  for (const std::string& name : spec_scale_order) {
    const auto it = scale_specs.find(name);
    if (it != scale_specs.end() && it->second.status == "pending_generation") {
      pending_scale_names.push_back(name);
      pending_scales_json.push_back(
          nlohmann::json{{"scale", name},
                         {"status", "pending_generation"},
                         {"note", "not generated and no shrunken substitute was measured "
                                  "under this label"}});
    }
  }
  summary_json["pending_scales"] = std::move(pending_scales_json);
  nlohmann::json scales_json = nlohmann::json::array();
  for (const auto& summary : summaries) {
    nlohmann::json entry;
    entry["scale"] = summary.scale.name;
    entry["lanelet_count"] = summary.scale.lanelets;
    entry["map_path"] = summary.map_path;
    entry["map_sha256"] = summary.map_sha256;
    entry["map_bytes"] = summary.map_bytes;
    entry["geometry_max_error_m"] = summary.geometry.max_error_m;
    entry["geometry_samples_checked"] = summary.geometry.checked;
    entry["peak_rss_bytes"] = summary.peak_rss_bytes;
    entry["ok"] = summary.ok;
    entry["error"] = summary.error;
    const auto stats_json = [](const Stats& stats) {
      return nlohmann::json{{"p50_ms", stats.p50_ms},
                            {"p95_ms", stats.p95_ms},
                            {"mean_ms", stats.mean_ms},
                            {"min_ms", stats.min_ms},
                            {"max_ms", stats.max_ms}};
    };
    entry["load"] = stats_json(summary.load);
    entry["query"] = stats_json(summary.query);
    entry["crop"] = stats_json(summary.crop);
    entry["route"] = stats_json(summary.route);
    entry["query_avg_op_ms"] = summary.query_avg_op_ms;
    entry["query_ops_per_second"] = summary.query_ops_per_second;
    entry["crop_avg_op_ms"] = summary.crop_avg_op_ms;
    entry["route_avg_op_ms"] = summary.route_avg_op_ms;
    entry["route_length_mean_m"] = summary.route_length_mean_m;
    entry["route_mismatches"] = summary.route_mismatches;
    entry["crop_mismatches"] = summary.crop_mismatches;
    entry["query_mismatches"] = summary.query_mismatches;
    entry["aux_import_with_validation_ms"] = summary.aux_validation_load_ms;
    entry["aux_validation_errors"] = summary.aux_validation_errors;
    entry["aux_validation_warnings"] = summary.aux_validation_warnings;
    if (!summary.archive_sha256.empty() || !summary.archive_relative_path.empty()) {
      nlohmann::json archive_json{{"sha256", summary.archive_sha256},
                                  {"relative_path", summary.archive_relative_path}};
      if (!summary.archive_sha256.empty()) {
        archive_json["sha256_matches_generated"] = summary.archive_sha256_matches;
      }
      entry["spec_archive"] = std::move(archive_json);
    }
    scales_json.push_back(std::move(entry));
  }
  summary_json["scales"] = std::move(scales_json);
  summary_json["metrics_file"] = "benchmark_metrics.csv";
  summary_json["exit_code"] = exit_code;
  {
    std::ofstream out(out_dir / "bench_summary.json");
    out << summary_json.dump(2) << '\n';
  }
  write_summary_markdown(out_dir / "summary.md", suite, summaries, pending_scale_names);
  for (const auto& summary : summaries) {
    write_scale_plots(out_dir / "plots", summary);
  }

  std::cout << "planner_map_bench suite=" << suite << " scales=" << scale_names.size()
            << " metrics_rows=" << rows.size() << " exit=" << exit_code << "\n";
  return exit_code;
}
