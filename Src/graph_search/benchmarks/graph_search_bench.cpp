// planner_graph_search_bench：X-B1 图搜索正式基准运行器 + graph_gen v1 生成器。
//
// 契约（Docs/expansion/benchmark_tiers.md §2.1/§2.3/§5/§6、
// Docs/component_development_plan.md §7.2/§7.3、tools/README.md §7.2）：
//
//   planner_graph_search_bench --suite v1 --output-dir <dir>      # 正式矩阵
//   planner_graph_search_bench --smoke --output-dir <dir>          # 烟测
//   planner_graph_search_bench --generate-only --output-dir <dir>  # 物化实例
//   planner_graph_search_bench --check --output-dir <dir>          # 校验实例
//   planner_graph_search_bench --list                              # 打印矩阵
//
// 输出（绝不写到 output-dir 之外，归档目录除外）：
//   benchmark_metrics.csv   逐 trial 一行（tools/README.md §7.2 固定列 +
//                           图/距离场列 + 组件列；预热不写，负例单次写）
//   bench_summary.json      聚合（P50/P95 nearest-rank、成功率、超时、门禁）
//   graph_instances.json    实例表（生成器版本/种子/顶点/边/对/SHA256/归档）
//   graph_instance_checks.json  生成器结构自检结果
//   graphs/<id>.graph.txt   物化实例（--generate-only；>10 MiB 另按 §3.4
//                           复制到 artifacts/data/sha256/<sha>/ 归档）
//   graphs/<id>.sha256 / <id>.meta.json
//   plots/*.svg + *.csv     每组冷启动 P95 柱状图与数据
//
// 协议要点：
//  * 点到点 7 个主后端 + 4 个零启发式对照（单列，不混入排名）+ 距离场 3 个；
//  * 每用例预热 5 + 测量 30（可调）；点到点预算 5 s、距离场 30 s；
//  * 冷启动每 trial 新建后端（构造单独计时），热启动复用一个后端实例；
//  * 计时边界：construct_ms / e2e_ms / preprocess_ms / search_ms /
//    postprocess_ms / wall_ms 分列；输入生成与磁盘输出在计时外；
//  * 峰值 RSS 用 /proc/self 1 ms 采样（采样估计，另有 VmHWM 参考列）；
//  * 超时单列（status=timeout，timeout=true），不进成功率剔除；
//  * 负例（断连、同点）单次运行，只记录状态与终止耗时，不进排名；
//  * 正确性门禁：返回路径逐边存在、重算 double 代价与独立参考 Dijkstra
//    一致（justinhj 使用已登记的 float32 门槛，见 graph_gen.hpp）；
//  * 退出码：0 完成；1 生成器/一致性/门禁失败；2 用法错误。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <nlohmann/json.hpp>

#include "graph_gen.hpp"
#include "sha256.hpp"

#include "planner/graph_search/adapters.hpp"
#include "planner/graph_search/types.hpp"

namespace fs = std::filesystem;

namespace {

using planner::graph_search::Control;
using planner::graph_search::DistanceFieldResult;
using planner::graph_search::DistanceFieldSearcher;
using planner::graph_search::Graph;
using planner::graph_search::GraphSearcher;
using planner::graph_search::Heuristic;
using planner::graph_search::Result;
using planner::graph_search::Status;
using planner::graph_search::bench::check_instance;
using planner::graph_search::bench::EdgeIndex;
using planner::graph_search::bench::Instance;
using planner::graph_search::bench::InstanceSpec;
using planner::graph_search::bench::PairSpec;
using planner::graph_search::bench::ReferenceField;
using planner::graph_search::bench::sha256_file_hex;
using planner::graph_search::bench::sha256_hex;
using planner::graph_search::bench::serialize_instance;

constexpr std::size_t kArchiveThresholdBytes = 10u * 1024u * 1024u;  // §3.4
constexpr double kTimeBudgetP2P = 5.0;    // §2.1
constexpr double kTimeBudgetField = 30.0; // §2.3
constexpr int kDefaultWarmup = 5;
constexpr int kDefaultRepeats = 30;

// ---------------------------------------------------------------------------
// 后端注册表
// ---------------------------------------------------------------------------

struct BackendSpec {
  std::string id;
  std::string kind;     // "astar" | "dijkstra" | "distance_field"
  std::string group;    // "p2p" | "zero_heuristic" | "distance_field"
  std::string variant;  // "main" | "zero_heuristic"
  std::string provenance;
  bool uses_heuristic = false;
  bool float32_native = false;
  bool compiled = true;
};

std::vector<BackendSpec> backend_registry() {
  std::vector<BackendSpec> backends;
#if defined(PLANNER_GRAPH_SEARCH_HAS_BOOST)
  backends.push_back({"boost_astar", "astar", "p2p", "main",
                      "Boost Graph 1.71 A* (float64)", true, false, true});
  backends.push_back({"boost_dijkstra", "dijkstra", "p2p", "main",
                      "Boost Graph 1.71 Dijkstra 语义 (float64)", false, false,
                      true});
  backends.push_back({"boost_distance_field", "distance_field",
                      "distance_field", "main",
                      "Boost Graph 1.71 完整距离场 (float64)", false, false, true});
#else
  backends.push_back({"boost_astar", "astar", "p2p", "main", "", true, false, false});
  backends.push_back({"boost_dijkstra", "dijkstra", "p2p", "main", "", false, false, false});
  backends.push_back({"boost_distance_field", "distance_field", "distance_field",
                      "main", "", false, false, false});
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_LMRP)
  backends.push_back({"lmrp_astar", "astar", "p2p", "main",
                      "libMultiRobotPlanning A* (float64)", true, false, true});
#else
  backends.push_back({"lmrp_astar", "astar", "p2p", "main", "", true, false, false});
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_POOL)
  backends.push_back({"justinhj_astar_pool", "astar", "p2p", "main",
                      "justinhj A* 内存池开 (float32)", true, true, true});
#else
  backends.push_back({"justinhj_astar_pool", "astar", "p2p", "main", "", true, true, false});
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_NOPOOL)
  backends.push_back({"justinhj_astar_nopool", "astar", "p2p", "main",
                      "justinhj A* 内存池关 (float32)", true, true, true});
#else
  backends.push_back({"justinhj_astar_nopool", "astar", "p2p", "main", "", true, true, false});
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_LEMON)
  backends.push_back({"lemon_dijkstra", "dijkstra", "p2p", "main",
                      "LEMON Dijkstra (float64)", false, false, true});
  backends.push_back({"lemon_distance_field", "distance_field", "distance_field",
                      "main", "LEMON 完整距离场 (float64)", false, false, true});
#else
  backends.push_back({"lemon_dijkstra", "dijkstra", "p2p", "main", "", false, false, false});
  backends.push_back({"lemon_distance_field", "distance_field", "distance_field",
                      "main", "", false, false, false});
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_IGRAPH)
  backends.push_back({"igraph_dijkstra", "dijkstra", "p2p", "main",
                      "igraph 1.0.1 Dijkstra (float64; GPL 隔离组)", false, false,
                      true});
  backends.push_back({"igraph_distance_field", "distance_field",
                      "distance_field", "main",
                      "igraph 1.0.1 完整距离场 (float64; GPL 隔离组)", false,
                      false, true});
#else
  backends.push_back({"igraph_dijkstra", "dijkstra", "p2p", "main", "", false, false, false});
  backends.push_back({"igraph_distance_field", "distance_field", "distance_field",
                      "main", "", false, false, false});
#endif
  // 零启发式对照组：可采纳的任意权重下界，单列，不混入主排名（§2.1/§4.2）。
  const std::vector<std::string> control_ids = {"boost_astar", "lmrp_astar",
                                                "justinhj_astar_pool",
                                                "justinhj_astar_nopool"};
  for (const std::string& control_id : control_ids) {
    for (const BackendSpec& spec : backends) {
      if (spec.id == control_id && spec.compiled) {
        BackendSpec control = spec;
        control.id = spec.id + "__zero";
        control.group = "zero_heuristic";
        control.variant = "zero_heuristic";
        control.uses_heuristic = false;
        control.provenance = spec.provenance + " / 零启发式对照";
        backends.push_back(control);
        break;
      }
    }
  }
  return backends;
}

std::unique_ptr<GraphSearcher> create_p2p_searcher(const std::string& id) {
  std::string base = id;
  const std::string suffix = "__zero";
  if (base.size() > suffix.size() &&
      base.compare(base.size() - suffix.size(), suffix.size(), suffix) == 0) {
    base = base.substr(0, base.size() - suffix.size());
  }
#if defined(PLANNER_GRAPH_SEARCH_HAS_BOOST)
  if (base == "boost_astar") {
    return planner::graph_search::make_boost_astar();
  }
  if (base == "boost_dijkstra") {
    return planner::graph_search::make_boost_dijkstra();
  }
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_LMRP)
  if (base == "lmrp_astar") {
    return planner::graph_search::make_lmrp_astar();
  }
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_POOL)
  if (base == "justinhj_astar_pool") {
    return planner::graph_search::make_justinhj_astar_pool();
  }
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_NOPOOL)
  if (base == "justinhj_astar_nopool") {
    return planner::graph_search::make_justinhj_astar_nopool();
  }
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_LEMON)
  if (base == "lemon_dijkstra") {
    return planner::graph_search::make_lemon_dijkstra();
  }
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_IGRAPH)
  if (base == "igraph_dijkstra") {
    return planner::graph_search::make_igraph_dijkstra();
  }
#endif
  return nullptr;
}

std::unique_ptr<DistanceFieldSearcher> create_df_searcher(const std::string& id) {
#if defined(PLANNER_GRAPH_SEARCH_HAS_BOOST)
  if (id == "boost_distance_field") {
    return planner::graph_search::make_boost_distance_field();
  }
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_LEMON)
  if (id == "lemon_distance_field") {
    return planner::graph_search::make_lemon_distance_field();
  }
#endif
#if defined(PLANNER_GRAPH_SEARCH_HAS_IGRAPH)
  if (id == "igraph_distance_field") {
    return planner::graph_search::make_igraph_distance_field();
  }
#endif
  return nullptr;
}

// ---------------------------------------------------------------------------
// 选项与工具
// ---------------------------------------------------------------------------

struct Options {
  std::string suite = "v1";
  std::string output_dir;
  std::string archive_root = "artifacts/data/sha256";
  bool generate_only = false;
  bool check = false;
  bool smoke = false;
  bool list = false;
  bool help = false;
  std::vector<std::string> instances;
  std::vector<std::string> scales;
  std::vector<std::string> classes;
  std::vector<std::string> backends;
  std::vector<std::string> cache_modes = {"cold", "warm"};
  int warmup = kDefaultWarmup;
  int repeats = kDefaultRepeats;
  double p2p_budget_s = kTimeBudgetP2P;
  double field_budget_s = kTimeBudgetField;
  long long rss_sample_us = 1000;
};

std::vector<std::string> split_csv(const std::string& text) {
  std::vector<std::string> values;
  std::string current;
  for (const char character : text) {
    if (character == ',') {
      if (!current.empty()) {
        values.push_back(current);
      }
      current.clear();
    } else {
      current += character;
    }
  }
  if (!current.empty()) {
    values.push_back(current);
  }
  return values;
}

std::string utc_now_iso() {
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
  const auto millis =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds)
          .count();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&time, &tm);
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                tm.tm_min, tm.tm_sec, static_cast<int>(millis));
  return buffer;
}

std::string hostname() {
  char buffer[256] = {0};
  if (gethostname(buffer, sizeof(buffer) - 1) != 0) {
    return "unknown";
  }
  return buffer;
}

std::string run_id() {
  const char* value = std::getenv("PLANNER_RUN_ID");
  return value != nullptr && value[0] != '\0' ? value : "local";
}

std::string csv_field(const std::string& value) {
  if (value.find_first_of(",\"\n") == std::string::npos) {
    return value;
  }
  std::string quoted = "\"";
  for (const char character : value) {
    if (character == '"') {
      quoted += "\"\"";
    } else {
      quoted += character;
    }
  }
  quoted += "\"";
  return quoted;
}

std::string number_field(double value, int precision = 9) {
  if (!std::isfinite(value)) {
    return "";
  }
  std::ostringstream out;
  out << std::setprecision(precision) << value;
  return out.str();
}

std::string integer_field(long long value) {
  if (value < 0) {
    return "";
  }
  return std::to_string(value);
}

double percentile_nearest_rank(std::vector<double> values, double percentile) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double rank = std::ceil(percentile / 100.0 * values.size());
  std::size_t index = rank <= 1.0 ? 0 : static_cast<std::size_t>(rank) - 1;
  index = std::min(index, values.size() - 1);
  return values[index];
}

double elapsed_ms(std::chrono::steady_clock::time_point start,
                  std::chrono::steady_clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

double cpu_ms_between(std::clock_t start, std::clock_t end) {
  return 1000.0 * static_cast<double>(end - start) /
         static_cast<double>(CLOCKS_PER_SEC);
}

long long read_proc_status_bytes(const char* key) {
  std::ifstream stream("/proc/self/status");
  if (!stream) {
    return -1;
  }
  std::string line;
  const std::size_t key_length = std::strlen(key);
  while (std::getline(stream, line)) {
    if (line.compare(0, key_length, key) == 0) {
      std::istringstream parser(line.substr(key_length));
      long long kilobytes = -1;
      parser >> kilobytes;
      return kilobytes >= 0 ? kilobytes * 1024 : -1;
    }
  }
  return -1;
}

// /proc/self 1 ms 采样器（采样估计；另记录 VmHWM 参考值）。
class RssSampler {
 public:
  explicit RssSampler(long long interval_us)
      : interval_us_(interval_us > 0 ? interval_us : 1000) {}

  void start() {
    running_.store(true);
    thread_ = std::thread([this] { loop(); });
  }

  void stop() {
    running_.store(false);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void begin() {
    peak_.store(0);
    active_.store(true);
  }

  long long end() {
    active_.store(false);
    const long long sampled = peak_.load();
    const long long current = read_proc_status_bytes("VmRSS:");
    return std::max(sampled, current);
  }

 private:
  void loop() {
    while (running_.load()) {
      if (active_.load()) {
        const long long rss = read_proc_status_bytes("VmRSS:");
        long long observed = peak_.load();
        while (rss > observed && !peak_.compare_exchange_weak(observed, rss)) {
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(interval_us_));
    }
  }

  long long interval_us_;
  std::atomic<bool> running_{false};
  std::atomic<bool> active_{false};
  std::atomic<long long> peak_{0};
  std::thread thread_;
};

// ---------------------------------------------------------------------------
// Trial 记录
// ---------------------------------------------------------------------------

struct TrialRow {
  std::string case_id;
  int repeat_id = 0;
  std::string cache_mode;
  std::uint64_t seed = 0;
  std::string status;
  std::string failure_reason;
  bool measured = false;
  std::string group;
  std::string backend_id;
  std::string variant;
  std::string kind;
  std::string instance_id;
  std::string scale;
  std::string graph_class;
  std::string pair_id;
  std::string source_id;
  std::string expected;
  double budget_s = 0.0;
  bool timeout = false;
  double construct_ms = std::numeric_limits<double>::quiet_NaN();
  double e2e_ms = std::numeric_limits<double>::quiet_NaN();
  double wall_ms = std::numeric_limits<double>::quiet_NaN();
  double cpu_ms = std::numeric_limits<double>::quiet_NaN();
  double preprocess_ms = std::numeric_limits<double>::quiet_NaN();
  double search_ms = std::numeric_limits<double>::quiet_NaN();
  double postprocess_ms = std::numeric_limits<double>::quiet_NaN();
  double adapter_wall_ms = std::numeric_limits<double>::quiet_NaN();
  long long peak_rss_bytes = -1;
  long long rss_hwm_bytes = -1;
  long long expanded_nodes = -1;
  long long path_edges = -1;
  int hops = -1;
  double native_cost = std::numeric_limits<double>::quiet_NaN();
  double recomputed_cost = std::numeric_limits<double>::quiet_NaN();
  double reference_cost = std::numeric_limits<double>::quiet_NaN();
  double cost_delta_rel = std::numeric_limits<double>::quiet_NaN();
  double path_length_m = std::numeric_limits<double>::quiet_NaN();
  long long unreachable_cells = -1;
  double max_distance_m = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
};

const char* kMetricsHeader =
    "run_id,case_id,repeat_id,cache_mode,seed,status,failure_reason,"
    "preprocess_ms,wall_ms,cpu_ms,peak_rss_bytes,expanded_nodes,"
    "collision_checks,path_length_m,gear_changes,endpoint_position_error_m,"
    "endpoint_yaw_error_rad,valid,instance_id,scale,class,neighborhood,"
    "pair_id,budget_s,timeout,source_id,unreachable_cells,max_distance_m,"
    "reference_cost,group,backend_id,variant,kind,measured,expected,"
    "construct_ms,e2e_ms,search_ms,postprocess_ms,adapter_wall_ms,path_edges,"
    "hops,native_cost,recomputed_cost,cost_delta_rel,rss_hwm_bytes";

void write_metrics_csv(const std::string& path,
                       const std::vector<TrialRow>& rows) {
  std::ofstream stream(path);
  stream << kMetricsHeader << "\n";
  const std::string run = run_id();
  for (const TrialRow& row : rows) {
    stream << csv_field(run) << "," << csv_field(row.case_id) << ","
           << row.repeat_id << "," << row.cache_mode << "," << row.seed << ","
           << csv_field(row.status) << "," << csv_field(row.failure_reason) << ","
           << number_field(row.preprocess_ms) << ","
           << number_field(row.wall_ms) << "," << number_field(row.cpu_ms) << ","
           << integer_field(row.peak_rss_bytes) << ","
           << integer_field(row.expanded_nodes) << ","
           << ""  // collision_checks：图搜索不适用
           << "," << number_field(row.path_length_m) << ","
           << ""  // gear_changes：图搜索不适用
           << "," << ""  // endpoint_position_error_m：图搜索不适用
           << "," << ""  // endpoint_yaw_error_rad：图搜索不适用
           << "," << (row.valid ? "true" : "false") << ","
           << csv_field(row.instance_id) << "," << csv_field(row.scale) << ","
           << csv_field(row.graph_class) << ","
           << ""  // neighborhood：图搜索不适用（栅格专有）
           << "," << csv_field(row.pair_id) << "," << number_field(row.budget_s, 6)
           << "," << (row.timeout ? "true" : "false") << ","
           << csv_field(row.source_id)
           << "," << integer_field(row.unreachable_cells) << ","
           << number_field(row.max_distance_m) << ","
           << number_field(row.reference_cost) << "," << csv_field(row.group)
           << "," << csv_field(row.backend_id) << "," << csv_field(row.variant)
           << "," << csv_field(row.kind) << ","
           << (row.measured ? "true" : "false") << ","
           << csv_field(row.expected) << ","
           << number_field(row.construct_ms) << ","
           << number_field(row.e2e_ms) << "," << number_field(row.search_ms)
           << "," << number_field(row.postprocess_ms) << ","
           << number_field(row.adapter_wall_ms) << ","
           << integer_field(row.path_edges) << "," << integer_field(row.hops)
           << "," << number_field(row.native_cost) << ","
           << number_field(row.recomputed_cost) << ","
           << number_field(row.cost_delta_rel) << ","
           << integer_field(row.rss_hwm_bytes) << "\n";
  }
}

// ---------------------------------------------------------------------------
// 实例数据（生成 + 参考 + 物化校验）
// ---------------------------------------------------------------------------

struct InstanceData {
  Instance instance;
  EdgeIndex index;
  std::string serialized;
  std::string serialized_sha256;
  std::size_t serialized_size = 0;
  bool materialized = false;
  std::string materialized_relative_path;
  bool archived = false;
  std::string archive_relative_path;
  std::vector<std::string> check_issues;
  std::map<std::string, double> pair_reference_cost;
  std::map<std::string, Heuristic> pair_heuristics;   // key = goal 字符串
  std::map<std::string, ReferenceField> source_reference;  // s01/s02
  std::string s02_source_id;
  int s02_vertex = 0;

  InstanceData(Instance generated, EdgeIndex built)
      : instance(std::move(generated)), index(std::move(built)) {}
};

std::string s01_source_id() { return "s01"; }

std::string pair_goal_key(int goal) { return std::to_string(goal); }

bool write_file(const std::string& path, const std::string& content,
                std::string* error) {
  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    if (error != nullptr) {
      *error = "cannot write file: " + path;
    }
    return false;
  }
  stream.write(content.data(), static_cast<std::streamsize>(content.size()));
  if (!stream) {
    if (error != nullptr) {
      *error = "write error: " + path;
    }
    return false;
  }
  return true;
}

bool read_file(const std::string& path, std::string* content,
               std::string* error) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    if (error != nullptr) {
      *error = "cannot read file: " + path;
    }
    return false;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  if (!stream.good() && !stream.eof()) {
    if (error != nullptr) {
      *error = "read error: " + path;
    }
    return false;
  }
  *content = buffer.str();
  return true;
}

// 生成全部选中实例并构建参考/启发式数据。
bool prepare_instances(const std::vector<InstanceSpec>& specs,
                       std::vector<std::unique_ptr<InstanceData>>* data,
                       std::string* error) {
  for (const InstanceSpec& spec : specs) {
    Instance generated;
    std::string generation_error;
    if (!planner::graph_search::bench::generate_instance(spec, &generated,
                                                         &generation_error)) {
      *error = "generate " + spec.instance_id + ": " + generation_error;
      return false;
    }
    EdgeIndex index(generated.graph);
    auto entry =
        std::make_unique<InstanceData>(std::move(generated), std::move(index));
    // 结构问题不在此中断：调用方决定门禁并写出 graph_instance_checks.json。
    check_instance(entry->instance, &entry->check_issues);
    entry->serialized = serialize_instance(entry->instance);
    entry->serialized_size = entry->serialized.size();
    entry->serialized_sha256 = sha256_hex(entry->serialized);
    // 参考代价（点到点对）与距离场参考（s01=0；s02=r05 目标）。全部可达对
    // 以 0 为起点（生成规则），参考只算一次。
    const ReferenceField reference_zero =
        planner::graph_search::bench::reference_dijkstra(entry->instance.graph,
                                                         0);
    for (const PairSpec& pair : entry->instance.pairs) {
      if (pair.expected == "reachable") {
        entry->pair_reference_cost[pair.pair_id] =
            reference_zero.dist[static_cast<std::size_t>(pair.goal)];
        entry->pair_heuristics[pair_goal_key(pair.goal)] = Heuristic(
            planner::graph_search::EuclideanHeuristic(entry->instance.xs,
                                                      entry->instance.ys,
                                                      pair.goal));
      }
    }
    const PairSpec* s02_pair = entry->instance.find_pair("r05");
    if (s02_pair == nullptr) {
      *error = spec.instance_id + ": r05 pair missing";
      return false;
    }
    entry->s02_source_id = "s02";
    entry->s02_vertex = s02_pair->goal;
    entry->source_reference[s01_source_id()] = reference_zero;
    entry->source_reference[entry->s02_source_id] =
        planner::graph_search::bench::reference_dijkstra(entry->instance.graph,
                                                         entry->s02_vertex);
    data->push_back(std::move(entry));
  }
  return true;
}

nlohmann::json pair_json(const PairSpec& pair) {
  nlohmann::json out;
  out["pair_id"] = pair.pair_id;
  out["start"] = pair.start;
  out["goal"] = pair.goal;
  out["hops"] = pair.hops;
  out["expected"] = pair.expected;
  return out;
}

nlohmann::json instance_record_json(const InstanceData& data,
                                    const std::string& run,
                                    const std::string& materialized_relative,
                                    const std::string& archive_relative) {
  const Instance& instance = data.instance;
  nlohmann::json record;
  record["instance_id"] = instance.spec.instance_id;
  record["class"] = instance.spec.graph_class;
  record["scale"] = instance.spec.scale;
  record["seed"] = instance.spec.seed;
  record["num_vertices"] = instance.graph.num_vertices;
  record["num_edges"] = instance.graph.edges.size();
  record["average_out_degree"] =
      static_cast<double>(instance.graph.edges.size()) /
      static_cast<double>(instance.graph.num_vertices);
  record["isolated_vertex"] = instance.isolated_vertex;
  record["generator"] = {{"name", "graph_gen"},
                         {"version", planner::graph_search::bench::kGraphGenVersion},
                         {"prng", "splitmix64"},
                         {"rules", "Src/graph_search/benchmarks/README.md"}};
  record["serialized_sha256"] = data.serialized_sha256;
  record["serialized_size_bytes"] = data.serialized_size;
  record["materialized"] = !materialized_relative.empty();
  record["relative_path"] = materialized_relative;
  record["archive"] = nlohmann::json(nullptr);
  if (!archive_relative.empty()) {
    record["archive"] = {
        {"relative_path", archive_relative},
        {"sha256", data.serialized_sha256},
        {"size_bytes", data.serialized_size},
        {"generated_by_run_id", run},
        {"strategy", "benchmark_tiers.md §3.4：单文件 >10 MiB 时 SHA256 寻址归档"}};
  }
  nlohmann::json pairs = nlohmann::json::array();
  for (const PairSpec& pair : instance.pairs) {
    pairs.push_back(pair_json(pair));
  }
  record["pairs"] = std::move(pairs);
  record["checks_passed"] = data.check_issues.empty();
  record["check_issues"] = data.check_issues;
  return record;
}

// ---------------------------------------------------------------------------
// 物化 / 校验（--generate-only / --check）
// ---------------------------------------------------------------------------

struct MaterializeOutcome {
  bool ok = true;
  bool archive_written = false;
  std::string relative_path;
  std::string archive_relative;
  std::vector<std::string> issues;
};

// 写盘后回读校验 SHA256/大小（§3.4：生成器每次校验输出 SHA256）。
bool verify_file(const std::string& path, const std::string& expected_sha256,
                 std::size_t expected_size, std::string* error) {
  std::size_t size = 0;
  std::string read_error;
  const std::string sha = sha256_file_hex(path, &size, &read_error);
  if (!read_error.empty()) {
    if (error != nullptr) {
      *error = read_error;
    }
    return false;
  }
  if (sha != expected_sha256 || size != expected_size) {
    if (error != nullptr) {
      *error = "post-write verification failed: " + path + " (sha256 " + sha +
               " size " + std::to_string(size) + ")";
    }
    return false;
  }
  return true;
}

// §3.4：单文件 >10 MiB 时复制到 SHA256 寻址归档目录
// （<archive_root>/<sha256>/<name>，另附 .sha256 与 .meta.json）。
MaterializeOutcome archive_instance(const InstanceData& data,
                                    const std::string& archive_root) {
  MaterializeOutcome outcome;
  if (data.serialized_size <= kArchiveThresholdBytes) {
    return outcome;
  }
  std::error_code filesystem_error;
  const std::string file_name = data.instance.spec.instance_id + ".graph.txt";
  const std::string archive_dir = archive_root + "/" + data.serialized_sha256;
  fs::create_directories(archive_dir, filesystem_error);
  if (filesystem_error) {
    outcome.ok = false;
    outcome.issues.push_back("cannot create archive dir: " +
                             filesystem_error.message());
    return outcome;
  }
  const std::string archive_path = archive_dir + "/" + file_name;
  const std::string checksum_path =
      archive_dir + "/" + data.instance.spec.instance_id + ".sha256";
  const std::string meta_path =
      archive_dir + "/" + data.instance.spec.instance_id + ".meta.json";
  std::string error;
  if (!write_file(archive_path, data.serialized, &error) ||
      !verify_file(archive_path, data.serialized_sha256, data.serialized_size,
                   &error) ||
      !write_file(checksum_path,
                  data.serialized_sha256 + "  " + file_name + "\n", &error)) {
    outcome.ok = false;
    outcome.issues.push_back(error);
    return outcome;
  }
  nlohmann::json meta;
  meta["instance_id"] = data.instance.spec.instance_id;
  meta["generator"] = {{"name", "graph_gen"},
                       {"version", planner::graph_search::bench::kGraphGenVersion}};
  meta["class"] = data.instance.spec.graph_class;
  meta["scale"] = data.instance.spec.scale;
  meta["seed"] = data.instance.spec.seed;
  meta["num_vertices"] = data.instance.graph.num_vertices;
  meta["num_edges"] = data.instance.graph.edges.size();
  meta["isolated_vertex"] = data.instance.isolated_vertex;
  meta["sha256"] = data.serialized_sha256;
  meta["size_bytes"] = data.serialized_size;
  meta["relative_path"] = archive_path;
  meta["generated_by_run_id"] = run_id();
  if (!write_file(meta_path, meta.dump(2) + "\n", &error)) {
    outcome.ok = false;
    outcome.issues.push_back(error);
    return outcome;
  }
  outcome.archive_written = true;
  outcome.archive_relative = archive_path;
  return outcome;
}

MaterializeOutcome materialize_instance(const InstanceData& data,
                                        const std::string& output_dir,
                                        const std::string& archive_root,
                                        bool write_archive_copy) {
  MaterializeOutcome outcome;
  const std::string graphs_dir = output_dir + "/graphs";
  std::error_code filesystem_error;
  fs::create_directories(graphs_dir, filesystem_error);
  if (filesystem_error) {
    outcome.ok = false;
    outcome.issues.push_back("cannot create graphs dir: " +
                             filesystem_error.message());
    return outcome;
  }
  const std::string file_name = data.instance.spec.instance_id + ".graph.txt";
  const std::string file_path = graphs_dir + "/" + file_name;
  const std::string checksum_path =
      graphs_dir + "/" + data.instance.spec.instance_id + ".sha256";
  const std::string meta_path =
      graphs_dir + "/" + data.instance.spec.instance_id + ".meta.json";
  std::string error;
  if (!write_file(file_path, data.serialized, &error) ||
      !verify_file(file_path, data.serialized_sha256, data.serialized_size,
                   &error)) {
    outcome.ok = false;
    outcome.issues.push_back(error);
    return outcome;
  }
  outcome.relative_path = "graphs/" + file_name;
  if (!write_file(checksum_path,
                  data.serialized_sha256 + "  " + file_name + "\n", &error)) {
    outcome.ok = false;
    outcome.issues.push_back(error);
    return outcome;
  }
  nlohmann::json meta;
  meta["instance_id"] = data.instance.spec.instance_id;
  meta["generator"] = {{"name", "graph_gen"},
                       {"version", planner::graph_search::bench::kGraphGenVersion}};
  meta["class"] = data.instance.spec.graph_class;
  meta["scale"] = data.instance.spec.scale;
  meta["seed"] = data.instance.spec.seed;
  meta["num_vertices"] = data.instance.graph.num_vertices;
  meta["num_edges"] = data.instance.graph.edges.size();
  meta["isolated_vertex"] = data.instance.isolated_vertex;
  meta["sha256"] = data.serialized_sha256;
  meta["size_bytes"] = data.serialized_size;
  meta["generated_by_run_id"] = run_id();
  nlohmann::json pairs = nlohmann::json::array();
  for (const PairSpec& pair : data.instance.pairs) {
    pairs.push_back(pair_json(pair));
  }
  meta["pairs"] = std::move(pairs);
  if (!write_file(meta_path, meta.dump(2) + "\n", &error)) {
    outcome.ok = false;
    outcome.issues.push_back(error);
    return outcome;
  }
  if (write_archive_copy) {
    MaterializeOutcome archive = archive_instance(data, archive_root);
    if (!archive.ok) {
      outcome.ok = false;
      for (const std::string& issue : archive.issues) {
        outcome.issues.push_back(issue);
      }
      return outcome;
    }
    outcome.archive_written = archive.archive_written;
    outcome.archive_relative = archive.archive_relative;
  }
  return outcome;
}

// 校验 output_dir 中已有的物化文件与内存生成内容一致（--check / 正式运行）。
std::vector<std::string> verify_materialized(
    const std::vector<std::unique_ptr<InstanceData>>& data,
    const std::string& output_dir, const std::string& archive_root) {
  std::vector<std::string> issues;
  for (const auto& entry : data) {
    const std::string file_name = entry->instance.spec.instance_id + ".graph.txt";
    const std::string file_path = output_dir + "/graphs/" + file_name;
    std::error_code filesystem_error;
    if (fs::exists(file_path, filesystem_error)) {
      std::size_t size = 0;
      std::string error;
      const std::string sha = sha256_file_hex(file_path, &size, &error);
      if (!error.empty()) {
        issues.push_back(file_name + ": " + error);
      } else if (sha != entry->serialized_sha256 || size != entry->serialized_size) {
        issues.push_back(file_name +
                         ": materialized file does not match deterministic "
                         "regeneration (sha256 " +
                         sha + " vs " + entry->serialized_sha256 + ")");
      }
    }
    // 实例表引用归档时校验归档文件（若存在）。
    const std::string table_path = output_dir + "/graph_instances.json";
    if (fs::exists(table_path, filesystem_error)) {
      std::string content;
      std::string error;
      if (read_file(table_path, &content, &error)) {
        try {
          const nlohmann::json table = nlohmann::json::parse(content);
          for (const auto& record : table.at("instances")) {
            if (record.at("instance_id").get<std::string>() !=
                entry->instance.spec.instance_id) {
              continue;
            }
            if (record.contains("archive") && record.at("archive").is_object()) {
              const std::string archive_path =
                  record.at("archive").at("relative_path").get<std::string>();
              const std::string archive_sha =
                  record.at("archive").at("sha256").get<std::string>();
              if (fs::exists(archive_path, filesystem_error)) {
                std::size_t size = 0;
                std::string sha_error;
                const std::string sha =
                    sha256_file_hex(archive_path, &size, &sha_error);
                if (!sha_error.empty()) {
                  issues.push_back(archive_path + ": " + sha_error);
                } else if (sha != archive_sha) {
                  issues.push_back(archive_path + ": archive sha256 mismatch");
                }
              }
            }
          }
        } catch (const std::exception& exception) {
          issues.push_back("graph_instances.json: parse error: " +
                           std::string(exception.what()));
        }
      } else {
        issues.push_back(error);
      }
    }
    (void)archive_root;
  }
  return issues;
}

// ---------------------------------------------------------------------------
// Trial 执行
// ---------------------------------------------------------------------------

void fill_common_row(TrialRow* row, const BackendSpec& backend,
                     const Instance& instance, const std::string& cache_mode,
                     int repeat_id, bool measured, double budget_s) {
  row->group = backend.group;
  row->backend_id = backend.id;
  row->variant = backend.variant;
  row->kind = backend.kind;
  row->instance_id = instance.spec.instance_id;
  row->scale = instance.spec.scale;
  row->graph_class = instance.spec.graph_class;
  row->cache_mode = cache_mode;
  row->repeat_id = repeat_id;
  row->seed = instance.spec.seed;
  row->measured = measured;
  row->budget_s = budget_s;
}

void fill_timing(TrialRow* row, double construct_ms,
                 std::chrono::steady_clock::time_point wall_start,
                 std::chrono::steady_clock::time_point wall_end,
                 std::clock_t cpu_start, std::clock_t cpu_end,
                 const planner::graph_search::Stats& stats,
                 RssSampler* sampler) {
  row->construct_ms = construct_ms;
  row->wall_ms = elapsed_ms(wall_start, wall_end);
  row->cpu_ms = cpu_ms_between(cpu_start, cpu_end);
  row->e2e_ms = (std::isfinite(construct_ms) ? construct_ms : 0.0) + row->wall_ms;
  row->adapter_wall_ms = stats.wall_ms;
  row->preprocess_ms = stats.preprocess_ms;
  row->search_ms = stats.search_ms;
  if (std::isfinite(stats.preprocess_ms) && std::isfinite(stats.search_ms)) {
    row->postprocess_ms =
        std::max(0.0, row->wall_ms - stats.preprocess_ms - stats.search_ms);
  }
  row->expanded_nodes = stats.expanded_nodes;
  row->peak_rss_bytes = sampler->end();
  row->rss_hwm_bytes = read_proc_status_bytes("VmHWM:");
}

TrialRow run_p2p_trial(const BackendSpec& backend, const Instance& instance,
                       const PairSpec& pair, const EdgeIndex& index,
                       double reference_cost, const Heuristic& heuristic,
                       GraphSearcher* reusable, const std::string& cache_mode,
                       int repeat_id, bool measured, double budget_s,
                       RssSampler* sampler) {
  TrialRow row;
  fill_common_row(&row, backend, instance, cache_mode, repeat_id, measured,
                  budget_s);
  row.pair_id = pair.pair_id;
  row.expected = pair.expected;
  row.case_id = instance.spec.instance_id + "/" + pair.pair_id;
  row.hops = pair.hops;

  std::unique_ptr<GraphSearcher> owned;
  GraphSearcher* searcher = reusable;
  double construct_ms = std::numeric_limits<double>::quiet_NaN();
  if (searcher == nullptr) {
    const auto construct_start = std::chrono::steady_clock::now();
    owned = create_p2p_searcher(backend.id);
    construct_ms = elapsed_ms(construct_start, std::chrono::steady_clock::now());
    searcher = owned.get();
  }
  if (searcher == nullptr) {
    row.status = "invalid_input";
    row.failure_reason = "backend is not compiled into this benchmark";
    return row;
  }

  Control control;
  control.budget_s = budget_s;
  sampler->begin();
  const auto wall_start = std::chrono::steady_clock::now();
  const std::clock_t cpu_start = std::clock();
  const Result result =
      searcher->search(instance.graph, pair.start, pair.goal, heuristic, control);
  const auto wall_end = std::chrono::steady_clock::now();
  const std::clock_t cpu_end = std::clock();
  fill_timing(&row, construct_ms, wall_start, wall_end, cpu_start, cpu_end,
              result.stats, sampler);

  row.status = planner::graph_search::status_to_string(result.status);
  row.timeout = result.status == Status::kTimeout;
  row.native_cost = result.cost;
  if (pair.expected == "reachable") {
    row.reference_cost = reference_cost;
    if (result.status == Status::kSuccess) {
      bool edge_missing = false;
      const double recomputed = index.path_cost(result.path, &edge_missing);
      row.path_edges = static_cast<long long>(result.path.size()) - 1;
      row.recomputed_cost = recomputed;
      if (std::isfinite(reference_cost) && reference_cost > 0.0 &&
          std::isfinite(recomputed)) {
        row.cost_delta_rel = (recomputed - reference_cost) / reference_cost;
      }
      row.path_length_m = recomputed;
      const double gate =
          backend.float32_native
              ? planner::graph_search::bench::float32_registered_tolerance(
                    reference_cost,
                    result.path.size() > 0 ? result.path.size() - 1 : 0)
              : planner::graph_search::cost_tolerance(reference_cost);
      const bool path_ok =
          !edge_missing && !result.path.empty() &&
          result.path.front() == pair.start && result.path.back() == pair.goal;
      const bool recomputed_ok =
          std::isfinite(recomputed) &&
          std::fabs(recomputed - reference_cost) <= gate;
      const bool native_ok = std::isfinite(result.cost) &&
                             std::fabs(result.cost - reference_cost) <= gate;
      row.valid = path_ok && recomputed_ok && native_ok;
      if (!row.valid) {
        std::ostringstream reason;
        reason << "correctness gate failed";
        if (!path_ok) {
          reason << "; path invalid (edges=" << row.path_edges
                 << ", missing=" << (edge_missing ? "true" : "false") << ")";
        }
        if (!recomputed_ok) {
          reason << "; recomputed " << number_field(recomputed)
                 << " vs reference " << number_field(reference_cost)
                 << " gate " << number_field(gate);
        }
        if (!native_ok) {
          reason << "; native " << number_field(result.cost)
                 << " vs reference " << number_field(reference_cost)
                 << " gate " << number_field(gate);
        }
        row.failure_reason = reason.str();
      }
    } else {
      row.valid = false;
      row.failure_reason = "expected success, got " + row.status;
      if (result.status == Status::kNoPath || result.status == Status::kTimeout) {
        row.failure_reason += " (" + result.message + ")";
      }
    }
  } else if (pair.expected == "unreachable") {
    row.valid = result.status == Status::kNoPath;
    if (!row.valid) {
      row.failure_reason = "expected no_path, got " + row.status;
    }
  } else {  // zero_cost
    row.valid = result.status == Status::kSuccess && result.path.size() == 1 &&
                result.path.front() == pair.start &&
                std::fabs(result.cost) <= 1e-8;
    row.path_length_m = row.valid ? 0.0
                                  : std::numeric_limits<double>::quiet_NaN();
    if (!row.valid) {
      row.failure_reason = "expected zero-cost single-vertex path, got " +
                           row.status;
    }
  }
  return row;
}

TrialRow run_df_trial(const BackendSpec& backend, const Instance& instance,
                      const std::string& source_id, int source_vertex,
                      const ReferenceField& reference,
                      DistanceFieldSearcher* reusable,
                      const std::string& cache_mode, int repeat_id,
                      bool measured, double budget_s, RssSampler* sampler) {
  TrialRow row;
  fill_common_row(&row, backend, instance, cache_mode, repeat_id, measured,
                  budget_s);
  row.source_id = source_id;
  row.expected = "distance_field";
  row.case_id = instance.spec.instance_id + "/" + source_id;

  std::unique_ptr<DistanceFieldSearcher> owned;
  DistanceFieldSearcher* searcher = reusable;
  double construct_ms = std::numeric_limits<double>::quiet_NaN();
  if (searcher == nullptr) {
    const auto construct_start = std::chrono::steady_clock::now();
    owned = create_df_searcher(backend.id);
    construct_ms = elapsed_ms(construct_start, std::chrono::steady_clock::now());
    searcher = owned.get();
  }
  if (searcher == nullptr) {
    row.status = "invalid_input";
    row.failure_reason = "backend is not compiled into this benchmark";
    return row;
  }

  Control control;
  control.budget_s = budget_s;
  sampler->begin();
  const auto wall_start = std::chrono::steady_clock::now();
  const std::clock_t cpu_start = std::clock();
  const DistanceFieldResult result =
      searcher->search(instance.graph, source_vertex, control);
  const auto wall_end = std::chrono::steady_clock::now();
  const std::clock_t cpu_end = std::clock();
  fill_timing(&row, construct_ms, wall_start, wall_end, cpu_start, cpu_end,
              result.stats, sampler);

  row.status = planner::graph_search::status_to_string(result.status);
  row.timeout = result.status == Status::kTimeout;
  if (result.status == Status::kSuccess) {
    long long unreachable = 0;
    double max_finite = 0.0;
    bool mismatched = false;
    for (int v = 0; v < instance.graph.num_vertices; ++v) {
      const double measured_distance = result.distances[static_cast<std::size_t>(v)];
      const double reference_distance = reference.dist[static_cast<std::size_t>(v)];
      const bool measured_finite = std::isfinite(measured_distance);
      const bool reference_finite = std::isfinite(reference_distance);
      if (!measured_finite) {
        ++unreachable;
      } else {
        max_finite = std::max(max_finite, measured_distance);
      }
      if (measured_finite != reference_finite) {
        mismatched = true;
        continue;
      }
      if (measured_finite &&
          std::fabs(measured_distance - reference_distance) >
              planner::graph_search::cost_tolerance(reference_distance)) {
        mismatched = true;
      }
    }
    row.unreachable_cells = unreachable;
    row.max_distance_m = max_finite;
    row.reference_cost = reference.max_finite;
    row.valid = !mismatched;
    if (mismatched) {
      row.failure_reason = "distance field mismatches reference";
    }
  } else {
    row.valid = false;
    row.failure_reason = "expected success, got " + row.status;
  }
  return row;
}

// ---------------------------------------------------------------------------
// 聚合与输出
// ---------------------------------------------------------------------------

struct Aggregate {
  std::string group;
  std::string backend_id;
  std::string variant;
  std::string kind;
  std::string provenance;
  long long rows = 0;
  long long measured_rows = 0;
  long long successes = 0;
  long long no_path = 0;
  long long timeouts = 0;
  long long invalid = 0;
  long long valid_successes = 0;
  long long negative_runs = 0;
  long long negative_ok = 0;
  long long negative_wrong = 0;
  double total_cpu_ms = 0.0;
  long long max_peak_rss_bytes = 0;
  long long max_expanded_nodes = -1;
  std::vector<double> success_wall_ms;
  std::vector<double> success_search_ms;
  std::vector<double> success_preprocess_ms;
  std::vector<double> success_postprocess_ms;
  std::vector<double> construct_ms;
  std::vector<double> cost_delta_rel;
  std::map<std::string, std::vector<double>> case_wall_ms;  // key: inst|pair|mode
  long long reachable_cases = 0;
  long long reachable_cases_with_success = 0;
};

std::string case_key(const TrialRow& row) {
  const std::string case_suffix =
      row.pair_id.empty() ? row.source_id : row.pair_id;
  return row.instance_id + "|" + case_suffix + "|" + row.cache_mode;
}

Aggregate& aggregate_for(std::map<std::string, Aggregate>* aggregates,
                         const TrialRow& row,
                         const std::map<std::string, BackendSpec>& registry) {
  Aggregate& aggregate = (*aggregates)[row.backend_id];
  aggregate.backend_id = row.backend_id;
  aggregate.group = row.group;
  aggregate.variant = row.variant;
  aggregate.kind = row.kind;
  const auto found = registry.find(row.backend_id);
  if (found != registry.end()) {
    aggregate.provenance = found->second.provenance;
  }
  return aggregate;
}

void accumulate(const std::vector<TrialRow>& rows,
                std::map<std::string, Aggregate>* aggregates,
                const std::map<std::string, BackendSpec>& registry) {
  for (const TrialRow& row : rows) {
    Aggregate& aggregate = aggregate_for(aggregates, row, registry);
    ++aggregate.rows;
    if (row.measured) {
      ++aggregate.measured_rows;
    }
    aggregate.total_cpu_ms += std::isfinite(row.cpu_ms) ? row.cpu_ms : 0.0;
    aggregate.max_peak_rss_bytes =
        std::max(aggregate.max_peak_rss_bytes, row.peak_rss_bytes);
    aggregate.max_expanded_nodes =
        std::max(aggregate.max_expanded_nodes, row.expanded_nodes);
    if (row.status == "success") {
      ++aggregate.successes;
    } else if (row.status == "no_path") {
      ++aggregate.no_path;
    } else if (row.status == "timeout") {
      ++aggregate.timeouts;
    } else {
      ++aggregate.invalid;
    }
    if (!row.measured) {
      ++aggregate.negative_runs;
      if (row.valid) {
        ++aggregate.negative_ok;
      } else {
        ++aggregate.negative_wrong;
      }
      continue;
    }
    if (row.valid && row.status == "success") {
      ++aggregate.valid_successes;
      aggregate.success_wall_ms.push_back(row.wall_ms);
      if (std::isfinite(row.search_ms)) {
        aggregate.success_search_ms.push_back(row.search_ms);
      }
      if (std::isfinite(row.preprocess_ms)) {
        aggregate.success_preprocess_ms.push_back(row.preprocess_ms);
      }
      if (std::isfinite(row.postprocess_ms)) {
        aggregate.success_postprocess_ms.push_back(row.postprocess_ms);
      }
      if (std::isfinite(row.cost_delta_rel)) {
        aggregate.cost_delta_rel.push_back(row.cost_delta_rel);
      }
      aggregate.case_wall_ms[case_key(row)].push_back(row.wall_ms);
    }
    if (std::isfinite(row.construct_ms)) {
      aggregate.construct_ms.push_back(row.construct_ms);
    }
  }
}

nlohmann::json aggregate_json(const Aggregate& aggregate,
                              const std::map<std::string, Aggregate>& all,
                              bool rank_eligible) {
  nlohmann::json out;
  out["backend_id"] = aggregate.backend_id;
  out["group"] = aggregate.group;
  out["variant"] = aggregate.variant;
  out["kind"] = aggregate.kind;
  out["provenance"] = aggregate.provenance;
  out["rows"] = aggregate.rows;
  out["measured_rows"] = aggregate.measured_rows;
  out["successes"] = aggregate.successes;
  out["no_path"] = aggregate.no_path;
  out["timeouts"] = aggregate.timeouts;
  out["invalid"] = aggregate.invalid;
  out["valid_successes"] = aggregate.valid_successes;
  out["negative_runs"] = aggregate.negative_runs;
  out["negative_ok"] = aggregate.negative_ok;
  out["negative_wrong"] = aggregate.negative_wrong;
  const long long denominator =
      aggregate.successes + aggregate.no_path + aggregate.timeouts;
  out["success_rate"] =
      denominator > 0
          ? nlohmann::json(static_cast<double>(aggregate.successes) /
                           static_cast<double>(denominator))
          : nlohmann::json(nullptr);
  out["p50_wall_ms"] = percentile_nearest_rank(aggregate.success_wall_ms, 50.0);
  out["p95_wall_ms"] = percentile_nearest_rank(aggregate.success_wall_ms, 95.0);
  out["p50_search_ms"] = percentile_nearest_rank(aggregate.success_search_ms, 50.0);
  out["p95_search_ms"] = percentile_nearest_rank(aggregate.success_search_ms, 95.0);
  out["p50_preprocess_ms"] =
      percentile_nearest_rank(aggregate.success_preprocess_ms, 50.0);
  out["p50_postprocess_ms"] =
      percentile_nearest_rank(aggregate.success_postprocess_ms, 50.0);
  out["p50_construct_ms"] = percentile_nearest_rank(aggregate.construct_ms, 50.0);
  out["total_cpu_ms"] = aggregate.total_cpu_ms;
  out["max_peak_rss_bytes"] = aggregate.max_peak_rss_bytes;
  out["max_expanded_nodes"] = aggregate.max_expanded_nodes;
  if (aggregate.cost_delta_rel.empty()) {
    out["mean_cost_delta_rel"] = nullptr;
    out["max_abs_cost_delta_rel"] = nullptr;
  } else {
    double sum = 0.0;
    double maximum = 0.0;
    for (const double value : aggregate.cost_delta_rel) {
      sum += value;
      maximum = std::max(maximum, std::fabs(value));
    }
    out["mean_cost_delta_rel"] = sum / aggregate.cost_delta_rel.size();
    out["max_abs_cost_delta_rel"] = maximum;
  }
  // 每个 (实例, 对, 模式) 的冷启动 P95 等权均值（§7.2 口径）。
  double p95_sum = 0.0;
  long long p95_cases = 0;
  double p95_sum_all = 0.0;
  long long p95_cases_all = 0;
  long long cases_with_success = 0;
  for (const auto& entry : aggregate.case_wall_ms) {
    const double p95 = percentile_nearest_rank(entry.second, 95.0);
    p95_sum_all += p95;
    ++p95_cases_all;
    if (entry.first.size() > 4 &&
        entry.first.compare(entry.first.size() - 4, 4, "cold") == 0) {
      p95_sum += p95;
      ++p95_cases;
    }
    if (!entry.second.empty()) {
      ++cases_with_success;
    }
  }
  out["mean_case_p95_cold_ms"] =
      p95_cases > 0 ? nlohmann::json(p95_sum / static_cast<double>(p95_cases))
                    : nlohmann::json(nullptr);
  out["mean_case_p95_all_ms"] =
      p95_cases_all > 0 ? nlohmann::json(p95_sum_all /
                                         static_cast<double>(p95_cases_all))
                        : nlohmann::json(nullptr);
  out["cases_with_success"] = cases_with_success;
  out["ranking_eligible"] = rank_eligible;
  (void)all;
  return out;
}

std::string svg_bar_chart(const std::string& title,
                          const std::vector<std::pair<std::string, double>>& data,
                          const std::string& unit) {
  double maximum = 0.0;
  for (const auto& entry : data) {
    maximum = std::max(maximum, entry.second);
  }
  if (!(maximum > 0.0)) {
    maximum = 1.0;
  }
  std::ostringstream svg;
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"820\" height=\""
      << 60 + 30 * data.size()
      << "\" font-family=\"sans-serif\" font-size=\"12\">\n";
  svg << "<text x=\"10\" y=\"20\" font-size=\"14\">" << title << "</text>\n";
  double y = 40;
  for (const auto& entry : data) {
    const double width = 520.0 * entry.second / maximum;
    svg << "<text x=\"10\" y=\"" << y + 12 << "\">" << entry.first << "</text>\n";
    svg << "<rect x=\"230\" y=\"" << y << "\" width=\"" << width
        << "\" height=\"16\" fill=\"#4477aa\"/>\n";
    svg << "<text x=\"" << 240 + width << "\" y=\"" << y + 12 << "\">"
        << std::fixed << std::setprecision(3) << entry.second << " " << unit
        << "</text>\n";
    y += 30;
  }
  svg << "</svg>\n";
  return svg.str();
}

void write_group_plots(const std::string& output_dir, const std::string& group,
                       const std::vector<Aggregate>& aggregates) {
  const std::string plots_dir = output_dir + "/plots";
  std::error_code filesystem_error;
  fs::create_directories(plots_dir, filesystem_error);
  if (filesystem_error) {
    std::fprintf(stderr, "warning: cannot create plots dir: %s\n",
                 filesystem_error.message().c_str());
    return;
  }
  std::vector<std::pair<std::string, double>> p95_data;
  std::vector<std::pair<std::string, double>> p50_data;
  std::ofstream csv(plots_dir + "/p50_p95_" + group + ".csv");
  csv << "backend_id,p50_ms,p95_ms,success_rate,rows\n";
  for (const Aggregate& aggregate : aggregates) {
    if (aggregate.group != group) {
      continue;
    }
    const double p95 = percentile_nearest_rank(aggregate.success_wall_ms, 95.0);
    const double p50 = percentile_nearest_rank(aggregate.success_wall_ms, 50.0);
    if (std::isfinite(p95)) {
      p95_data.push_back({aggregate.backend_id, p95});
    }
    if (std::isfinite(p50)) {
      p50_data.push_back({aggregate.backend_id, p50});
    }
    const long long denominator =
        aggregate.successes + aggregate.no_path + aggregate.timeouts;
    csv << aggregate.backend_id << "," << number_field(p50) << ","
        << number_field(p95) << ","
        << (denominator > 0
                ? number_field(static_cast<double>(aggregate.successes) /
                               static_cast<double>(denominator))
                : "")
        << "," << aggregate.rows << "\n";
  }
  std::ofstream(plots_dir + "/p95_wall_" + group + ".svg")
      << svg_bar_chart("wall_ms P95 (cold+warm valid successes) - " + group,
                       p95_data, "ms");
  std::ofstream(plots_dir + "/p50_wall_" + group + ".svg")
      << svg_bar_chart("wall_ms P50 (cold+warm valid successes) - " + group,
                       p50_data, "ms");
}

// ---------------------------------------------------------------------------
// 矩阵运行
// ---------------------------------------------------------------------------

int run_benchmark(const Options& options,
                  const std::vector<InstanceSpec>& instance_specs,
                  const std::vector<BackendSpec>& backends,
                  const std::string& output_dir) {
  std::error_code filesystem_error;
  fs::create_directories(output_dir, filesystem_error);
  if (filesystem_error) {
    std::fprintf(stderr, "cannot create output dir %s: %s\n", output_dir.c_str(),
                 filesystem_error.message().c_str());
    return 1;
  }

  std::vector<std::unique_ptr<InstanceData>> instances;
  std::string error;
  if (!prepare_instances(instance_specs, &instances, &error)) {
    std::fprintf(stderr, "instance preparation failed: %s\n", error.c_str());
    return 1;
  }
  const std::vector<std::string> materialize_issues =
      verify_materialized(instances, output_dir, options.archive_root);
  if (!materialize_issues.empty()) {
    for (const std::string& issue : materialize_issues) {
      std::fprintf(stderr, "materialized input mismatch: %s\n", issue.c_str());
    }
    return 1;
  }

  // >10 MiB 输入按 §3.4 写入 SHA256 寻址归档（输入生成与磁盘输出在计时外）。
  std::map<std::string, std::string> archive_paths;
  for (const auto& entry : instances) {
    if (entry->serialized_size <= kArchiveThresholdBytes) {
      continue;
    }
    MaterializeOutcome archive =
        archive_instance(*entry, options.archive_root);
    if (!archive.ok) {
      for (const std::string& issue : archive.issues) {
        std::fprintf(stderr, "archive %s: %s\n",
                     entry->instance.spec.instance_id.c_str(), issue.c_str());
      }
      return 1;
    }
    archive_paths[entry->instance.spec.instance_id] = archive.archive_relative;
    std::printf("archived %s -> %s (%zu bytes)\n",
                entry->instance.spec.instance_id.c_str(),
                archive.archive_relative.c_str(), entry->serialized_size);
  }

  // 实例表（即使未物化也写；sha256 为序列化内容哈希）。
  {
    nlohmann::json table;
    table["schema_version"] = 1;
    table["type"] = "graph_gen_instance_table";
    table["generator"] = {{"name", "graph_gen"},
                          {"version", planner::graph_search::bench::kGraphGenVersion},
                          {"prng", "splitmix64"},
                          {"rules", "Src/graph_search/benchmarks/README.md"}};
    table["generated_by_run_id"] = run_id();
    nlohmann::json entries = nlohmann::json::array();
    for (const auto& entry : instances) {
      std::string relative;
      if (fs::exists(output_dir + "/graphs/" +
                         entry->instance.spec.instance_id + ".graph.txt")) {
        relative = "graphs/" + entry->instance.spec.instance_id + ".graph.txt";
      }
      const auto archive_found =
          archive_paths.find(entry->instance.spec.instance_id);
      entries.push_back(instance_record_json(
          *entry, run_id(), relative,
          archive_found != archive_paths.end() ? archive_found->second : ""));
    }
    table["instances"] = std::move(entries);
    std::string write_error;
    if (!write_file(output_dir + "/graph_instances.json",
                    table.dump(2) + "\n", &write_error)) {
      std::fprintf(stderr, "%s\n", write_error.c_str());
      return 1;
    }
  }
  {
    nlohmann::json checks;
    checks["schema_version"] = 1;
    checks["type"] = "graph_gen_instance_checks";
    checks["all_passed"] = true;
    nlohmann::json entries = nlohmann::json::array();
    for (const auto& entry : instances) {
      nlohmann::json record;
      record["instance_id"] = entry->instance.spec.instance_id;
      record["passed"] = entry->check_issues.empty();
      record["issues"] = entry->check_issues;
      record["serialized_sha256"] = entry->serialized_sha256;
      record["serialized_size_bytes"] = entry->serialized_size;
      if (!entry->check_issues.empty()) {
        checks["all_passed"] = false;
      }
      entries.push_back(std::move(record));
    }
    checks["instances"] = std::move(entries);
    std::string write_error;
    if (!write_file(output_dir + "/graph_instance_checks.json",
                    checks.dump(2) + "\n", &write_error)) {
      std::fprintf(stderr, "%s\n", write_error.c_str());
      return 1;
    }
    if (!checks["all_passed"].get<bool>()) {
      for (const auto& entry : instances) {
        for (const std::string& issue : entry->check_issues) {
          std::fprintf(stderr, "instance check failed: %s\n", issue.c_str());
        }
      }
      return 1;
    }
  }

  // 预算与缓存模式。
  const bool run_cold = std::find(options.cache_modes.begin(),
                                  options.cache_modes.end(),
                                  "cold") != options.cache_modes.end();
  const bool run_warm = std::find(options.cache_modes.begin(),
                                  options.cache_modes.end(),
                                  "warm") != options.cache_modes.end();
  if (!run_cold && !run_warm) {
    std::fprintf(stderr, "no cache mode selected (cold/warm)\n");
    return 2;
  }

  RssSampler sampler(options.rss_sample_us);
  sampler.start();

  std::vector<TrialRow> rows;
  const auto started_at = utc_now_iso();

  // 点到点组（含零启发式对照）。
  for (const BackendSpec& backend : backends) {
    if (backend.kind == "distance_field") {
      continue;
    }
    const bool is_control = backend.group == "zero_heuristic";
    std::printf("[p2p] backend=%s variant=%s%s\n", backend.id.c_str(),
                backend.variant.c_str(), is_control ? " (control)" : "");
    std::fflush(stdout);
    for (const auto& entry : instances) {
      const Instance& instance = entry->instance;
      for (const PairSpec& pair : instance.pairs) {
        const bool reachable = pair.expected == "reachable";
        if (reachable) {
          const auto cost_found = entry->pair_reference_cost.find(pair.pair_id);
          const double reference_cost =
              cost_found != entry->pair_reference_cost.end()
                  ? cost_found->second
                  : std::numeric_limits<double>::quiet_NaN();
          // 与负例分支同口径：仅使用启发式的后端取预计算欧氏启发式；
          // Dijkstra 语义与零启发式对照（uses_heuristic=false）用零启发式。
          const auto heuristic_found =
              entry->pair_heuristics.find(pair_goal_key(pair.goal));
          const Heuristic heuristic =
              backend.uses_heuristic
                  ? (heuristic_found != entry->pair_heuristics.end()
                         ? heuristic_found->second
                         : planner::graph_search::zero_heuristic())
                  : planner::graph_search::zero_heuristic();
          for (int mode = 0; mode < 2; ++mode) {
            const bool cold = mode == 0;
            if ((cold && !run_cold) || (!cold && !run_warm)) {
              continue;
            }
            const std::string cache_mode = cold ? "cold" : "warm";
            std::unique_ptr<GraphSearcher> warm_searcher;
            if (!cold) {
              warm_searcher = create_p2p_searcher(backend.id);
            }
            for (int trial = 0; trial < options.warmup + options.repeats;
                 ++trial) {
              const bool measured = trial >= options.warmup;
              const int repeat_id = trial - options.warmup;
              TrialRow row = run_p2p_trial(
                  backend, instance, pair, entry->index, reference_cost,
                  heuristic, cold ? nullptr : warm_searcher.get(), cache_mode,
                  repeat_id, measured, options.p2p_budget_s, &sampler);
              if (measured) {
                rows.push_back(std::move(row));
              }
            }
          }
        } else {
          // 负例/边界对：单次运行（冷启动），只记录状态与终止耗时。
          const double reference_cost =
              pair.expected == "zero_cost" ? 0.0
                                           : std::numeric_limits<double>::quiet_NaN();
          // 使用与主组相同的启发式配置（A* 用欧氏；Dijkstra/对照用零）。
          const Heuristic heuristic =
              backend.uses_heuristic
                  ? Heuristic(planner::graph_search::EuclideanHeuristic(
                        instance.xs, instance.ys, pair.goal))
                  : planner::graph_search::zero_heuristic();
          TrialRow row = run_p2p_trial(backend, instance, pair, entry->index,
                                       reference_cost, heuristic, nullptr,
                                       "cold", 0, false, options.p2p_budget_s,
                                       &sampler);
          rows.push_back(std::move(row));
        }
      }
    }
  }

  // 距离场组（3 个后端 × 2 个源点）。
  for (const BackendSpec& backend : backends) {
    if (backend.kind != "distance_field") {
      continue;
    }
    std::printf("[distance_field] backend=%s\n", backend.id.c_str());
    std::fflush(stdout);
    for (const auto& entry : instances) {
      const Instance& instance = entry->instance;
      const std::vector<std::pair<std::string, int>> sources = {
          {s01_source_id(), 0}, {entry->s02_source_id, entry->s02_vertex}};
      for (const auto& source : sources) {
        const auto reference_found = entry->source_reference.find(source.first);
        if (reference_found == entry->source_reference.end()) {
          std::fprintf(stderr, "missing reference for source %s\n",
                       source.first.c_str());
          return 1;
        }
        const ReferenceField& reference = reference_found->second;
        for (int mode = 0; mode < 2; ++mode) {
          const bool cold = mode == 0;
          if ((cold && !run_cold) || (!cold && !run_warm)) {
            continue;
          }
          const std::string cache_mode = cold ? "cold" : "warm";
          std::unique_ptr<DistanceFieldSearcher> warm_searcher;
          if (!cold) {
            warm_searcher = create_df_searcher(backend.id);
          }
          for (int trial = 0; trial < options.warmup + options.repeats; ++trial) {
            const bool measured = trial >= options.warmup;
            const int repeat_id = trial - options.warmup;
            TrialRow row = run_df_trial(
                backend, instance, source.first, source.second, reference,
                cold ? nullptr : warm_searcher.get(), cache_mode, repeat_id,
                measured, options.field_budget_s, &sampler);
            if (measured) {
              rows.push_back(std::move(row));
            }
          }
        }
      }
    }
  }
  sampler.stop();

  write_metrics_csv(output_dir + "/benchmark_metrics.csv", rows);

  // 聚合。
  std::map<std::string, BackendSpec> registry;
  for (const BackendSpec& backend : backends) {
    registry[backend.id] = backend;
  }
  std::map<std::string, Aggregate> aggregates;
  accumulate(rows, &aggregates, registry);

  // 可排名性：主组、无 invalid、无 timeout、每个用例都有有效成功、
  // 负例状态正确。
  std::map<std::string, bool> rank_eligible;
  for (auto& entry : aggregates) {
    Aggregate& aggregate = entry.second;
    bool eligible = aggregate.group == "p2p" && aggregate.invalid == 0 &&
                    aggregate.timeouts == 0 && aggregate.negative_wrong == 0 &&
                    aggregate.valid_successes > 0;
    if (eligible) {
      // 每个 (实例, 对) 的冷启动用例都要有有效成功。
      std::set<std::string> reachable_cases;
      std::set<std::string> success_cases;
      for (const TrialRow& row : rows) {
        if (row.backend_id != aggregate.backend_id || !row.measured ||
            row.expected != "reachable") {
          continue;
        }
        reachable_cases.insert(row.instance_id + "|" + row.pair_id);
        if (row.valid && row.status == "success") {
          success_cases.insert(row.instance_id + "|" + row.pair_id);
        }
      }
      eligible = reachable_cases == success_cases;
    }
    rank_eligible[entry.first] = eligible;
  }

  // 排名（信息性；正式选型属独占窗口阶段，零启发式对照与距离场不参与
  // 点到点排名）。
  std::vector<std::pair<std::string, double>> p2p_ranking;
  for (const auto& entry : aggregates) {
    const Aggregate& aggregate = entry.second;
    if (aggregate.group != "p2p" || !rank_eligible[entry.first]) {
      continue;
    }
    double p95_sum = 0.0;
    long long p95_cases = 0;
    for (const auto& case_entry : aggregate.case_wall_ms) {
      if (case_entry.first.size() <= 4 ||
          case_entry.first.compare(case_entry.first.size() - 4, 4, "cold") != 0) {
        continue;
      }
      p95_sum += percentile_nearest_rank(case_entry.second, 95.0);
      ++p95_cases;
    }
    if (p95_cases > 0) {
      p2p_ranking.push_back(
          {entry.first, p95_sum / static_cast<double>(p95_cases)});
    }
  }
  std::sort(p2p_ranking.begin(), p2p_ranking.end(),
            [](const auto& a, const auto& b) {
              if (a.second != b.second) {
                return a.second < b.second;
              }
              return a.first < b.first;
            });

  // summary。
  nlohmann::json summary;
  summary["schema_version"] = 1;
  summary["type"] = "graph_search_bench_summary";
  summary["suite"] = options.suite;
  summary["run_id"] = run_id();
  summary["started_at"] = started_at;
  summary["finished_at"] = utc_now_iso();
  summary["host"] = hostname();
  summary["smoke"] = options.smoke;
  summary["generator"] = {{"name", "graph_gen"},
                          {"version", planner::graph_search::bench::kGraphGenVersion},
                          {"prng", "splitmix64"}};
  summary["protocol"] = {
      {"warmup", options.warmup},
      {"repeats", options.repeats},
      {"p2p_budget_s", options.p2p_budget_s},
      {"field_budget_s", options.field_budget_s},
      {"cache_modes", options.cache_modes},
      {"rss_sample_us", options.rss_sample_us},
      {"percentile", "nearest-rank"},
      {"ranking_basis",
       "冷启动每 (实例, 对, 源点) P95 等权均值；信息性排名，正式选型属独占窗口阶段；零启发式对照与距离场不参与点到点排名"},
      {"correctness_gate",
       "返回路径逐边存在 + double 重算代价与独立参考 Dijkstra 一致（§6.2 容差；justinhj 用已登记 float32 门槛）"},
      {"negative_cases", "断连/同点单次运行，不进排名"}};
  nlohmann::json instance_json = nlohmann::json::array();
  for (const auto& entry : instances) {
    std::string relative;
    if (fs::exists(output_dir + "/graphs/" +
                       entry->instance.spec.instance_id + ".graph.txt")) {
      relative = "graphs/" + entry->instance.spec.instance_id + ".graph.txt";
    }
    const auto archive_found =
        archive_paths.find(entry->instance.spec.instance_id);
    instance_json.push_back(instance_record_json(
        *entry, run_id(), relative,
        archive_found != archive_paths.end() ? archive_found->second : ""));
  }
  summary["instances"] = std::move(instance_json);
  summary["checks"] = {{"all_passed", true},
                       {"materialized_mismatches", materialize_issues}};

  nlohmann::json group_json;
  const std::vector<std::string> groups = {"p2p", "zero_heuristic",
                                           "distance_field"};
  for (const std::string& group : groups) {
    nlohmann::json backends_json = nlohmann::json::object();
    for (const auto& entry : aggregates) {
      if (entry.second.group != group) {
        continue;
      }
      backends_json[entry.first] =
          aggregate_json(entry.second, aggregates, rank_eligible[entry.first]);
    }
    group_json[group]["backends"] = std::move(backends_json);
    if (group == "p2p") {
      nlohmann::json ranking = nlohmann::json::array();
      for (const auto& entry : p2p_ranking) {
        ranking.push_back({{"backend_id", entry.first},
                           {"mean_case_p95_cold_ms", entry.second}});
      }
      group_json[group]["ranking"] = std::move(ranking);
    }
  }
  summary["groups"] = std::move(group_json);

  nlohmann::json negative = nlohmann::json::array();
  long long measured_rows = 0;
  for (const TrialRow& row : rows) {
    if (row.measured) {
      ++measured_rows;
      continue;
    }
    negative.push_back(
        {{"backend_id", row.backend_id},
         {"instance_id", row.instance_id},
         {"pair_id", row.pair_id},
         {"expected", row.expected},
         {"status", row.status},
         {"wall_ms", row.wall_ms},
         {"timeout", row.timeout},
         {"valid", row.valid},
         {"failure_reason", row.failure_reason}});
  }
  summary["negative_cases"] = std::move(negative);
  summary["totals"] = {{"rows", rows.size()},
                       {"measured_rows", measured_rows},
                       {"warmup_runs_per_case",
                        static_cast<long long>(options.warmup)},
                       {"cache_modes", options.cache_modes}};
  summary["notes"] = {
      "图正式档矩阵：6 实例 × 8 可达对 × (预热+测量) × 冷/热；点到点 7 主后端 + 零启发式对照；距离场 3 后端 × 2 源点",
      "本运行不产生最终选型结论；选型在独占窗口阶段按 §7.2 口径进行",
      "justinhj 原生 float32 代价使用已登记门槛（README §justinhj float32 口径）",
      "峰值 RSS 为 /proc/self 1 ms 采样估计；VmHWM 单列为进程高水位参考"};

  std::string write_error;
  if (!write_file(output_dir + "/bench_summary.json",
                  summary.dump(2) + "\n", &write_error)) {
    std::fprintf(stderr, "%s\n", write_error.c_str());
    return 1;
  }

  std::vector<Aggregate> aggregate_values;
  for (const auto& entry : aggregates) {
    aggregate_values.push_back(entry.second);
  }
  write_group_plots(output_dir, "p2p", aggregate_values);
  write_group_plots(output_dir, "zero_heuristic", aggregate_values);
  write_group_plots(output_dir, "distance_field", aggregate_values);

  // 门禁：任何 invalid trial（含负例错误）都视为失败并保留证据。
  long long invalid_trials = 0;
  for (const TrialRow& row : rows) {
    if (!row.valid) {
      ++invalid_trials;
    }
  }
  std::printf("rows=%zu measured=%lld invalid=%lld\n", rows.size(),
              measured_rows, invalid_trials);
  if (invalid_trials != 0) {
    std::fprintf(stderr,
                 "correctness gate failed: %lld invalid trial(s); evidence "
                 "kept, exit code 1\n",
                 invalid_trials);
    return 1;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// 生成 / 校验模式
// ---------------------------------------------------------------------------

int run_generate_only(const Options& options,
                      const std::vector<InstanceSpec>& instance_specs,
                      const std::string& output_dir) {
  std::error_code filesystem_error;
  fs::create_directories(output_dir, filesystem_error);
  if (filesystem_error) {
    std::fprintf(stderr, "cannot create output dir %s: %s\n", output_dir.c_str(),
                 filesystem_error.message().c_str());
    return 1;
  }
  std::vector<std::unique_ptr<InstanceData>> instances;
  std::string error;
  if (!prepare_instances(instance_specs, &instances, &error)) {
    std::fprintf(stderr, "instance preparation failed: %s\n", error.c_str());
    return 1;
  }
  bool ok = true;
  nlohmann::json entries = nlohmann::json::array();
  nlohmann::json checks = nlohmann::json::array();
  for (const auto& entry : instances) {
    MaterializeOutcome outcome =
        materialize_instance(*entry, output_dir, options.archive_root, true);
    if (!outcome.ok) {
      for (const std::string& issue : outcome.issues) {
        std::fprintf(stderr, "materialize %s: %s\n",
                     entry->instance.spec.instance_id.c_str(), issue.c_str());
      }
      ok = false;
      continue;
    }
    nlohmann::json record =
        instance_record_json(*entry, run_id(), outcome.relative_path,
                             outcome.archive_relative);
    record["materialized"] = true;
    entries.push_back(std::move(record));
    checks.push_back({{"instance_id", entry->instance.spec.instance_id},
                      {"passed", entry->check_issues.empty()},
                      {"issues", entry->check_issues},
                      {"serialized_sha256", entry->serialized_sha256},
                      {"serialized_size_bytes", entry->serialized_size}});
    if (!entry->check_issues.empty()) {
      ok = false;
      for (const std::string& issue : entry->check_issues) {
        std::fprintf(stderr, "instance check failed: %s\n", issue.c_str());
      }
    }
    std::printf(
        "%s: n=%d m=%zu sha256=%s size=%zu archive=%s\n",
        entry->instance.spec.instance_id.c_str(),
        entry->instance.graph.num_vertices, entry->instance.graph.edges.size(),
        entry->serialized_sha256.c_str(), entry->serialized_size,
        outcome.archive_written ? outcome.archive_relative.c_str() : "-");
  }
  nlohmann::json table;
  table["schema_version"] = 1;
  table["type"] = "graph_gen_instance_table";
  table["generator"] = {{"name", "graph_gen"},
                        {"version", planner::graph_search::bench::kGraphGenVersion},
                        {"prng", "splitmix64"},
                        {"rules", "Src/graph_search/benchmarks/README.md"}};
  table["generated_by_run_id"] = run_id();
  table["archive_threshold_bytes"] = kArchiveThresholdBytes;
  table["archive_strategy"] =
      "benchmark_tiers.md §3.4：>10 MiB 单文件复制到 SHA256 寻址归档目录（另附 .sha256 与 .meta.json）";
  table["instances"] = std::move(entries);
  std::string write_error;
  if (!write_file(output_dir + "/graph_instances.json", table.dump(2) + "\n",
                  &write_error)) {
    std::fprintf(stderr, "%s\n", write_error.c_str());
    return 1;
  }
  nlohmann::json check_document;
  check_document["schema_version"] = 1;
  check_document["type"] = "graph_gen_instance_checks";
  check_document["all_passed"] = ok;
  check_document["instances"] = std::move(checks);
  if (!write_file(output_dir + "/graph_instance_checks.json",
                  check_document.dump(2) + "\n", &write_error)) {
    std::fprintf(stderr, "%s\n", write_error.c_str());
    return 1;
  }
  return ok ? 0 : 1;
}

int run_check(const Options& options,
              const std::vector<InstanceSpec>& instance_specs,
              const std::string& output_dir) {
  std::vector<std::unique_ptr<InstanceData>> instances;
  std::string error;
  if (!prepare_instances(instance_specs, &instances, &error)) {
    std::fprintf(stderr, "instance preparation failed: %s\n", error.c_str());
    return 1;
  }
  bool ok = true;
  nlohmann::json checks = nlohmann::json::array();
  const std::vector<std::string> materialized_issues =
      verify_materialized(instances, output_dir, options.archive_root);
  for (const auto& entry : instances) {
    nlohmann::json record;
    record["instance_id"] = entry->instance.spec.instance_id;
    record["passed"] = entry->check_issues.empty();
    record["issues"] = entry->check_issues;
    record["serialized_sha256"] = entry->serialized_sha256;
    record["serialized_size_bytes"] = entry->serialized_size;
    const std::string file_name =
        entry->instance.spec.instance_id + ".graph.txt";
    record["materialized_present"] = fs::exists(
        output_dir + "/graphs/" + file_name);
    if (!entry->check_issues.empty()) {
      ok = false;
    }
    checks.push_back(std::move(record));
    std::printf("%s: checks=%s sha256=%s size=%zu\n",
                entry->instance.spec.instance_id.c_str(),
                entry->check_issues.empty() ? "PASS" : "FAIL",
                entry->serialized_sha256.c_str(), entry->serialized_size);
    for (const std::string& issue : entry->check_issues) {
      std::printf("  issue: %s\n", issue.c_str());
    }
  }
  for (const std::string& issue : materialized_issues) {
    std::printf("materialized: FAIL: %s\n", issue.c_str());
    ok = false;
  }
  if (materialized_issues.empty()) {
    std::printf("materialized: OK (no mismatch; files absent are allowed)\n");
  }
  std::error_code filesystem_error;
  fs::create_directories(output_dir, filesystem_error);
  nlohmann::json document;
  document["schema_version"] = 1;
  document["type"] = "graph_gen_instance_checks";
  document["all_passed"] = ok;
  document["materialized_mismatches"] = materialized_issues;
  document["instances"] = std::move(checks);
  std::string write_error;
  if (!write_file(output_dir + "/graph_instance_checks.json",
                  document.dump(2) + "\n", &write_error)) {
    std::fprintf(stderr, "%s\n", write_error.c_str());
    return 1;
  }
  return ok ? 0 : 1;
}

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

void print_usage() {
  std::printf(
      "usage: planner_graph_search_bench [--suite v1] --output-dir <dir> "
      "[--smoke]\n"
      "       planner_graph_search_bench --generate-only --output-dir <dir>\n"
      "       planner_graph_search_bench --check --output-dir <dir>\n"
      "       planner_graph_search_bench --list\n"
      "options:\n"
      "  --instances a,b    实例 ID 过滤（默认全部 6 个）\n"
      "  --scales a,b       规模过滤：1k,10k,100k\n"
      "  --classes a,b      类别过滤：road,rand\n"
      "  --backends a,b     后端 ID 过滤（含 __zero 对照）\n"
      "  --cache-modes a,b  默认 cold,warm\n"
      "  --warmup N         默认 5\n"
      "  --repeats N        默认 30\n"
      "  --p2p-budget-s F   默认 5\n"
      "  --field-budget-s F 默认 30\n"
      "  --archive-root DIR 默认 artifacts/data/sha256\n"
      "  --rss-sample-us N  默认 1000\n");
}

bool parse_options(int argc, char** argv, Options* options,
                   std::string* error) {
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    auto take_value = [&](std::string* out) {
      if (i + 1 >= argc) {
        *error = "missing value for " + argument;
        return false;
      }
      *out = argv[++i];
      return true;
    };
    std::string value;
    if (argument == "--suite") {
      if (!take_value(&options->suite)) return false;
    } else if (argument == "--output-dir") {
      if (!take_value(&options->output_dir)) return false;
    } else if (argument == "--archive-root") {
      if (!take_value(&options->archive_root)) return false;
    } else if (argument == "--generate-only") {
      options->generate_only = true;
    } else if (argument == "--check") {
      options->check = true;
    } else if (argument == "--smoke") {
      options->smoke = true;
    } else if (argument == "--list") {
      options->list = true;
    } else if (argument == "--help" || argument == "-h") {
      options->help = true;
    } else if (argument == "--instances") {
      if (!take_value(&value)) return false;
      options->instances = split_csv(value);
    } else if (argument == "--scales") {
      if (!take_value(&value)) return false;
      options->scales = split_csv(value);
    } else if (argument == "--classes") {
      if (!take_value(&value)) return false;
      options->classes = split_csv(value);
    } else if (argument == "--backends") {
      if (!take_value(&value)) return false;
      options->backends = split_csv(value);
    } else if (argument == "--cache-modes") {
      if (!take_value(&value)) return false;
      options->cache_modes = split_csv(value);
    } else if (argument == "--warmup") {
      if (!take_value(&value)) return false;
      options->warmup = std::atoi(value.c_str());
    } else if (argument == "--repeats") {
      if (!take_value(&value)) return false;
      options->repeats = std::atoi(value.c_str());
    } else if (argument == "--p2p-budget-s") {
      if (!take_value(&value)) return false;
      options->p2p_budget_s = std::atof(value.c_str());
    } else if (argument == "--field-budget-s") {
      if (!take_value(&value)) return false;
      options->field_budget_s = std::atof(value.c_str());
    } else if (argument == "--rss-sample-us") {
      if (!take_value(&value)) return false;
      options->rss_sample_us = std::atoll(value.c_str());
    } else {
      *error = "unknown argument: " + argument;
      return false;
    }
  }
  return true;
}

std::vector<InstanceSpec> select_instances(const Options& options,
                                           std::string* error) {
  std::vector<InstanceSpec> selected;
  for (const InstanceSpec& spec :
       planner::graph_search::bench::default_instance_specs()) {
    if (!options.instances.empty() &&
        std::find(options.instances.begin(), options.instances.end(),
                  spec.instance_id) == options.instances.end()) {
      continue;
    }
    if (!options.scales.empty() &&
        std::find(options.scales.begin(), options.scales.end(), spec.scale) ==
            options.scales.end()) {
      continue;
    }
    if (!options.classes.empty() &&
        std::find(options.classes.begin(), options.classes.end(),
                  spec.graph_class) == options.classes.end()) {
      continue;
    }
    selected.push_back(spec);
  }
  if (selected.empty()) {
    *error = "no instances selected";
  }
  return selected;
}

std::vector<BackendSpec> select_backends(const Options& options,
                                         std::string* error) {
  std::vector<BackendSpec> all = backend_registry();
  std::vector<BackendSpec> selected;
  if (options.backends.empty()) {
    for (const BackendSpec& backend : all) {
      if (backend.compiled) {
        selected.push_back(backend);
      }
    }
    return selected;
  }
  for (const std::string& requested : options.backends) {
    bool found = false;
    for (const BackendSpec& backend : all) {
      if (backend.id == requested) {
        found = true;
        if (!backend.compiled) {
          *error = "backend '" + requested +
                   "' is registered but not compiled into this benchmark";
          return selected;
        }
        selected.push_back(backend);
        break;
      }
    }
    if (!found) {
      std::string available;
      for (const BackendSpec& backend : all) {
        if (backend.compiled) {
          available += (available.empty() ? "" : ",") + backend.id;
        }
      }
      *error = "unknown backend '" + requested + "'; available: " + available;
      return selected;
    }
  }
  return selected;
}

int print_matrix() {
  std::printf("instances:\n");
  for (const InstanceSpec& spec :
       planner::graph_search::bench::default_instance_specs()) {
    std::printf("  %-16s class=%-5s scale=%-5s n=%-7d seed=%llu\n",
                spec.instance_id.c_str(), spec.graph_class.c_str(),
                spec.scale.c_str(), spec.num_vertices,
                static_cast<unsigned long long>(spec.seed));
  }
  std::printf("backends:\n");
  for (const BackendSpec& backend : backend_registry()) {
    std::printf("  %-28s group=%-15s kind=%-14s compiled=%s%s\n",
                backend.id.c_str(), backend.group.c_str(), backend.kind.c_str(),
                backend.compiled ? "yes" : "no",
                backend.compiled ? "" : " (not linked)");
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  std::string error;
  if (!parse_options(argc, argv, &options, &error)) {
    std::fprintf(stderr, "%s\n", error.c_str());
    print_usage();
    return 2;
  }
  if (options.help) {
    print_usage();
    return 0;
  }
  if (options.list) {
    return print_matrix();
  }
  if (options.suite != "v1") {
    std::fprintf(stderr, "unsupported suite: %s (only v1 is defined)\n",
                 options.suite.c_str());
    return 2;
  }
  if (options.output_dir.empty()) {
    std::fprintf(stderr, "--output-dir is required\n");
    return 2;
  }
  if (options.warmup < 0 || options.repeats < 1) {
    std::fprintf(stderr, "--warmup must be >= 0 and --repeats >= 1\n");
    return 2;
  }
  if (options.p2p_budget_s <= 0.0 || options.field_budget_s <= 0.0) {
    std::fprintf(stderr, "budgets must be positive\n");
    return 2;
  }
  if (options.smoke) {
    options.warmup = 1;
    options.repeats = 2;
    if (options.instances.empty() && options.scales.empty() &&
        options.classes.empty()) {
      options.scales = {"1k"};
    }
    if (options.backends.empty()) {
      options.backends = {"boost_astar", "boost_dijkstra",
                          "boost_distance_field"};
    }
  }
  std::vector<InstanceSpec> instance_specs = select_instances(options, &error);
  if (instance_specs.empty()) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 2;
  }
  if (options.generate_only || options.check) {
    // 生成/校验模式不需要后端选择。
    if (options.generate_only) {
      return run_generate_only(options, instance_specs, options.output_dir);
    }
    return run_check(options, instance_specs, options.output_dir);
  }
  std::vector<BackendSpec> backends = select_backends(options, &error);
  if (backends.empty()) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 2;
  }
  return run_benchmark(options, instance_specs, backends, options.output_dir);
}
