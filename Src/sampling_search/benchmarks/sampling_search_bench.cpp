// planner_sampling_search_bench —— X-B1 RRT* 族基准运行器。
//
// 用法（详见 benchmarks/README.md）：
//   planner_sampling_search_bench --suite v1 --output-dir <dir> [--smoke]
//       [--child-grace-s <s>]
//   planner_sampling_search_bench --single --instance <id> --backend <id>
//       --budget <s> --seed <n> --trial-out <file>
//   planner_sampling_search_bench --dump-instances <dir>
//
// 进程模型（OMPL 进程级种子）：每个 trial 由一个独立子进程执行
// （`--single`），驱动循环顺序派发。OMPL 的 RNG::setSeed 是进程全局单例，
// 进程内首次使用后再播种会被忽略（provenance.md §2），因此每 trial 独立
// 进程是 OMPL 种子可复现的必要条件；该模式同时给出干净的每 trial 峰值 RSS
// 与崩溃隔离。所有后端统一走该模式（不区分 OMPL）。
//
// 输出（--output-dir，§7.2 契约与 benchmark_tiers §5）：
//   benchmark_metrics.csv  §7.2 契约文件：每 trial 一行（含 §9 固定列、
//                          RRT* 专有列与容量档列）；任务运行器并入运行级
//                          metrics.csv。组件绝不写入 --output-dir 以外位置。
//   metrics.csv            与 benchmark_metrics.csv 相同的独立副本（便于脱离
//                          入口查看；运行级 metrics.csv 由入口合并生成）。
//   summary.json / summary.md  聚合（成功率含超时/失败、P50/P95、逐场景/逐
//                          预算分解；nearest-rank）。
//   trials.jsonl           逐 trial 原始记录（含子进程退出码与日志路径）。
//   instances.json / instances/<id>.json / instances.sha256.txt
//                          实例表与规范 JSON SHA256（生成器证据）。
//   children_failures.log  非成功/异常退出的子进程日志（失败保留）。
//   plots/*.svg + *.csv    成功率/首次解 P50/容量峰值 RSS 图。
//
// 退出码：0 正常完成（正式套件：失败 trial 记录后继续，不改变退出码）；
// 1 烟测自检失败；2 用法错误；3 环境/生成器/输出错误。

#include <fcntl.h>
#include <signal.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/sampling_search/planner.hpp"
#include "planner/sampling_search/types.hpp"

#include "scenario_gen.hpp"
#include "sha256.hpp"

namespace planner::sampling_search::bench {
namespace {

using Clock = std::chrono::steady_clock;

// ---------------------------------------------------------------------------
// 小工具
// ---------------------------------------------------------------------------

double seconds_since(const Clock::time_point& t0) {
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

std::string format_number(double value) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.10g", value);
  return buffer;
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

std::uint64_t current_rss_bytes() {
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    if (line.rfind("VmRSS:", 0) == 0) {
      std::istringstream stream(line.substr(6));
      std::uint64_t kib = 0;
      stream >> kib;
      return kib * 1024ULL;
    }
  }
  return 0;
}

std::uint64_t peak_rss_bytes() {
  struct rusage usage {};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    return 0;
  }
  return static_cast<std::uint64_t>(usage.ru_maxrss) * 1024ULL;
}

std::string status_name(Status status) {
  switch (status) {
    case Status::kSuccess:
      return "success";
    case Status::kNoPath:
      return "no_path";
    case Status::kTimeout:
      return "timeout";
    case Status::kCancelled:
      return "cancelled";
    case Status::kInvalidInput:
      return "invalid_input";
    default:
      return "unknown";
  }
}

// ---------------------------------------------------------------------------
// 后端登记（CLI id → 工厂/可用性/变体）
// ---------------------------------------------------------------------------

struct BackendSpec {
  const char* id;        // CLI 标识
  const char* backend;   // Statistics::backend
  const char* variant;   // Statistics::variant
  const char* group;     // continuous | grid
  const char* first_solution_source;
  std::unique_ptr<Planner> (*factory)();
  bool (*available)();
};

const std::vector<BackendSpec>& backend_specs() {
  static const std::vector<BackendSpec> specs = {
      {"ompl", "ompl", "rrtstar", "continuous",
       "intermediate_solution_callback", &make_ompl_rrtstar,
       &ompl_rrtstar_available},
      {"ompl-informed", "ompl", "informed_rrtstar", "continuous",
       "intermediate_solution_callback", &make_ompl_informed_rrtstar,
       &ompl_informed_rrtstar_available},
      {"ch0p1k3", "ch0p1k3", "rrtstar", "continuous",
       "outer_loop_goal_domain", &make_ch0p1k3_rrtstar,
       &ch0p1k3_rrtstar_available},
      {"klampt", "klampt", "rrtstar", "continuous",
       "outer_planmore_loop", &make_klampt_rrtstar,
       &klampt_rrtstar_available},
      {"vss2sn", "vss2sn", "rrtstar", "grid",
       "first_solution_callback_patch", &make_vss2sn_rrtstar,
       &vss2sn_rrtstar_available},
  };
  return specs;
}

const BackendSpec* find_backend(const std::string& id) {
  for (const BackendSpec& spec : backend_specs()) {
    if (id == spec.id) {
      return &spec;
    }
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// trial 记录（子进程 JSON ⇄ 驱动 CSV/JSONL）
// ---------------------------------------------------------------------------

struct Record {
  std::string status{"failed"};
  std::string failure_reason;
  std::optional<double> first_solution_ms;
  std::optional<double> best_length_m;
  std::optional<double> best_cost;
  std::optional<std::uint64_t> improvements;
  std::optional<std::uint64_t> iterations;
  std::optional<std::uint64_t> nodes;
  std::optional<std::uint64_t> edge_checks;
  std::optional<double> wall_ms;
  std::optional<double> cpu_ms;
  std::optional<std::uint64_t> peak_rss_bytes;
  std::optional<double> preprocess_ms;
  std::optional<double> endpoint_position_error_m;
  std::optional<std::uint64_t> rss_input_bytes;
  std::optional<std::uint64_t> rss_search_bytes;
  std::optional<std::uint64_t> rss_output_bytes;
  std::string native_params;
  std::string instance_sha256;
  std::string edge_check_step;
  bool valid{false};
  bool solved{false};
};

template <typename T>
void put_optional(nlohmann::json* j, const char* key,
                  const std::optional<T>& value) {
  if (value.has_value()) {
    (*j)[key] = *value;
  } else {
    (*j)[key] = nullptr;
  }
}

template <typename T>
std::optional<T> get_optional(const nlohmann::json& j, const char* key) {
  if (!j.contains(key) || j.at(key).is_null()) {
    return std::nullopt;
  }
  return j.at(key).get<T>();
}

nlohmann::json record_to_json(const Record& record) {
  nlohmann::json j;
  j["status"] = record.status;
  j["failure_reason"] = record.failure_reason;
  put_optional(&j, "first_solution_ms", record.first_solution_ms);
  put_optional(&j, "best_length_m", record.best_length_m);
  put_optional(&j, "best_cost", record.best_cost);
  put_optional(&j, "improvements", record.improvements);
  put_optional(&j, "iterations", record.iterations);
  put_optional(&j, "nodes", record.nodes);
  put_optional(&j, "edge_checks", record.edge_checks);
  put_optional(&j, "wall_ms", record.wall_ms);
  put_optional(&j, "cpu_ms", record.cpu_ms);
  put_optional(&j, "peak_rss_bytes", record.peak_rss_bytes);
  put_optional(&j, "preprocess_ms", record.preprocess_ms);
  put_optional(&j, "endpoint_position_error_m", record.endpoint_position_error_m);
  put_optional(&j, "rss_input_bytes", record.rss_input_bytes);
  put_optional(&j, "rss_search_bytes", record.rss_search_bytes);
  put_optional(&j, "rss_output_bytes", record.rss_output_bytes);
  j["native_params"] = record.native_params;
  j["instance_sha256"] = record.instance_sha256;
  j["edge_check_step"] = record.edge_check_step;
  j["valid"] = record.valid;
  j["solved"] = record.solved;
  return j;
}

Record record_from_json(const nlohmann::json& j) {
  Record record;
  record.status = j.value("status", std::string("failed"));
  record.failure_reason = j.value("failure_reason", std::string());
  record.first_solution_ms = get_optional<double>(j, "first_solution_ms");
  record.best_length_m = get_optional<double>(j, "best_length_m");
  record.best_cost = get_optional<double>(j, "best_cost");
  record.improvements = get_optional<std::uint64_t>(j, "improvements");
  record.iterations = get_optional<std::uint64_t>(j, "iterations");
  record.nodes = get_optional<std::uint64_t>(j, "nodes");
  record.edge_checks = get_optional<std::uint64_t>(j, "edge_checks");
  record.wall_ms = get_optional<double>(j, "wall_ms");
  record.cpu_ms = get_optional<double>(j, "cpu_ms");
  record.peak_rss_bytes = get_optional<std::uint64_t>(j, "peak_rss_bytes");
  record.preprocess_ms = get_optional<double>(j, "preprocess_ms");
  record.endpoint_position_error_m =
      get_optional<double>(j, "endpoint_position_error_m");
  record.rss_input_bytes = get_optional<std::uint64_t>(j, "rss_input_bytes");
  record.rss_search_bytes = get_optional<std::uint64_t>(j, "rss_search_bytes");
  record.rss_output_bytes = get_optional<std::uint64_t>(j, "rss_output_bytes");
  record.native_params = j.value("native_params", std::string());
  record.instance_sha256 = j.value("instance_sha256", std::string());
  record.edge_check_step = j.value("edge_check_step", std::string());
  record.valid = j.value("valid", false);
  record.solved = j.value("solved", false);
  return record;
}

// ---------------------------------------------------------------------------
// --single：单 trial 子进程
// ---------------------------------------------------------------------------

int run_single(const std::string& instance_id, const std::string& backend_id,
               double budget_s, std::uint64_t seed, const std::string& out_path) {
  const BackendSpec* spec = find_backend(backend_id);
  if (spec == nullptr) {
    std::fprintf(stderr, "unknown backend id: %s\n", backend_id.c_str());
    return 3;
  }
  if (!spec->available()) {
    std::fprintf(stderr, "backend not available in this build: %s\n",
                 backend_id.c_str());
    return 3;
  }
  const std::string group = group_of(instance_id);
  const bool is_grid = group == "grid" || group == "capacity_grid";
  const bool is_capacity =
      group == "capacity_continuous" || group == "capacity_grid";

  Record record;
  try {
    const auto preprocess_start = Clock::now();
    Problem problem;
    std::string instance_hash;
    if (is_grid) {
      const GridInstance instance = make_grid_instance(instance_id);
      problem = make_problem(instance, budget_s, seed);
      instance_hash = instance_sha256(instance);
    } else {
      const ContinuousInstance instance =
          make_continuous_instance(instance_id);
      problem = make_problem(instance, budget_s, seed);
      instance_hash = instance_sha256(instance);
    }
    record.instance_sha256 = instance_hash;
    record.edge_check_step = format_number(problem.edge_check_step);
    record.preprocess_ms = seconds_since(preprocess_start) * 1000.0;

    std::unique_ptr<Planner> planner = spec->factory();
    if (planner == nullptr) {
      std::fprintf(stderr, "factory returned null for backend %s\n",
                   backend_id.c_str());
      return 3;
    }

    const std::uint64_t rss_before = is_capacity ? current_rss_bytes() : 0;
    struct rusage usage_before {};
    getrusage(RUSAGE_SELF, &usage_before);
    const auto wall_start = Clock::now();
    CancelToken cancel;
    Result result = planner->plan(problem, &cancel);
    const double wall_ms = seconds_since(wall_start) * 1000.0;
    struct rusage usage_after {};
    getrusage(RUSAGE_SELF, &usage_after);

    record.wall_ms = wall_ms;
    const double user_s = static_cast<double>(usage_after.ru_utime.tv_sec -
                                              usage_before.ru_utime.tv_sec) +
                          static_cast<double>(usage_after.ru_utime.tv_usec -
                                              usage_before.ru_utime.tv_usec) *
                              1e-6;
    const double sys_s = static_cast<double>(usage_after.ru_stime.tv_sec -
                                             usage_before.ru_stime.tv_sec) +
                         static_cast<double>(usage_after.ru_stime.tv_usec -
                                             usage_before.ru_stime.tv_usec) *
                             1e-6;
    record.cpu_ms = (user_s + sys_s) * 1000.0;
    record.peak_rss_bytes = peak_rss_bytes();
    record.native_params = result.stats.native_params;

    const bool vss2sn = std::strcmp(spec->backend, "vss2sn") == 0;
    if (result.stats.first_solution_s >= 0.0) {
      record.first_solution_ms = result.stats.first_solution_s * 1000.0;
    }
    record.iterations = result.stats.iterations;
    if (!vss2sn) {
      record.improvements = result.stats.improvements;
      record.edge_checks = result.stats.edge_checks;
    }
    if (result.stats.nodes > 0) {
      record.nodes = result.stats.nodes;
    }

    const bool solved = result.status == Status::kSuccess;
    if (solved) {
      const std::vector<Point2d>& path = result.path;
      const double start_gap =
          path.empty() ? std::numeric_limits<double>::infinity()
                       : std::hypot(path.front().x_m - problem.start.x_m,
                                    path.front().y_m - problem.start.y_m);
      const double goal_gap =
          path.empty() ? std::numeric_limits<double>::infinity()
                       : std::hypot(path.back().x_m - problem.goal.x_m,
                                    path.back().y_m - problem.goal.y_m);
      record.endpoint_position_error_m = std::max(start_gap, goal_gap);
      if (path.size() >= 2 && path_edges_valid(problem, path) &&
          *record.endpoint_position_error_m <= 1e-6) {
        record.status = "success";
        record.valid = true;
        record.solved = true;
        record.best_length_m = polyline_length(path);
        record.best_cost = result.cost;
      } else {
        record.status = "invalid_output";
        record.failure_reason =
            "adapter reported success but harness re-check failed "
            "(edges/endpoints)";
        record.valid = false;
        record.solved = false;
      }
    } else {
      record.status = status_name(result.status);
      record.failure_reason = result.message;
      record.valid = false;
      record.solved = false;
    }

    if (is_capacity) {
      record.rss_input_bytes = rss_before;
      const std::uint64_t peak = record.peak_rss_bytes.value_or(0);
      if (peak >= rss_before) {
        record.rss_search_bytes = peak - rss_before;
      }
      record.rss_output_bytes =
          static_cast<std::uint64_t>(result.path.size()) * sizeof(Point2d);
    }
  } catch (const std::exception& error) {
    record.status = "failed";
    record.failure_reason = std::string("harness exception: ") + error.what();
    record.valid = false;
    record.solved = false;
  }

  std::ofstream out(out_path);
  if (!out.is_open()) {
    std::fprintf(stderr, "cannot write trial record: %s\n", out_path.c_str());
    return 3;
  }
  out << record_to_json(record).dump() << '\n';
  return 0;
}

// ---------------------------------------------------------------------------
// 矩阵枚举
// ---------------------------------------------------------------------------

struct Trial {
  std::string group;
  std::string instance_id;
  std::string class_name;
  std::string scale;
  std::string backend_id;
  std::string backend;
  std::string variant;
  double budget_s{0.0};
  int seed{0};
  int repeat_id{0};
  std::string case_id;
  bool capacity{false};
  // 驱动侧结果
  Record record;
  std::string child_log_path;
  int child_exit_code{-1};
  bool child_interrupted{false};
  bool record_missing{false};
};

std::string instance_class(const std::string& id) {
  const std::string group = group_of(id);
  if (group == "continuous" || group == "capacity_continuous") {
    // cont[-cap]-<class>-<size>-s<seed>
    std::vector<std::string> parts;
    std::string token;
    for (const char ch : id) {
      if (ch == '-') {
        parts.push_back(token);
        token.clear();
      } else {
        token.push_back(ch);
      }
    }
    parts.push_back(token);
    std::size_t index = 1;
    if (parts[1] == "cap") {
      index = 2;
    }
    return parts[index];
  }
  // grid-<class>-<size>-s<seed>
  const std::size_t first = id.find('-');
  const std::size_t second = id.find('-', first + 1);
  return id.substr(first + 1, second - first - 1);
}

std::string instance_scale(const std::string& id) {
  const std::string group = group_of(id);
  std::vector<std::string> parts;
  std::string token;
  for (const char ch : id) {
    if (ch == '-') {
      parts.push_back(token);
      token.clear();
    } else {
      token.push_back(ch);
    }
  }
  parts.push_back(token);
  if (group == "continuous" || group == "capacity_continuous") {
    std::size_t index = 1;
    if (parts[1] == "cap") {
      index = 2;
    }
    return parts[index + 1];
  }
  return parts[2];
}

Trial make_trial(const std::string& instance_id, const BackendSpec& spec,
                 double budget_s, int seed) {
  Trial trial;
  trial.group = group_of(instance_id);
  trial.instance_id = instance_id;
  trial.class_name = instance_class(instance_id);
  trial.scale = instance_scale(instance_id);
  trial.backend_id = spec.id;
  trial.backend = spec.backend;
  trial.variant = spec.variant;
  trial.budget_s = budget_s;
  trial.seed = seed;
  trial.repeat_id = seed;
  trial.capacity = trial.group.rfind("capacity_", 0) == 0;
  trial.case_id = instance_id + "__" + spec.id + "__b" + format_number(budget_s);
  return trial;
}

std::vector<Trial> build_matrix(bool smoke, std::vector<std::string>* skipped) {
  std::vector<Trial> trials;
  const std::vector<double> formal_budgets = {0.1, 0.5, 1.0, 5.0};
  const int formal_seeds = 30;
  for (const BackendSpec& spec : backend_specs()) {
    if (!spec.available()) {
      skipped->push_back(spec.id);
    }
  }
  const auto available = [&](const BackendSpec& spec) {
    return spec.available();
  };

  if (!smoke) {
    // 连续组：4 目标（Informed 单列）× 6 场景 × 4 预算 × 种子 0–29。
    for (const BackendSpec& spec : backend_specs()) {
      if (std::strcmp(spec.group, "continuous") != 0 || !available(spec)) {
        continue;
      }
      for (const std::string& instance : continuous_instance_ids(false)) {
        for (const double budget : formal_budgets) {
          for (int seed = 0; seed < formal_seeds; ++seed) {
            trials.push_back(make_trial(instance, spec, budget, seed));
          }
        }
      }
    }
    // 栅格组：vss2sn × 6 类 × 3 规模（seed 0 几何）× 4 预算 × 种子 0–29。
    for (const BackendSpec& spec : backend_specs()) {
      if (std::strcmp(spec.group, "grid") != 0 || !available(spec)) {
        continue;
      }
      for (const std::string& instance : grid_instance_ids(false)) {
        for (const double budget : formal_budgets) {
          for (int seed = 0; seed < formal_seeds; ++seed) {
            trials.push_back(make_trial(instance, spec, budget, seed));
          }
        }
      }
    }
    // 容量档：连续 2 实例 × 4 目标 × 5 s × seed 0；栅格 12 实例 × vss2sn × 5 s。
    for (const BackendSpec& spec : backend_specs()) {
      if (std::strcmp(spec.group, "continuous") != 0 || !available(spec)) {
        continue;
      }
      for (const std::string& instance : continuous_instance_ids(true)) {
        if (group_of(instance) != "capacity_continuous") {
          continue;
        }
        trials.push_back(make_trial(instance, spec, 5.0, 0));
      }
    }
    for (const BackendSpec& spec : backend_specs()) {
      if (std::strcmp(spec.group, "grid") != 0 || !available(spec)) {
        continue;
      }
      for (const std::string& instance : grid_instance_ids(true)) {
        if (group_of(instance) != "capacity_grid") {
          continue;
        }
        trials.push_back(make_trial(instance, spec, 5.0, 0));
      }
    }
    return trials;
  }

  // 烟测子集：小预算、少种子；含每 trial 进程模式验证与容量档路径。
  const std::vector<double> smoke_budgets = {0.1, 0.5};
  const std::vector<std::string> smoke_continuous = {"cont-open-100-s0",
                                                     "cont-narrow-100-s0"};
  const std::vector<std::string> smoke_grid = {"grid-open-128-s0",
                                               "grid-maze-128-s0"};
  for (const BackendSpec& spec : backend_specs()) {
    if (std::strcmp(spec.group, "continuous") != 0 || !available(spec)) {
      continue;
    }
    for (const std::string& instance : smoke_continuous) {
      for (const double budget : smoke_budgets) {
        for (int seed = 0; seed < 2; ++seed) {
          trials.push_back(make_trial(instance, spec, budget, seed));
        }
      }
    }
  }
  for (const BackendSpec& spec : backend_specs()) {
    if (std::strcmp(spec.group, "grid") != 0 || !available(spec)) {
      continue;
    }
    for (const std::string& instance : smoke_grid) {
      for (const double budget : smoke_budgets) {
        for (int seed = 0; seed < 2; ++seed) {
          trials.push_back(make_trial(instance, spec, budget, seed));
        }
      }
    }
  }
  for (const BackendSpec& spec : backend_specs()) {
    if (std::strcmp(spec.group, "continuous") != 0 || !available(spec)) {
      continue;
    }
    trials.push_back(
        make_trial("cont-cap-open-200-s0", spec, 0.1, 0));
    if (std::strcmp(spec.id, "ompl") == 0) {
      trials.push_back(make_trial("cont-cap-sparse-200-s0", spec, 0.1, 0));
    }
  }
  for (const BackendSpec& spec : backend_specs()) {
    if (std::strcmp(spec.group, "grid") != 0 || !available(spec)) {
      continue;
    }
    trials.push_back(make_trial("grid-open-2048-s0", spec, 0.1, 0));
  }
  return trials;
}

// ---------------------------------------------------------------------------
// 子进程派发
// ---------------------------------------------------------------------------

struct ChildOutcome {
  int exit_code{-1};
  bool interrupted{false};
  bool record_read{false};
  std::string record_json;
  std::string log;
};

std::string read_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

ChildOutcome run_child(const std::string& executable, const Trial& trial,
                       const std::string& record_path,
                       const std::string& log_path, double hard_timeout_s) {
  ChildOutcome outcome;
  std::vector<std::string> args = {
      executable,        "--single",
      "--instance",      trial.instance_id,
      "--backend",       trial.backend_id,
      "--budget",        format_number(trial.budget_s),
      "--seed",          std::to_string(trial.seed),
      "--trial-out",     record_path};
  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (std::string& arg : args) {
    argv.push_back(const_cast<char*>(arg.c_str()));
  }
  argv.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    outcome.exit_code = -1;
    outcome.log = "fork failed";
    return outcome;
  }
  if (pid == 0) {
    setpgid(0, 0);
    const int fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd >= 0) {
      dup2(fd, 1);
      dup2(fd, 2);
      if (fd > 2) {
        close(fd);
      }
    }
    execvp(executable.c_str(), argv.data());
    _exit(127);
  }

  const auto deadline =
      Clock::now() + std::chrono::duration<double>(hard_timeout_s);
  int status = 0;
  bool finished = false;
  while (!finished) {
    const pid_t waited = waitpid(pid, &status, WNOHANG);
    if (waited == pid) {
      finished = true;
      break;
    }
    if (waited < 0) {
      finished = true;
      outcome.exit_code = -1;
      break;
    }
    if (Clock::now() >= deadline) {
      outcome.interrupted = true;
      if (kill(-pid, SIGTERM) != 0) {
        kill(pid, SIGTERM);
      }
      const auto grace_deadline = Clock::now() + std::chrono::seconds(5);
      while (Clock::now() < grace_deadline) {
        if (waitpid(pid, &status, WNOHANG) == pid) {
          finished = true;
          break;
        }
        usleep(50000);
      }
      if (!finished) {
        if (kill(-pid, SIGKILL) != 0) {
          kill(pid, SIGKILL);
        }
        waitpid(pid, &status, 0);
        finished = true;
      }
      break;
    }
    usleep(5000);
  }

  if (!outcome.interrupted) {
    if (WIFEXITED(status)) {
      outcome.exit_code = WEXITSTATUS(status);
    } else if (WIFSIGNALED(status)) {
      outcome.exit_code = 128 + WTERMSIG(status);
    } else {
      outcome.exit_code = -1;
    }
  }
  outcome.log = read_file(log_path);
  const std::string record = read_file(record_path);
  if (!record.empty()) {
    outcome.record_read = true;
    outcome.record_json = record;
  }
  return outcome;
}

// ---------------------------------------------------------------------------
// 聚合
// ---------------------------------------------------------------------------

struct Aggregate {
  int trials{0};
  int success{0};
  int timeout{0};
  int no_path{0};
  int cancelled{0};
  int invalid{0};
  int failed{0};
  int interrupted{0};
  std::vector<double> first_solution_ms;
  std::vector<double> wall_ms;
  std::vector<double> best_length_m;

  int normal_denominator() const {
    return success + timeout + no_path + invalid + failed;
  }
};

void add_trial(Aggregate* aggregate, const Trial& trial) {
  ++aggregate->trials;
  const std::string& status = trial.record.status;
  if (status == "success") {
    ++aggregate->success;
  } else if (status == "timeout") {
    ++aggregate->timeout;
  } else if (status == "no_path") {
    ++aggregate->no_path;
  } else if (status == "cancelled") {
    ++aggregate->cancelled;
  } else if (status == "invalid_output" || status == "invalid_input") {
    ++aggregate->invalid;
  } else if (status == "interrupted") {
    ++aggregate->interrupted;
  } else {
    ++aggregate->failed;
  }
  if (trial.record.first_solution_ms.has_value() && status == "success") {
    aggregate->first_solution_ms.push_back(*trial.record.first_solution_ms);
  }
  if (trial.record.wall_ms.has_value() && status != "interrupted") {
    aggregate->wall_ms.push_back(*trial.record.wall_ms);
  }
  if (trial.record.best_length_m.has_value() && status == "success") {
    aggregate->best_length_m.push_back(*trial.record.best_length_m);
  }
}

nlohmann::json aggregate_to_json(const Aggregate& aggregate) {
  nlohmann::json j;
  j["trials"] = aggregate.trials;
  j["success"] = aggregate.success;
  j["timeout"] = aggregate.timeout;
  j["no_path"] = aggregate.no_path;
  j["cancelled"] = aggregate.cancelled;
  j["invalid"] = aggregate.invalid;
  j["failed"] = aggregate.failed;
  j["interrupted"] = aggregate.interrupted;
  const int denominator = aggregate.normal_denominator();
  if (denominator > 0) {
    j["success_rate"] = static_cast<double>(aggregate.success) / denominator;
  } else {
    j["success_rate"] = nullptr;
  }
  const auto p50_first =
      percentile_nearest_rank(aggregate.first_solution_ms, 50.0);
  const auto p95_first =
      percentile_nearest_rank(aggregate.first_solution_ms, 95.0);
  const auto p50_wall = percentile_nearest_rank(aggregate.wall_ms, 50.0);
  const auto p95_wall = percentile_nearest_rank(aggregate.wall_ms, 95.0);
  const auto p50_length =
      percentile_nearest_rank(aggregate.best_length_m, 50.0);
  const auto p95_length =
      percentile_nearest_rank(aggregate.best_length_m, 95.0);
  j["first_solution_ms_p50"] =
      std::isnan(p50_first) ? nlohmann::json(nullptr)
                            : nlohmann::json(p50_first);
  j["first_solution_ms_p95"] =
      std::isnan(p95_first) ? nlohmann::json(nullptr)
                            : nlohmann::json(p95_first);
  j["wall_ms_p50"] =
      std::isnan(p50_wall) ? nlohmann::json(nullptr) : nlohmann::json(p50_wall);
  j["wall_ms_p95"] =
      std::isnan(p95_wall) ? nlohmann::json(nullptr) : nlohmann::json(p95_wall);
  j["best_length_m_p50"] =
      std::isnan(p50_length) ? nlohmann::json(nullptr)
                             : nlohmann::json(p50_length);
  j["best_length_m_p95"] =
      std::isnan(p95_length) ? nlohmann::json(nullptr)
                             : nlohmann::json(p95_length);
  return j;
}

// ---------------------------------------------------------------------------
// 输出：CSV / JSONL / summary / 实例表 / 图
// ---------------------------------------------------------------------------

void write_metrics_csv(const std::filesystem::path& path,
                       const std::string& run_id,
                       const std::vector<Trial>& trials) {
  std::ofstream out(path);
  out << "run_id,case_id,repeat_id,cache_mode,seed,status,failure_reason,"
         "preprocess_ms,wall_ms,cpu_ms,peak_rss_bytes,expanded_nodes,"
         "collision_checks,path_length_m,gear_changes,endpoint_position_error_m,"
         "endpoint_yaw_error_rad,valid,budget_s,first_solution_ms,solved,"
         "best_length_m,best_cost,improvements,group,instance_id,class,scale,"
         "backend,variant,edge_check_step,iterations,nodes,"
         "first_solution_source,native_params,instance_sha256,rss_input_bytes,"
         "rss_search_bytes,rss_output_bytes,conversion_ms\n";
  const auto optional_number = [](const std::optional<double>& value) {
    return value.has_value() ? format_number(*value) : std::string();
  };
  const auto optional_integer = [](const std::optional<std::uint64_t>& value) {
    return value.has_value() ? std::to_string(*value) : std::string();
  };
  for (const Trial& trial : trials) {
    const Record& record = trial.record;
    const BackendSpec* spec = find_backend(trial.backend_id);
    const std::string source =
        spec != nullptr ? spec->first_solution_source : "";
    out << csv_field(run_id) << ',' << csv_field(trial.case_id) << ','
        << trial.repeat_id << ",cold," << trial.seed << ','
        << csv_field(record.status) << ',' << csv_field(record.failure_reason)
        << ',' << optional_number(record.preprocess_ms) << ','
        << optional_number(record.wall_ms) << ','
        << optional_number(record.cpu_ms) << ','
        << optional_integer(record.peak_rss_bytes) << ',' << ','
        << optional_integer(record.edge_checks) << ','
        << optional_number(record.best_length_m) << ','
        << ',' << optional_number(record.endpoint_position_error_m) << ','
        << ',' << (record.valid ? "true" : "false") << ','
        << format_number(trial.budget_s) << ','
        << optional_number(record.first_solution_ms) << ','
        << (record.solved ? 1 : 0) << ','
        << optional_number(record.best_length_m) << ','
        << optional_number(record.best_cost) << ','
        << optional_integer(record.improvements) << ','
        << csv_field(trial.group) << ',' << csv_field(trial.instance_id) << ','
        << csv_field(trial.class_name) << ',' << csv_field(trial.scale) << ','
        << csv_field(trial.backend) << ',' << csv_field(trial.variant) << ','
        << csv_field(record.edge_check_step) << ','
        << optional_integer(record.iterations) << ','
        << optional_integer(record.nodes) << ',' << csv_field(source) << ','
        << csv_field(record.native_params) << ','
        << csv_field(record.instance_sha256) << ','
        << optional_integer(record.rss_input_bytes) << ','
        << optional_integer(record.rss_search_bytes) << ','
        << optional_integer(record.rss_output_bytes) << ','
        << "\n";
  }
}

void write_trials_jsonl(const std::filesystem::path& path,
                        const std::vector<Trial>& trials) {
  std::ofstream out(path);
  int index = 0;
  for (const Trial& trial : trials) {
    nlohmann::json j;
    j["index"] = index++;
    j["case_id"] = trial.case_id;
    j["group"] = trial.group;
    j["instance_id"] = trial.instance_id;
    j["class"] = trial.class_name;
    j["scale"] = trial.scale;
    j["backend_id"] = trial.backend_id;
    j["backend"] = trial.backend;
    j["variant"] = trial.variant;
    j["budget_s"] = trial.budget_s;
    j["seed"] = trial.seed;
    j["child_exit_code"] = trial.child_exit_code;
    j["child_interrupted"] = trial.child_interrupted;
    j["record_missing"] = trial.record_missing;
    j["record"] = record_to_json(trial.record);
    out << j.dump() << '\n';
  }
}

struct InstanceEntry {
  std::string id;
  std::string group;
  std::string class_name;
  std::string scale;
  std::uint64_t geometry_seed{0};
  std::string generator;
  std::string sha256;
  std::string canonical_json;
  std::uint64_t bytes{0};
};

std::vector<InstanceEntry> collect_instances(const std::vector<Trial>& trials) {
  std::map<std::string, InstanceEntry> unique;
  for (const Trial& trial : trials) {
    if (unique.count(trial.instance_id) != 0) {
      continue;
    }
    InstanceEntry entry;
    entry.id = trial.instance_id;
    entry.group = trial.group;
    entry.class_name = trial.class_name;
    entry.scale = trial.scale;
    if (entry.group == "grid" || entry.group == "capacity_grid") {
      const GridInstance instance = make_grid_instance(trial.instance_id);
      entry.geometry_seed = instance.geometry_seed;
      entry.generator = kGridGenerator;
      entry.canonical_json = canonical_instance_json(instance);
      entry.sha256 = instance_sha256(instance);
    } else {
      const ContinuousInstance instance =
          make_continuous_instance(trial.instance_id);
      entry.geometry_seed = instance.geometry_seed;
      entry.generator = kContinuousGenerator;
      entry.canonical_json = canonical_instance_json(instance);
      entry.sha256 = instance_sha256(instance);
    }
    entry.bytes = entry.canonical_json.size();
    unique.emplace(trial.instance_id, std::move(entry));
  }
  std::vector<InstanceEntry> entries;
  for (auto& item : unique) {
    entries.push_back(std::move(item.second));
  }
  return entries;
}

void write_instances(const std::filesystem::path& out_dir,
                     const std::vector<InstanceEntry>& entries) {
  std::filesystem::create_directories(out_dir / "instances");
  nlohmann::json table;
  table["schema_version"] = 1;
  table["type"] = "planner.sampling_search.instances";
  table["generators"] = {{"continuous", kContinuousGenerator},
                         {"grid", kGridGenerator}};
  table["count"] = entries.size();
  nlohmann::json list = nlohmann::json::array();
  std::ofstream hashes(out_dir / "instances.sha256.txt");
  for (const InstanceEntry& entry : entries) {
    const std::string relative =
        std::string("instances/") + entry.id + ".json";
    std::ofstream single(out_dir / relative);
    single << entry.canonical_json << '\n';
    nlohmann::json item;
    item["id"] = entry.id;
    item["group"] = entry.group;
    item["class"] = entry.class_name;
    item["scale"] = entry.scale;
    item["geometry_seed"] = entry.geometry_seed;
    item["generator"] = entry.generator;
    item["sha256"] = entry.sha256;
    item["bytes"] = entry.bytes;
    item["canonical_file"] = relative;
    list.push_back(std::move(item));
    hashes << entry.sha256 << "  " << entry.id << '\n';
  }
  table["instances"] = std::move(list);
  std::ofstream out(out_dir / "instances.json");
  out << std::setw(2) << table << '\n';
}

std::string xml_escape(const std::string& text) {
  std::string out;
  for (const char ch : text) {
    switch (ch) {
      case '&':
        out += "&amp;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      case '"':
        out += "&quot;";
        break;
      default:
        out += ch;
    }
  }
  return out;
}

struct ChartSeries {
  std::string name;
  std::vector<std::optional<double>> values;
};

void write_grouped_bar_chart(const std::filesystem::path& base,
                             const std::string& title,
                             const std::vector<std::string>& categories,
                             const std::vector<ChartSeries>& series,
                             const std::string& y_label) {
  const char* palette[] = {"#4e79a7", "#f28e2b", "#e15759", "#76b7b2",
                           "#59a14f", "#edc948", "#b07aa1", "#ff9da7"};
  const int palette_size = 8;
  // CSV
  {
    std::ofstream csv(base.string() + ".csv");
    csv << "category,series,value\n";
    for (std::size_t c = 0; c < categories.size(); ++c) {
      for (std::size_t s = 0; s < series.size(); ++s) {
        csv << csv_field(categories[c]) << ',' << csv_field(series[s].name)
            << ',';
        if (c < series[s].values.size() && series[s].values[c].has_value()) {
          csv << format_number(*series[s].values[c]);
        }
        csv << '\n';
      }
    }
  }
  // SVG
  const int series_count = static_cast<int>(series.size());
  const int category_count = static_cast<int>(categories.size());
  const int bar_width = 34;
  const int group_gap = 26;
  const int left = 80;
  const int top = 80;
  const int bottom = 70;
  const int right = 30;
  const int group_width = std::max(1, series_count) * bar_width;
  const int plot_width =
      category_count * (group_width + group_gap) + group_gap;
  const int plot_height = 320;
  const int width = left + plot_width + right;
  const int height = top + plot_height + bottom;
  double max_value = 0.0;
  for (const ChartSeries& s : series) {
    for (const auto& value : s.values) {
      if (value.has_value()) {
        max_value = std::max(max_value, *value);
      }
    }
  }
  if (max_value <= 0.0) {
    max_value = 1.0;
  }
  const double y_max = max_value * 1.15;
  std::ofstream svg(base.string() + ".svg");
  svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
      << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << ' '
      << height << "\">\n";
  svg << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
  svg << "<text x=\"" << left << "\" y=\"34\" font-family=\"sans-serif\" "
         "font-size=\"18\" fill=\"#111\">"
      << xml_escape(title) << "</text>\n";
  // 图例
  int legend_x = left;
  for (int s = 0; s < series_count; ++s) {
    const char* color = palette[s % palette_size];
    svg << "<rect x=\"" << legend_x << "\" y=\"48\" width=\"14\" height=\"14\" "
           "fill=\""
        << color << "\"/>\n";
    svg << "<text x=\"" << legend_x + 18 << "\" y=\"60\" "
           "font-family=\"sans-serif\" font-size=\"12\" fill=\"#333\">"
        << xml_escape(series[s].name) << "</text>\n";
    legend_x += 26 + static_cast<int>(series[s].name.size()) * 8;
  }
  const auto y_pixel = [&](double value) {
    return top + plot_height - value / y_max * plot_height;
  };
  // 轴与刻度
  svg << "<line x1=\"" << left << "\" y1=\"" << top + plot_height << "\" x2=\""
      << left + plot_width << "\" y2=\"" << top + plot_height
      << "\" stroke=\"#333\"/>\n";
  svg << "<line x1=\"" << left << "\" y1=\"" << top << "\" x2=\"" << left
      << "\" y2=\"" << top + plot_height << "\" stroke=\"#333\"/>\n";
  for (int tick = 0; tick <= 5; ++tick) {
    const double value = y_max * tick / 5.0;
    const double y = y_pixel(value);
    svg << "<line x1=\"" << left - 4 << "\" y1=\"" << y << "\" x2=\"" << left
        << "\" y2=\"" << y << "\" stroke=\"#333\"/>\n";
    svg << "<text x=\"" << left - 8 << "\" y=\"" << y + 4
        << "\" text-anchor=\"end\" font-family=\"sans-serif\" font-size=\"11\" "
           "fill=\"#333\">"
        << format_number(value) << "</text>\n";
  }
  svg << "<text x=\"" << left - 60 << "\" y=\"" << top - 8
      << "\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#333\">"
      << xml_escape(y_label) << "</text>\n";
  for (int c = 0; c < category_count; ++c) {
    const int group_x = left + group_gap + c * (group_width + group_gap);
    for (int s = 0; s < series_count; ++s) {
      const int x = group_x + s * bar_width;
      const char* color = palette[s % palette_size];
      if (c < static_cast<int>(series[s].values.size()) &&
          series[s].values[c].has_value()) {
        const double value = *series[s].values[c];
        const double y = y_pixel(value);
        const double bar_height = top + plot_height - y;
        svg << "<rect x=\"" << x << "\" y=\"" << y << "\" width=\""
            << bar_width - 4 << "\" height=\"" << std::max(0.0, bar_height)
            << "\" fill=\"" << color << "\"/>\n";
        svg << "<text x=\"" << x + (bar_width - 4) / 2 << "\" y=\"" << y - 4
            << "\" text-anchor=\"middle\" font-family=\"sans-serif\" "
               "font-size=\"10\" fill=\"#333\">"
            << format_number(value) << "</text>\n";
      }
    }
    svg << "<text x=\"" << group_x + group_width / 2 << "\" y=\""
        << top + plot_height + 20
        << "\" text-anchor=\"middle\" font-family=\"sans-serif\" "
           "font-size=\"12\" fill=\"#333\">"
        << xml_escape(categories[c]) << "</text>\n";
  }
  svg << "</svg>\n";
}

void write_plots(const std::filesystem::path& out_dir,
                 const std::vector<Trial>& trials) {
  std::filesystem::create_directories(out_dir / "plots");
  // 连续组：按后端 × 预算聚合（跨 6 场景）。
  {
    std::vector<std::string> categories;
    std::vector<std::string> budgets = {"0.1", "0.5", "1", "5"};
    std::map<std::string, Aggregate> aggregates;  // backend|budget
    for (const Trial& trial : trials) {
      if (trial.group != "continuous") {
        continue;
      }
      add_trial(&aggregates[trial.backend_id + "|" +
                            format_number(trial.budget_s)],
                trial);
    }
    for (const BackendSpec& spec : backend_specs()) {
      if (std::strcmp(spec.group, "continuous") == 0) {
        categories.push_back(spec.id);
      }
    }
    std::vector<ChartSeries> success_series;
    std::vector<ChartSeries> first_series;
    for (const std::string& budget : budgets) {
      ChartSeries success;
      success.name = budget + " s";
      ChartSeries first;
      first.name = budget + " s";
      for (const std::string& category : categories) {
        const auto it = aggregates.find(category + "|" + budget);
        if (it == aggregates.end() || it->second.normal_denominator() == 0) {
          success.values.push_back(std::nullopt);
        } else {
          success.values.push_back(
              static_cast<double>(it->second.success) /
              it->second.normal_denominator());
        }
        const double p50 =
            percentile_nearest_rank(it == aggregates.end()
                                        ? std::vector<double>()
                                        : it->second.first_solution_ms,
                                    50.0);
        if (std::isnan(p50)) {
          first.values.push_back(std::nullopt);
        } else {
          first.values.push_back(p50);
        }
      }
      success_series.push_back(std::move(success));
      first_series.push_back(std::move(first));
    }
    write_grouped_bar_chart(out_dir / "plots" / "continuous_success_rate",
                            "RRT* continuous group: success rate by budget",
                            categories, success_series, "success rate");
    write_grouped_bar_chart(
        out_dir / "plots" / "continuous_first_solution_p50_ms",
        "RRT* continuous group: first-solution P50 (success trials)",
        categories, first_series, "ms");
  }
  // 栅格组：按规模 × 预算（vss2sn，跨 6 类）。
  {
    std::vector<std::string> categories = {"128", "512", "1024"};
    std::vector<std::string> budgets = {"0.1", "0.5", "1", "5"};
    std::map<std::string, Aggregate> aggregates;  // scale|budget
    for (const Trial& trial : trials) {
      if (trial.group != "grid") {
        continue;
      }
      add_trial(&aggregates[trial.scale + "|" +
                            format_number(trial.budget_s)],
                trial);
    }
    std::vector<ChartSeries> success_series;
    for (const std::string& budget : budgets) {
      ChartSeries success;
      success.name = budget + " s";
      for (const std::string& category : categories) {
        const auto it = aggregates.find(category + "|" + budget);
        if (it == aggregates.end() || it->second.normal_denominator() == 0) {
          success.values.push_back(std::nullopt);
        } else {
          success.values.push_back(
              static_cast<double>(it->second.success) /
              it->second.normal_denominator());
        }
      }
      success_series.push_back(std::move(success));
    }
    write_grouped_bar_chart(out_dir / "plots" / "grid_success_rate",
                            "RRT* grid group (vss2sn): success rate by scale",
                            categories, success_series, "success rate");
  }
  // 容量档：峰值 RSS。
  {
    std::vector<std::string> categories;
    ChartSeries series;
    series.name = "peak RSS (MiB)";
    for (const Trial& trial : trials) {
      if (!trial.capacity) {
        continue;
      }
      categories.push_back(trial.instance_id + "/" + trial.backend_id);
      if (trial.record.peak_rss_bytes.has_value()) {
        series.values.push_back(
            static_cast<double>(*trial.record.peak_rss_bytes) / (1024.0 * 1024.0));
      } else {
        series.values.push_back(std::nullopt);
      }
    }
    if (!categories.empty()) {
      write_grouped_bar_chart(out_dir / "plots" / "capacity_peak_rss_mib",
                              "Capacity tier: peak RSS per single run",
                              categories, {series}, "MiB");
    }
  }
}

std::string aggregate_row_md(const std::string& label,
                             const std::string& budget,
                             const Aggregate& aggregate) {
  std::ostringstream out;
  const int denominator = aggregate.normal_denominator();
  const std::string rate =
      denominator > 0 ? format_number(static_cast<double>(aggregate.success) /
                                      denominator)
                      : "-";
  const double p50 =
      percentile_nearest_rank(aggregate.first_solution_ms, 50.0);
  const double p95 =
      percentile_nearest_rank(aggregate.first_solution_ms, 95.0);
  const double wall50 = percentile_nearest_rank(aggregate.wall_ms, 50.0);
  out << "| " << label << " | " << budget << " | " << aggregate.trials
      << " | " << aggregate.success << " | " << aggregate.timeout << " | "
      << aggregate.failed << " | " << aggregate.interrupted << " | " << rate
      << " | ";
  if (std::isnan(p50)) {
    out << "- | - | ";
  } else {
    out << format_number(p50) << " | " << format_number(p95) << " | ";
  }
  out << (std::isnan(wall50) ? std::string("-") : format_number(wall50))
      << " |\n";
  return out.str();
}

void write_summary_md(const std::filesystem::path& path,
                      const std::string& suite,
                      const std::string& run_id,
                      const std::vector<Trial>& trials,
                      bool smoke) {
  std::ofstream out(path);
  out << "# planner_sampling_search_bench summary (" << suite << ")\n\n";
  out << "- run_id: `" << (run_id.empty() ? "(manual)" : run_id) << "`\n";
  out << "- trials: " << trials.size() << "（每 trial 独立子进程）\n";
  out << "- 成功率口径：success / (success + timeout + no_path + invalid + "
         "failed)；cancelled 与 interrupted 单列（benchmark_tiers §5.2）\n";
  out << "- 首次解口径：各后端原生观测点（`first_solution_source` 列）；"
         "跨实现不声称统一（见 README §5 遗留）\n\n";
  const std::vector<std::string> groups = {"continuous", "grid",
                                           "capacity_continuous",
                                           "capacity_grid"};
  for (const std::string& group : groups) {
    bool any = false;
    for (const Trial& trial : trials) {
      if (trial.group == group) {
        any = true;
        break;
      }
    }
    if (!any) {
      continue;
    }
    out << "## " << group << "\n\n";
    out << "| backend | budget | trials | success | timeout | failed | "
           "interrupted | success_rate | first_p50_ms | first_p95_ms | "
           "wall_p50_ms |\n";
    out << "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | "
           "---: | ---: |\n";
    std::vector<std::string> backend_ids;
    for (const BackendSpec& spec : backend_specs()) {
      backend_ids.push_back(spec.id);
    }
    std::vector<double> budgets;
    for (const Trial& trial : trials) {
      if (trial.group == group) {
        budgets.push_back(trial.budget_s);
      }
    }
    std::sort(budgets.begin(), budgets.end());
    budgets.erase(std::unique(budgets.begin(), budgets.end()), budgets.end());
    for (const std::string& backend : backend_ids) {
      for (const double budget : budgets) {
        Aggregate aggregate;
        bool any_trial = false;
        for (const Trial& trial : trials) {
          if (trial.group == group && trial.backend_id == backend &&
              trial.budget_s == budget) {
            add_trial(&aggregate, trial);
            any_trial = true;
          }
        }
        if (any_trial) {
          out << aggregate_row_md(backend, format_number(budget), aggregate);
        }
      }
    }
    out << "\n";
  }
  if (smoke) {
    out << "## 烟测说明\n\n"
        << "本 run 为烟测子集（小预算/少种子，含每 trial 进程模式与容量档"
           "路径验证），不构成正式排名。\n";
  }
}

// ---------------------------------------------------------------------------
// 驱动主循环
// ---------------------------------------------------------------------------

std::string executable_path() {
  char buffer[4096];
  const ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (length <= 0) {
    return {};
  }
  buffer[length] = '\0';
  return buffer;
}

int run_driver(bool smoke, const std::string& output_dir, double grace_s) {
  const std::string executable = executable_path();
  if (executable.empty()) {
    std::fprintf(stderr, "cannot resolve /proc/self/exe\n");
    return 3;
  }
  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    std::fprintf(stderr, "cannot create output dir: %s\n", output_dir.c_str());
    return 3;
  }
  std::vector<std::string> skipped;
  std::vector<Trial> trials = build_matrix(smoke, &skipped);
  if (trials.empty()) {
    std::fprintf(stderr, "matrix is empty (no available backends?)\n");
    return 3;
  }
  const char* run_id_env = std::getenv("PLANNER_RUN_ID");
  const std::string run_id = run_id_env != nullptr ? run_id_env : "";

  // 生成器自检（实例可生成、起终点有效、连通性规则成立）。
  std::string self_check_error = generator_self_check(false);
  if (!self_check_error.empty()) {
    std::fprintf(stderr, "generator self-check failed: %s\n",
                 self_check_error.c_str());
    return 3;
  }

  // 驱动侧实例表与 SHA256（子进程记录须一致）。
  std::vector<InstanceEntry> instances;
  try {
    instances = collect_instances(trials);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "instance generation failed: %s\n", error.what());
    return 3;
  }
  std::map<std::string, std::string> expected_hash;
  for (const InstanceEntry& entry : instances) {
    expected_hash[entry.id] = entry.sha256;
  }

  const std::filesystem::path work_dir = output_dir;
  const std::string trial_dir = work_dir.string() + "/.trial_tmp";
  std::filesystem::create_directories(trial_dir, ec);
  std::ofstream failures(work_dir / "children_failures.log");

  int index = 0;
  int child_processes = 0;
  int child_exit_zero = 0;
  int records_parsed = 0;
  int hash_mismatches = 0;
  for (Trial& trial : trials) {
    ++index;
    const std::string record_path =
        trial_dir + "/trial_" + std::to_string(index) + ".json";
    const std::string log_path =
        trial_dir + "/trial_" + std::to_string(index) + ".log";
    const double hard_timeout_s = trial.budget_s + grace_s;
    const ChildOutcome outcome =
        run_child(executable, trial, record_path, log_path, hard_timeout_s);
    ++child_processes;
    trial.child_exit_code = outcome.exit_code;
    trial.child_interrupted = outcome.interrupted;
    trial.child_log_path = log_path;
    if (outcome.interrupted) {
      trial.record.status = "interrupted";
      trial.record.failure_reason =
          "hard timeout: no child record within budget + " +
          format_number(grace_s) + " s grace";
      trial.record_missing = true;
    } else if (!outcome.record_read) {
      trial.record.status = "failed";
      trial.record.failure_reason =
          "child exited with code " + std::to_string(outcome.exit_code) +
          " without a trial record";
      trial.record_missing = true;
    } else {
      ++records_parsed;
      if (outcome.exit_code == 0) {
        ++child_exit_zero;
      }
      try {
        trial.record = record_from_json(nlohmann::json::parse(
            outcome.record_json.substr(0, outcome.record_json.find('\n'))));
      } catch (const std::exception& error) {
        trial.record = Record{};
        trial.record.status = "failed";
        trial.record.failure_reason =
            std::string("cannot parse child record: ") + error.what();
        trial.record_missing = true;
      }
      const auto expected = expected_hash.find(trial.instance_id);
      if (expected != expected_hash.end() &&
          trial.record.instance_sha256 != expected->second) {
        ++hash_mismatches;
        trial.record.status = "failed";
        trial.record.failure_reason =
            "instance SHA256 mismatch between driver and child";
        trial.record.valid = false;
        trial.record.solved = false;
      }
    }
    const bool keep_log = outcome.exit_code != 0 ||
                          trial.record.status != "success";
    if (keep_log && !outcome.log.empty()) {
      failures << "== trial " << index << " " << trial.case_id
               << " exit=" << outcome.exit_code
               << " status=" << trial.record.status << " ==\n"
               << outcome.log;
      if (outcome.log.back() != '\n') {
        failures << '\n';
      }
    }
    std::filesystem::remove(record_path, ec);
    std::filesystem::remove(log_path, ec);
    std::printf("[%d/%zu] %-28s %-13s b=%-4s seed=%2d -> %-14s wall=%s ms\n",
                index, trials.size(), trial.instance_id.c_str(),
                trial.backend_id.c_str(), format_number(trial.budget_s).c_str(),
                trial.seed, trial.record.status.c_str(),
                trial.record.wall_ms.has_value()
                    ? format_number(*trial.record.wall_ms).c_str()
                    : "-");
    std::fflush(stdout);
  }
  std::filesystem::remove_all(trial_dir, ec);

  // 输出：CSV / JSONL / 实例表 / 图 / summary。
  write_metrics_csv(work_dir / "benchmark_metrics.csv", run_id, trials);
  write_metrics_csv(work_dir / "metrics.csv", run_id, trials);
  write_trials_jsonl(work_dir / "trials.jsonl", trials);
  write_instances(work_dir, instances);
  write_plots(work_dir, trials);

  nlohmann::json summary;
  summary["schema_version"] = 1;
  summary["type"] = "planner_sampling_search_benchmark_summary";
  summary["suite"] = smoke ? "v1-smoke" : "v1";
  summary["run_id"] = run_id;
  summary["matrix"]["trials"] = static_cast<int>(trials.size());
  summary["matrix"]["child_processes"] = child_processes;
  summary["matrix"]["skipped_backends"] = skipped;
  summary["generators"] = {{"continuous", kContinuousGenerator},
                           {"grid", kGridGenerator}};
  summary["harness"] = {
      {"trial_process_mode",
       "one child process per trial; OMPL RNG::setSeed is a process-global "
       "singleton (provenance.md §2), so per-trial processes are required for "
       "reproducible seeding; peak RSS is per-trial process peak"},
      {"per_trial_process",
       child_processes == static_cast<int>(trials.size())},
      {"records_parsed", records_parsed},
      {"child_exit_zero", child_exit_zero},
      {"instance_sha256_mismatches", hash_mismatches},
      {"generator_self_check", "passed"},
      {"ok", child_processes == static_cast<int>(trials.size()) &&
                 records_parsed == static_cast<int>(trials.size()) &&
                 hash_mismatches == 0}};
  summary["first_solution_definition"] = {
      {"ompl", "ProblemDefinition::setIntermediateSolutionCallback first call"},
      {"ch0p1k3", "outer driver loop, first goal-domain node"},
      {"klampt", "outer PlanMore()+IsSolved() loop first observed solution"},
      {"vss2sn", "upstream-patched first-solution callback"},
      {"note",
       "cross-implementation first-solution semantics differ; per-backend "
       "source recorded in the first_solution_source column"}};
  summary["metrics_file"] = "benchmark_metrics.csv";
  summary["metrics_copy"] = "metrics.csv";
  summary["cache_mode"] = "cold";
  summary["notes"] = {
      "success rate denominator = success + timeout + no_path + invalid + "
      "failed; cancelled and interrupted are listed separately "
      "(benchmark_tiers §5.2)",
      "grid group uses seed-0 geometry per benchmark_tiers §2.4; "
      "cross-family consistency with grid_scenario_gen v1 pending",
      "edge_check_step is recorded per instance; continuous fixture uses "
      "exact segment-rectangle validity, grid fixture samples at resolution",
      "vss2sn collision_checks/improvements are not instrumented natively and "
      "are left empty (not 0)"};
  nlohmann::json groups_json;
  const std::vector<std::string> groups = {"continuous", "grid",
                                           "capacity_continuous",
                                           "capacity_grid"};
  for (const std::string& group : groups) {
    nlohmann::json backends_json;
    for (const BackendSpec& spec : backend_specs()) {
      bool any = false;
      for (const Trial& trial : trials) {
        if (trial.group == group && trial.backend_id == spec.id) {
          any = true;
          break;
        }
      }
      if (!any) {
        continue;
      }
      nlohmann::json backend_json;
      backend_json["backend"] = spec.backend;
      backend_json["variant"] = spec.variant;
      Aggregate overall;
      std::map<std::string, Aggregate> by_budget;
      std::map<std::string, Aggregate> by_instance;
      for (const Trial& trial : trials) {
        if (trial.group != group || trial.backend_id != spec.id) {
          continue;
        }
        add_trial(&overall, trial);
        add_trial(&by_budget[format_number(trial.budget_s)], trial);
        add_trial(&by_instance[trial.instance_id], trial);
      }
      backend_json["overall"] = aggregate_to_json(overall);
      nlohmann::json budgets_json;
      for (const auto& item : by_budget) {
        budgets_json[item.first] = aggregate_to_json(item.second);
      }
      backend_json["by_budget"] = budgets_json;
      nlohmann::json instances_json;
      for (const auto& item : by_instance) {
        instances_json[item.first] = aggregate_to_json(item.second);
      }
      backend_json["by_instance"] = instances_json;
      backends_json[spec.id] = backend_json;
    }
    if (!backends_json.empty()) {
      groups_json[group]["backends"] = backends_json;
    }
  }
  summary["groups"] = groups_json;
  {
    std::ofstream out(work_dir / "summary.json");
    out << std::setw(2) << summary << '\n';
  }
  write_summary_md(work_dir / "summary.md", summary["suite"].get<std::string>(),
                   run_id, trials, smoke);

  std::printf("summary: trials=%zu child_processes=%d records=%d "
              "hash_mismatches=%d skipped_backends=%zu\n",
              trials.size(), child_processes, records_parsed, hash_mismatches,
              skipped.size());
  std::printf("outputs: %s/benchmark_metrics.csv, summary.json, summary.md, "
              "instances.json, plots/\n",
              output_dir.c_str());

  if (smoke && !summary["harness"]["ok"].get<bool>()) {
    std::fprintf(stderr, "smoke harness self-check FAILED\n");
    return 1;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// --dump-instances
// ---------------------------------------------------------------------------

int dump_instances(const std::string& dir) {
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    std::fprintf(stderr, "cannot create dir: %s\n", dir.c_str());
    return 3;
  }
  const std::string self_check_error = generator_self_check(true);
  std::vector<Trial> trials;
  for (const std::string& id : continuous_instance_ids(true)) {
    Trial trial;
    trial.instance_id = id;
    trial.group = group_of(id);
    trial.class_name = instance_class(id);
    trial.scale = instance_scale(id);
    trials.push_back(std::move(trial));
  }
  for (const std::string& id : grid_instance_ids(true)) {
    Trial trial;
    trial.instance_id = id;
    trial.group = group_of(id);
    trial.class_name = instance_class(id);
    trial.scale = instance_scale(id);
    trials.push_back(std::move(trial));
  }
  const std::vector<InstanceEntry> instances = collect_instances(trials);
  write_instances(dir, instances);
  std::printf("instances: %zu (self-check: %s)\n", instances.size(),
              self_check_error.empty() ? "passed"
                                       : self_check_error.c_str());
  return self_check_error.empty() ? 0 : 1;
}

int print_usage() {
  std::fprintf(stderr,
               "usage:\n"
               "  planner_sampling_search_bench --suite v1 --output-dir <dir> "
               "[--smoke] [--child-grace-s <s>]\n"
               "  planner_sampling_search_bench --single --instance <id> "
               "--backend <id> --budget <s> --seed <n> --trial-out <file>\n"
               "  planner_sampling_search_bench --dump-instances <dir>\n");
  return 2;
}

int bench_main(int argc, char** argv) {
  std::string suite;
  std::string output_dir;
  std::string dump_dir;
  std::string instance;
  std::string backend;
  std::string trial_out;
  std::string budget_text;
  std::string seed_text;
  bool smoke = false;
  bool single = false;
  double grace_s = 60.0;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", name);
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--suite") {
      suite = next("--suite");
    } else if (arg == "--output-dir") {
      output_dir = next("--output-dir");
    } else if (arg == "--dump-instances") {
      dump_dir = next("--dump-instances");
    } else if (arg == "--single") {
      single = true;
    } else if (arg == "--smoke") {
      smoke = true;
    } else if (arg == "--instance") {
      instance = next("--instance");
    } else if (arg == "--backend") {
      backend = next("--backend");
    } else if (arg == "--budget") {
      budget_text = next("--budget");
    } else if (arg == "--seed") {
      seed_text = next("--seed");
    } else if (arg == "--trial-out") {
      trial_out = next("--trial-out");
    } else if (arg == "--child-grace-s") {
      grace_s = std::stod(next("--child-grace-s"));
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    } else {
      std::fprintf(stderr, "unrecognized argument: %s\n", arg.c_str());
      return 2;
    }
  }
  if (!dump_dir.empty()) {
    return dump_instances(dump_dir);
  }
  if (single) {
    if (instance.empty() || backend.empty() || budget_text.empty() ||
        seed_text.empty() || trial_out.empty()) {
      return print_usage();
    }
    return run_single(instance, backend, std::stod(budget_text),
                      static_cast<std::uint64_t>(std::stoull(seed_text)),
                      trial_out);
  }
  if (suite != "v1") {
    std::fprintf(stderr, "unsupported suite '%s' (only 'v1' is registered)\n",
                 suite.c_str());
    return 2;
  }
  if (output_dir.empty()) {
    std::fprintf(stderr, "--output-dir is required\n");
    return 2;
  }
  return run_driver(smoke, output_dir, grace_s);
}

}  // namespace
}  // namespace planner::sampling_search::bench

int main(int argc, char** argv) {
  using namespace planner::sampling_search::bench;
  try {
    return bench_main(argc, argv);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "fatal: %s\n", error.what());
    return 3;
  }
}
