// planner_collision_bench --suite v1 --output-dir <dir>
//
// 正式碰撞微基准测试（Docs/component_development_plan.md 7.2/7.3 与 6.2；
// tools/README.md 第 7.2 节“基准输出契约”）。协议：每个配置预热 5 批、
// 随后测量 30 批，顺序单线程；确定性合成查询集与碰撞检查器均在计时区域之外
// 构建。
//
// 配置（batch = 对固定查询列表的一遍遍历）：
//   pose_typical        check_pose   典型场景   plain        10,000 ops
//   pose_dense          check_pose   密集场景   plain        10,000 ops
//   pose_dense_accel    check_pose   密集场景   accelerated  10,000 ops
//   motion_typical      check_motion 典型场景   plain        10,000 ops
//   motion_dense        check_motion 密集场景   plain        10,000 ops
//   motion_dense_accel  check_motion 密集场景   accelerated  10,000 ops
//   path_dense          check_path   密集场景   plain         2,000 ops
//   pose_dense_cold     check_pose   密集场景   plain，每次操作
//                                                     构建    1,000 ops
//
// 报告值：批 P50/P95 墙钟时间（每批毫秒数）以及由批时间推导的摊还
// ns/op / ops-per-second。摊还值绝不以单次调用 P95 的形式呈现。
//
// 正确性冒烟（计时区域之外）：
//   * 手算的典型场景用例（无障碍、s=2.2 m 处障碍物接触、倒车接触、未知栅格、
//     地图外）必须与库一致；
//   * 每个测量批重新汇总其碰撞计数与首碰弧长和，并与同一查询列表的未计时参考遍
//     对比；plain 与 accelerated 检查器必须完全一致。
//
// 输出写入 --output-dir（绝不写到目录之外）：
//   benchmark_metrics.csv  每次测量试验一行，含第 9 节的固定列以及组件列
//                          operation/phase/batch_size/ns_per_op/ops_per_s
//   summary.json           机器可读的汇总与检查结果
//   summary.md             人类可读的表格
//   plots/planner_collision_latency.svg + .csv
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
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/collision/collision_checker.hpp"

namespace {

using planner::collision::CheckOptions;
using planner::collision::CheckResult;
using planner::collision::CheckStatus;
using planner::collision::CollisionChecker;
using planner::collision::CollisionReason;
using planner::collision::MotionSegment;
using planner::collision::Obstacle;
using planner::collision::Scene;
using planner::common::Direction;
using planner::common::PathPoint;
using planner::common::PathSegment;
using planner::common::Point2d;
using planner::common::Pose2d;
using planner::common::VehicleModel;

constexpr int kWarmup = 5;
constexpr int kMeasured = 30;
constexpr std::uint64_t kSeed = 20261010ULL;
constexpr double kPi = 3.14159265358979323846;
constexpr int kPoseBatch = 10000;
constexpr int kMotionBatch = 10000;
constexpr int kPathBatch = 2000;
constexpr int kColdBatch = 1000;

class SplitMix64 {
 public:
  explicit SplitMix64(std::uint64_t seed) : state_(seed) {}
  std::uint64_t next() {
    state_ += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }
  double uniform() {
    return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0);
  }
  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }
  int uniform_int(int lo, int hi) {
    return lo + static_cast<int>(uniform() * (hi - lo + 1));
  }
  bool coin(double p = 0.5) { return uniform() < p; }

 private:
  std::uint64_t state_;
};

VehicleModel benchmark_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "default_test_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

// ---------------------------------------------------------------------------
// 确定性场景与查询集（在计时区域之外构建）
// ---------------------------------------------------------------------------

Scene make_free_grid(double width_m, double height_m,
                     const std::string& map_id) {
  Scene scene;
  scene.grid.map_id = map_id;
  scene.grid.map_version = "v1";
  scene.grid.resolution_m = 0.1;
  scene.grid.width = static_cast<int>(std::lround(width_m / 0.1));
  scene.grid.height = static_cast<int>(std::lround(height_m / 0.1));
  scene.grid.origin = Pose2d{0.0, 0.0, 0.0};
  scene.grid.cells.assign(
      static_cast<std::size_t>(scene.grid.width) * scene.grid.height, 0);
  return scene;
}

void paint_cells(planner::common::GridView& grid, double x_min, double y_min,
                 double x_max, double y_max, int value) {
  const int col0 = static_cast<int>(std::floor(x_min / grid.resolution_m));
  const int col1 = static_cast<int>(std::ceil(x_max / grid.resolution_m)) - 1;
  const int row0 = static_cast<int>(std::floor(y_min / grid.resolution_m));
  const int row1 = static_cast<int>(std::ceil(y_max / grid.resolution_m)) - 1;
  for (int row = std::max(row0, 0); row <= std::min(row1, grid.height - 1);
       ++row) {
    for (int col = std::max(col0, 0); col <= std::min(col1, grid.width - 1);
         ++col) {
      grid.cells[static_cast<std::size_t>(row) * grid.width + col] = value;
    }
  }
}

// 典型场景：40 m x 40 m 自由栅格，含两面已知墙；下方的手算冒烟用例针对此布局。
Scene make_typical_scene() {
  Scene scene = make_free_grid(40.0, 40.0, "bench_typical");
  scene.obstacles.push_back(Obstacle::rectangle("wall_a", 8.0, 1.0, 9.0, 4.0));
  scene.obstacles.push_back(
      Obstacle::rectangle("block_b", 20.0, 20.0, 22.0, 23.0));
  return scene;
}

// 密集场景：60 m x 60 m，含确定性矩形障碍物阵列、若干凸多边形与未知区域。
Scene make_dense_scene() {
  Scene scene = make_free_grid(60.0, 60.0, "bench_dense");
  SplitMix64 rng(kSeed ^ 0xD1CE5EEDULL);
  int obstacle_id = 0;
  for (double x = 3.0; x < 57.0; x += 6.0) {
    for (double y = 3.0; y < 57.0; y += 6.0) {
      const double w = rng.uniform(0.8, 3.5);
      const double h = rng.uniform(0.8, 3.5);
      const double jx = rng.uniform(-1.5, 1.5);
      const double jy = rng.uniform(-1.5, 1.5);
      const double x0 = x + jx;
      const double y0 = y + jy;
      scene.obstacles.push_back(Obstacle::rectangle(
          "obs_" + std::to_string(obstacle_id++), x0, y0, x0 + w, y0 + h));
      paint_cells(scene.grid, x0, y0, x0 + w, y0 + h, 1);
    }
  }
  // 若干凸多边形（旋转正方形），用于增加形状多样性。
  for (int i = 0; i < 6; ++i) {
    const double cx = rng.uniform(6.0, 54.0);
    const double cy = rng.uniform(6.0, 54.0);
    const double r = rng.uniform(1.0, 2.5);
    const double angle = rng.uniform(0.0, kPi);
    std::vector<Point2d> vertices;
    for (int k = 0; k < 4; ++k) {
      const double a = angle + k * 0.5 * kPi;
      vertices.push_back(Point2d{cx + r * std::cos(a), cy + r * std::sin(a)});
    }
    scene.obstacles.push_back(
        Obstacle::convex_polygon("poly_" + std::to_string(i), vertices));
  }
  // 未知区域（约占地图 4%）。
  for (int i = 0; i < 8; ++i) {
    const double x0 = rng.uniform(1.0, 55.0);
    const double y0 = rng.uniform(1.0, 55.0);
    paint_cells(scene.grid, x0, y0, x0 + rng.uniform(1.0, 4.0),
                y0 + rng.uniform(1.0, 4.0), 2);
  }
  return scene;
}

std::vector<Pose2d> make_pose_queries(int count, double width_m,
                                      double height_m, std::uint64_t seed) {
  SplitMix64 rng(seed);
  std::vector<Pose2d> poses;
  poses.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    poses.push_back(Pose2d{rng.uniform(1.0, width_m - 1.0),
                           rng.uniform(1.0, height_m - 1.0),
                           rng.uniform(-kPi, kPi)});
  }
  return poses;
}

std::vector<MotionSegment> make_motion_queries(int count, double width_m,
                                               double height_m,
                                               std::uint64_t seed) {
  SplitMix64 rng(seed);
  std::vector<MotionSegment> motions;
  motions.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    MotionSegment motion;
    motion.start = Pose2d{rng.uniform(1.0, width_m - 1.0),
                          rng.uniform(1.0, height_m - 1.0),
                          rng.uniform(-kPi, kPi)};
    motion.length_m = rng.uniform(0.05, 5.0);
    motion.direction = rng.coin() ? Direction::kForward : Direction::kReverse;
    motion.curvature_inv_m =
        rng.coin(0.2) ? 0.0 : rng.uniform(-0.195108, 0.195108);
    motions.push_back(motion);
  }
  return motions;
}

Pose2d pose_on_arc(const Pose2d& start, double s, double kappa) {
  const double b = kappa * s;
  const double cy = std::cos(start.yaw_rad);
  const double sy = std::sin(start.yaw_rad);
  const auto sinc = [](double x) {
    return std::abs(x) < 1e-8 ? 1.0 - x * x / 6.0 : std::sin(x) / x;
  };
  const double q = 0.5 * b;
  const double quartic = q * sinc(q) * sinc(q);
  const double dsin = cy * sinc(b) - sy * quartic;
  const double dcos = -sy * sinc(b) - cy * quartic;
  Pose2d out;
  out.x_m = start.x_m + s * dsin;
  out.y_m = start.y_m - s * dcos;
  out.yaw_rad = std::atan2(std::sin(start.yaw_rad + b),
                           std::cos(start.yaw_rad + b));
  return out;
}

std::vector<std::vector<PathSegment>> make_path_queries(int count,
                                                        double width_m,
                                                        double height_m,
                                                        std::uint64_t seed) {
  SplitMix64 rng(seed);
  std::vector<std::vector<PathSegment>> paths;
  paths.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const Pose2d start{rng.uniform(1.0, width_m - 1.0),
                       rng.uniform(1.0, height_m - 1.0),
                       rng.uniform(-kPi, kPi)};
    const double curvature = rng.coin(0.3) ? 0.0 : rng.uniform(-0.15, 0.15);
    PathSegment segment;
    double s = 0.0;
    for (int k = 0; k < 10; ++k) {
      PathPoint point;
      point.pose = pose_on_arc(start, s, curvature);
      point.s_m = s;
      point.curvature_inv_m = curvature;
      point.direction = Direction::kForward;
      segment.points.push_back(point);
      s += 0.5;
    }
    paths.push_back({segment});
  }
  return paths;
}

// ---------------------------------------------------------------------------
// 测量辅助
// ---------------------------------------------------------------------------

double percentile(const std::vector<double>& sorted, double percent) {
  if (sorted.empty()) {
    return 0.0;
  }
  const std::size_t rank = std::max<std::size_t>(
      1, std::min<std::size_t>(
             sorted.size(),
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

Stats summarize(std::vector<double> ms) {
  Stats stats;
  if (ms.empty()) {
    return stats;
  }
  std::sort(ms.begin(), ms.end());
  stats.min_ms = ms.front();
  stats.max_ms = ms.back();
  stats.p50_ms = percentile(ms, 50.0);
  stats.p95_ms = percentile(ms, 95.0);
  double sum = 0.0;
  for (const double value : ms) {
    sum += value;
  }
  stats.mean_ms = sum / static_cast<double>(ms.size());
  return stats;
}

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
  trial.cpu_ms =
      1000.0 * static_cast<double>(cpu_end - cpu_start) / CLOCKS_PER_SEC;
  return trial;
}

std::string format_number(double value) {
  std::ostringstream out;
  out << std::setprecision(12) << value;
  return out.str();
}

// 第 7.2 节契约文件中的一次测量试验。未测量的字段写为空单元格（绝不写 0）。
struct OutputRow {
  std::string case_id;
  std::string operation;
  std::string cache_mode{"warm"};
  int repeat_id{0};
  int batch_size{1};
  double wall_ms{0.0};
  double cpu_ms{0.0};
  long collision_checks{0};
};

void write_benchmark_metrics(const std::filesystem::path& path,
                             const std::string& run_id,
                             const std::vector<OutputRow>& rows) {
  std::ofstream out(path);
  out << "run_id,case_id,repeat_id,cache_mode,seed,status,failure_reason,"
         "preprocess_ms,wall_ms,cpu_ms,peak_rss_bytes,expanded_nodes,"
         "collision_checks,path_length_m,gear_changes,endpoint_position_error_m,"
         "endpoint_yaw_error_rad,valid,operation,phase,batch_size,ns_per_op,"
         "ops_per_s\n";
  for (const OutputRow& row : rows) {
    const double ns_per_op =
        row.wall_ms * 1e6 / static_cast<double>(std::max(1, row.batch_size));
    const double ops_per_s =
        row.wall_ms > 0.0
            ? static_cast<double>(row.batch_size) / (row.wall_ms * 1e-3)
            : 0.0;
    out << run_id << ',' << row.case_id << ',' << row.repeat_id << ','
        << row.cache_mode << ',' << kSeed << ",success,,"
        << ',' << format_number(row.wall_ms) << ','
        << format_number(row.cpu_ms) << ",,," << row.collision_checks
        << ",,,,,true," << row.operation << ",measure," << row.batch_size
        << ',' << format_number(ns_per_op) << ','
        << format_number(ops_per_s) << '\n';
  }
}

// 一次批运行的汇总，用于基准测试内的正确性检查。
struct BatchAggregate {
  long collisions{0};
  double first_s_sum{0.0};
};

struct ConfigResult {
  std::string id;
  std::string operation;
  std::string scene;
  std::string mode;  // "plain" | "accelerated" | "plain_cold"
  int batch_size{0};
  std::string cache_mode{"warm"};
  Stats stats;
  double ns_per_op{0.0};   // 摊还值，来自批 P50
  double ops_per_s{0.0};   // 摊还值，来自批 P50
  long expected_collisions{0};
  double expected_first_s_sum{0.0};
  bool check_ok{true};
};

// ---------------------------------------------------------------------------
// 批运行器（被计时的 lambda）
// ---------------------------------------------------------------------------

BatchAggregate run_pose_batch(const CollisionChecker& checker,
                              const std::vector<Pose2d>& poses) {
  BatchAggregate aggregate;
  for (const Pose2d& pose : poses) {
    const CheckResult result = checker.check_pose(pose);
    if (result.collided()) {
      ++aggregate.collisions;
      aggregate.first_s_sum += result.first_s_m;
    }
  }
  return aggregate;
}

BatchAggregate run_motion_batch(const CollisionChecker& checker,
                                const std::vector<MotionSegment>& motions) {
  BatchAggregate aggregate;
  for (const MotionSegment& motion : motions) {
    const CheckResult result = checker.check_motion(motion);
    if (result.collided()) {
      ++aggregate.collisions;
      aggregate.first_s_sum += result.first_s_m;
    }
  }
  return aggregate;
}

BatchAggregate run_path_batch(
    const CollisionChecker& checker,
    const std::vector<std::vector<PathSegment>>& paths) {
  BatchAggregate aggregate;
  for (const auto& path : paths) {
    const CheckResult result = checker.check_path(path);
    if (result.collided()) {
      ++aggregate.collisions;
      aggregate.first_s_sum += result.first_s_m;
    }
  }
  return aggregate;
}

// 冷启动变体：检查器（含场景校验与索引构建）在计时区域内按操作逐个构建。
BatchAggregate run_pose_cold_batch(const VehicleModel& vehicle,
                                   const Scene& scene,
                                   const std::vector<Pose2d>& poses) {
  BatchAggregate aggregate;
  const CheckOptions options;
  for (const Pose2d& pose : poses) {
    CollisionChecker checker(vehicle, scene, options);
    const CheckResult result = checker.check_pose(pose);
    if (result.collided()) {
      ++aggregate.collisions;
      aggregate.first_s_sum += result.first_s_m;
    }
  }
  return aggregate;
}

// ---------------------------------------------------------------------------
// 手算正确性冒烟（典型场景）
// ---------------------------------------------------------------------------

struct SmokeResult {
  bool ok{true};
  std::vector<std::string> failures;
};

void expect_clear(SmokeResult& smoke, const std::string& name,
                  const CheckResult& result) {
  if (!result.is_clear()) {
    smoke.ok = false;
    smoke.failures.push_back(name + ": expected clear, got " +
                             planner::collision::to_string(result.status) +
                             " (" + result.detail + ")");
  }
}

void expect_collision(SmokeResult& smoke, const std::string& name,
                      const CheckResult& result,
                      CollisionReason expected_reason) {
  if (!result.collided() || result.reason_code != expected_reason) {
    smoke.ok = false;
    smoke.failures.push_back(
        name + ": expected collision reason " +
        planner::collision::to_string(expected_reason) + ", got " +
        planner::collision::to_string(result.status) + "/" +
        planner::collision::to_string(result.reason_code) + " (" +
        result.detail + ")");
  }
}

void expect_near(SmokeResult& smoke, const std::string& name, double actual,
                 double expected, double tolerance) {
  if (!(std::abs(actual - expected) <= tolerance)) {
    smoke.ok = false;
    std::ostringstream out;
    out << name << ": expected " << expected << " +/- " << tolerance
        << ", got " << actual;
    smoke.failures.push_back(out.str());
  }
}

SmokeResult run_hand_computed_smoke() {
  SmokeResult smoke;
  const VehicleModel vehicle = benchmark_vehicle();
  const Scene scene = make_typical_scene();
  CollisionChecker checker(vehicle, scene, CheckOptions{});
  CollisionChecker accelerated(vehicle, scene, [] {
    CheckOptions options;
    options.enable_acceleration = true;
    return options;
  }());
  for (const CollisionChecker* instance : {&checker, &accelerated}) {
    // (2, 2, 0) 处足迹：x 在 [0.9, 5.8]，不与 wall_a 接触（x >= 8）。
    expect_clear(smoke, "pose_clear", instance->check_pose({2.0, 2.0, 0.0}));
    // (5, 2, 0) 处足迹：x 在 [3.9, 8.8]，接触 wall_a。
    expect_collision(smoke, "pose_wall_collision",
                     instance->check_pose({5.0, 2.0, 0.0}),
                     CollisionReason::kObstacle);
    // 从 (2, 2, 0) 前进：前保险杠 5.8 -> 在 s = 2.2 处接触。
    MotionSegment motion;
    motion.start = Pose2d{2.0, 2.0, 0.0};
    motion.length_m = 5.0;
    motion.direction = Direction::kForward;
    motion.curvature_inv_m = 0.0;
    const CheckResult forward = instance->check_motion(motion);
    expect_collision(smoke, "motion_wall_collision", forward,
                     CollisionReason::kObstacle);
    expect_near(smoke, "motion_first_contact_s", forward.first_s_m, 2.2,
                1.2e-3);
    // 从 (5, 6, 0) 倒车 2 m：后保险杠 3.9 -> 1.9；不与 wall_a（y 在 [1, 4]）
    // 和 block_b 接触。
    MotionSegment reverse = motion;
    reverse.start = Pose2d{5.0, 6.0, 0.0};
    reverse.length_m = 2.0;
    reverse.direction = Direction::kReverse;
    expect_clear(smoke, "reverse_clear", instance->check_motion(reverse));
  }
  // 同一场景上的栅格状态与地图边界。
  Scene unknown_scene = scene;
  paint_cells(unknown_scene.grid, 30.0, 30.0, 30.4, 30.4, 2);
  CollisionChecker unknown_checker(vehicle, unknown_scene, CheckOptions{});
  expect_collision(smoke, "unknown_cell",
                   unknown_checker.check_pose({29.0, 29.0, 0.0}),
                   CollisionReason::kGridUnknown);
  expect_collision(smoke, "outside_map",
                   checker.check_pose({100.0, 100.0, 0.0}),
                   CollisionReason::kOutsideMap);
  return smoke;
}

// 对查询列表的未计时参考遍，用于校验每个测量批的汇总（批运行器必须重现库结果）。
template <typename Queries, typename Fn>
BatchAggregate reference_aggregate(const Queries& queries, Fn&& check) {
  BatchAggregate aggregate;
  for (const auto& query : queries) {
    const CheckResult result = check(query);
    if (result.collided()) {
      ++aggregate.collisions;
      aggregate.first_s_sum += result.first_s_m;
    }
  }
  return aggregate;
}

bool aggregate_matches(const BatchAggregate& actual,
                       const BatchAggregate& expected) {
  return actual.collisions == expected.collisions &&
         actual.first_s_sum == expected.first_s_sum;
}

}  // namespace

int main(int argc, char** argv) {
  std::string suite = "v1";
  std::string output_dir = "planner_collision_bench_out";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--suite" && i + 1 < argc) {
      suite = argv[++i];
    } else if (arg == "--output-dir" && i + 1 < argc) {
      output_dir = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "usage: planner_collision_bench --suite v1 "
                   "--output-dir <dir>\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 2;
    }
  }
  if (suite != "v1") {
    std::cerr << "unsupported suite: " << suite << " (only v1)\n";
    return 2;
  }
  const char* run_id_env = std::getenv("PLANNER_RUN_ID");
  const std::string run_id = run_id_env != nullptr ? run_id_env
                                                   : "unregistered";

  std::error_code error;
  const std::filesystem::path out_dir(output_dir);
  std::filesystem::create_directories(out_dir / "plots", error);
  if (error) {
    std::cerr << "cannot create output directory " << out_dir << ": "
              << error.message() << "\n";
    return 2;
  }

  // 输入（计时区域之外）。
  const VehicleModel vehicle = benchmark_vehicle();
  const Scene typical_scene = make_typical_scene();
  const Scene dense_scene = make_dense_scene();
  const std::vector<Pose2d> typical_poses =
      make_pose_queries(kPoseBatch, 40.0, 40.0, kSeed ^ 0x11ULL);
  const std::vector<Pose2d> dense_poses =
      make_pose_queries(kPoseBatch, 60.0, 60.0, kSeed ^ 0x22ULL);
  const std::vector<MotionSegment> typical_motions =
      make_motion_queries(kMotionBatch, 40.0, 40.0, kSeed ^ 0x33ULL);
  const std::vector<MotionSegment> dense_motions =
      make_motion_queries(kMotionBatch, 60.0, 60.0, kSeed ^ 0x44ULL);
  const std::vector<std::vector<PathSegment>> dense_paths =
      make_path_queries(kPathBatch, 60.0, 60.0, kSeed ^ 0x55ULL);
  std::vector<Pose2d> cold_poses(dense_poses.begin(),
                                 dense_poses.begin() + kColdBatch);

  CheckOptions plain_options;
  plain_options.enable_acceleration = false;
  CheckOptions accelerated_options;
  accelerated_options.enable_acceleration = true;
  CollisionChecker typical_checker(vehicle, typical_scene, plain_options);
  CollisionChecker dense_checker(vehicle, dense_scene, plain_options);
  CollisionChecker dense_accelerated(vehicle, dense_scene,
                                     accelerated_options);
  for (const CollisionChecker* instance :
       {&typical_checker, &dense_checker, &dense_accelerated}) {
    if (!instance->valid()) {
      std::cerr << "internal error: benchmark scene invalid: "
                << instance->invalid_reason() << "\n";
      return 1;
    }
  }

  // 正确性冒烟，不计时。
  SmokeResult smoke = run_hand_computed_smoke();
  bool aggregates_ok = true;

  // 未计时的参考汇总。
  const BatchAggregate ref_pose_typical = reference_aggregate(
      typical_poses, [&](const Pose2d& p) { return typical_checker.check_pose(p); });
  const BatchAggregate ref_pose_dense = reference_aggregate(
      dense_poses, [&](const Pose2d& p) { return dense_checker.check_pose(p); });
  const BatchAggregate ref_pose_dense_accel = reference_aggregate(
      dense_poses, [&](const Pose2d& p) { return dense_accelerated.check_pose(p); });
  const BatchAggregate ref_motion_typical = reference_aggregate(
      typical_motions,
      [&](const MotionSegment& m) { return typical_checker.check_motion(m); });
  const BatchAggregate ref_motion_dense = reference_aggregate(
      dense_motions,
      [&](const MotionSegment& m) { return dense_checker.check_motion(m); });
  const BatchAggregate ref_motion_dense_accel = reference_aggregate(
      dense_motions,
      [&](const MotionSegment& m) { return dense_accelerated.check_motion(m); });
  const BatchAggregate ref_path_dense = reference_aggregate(
      dense_paths, [&](const std::vector<PathSegment>& p) {
        return dense_checker.check_path(p);
      });
  // 冷启动参考：每个查询构造一次检查器（与测量的冷批运行器一致）。
  BatchAggregate ref_pose_cold;
  for (const Pose2d& pose : cold_poses) {
    const BatchAggregate one =
        run_pose_cold_batch(vehicle, dense_scene, {pose});
    ref_pose_cold.collisions += one.collisions;
    ref_pose_cold.first_s_sum += one.first_s_sum;
  }
  // plain 与 accelerated 必须在密集查询集上完全一致。
  if (!aggregate_matches(ref_pose_dense, ref_pose_dense_accel) ||
      !aggregate_matches(ref_motion_dense, ref_motion_dense_accel)) {
    aggregates_ok = false;
    smoke.failures.push_back(
        "plain and accelerated reference aggregates differ on the dense sets");
  }

  // 测量协议：预热 5 批、测量 30 批，顺序单线程。
  std::vector<ConfigResult> configs;
  std::vector<OutputRow> rows;
  bool batches_ok = true;

  const auto measure = [&](const std::string& id, const std::string& operation,
                           const std::string& scene_name,
                           const std::string& mode, int batch_size,
                           const std::string& cache_mode,
                           const BatchAggregate& expected,
                           const std::function<BatchAggregate()>& batch) {
    ConfigResult config;
    config.id = id;
    config.operation = operation;
    config.scene = scene_name;
    config.mode = mode;
    config.batch_size = batch_size;
    config.cache_mode = cache_mode;
    config.expected_collisions = expected.collisions;
    config.expected_first_s_sum = expected.first_s_sum;
    for (int i = 0; i < kWarmup; ++i) {
      batch();
    }
    std::vector<double> wall_ms;
    std::vector<double> cpu_ms;
    wall_ms.reserve(kMeasured);
    cpu_ms.reserve(kMeasured);
    for (int repeat = 0; repeat < kMeasured; ++repeat) {
      BatchAggregate aggregate;
      const Trial trial = run_trial([&]() { aggregate = batch(); });
      wall_ms.push_back(trial.wall_ms);
      cpu_ms.push_back(trial.cpu_ms);
      if (!aggregate_matches(aggregate, expected)) {
        config.check_ok = false;
        batches_ok = false;
      }
      OutputRow row;
      row.case_id = id;
      row.operation = operation;
      row.cache_mode = cache_mode;
      row.repeat_id = repeat;
      row.batch_size = batch_size;
      row.wall_ms = trial.wall_ms;
      row.cpu_ms = trial.cpu_ms;
      row.collision_checks = batch_size;
      rows.push_back(row);
    }
    config.stats = summarize(wall_ms);
    config.ns_per_op = config.stats.p50_ms * 1e6 / batch_size;
    config.ops_per_s = batch_size / (config.stats.p50_ms * 1e-3);
    configs.push_back(config);
  };

  measure("pose_typical", "check_pose", "typical", "plain", kPoseBatch, "warm",
          ref_pose_typical,
          [&]() { return run_pose_batch(typical_checker, typical_poses); });
  measure("pose_dense", "check_pose", "dense", "plain", kPoseBatch, "warm",
          ref_pose_dense,
          [&]() { return run_pose_batch(dense_checker, dense_poses); });
  measure("pose_dense_accel", "check_pose", "dense", "accelerated",
          kPoseBatch, "warm", ref_pose_dense_accel,
          [&]() { return run_pose_batch(dense_accelerated, dense_poses); });
  measure("motion_typical", "check_motion", "typical", "plain", kMotionBatch,
          "warm", ref_motion_typical,
          [&]() { return run_motion_batch(typical_checker, typical_motions); });
  measure("motion_dense", "check_motion", "dense", "plain", kMotionBatch,
          "warm", ref_motion_dense,
          [&]() { return run_motion_batch(dense_checker, dense_motions); });
  measure("motion_dense_accel", "check_motion", "dense", "accelerated",
          kMotionBatch, "warm", ref_motion_dense_accel,
          [&]() { return run_motion_batch(dense_accelerated, dense_motions); });
  measure("path_dense", "check_path", "dense", "plain", kPathBatch, "warm",
          ref_path_dense,
          [&]() { return run_path_batch(dense_checker, dense_paths); });
  measure("pose_dense_cold", "check_pose", "dense", "plain_cold", kColdBatch,
          "cold", ref_pose_cold,
          [&]() { return run_pose_cold_batch(vehicle, dense_scene, cold_poses); });

  write_benchmark_metrics(out_dir / "benchmark_metrics.csv", run_id, rows);

  // 图表：成对的 SVG + CSV，数值以文字印出（而非仅靠颜色区分）。
  {
    std::ofstream csv(out_dir / "plots" / "planner_collision_latency.csv");
    csv << "config,operation,scene,mode,cache_mode,batch_size,p50_ms,p95_ms,"
           "mean_ms,min_ms,max_ms,ns_per_op,ops_per_s,collisions\n";
    csv << std::setprecision(12);
    double max_p95 = 0.0;
    for (const ConfigResult& config : configs) {
      max_p95 = std::max(max_p95, config.stats.p95_ms);
      csv << config.id << ',' << config.operation << ',' << config.scene << ','
          << config.mode << ',' << config.cache_mode << ',' << config.batch_size
          << ',' << config.stats.p50_ms << ',' << config.stats.p95_ms << ','
          << config.stats.mean_ms << ',' << config.stats.min_ms << ','
          << config.stats.max_ms << ',' << config.ns_per_op << ','
          << config.ops_per_s << ',' << config.expected_collisions << '\n';
    }
    const int row_height = 46;
    const int top = 58;
    const int left = 330;
    const int bar_width = 360;
    const int height =
        top + row_height * static_cast<int>(configs.size()) + 24;
    std::ofstream svg(out_dir / "plots" / "planner_collision_latency.svg");
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"860\" height=\""
        << height << "\" viewBox=\"0 0 860 " << height << "\">\n";
    svg << "<title>planner_collision suite v1 latency (P50/P95 per batch, ms)"
           "</title>\n";
    svg << "<rect width=\"860\" height=\"" << height
        << "\" fill=\"#ffffff\"/>\n";
    svg << "<text x=\"20\" y=\"24\" font-size=\"15\" font-family=\"sans-serif\">"
           "planner_collision v1 latency: P50/P95 per batch (ms); values "
           "printed as text</text>\n";
    svg << "<text x=\"" << left
        << "\" y=\"42\" font-size=\"11\" font-family=\"sans-serif\" "
           "fill=\"#333333\">P50 (blue) / P95 (orange), width scaled to max "
           "P95 = "
        << format_number(max_p95) << " ms</text>\n";
    const double scale =
        max_p95 > 0.0 ? static_cast<double>(bar_width) / max_p95 : 0.0;
    int row = 0;
    for (const ConfigResult& config : configs) {
      const int y = top + row * row_height;
      svg << "<text x=\"20\" y=\"" << y + 14
          << "\" font-size=\"12\" font-family=\"sans-serif\">" << config.id
          << " (" << config.operation << ", batch=" << config.batch_size
          << ")</text>\n";
      const int w50 =
          static_cast<int>(std::lround(config.stats.p50_ms * scale));
      const int w95 =
          static_cast<int>(std::lround(config.stats.p95_ms * scale));
      svg << "<rect x=\"" << left << "\" y=\"" << y + 2 << "\" width=\""
          << std::max(1, w50) << "\" height=\"14\" fill=\"#4477aa\"/>\n";
      svg << "<text x=\"" << left + std::max(1, w50) + 6 << "\" y=\"" << y + 14
          << "\" font-size=\"11\" font-family=\"sans-serif\">P50="
          << format_number(config.stats.p50_ms) << " ms</text>\n";
      svg << "<rect x=\"" << left << "\" y=\"" << y + 20 << "\" width=\""
          << std::max(1, w95) << "\" height=\"14\" fill=\"#ee8866\"/>\n";
      svg << "<text x=\"" << left + std::max(1, w95) + 6 << "\" y=\"" << y + 32
          << "\" font-size=\"11\" font-family=\"sans-serif\">P95="
          << format_number(config.stats.p95_ms) << " ms</text>\n";
      ++row;
    }
    svg << "</svg>\n";
  }

  // 摘要（机器可读 + 人类可读）。
  const bool checks_ok = smoke.ok && aggregates_ok && batches_ok;
  {
    nlohmann::json summary;
    summary["schema_version"] = 1;
    summary["type"] = "planner_collision_benchmark_summary";
    summary["suite"] = suite;
    summary["run_id"] = run_id;
    summary["seed"] = kSeed;
    summary["warmup"] = kWarmup;
    summary["measured"] = kMeasured;
    summary["threading"] = "sequential_single_threaded";
    summary["metrics_file"] = "benchmark_metrics.csv";
    summary["note"] =
        "P50/P95 are per-batch wall times; ns_per_op and ops_per_s are "
        "amortized batch values, not single-call percentiles";
    summary["vehicle"] = {{"id", vehicle.id},
                          {"wheelbase_m", vehicle.wheelbase_m},
                          {"width_m", vehicle.width_m},
                          {"safety_margin_m", vehicle.safety_margin_m}};
    summary["scenes"] = {
        {"typical", {{"size_m", {40.0, 40.0}},
                     {"resolution_m", 0.1},
                     {"obstacles", static_cast<int>(typical_scene.obstacles.size())}}},
        {"dense", {{"size_m", {60.0, 60.0}},
                   {"resolution_m", 0.1},
                   {"obstacles", static_cast<int>(dense_scene.obstacles.size())}}}};
    summary["query_counts"] = {{"pose_typical", kPoseBatch},
                               {"pose_dense", kPoseBatch},
                               {"motion_typical", kMotionBatch},
                               {"motion_dense", kMotionBatch},
                               {"path_dense", kPathBatch},
                               {"pose_dense_cold", kColdBatch}};
    summary["checks"] = {
        {"hand_computed_smoke", smoke.ok},
        {"plain_vs_accelerated_aggregates", aggregates_ok},
        {"batch_aggregates_match_library", batches_ok},
        {"failures", smoke.failures}};
    nlohmann::json configs_json = nlohmann::json::array();
    for (const ConfigResult& config : configs) {
      configs_json.push_back(
          {{"config", config.id},
           {"operation", config.operation},
           {"scene", config.scene},
           {"mode", config.mode},
           {"cache_mode", config.cache_mode},
           {"batch_size", config.batch_size},
           {"p50_ms", config.stats.p50_ms},
           {"p95_ms", config.stats.p95_ms},
           {"mean_ms", config.stats.mean_ms},
           {"min_ms", config.stats.min_ms},
           {"max_ms", config.stats.max_ms},
           {"ns_per_op", config.ns_per_op},
           {"ops_per_s", config.ops_per_s},
           {"collisions_per_batch", config.expected_collisions},
           {"batch_check_ok", config.check_ok}});
    }
    summary["configs"] = configs_json;
    summary["passed"] = checks_ok;
    std::ofstream json_out(out_dir / "summary.json");
    json_out << std::setw(2) << summary << '\n';
  }
  {
    std::ofstream md(out_dir / "summary.md");
    md << "# planner_collision 基准摘要（suite " << suite << "）\n\n";
    md << "- 协议：每个配置预热 " << kWarmup << " 批、测量 " << kMeasured
       << " 批，顺序单线程；查询集为确定性合成输入（seed " << kSeed
       << "），输入生成、检查器构建与输出不计时。\n";
    md << "- 车辆：wheelbase=" << vehicle.wheelbase_m
       << " m, width=" << vehicle.width_m
       << " m, safety_margin=" << vehicle.safety_margin_m << " m。\n";
    md << "- 批大小：pose=" << kPoseBatch << "，motion=" << kMotionBatch
       << "，path=" << kPathBatch << "，cold pose=" << kColdBatch << "。\n";
    md << "- 指标文件：`benchmark_metrics.csv`（§7.2 契约；每行一个测量批，"
          "预热不计；未测量列留空）。`cache_mode=warm` 复用已构建检查器，"
          "`cold` 表示每次操作内重新构建检查器（含场景校验）。\n";
    md << "- P50/P95 为**每批**耗时；ns/op 与 ops/s 为批均值摊还值，"
          "不得当作单次调用 P95。\n\n";
    md << "## 批操作（单位：ms；P50/P95 为每批耗时）\n\n";
    md << "| config | operation | scene | mode | batch | P50 | P95 | mean | "
          "ns/op (P50) | ops/s (P50) | collisions/batch | check |\n";
    md << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n";
    md << std::fixed << std::setprecision(4);
    for (const ConfigResult& config : configs) {
      md << "| " << config.id << " | " << config.operation << " | "
         << config.scene << " | " << config.mode << " | " << config.batch_size
         << " | " << config.stats.p50_ms << " | " << config.stats.p95_ms
         << " | " << config.stats.mean_ms << " | " << std::setprecision(1)
         << config.ns_per_op << " | " << config.ops_per_s << " | "
         << config.expected_collisions << " | "
         << (config.check_ok ? "ok" : "FAILED") << " |\n";
      md << std::setprecision(4);
    }
    md << "\n## 正确性冒烟\n\n";
    md << "- 手算用例（typical 场景，朴素与加速检查器各一遍）："
       << (smoke.ok ? "通过" : "失败") << "\n";
    for (const std::string& failure : smoke.failures) {
      md << "  - " << failure << "\n";
    }
    md << "- 每个测量批的碰撞计数与首碰弧长和与未计时参考遍一致；"
          "朴素与加速在密集查询集上完全一致："
       << (checks_ok ? "通过" : "失败") << "\n";
    md << "- 总判定：" << (checks_ok ? "通过" : "失败") << "\n";
    md << "- 图表：`plots/planner_collision_latency.svg` / "
          "`plots/planner_collision_latency.csv`\n";
  }

  std::cout << "planner_collision_bench suite=" << suite
            << " configs=" << configs.size() << " rows=" << rows.size()
            << " checks=" << (checks_ok ? "ok" : "FAILED")
            << " run_id=" << run_id << " output_dir=" << out_dir << "\n";
  return checks_ok ? 0 : 1;
}
