#pragma once

// 确定性随机用例生成器，用于 Docs/component_development_plan.md §7.1 与
// tests/fixtures/v1/collision/random_spec.json 规定的差分对比。
//
// 该生成器只生成内存中的用例；调用方（差分运行器）负责输出证据。有两点
// 很关键：
//   * 每个用例都可由 (regime.seed, ordinal, attempt) 复现；
//   * 用例对象完全确定检查输入，因此生产检查器、独立参考实现与可选的
//     加速路径消费完全相同的字节。
//
// 分层：每类检查一半为宽范围随机；一半集中于五个对抗族（角点擦碰、精确
// 接触、薄障碍物、倒车运动、地图边缘）。按 random_spec.json 中的分层
// 定义，reverse_motion 仅适用于运动检查。

#include <limits>
#include <string>
#include <vector>

#include "planner/collision/collision_checker.hpp"
#include "planner/collision/reference.hpp"
#include "planner/collision/scene.hpp"

namespace planner::collision::differential {

// 生成语义变化时递增；记录在每个证据文件中。
inline constexpr const char* kGeneratorVersion = "1.0.0";

const VehicleModel& default_test_vehicle();

// 与 tests/fixtures/v1/collision/random_spec.json 对应的参数。
struct RegimeSpec {
  std::string name;       // "dev_regression" 或 "acceptance"
  std::string id_prefix;  // "cr-dev" / "cr-acc"
  int seed{0};
  int pose_checks{0};
  int motion_checks{0};
};

// dev_regression：10,000 个用例，种子 1000（5,000 位姿 + 5,000 运动）。
RegimeSpec dev_regime_spec();
// acceptance：每个种子 0..99 生成 1,000 个用例（500 位姿 + 500 运动）。
RegimeSpec acceptance_regime_spec(int seed);

enum class Stratum {
  kWideRandom,
  kCornerGraze,
  kExactContact,
  kThinObstacle,
  kReverseMotion,
  kMapEdge
};

const char* to_string(Stratum stratum) noexcept;

struct CaseAllocation {
  bool motion{false};  // false = 位姿检查
  Stratum stratum{Stratum::kWideRandom};
};

// 整个 regime 的确定性分配，位姿用例在前（与 random_spec.json 中的 seq
// 范围一致：500/500 时为位姿 0000..0499、运动 0500..0999；dev 类似为
// 00000..04999 / 05000..09999）。
std::vector<CaseAllocation> build_regime_allocation(const RegimeSpec& regime);

// cr-<dev|acc>-s<seed>-<seq>-<pose|motion>；seq 补零，替换用例在标称范围
// 之后继续编号并保持唯一。
std::string format_case_id(const RegimeSpec& regime, int sequence,
                           bool motion);

struct DifferentialCase {
  std::string case_id;
  int seed{0};
  std::string regime;
  std::string check_kind;  // "pose" | "motion"
  Stratum stratum{Stratum::kWideRandom};
  VehicleModel vehicle;
  Pose2d start;  // 位姿，或运动起点
  double length_m{0.0};
  Direction direction{Direction::kForward};
  double curvature_inv_m{0.0};
  Scene scene;
  std::string replacement_of;  // 本用例为替换用例时的原 case_id

  // 集中分层会记录其设计间隙与实测间隙（特征障碍物与足迹 / 扫掠足迹的
  // 距离）。宽范围随机以及没有设计间隙的构造为 NaN。
  double design_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double measured_clearance_m{std::numeric_limits<double>::quiet_NaN()};
};

// 生成一个用例。`ordinal` 是运行内的生成计数器，控制 RNG 与用例 id
// （sequence）；当集中构造未落入其分层窗口时，`attempt` 允许确定性重试。
DifferentialCase generate_case(const RegimeSpec& regime,
                               const CaseAllocation& slot, int ordinal,
                               int attempt,
                               const std::string& replacement_of = "");

// 对生成的用例运行生产检查器。
CheckResult production_check(const DifferentialCase& test_case,
                             const CheckOptions& options);

// 对生成的用例运行独立参考实现。
ReferenceResult reference_check(const DifferentialCase& test_case);

}  // namespace planner::collision::differential
