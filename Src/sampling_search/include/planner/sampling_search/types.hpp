#pragma once

// planner::sampling_search —— 二维 RRT* 多实现适配的公共契约（X-S4）。
//
// 契约（组件扩展计划 §4.2 与候选登记 §4.2）：
// - 输入：二维边界、起终点、统一点/边有效性接口（沿边检查一致，非仅端点）、
//   种子、长度目标与预算；
// - 输出：状态/路径/代价/统计；no_path / timeout / cancel / invalid_input
//   明确区分；预算耗尽无解不作为不可达证明；
// - 本轮为二维几何采样规划评估，不宣称满足汽车运动学约束；
// - 公共头文件不暴露任何上游类型（OMPL / Ch0p1k3 / Klamp't / vss2sn /
//   ai-winter 的类型只出现在各自的私有适配目标内）。
//
// 状态语义（全适配器一致，测试逐项核对）：
// - kSuccess     ：预算内找到有效解（返回预算末最优解；长度目标达到时提前结束）；
// - kNoPath      ：仅在能证明不可达时使用（采样后端一般无法证明；本轮适配器
//                  不返回该状态，预算耗尽无解一律 kTimeout）；
// - kTimeout     ：预算耗尽且没有解（不是不可达证明）；
// - kCancelled   ：收到取消请求；丢弃部分结果（path 为空）；
// - kInvalidInput：输入被拒绝（越界、起终点无效、预算/步长非法、回调缺失等）。

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <planner/common/types.hpp>

namespace planner::sampling_search {

using common::Point2d;
using common::Status;

// 轴对齐二维边界（闭区间）；max > min 且各值有限。
struct Bounds2d {
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};

  double width() const { return max_x - min_x; }
  double height() const { return max_y - min_y; }

  bool contains(const Point2d& p, double tolerance = 0.0) const {
    return p.x_m >= min_x - tolerance && p.x_m <= max_x + tolerance &&
           p.y_m >= min_y - tolerance && p.y_m <= max_y + tolerance;
  }
};

// 统一点有效性回调：点是否位于自由空间（界内且不在障碍内）。
using PointValidityFn = std::function<bool(const Point2d&)>;

// 统一沿边有效性回调：整条线段是否有效。
// 契约要求实现方沿边采样检查（非仅端点），采样步长由夹具侧决定并记录在
// Problem::edge_check_step；适配器对需要显式步长的后端传入同一步长。
using EdgeValidityFn = std::function<bool(const Point2d&, const Point2d&)>;

// 协作式取消标志。调用方可在另一个线程设置 cancel()；
// 适配器在每个可检查点轮询 cancelled()。
class CancelToken {
 public:
  CancelToken() = default;
  CancelToken(const CancelToken&) = delete;
  CancelToken& operator=(const CancelToken&) = delete;

  void cancel() { flag_.store(true, std::memory_order_relaxed); }
  bool cancelled() const { return flag_.load(std::memory_order_relaxed); }

 private:
  std::atomic<bool> flag_{false};
};

// 统一输入。point_valid / edge_valid 必填；edge_check_step、budget_s 必须
// 为正有限值；start/goal 必须有限、在界内且点有效。
struct Problem {
  Bounds2d bounds{};
  Point2d start{};
  Point2d goal{};
  // 统一沿边检查步长（米）；同时作为需要显式步长的后端的运动校验步长。
  double edge_check_step{1.0};
  // 长度目标（米）：> 0 时达到该代价即提前结束；0 表示不设置。
  double length_target{0.0};
  // 墙钟预算（秒）；耗尽无解报 kTimeout，不作为不可达证明。
  double budget_s{1.0};
  // 统一输入的种子编号映射。不同实现不声称同编号种子产生同样采样序列。
  std::uint64_t seed{0};
  PointValidityFn point_valid;
  EdgeValidityFn edge_valid;
};

// 单次运行统计。缺失的字段保持默认值，不用 0 冒充已测量。
struct Statistics {
  // 后端报告的迭代/采样次数（不可得时为 0 并在 native_params 说明）。
  std::uint64_t iterations{0};
  // 统一有效性回调调用次数（可计数的适配器填写；不可得时为 0）。
  std::uint64_t edge_checks{0};
  // 首次找到有效解的时间（秒）；< 0 表示未观测到（后端无回调或未找到解）。
  double first_solution_s{-1.0};
  // 首次解之后的改进次数（不可得时为 0）。
  std::uint64_t improvements{0};
  // 后端树/图规模（里程碑数，不可得时为 0）。
  std::uint64_t nodes{0};
  // 生效预算与实测墙钟（秒）。
  double budget_s{0.0};
  double wall_s{0.0};
  // 适配器名与变体名（如 "ompl" / "rrtstar"、"informed_rrtstar"）。
  std::string backend;
  std::string variant;
  // 原生参数与偏差披露（自由文本，便于报告与 X-B1 复核）。
  std::string native_params;
};

struct Result {
  Status status{Status::kInvalidInput};
  // 成功时为完整折线（含起点与终点）；其余状态为空。
  std::vector<Point2d> path;
  // 路径长度（米）；无解时为 0。
  double cost{0.0};
  Statistics stats{};
  // 拒绝原因 / 状态说明（自由文本）。
  std::string message;

  bool success() const { return status == Status::kSuccess; }
};

// 统一后端接口。
class Planner {
 public:
  virtual ~Planner() = default;
  // 适配器标识（稳定字符串，供测试筛选与报告）。
  virtual const char* name() const = 0;
  // 执行一次规划。problem 在整个调用期间保持有效；cancel 可为 nullptr。
  virtual Result plan(const Problem& problem, CancelToken* cancel) = 0;
};

// ---------------------------------------------------------------------------
// 共享助手（库内实现，适配器与测试共用）
// ---------------------------------------------------------------------------

// 校验统一输入；返回空字符串表示通过，否则返回拒绝原因。
std::string validate_problem(const Problem& problem);

// 折线长度（米）；相邻点距离之和。点数 < 2 时返回 0。
double polyline_length(const std::vector<Point2d>& path);

// 用统一沿边回调复检整条路径的每条边（测试/结果自检用）。
bool path_edges_valid(const Problem& problem, const std::vector<Point2d>& path);

// 构造一个 kInvalidInput 结果。
Result invalid_result(const std::string& message);

// 起终点重合（欧氏距离 ≤ 1e-9）的平凡情形：所有适配器在输入有效且该判定
// 为真时返回单点成功路径（cost 0），不调用后端。构造器填入统一统计字段
// （backend/variant/budget/wall=0/first_solution=0）。
bool is_trivial_problem(const Problem& problem);
Result trivial_result(const Problem& problem, const char* backend,
                      const char* variant);

}  // namespace planner::sampling_search
