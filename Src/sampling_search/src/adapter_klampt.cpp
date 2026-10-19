// Klamp't / KrisLibrary 原生 rrt* 适配器（X-S4，连续空间组）。
//
// 上游：krishauser/KrisLibrary，commit f1c9ba76e287d9a3966b1ab3f870fa96e159ef7f
// （BSD-3-Clause）。经 FetchContent 以固定 URL + SHA256 获取；最小 48 文件
// 构建清单（见 CMakeLists.txt 与 provenance.md），不含 GLdraw/robotics/
// meshing/image/ROS/OMPL。
//
// 适配面（探针 supplement_klampt_probe.md §2.4）只涉及公共虚接口：
// - CSpace::Sample     → 边界内均匀采样（KrisLibrary Rand，Srand(seed) 播种）
// - CSpace::IsFeasible → 统一点有效性回调
// - CSpace::PathChecker→ 统一沿边有效性回调（非仅端点）
// - CSpace::Properties → volume/diameter/minimum/maximum（缺 diameter 时
//   连接半径退化为 1，探针实测无解，必须报告）
// - 手动 PlanMore()+IsSolved() 循环 → 墙钟预算、取消、首次解时间
// 采样策略、选父（KNN+ConnectEdge+decrease）、重连（近邻循环）全部在库内核，
// 未触碰；无上游补丁。
//
// 原生语义披露（登记于 provenance.md）：
// - 原生 "rrt*" = PRMStarPlanner(rrg=true)，属 RRT*/RRG* 家族：边维护为图式
//   （潜在改进即连边）+ 最短路父指针更新，与 OMPL RRTstar 的严格单父树语义
//   不同构；
// - 工厂 bidirectional 默认 true；本适配器固定 false 以与 OMPL RRTstar
//   （单向）对齐；
// - 无中间解回调：首次解时间由外层 PlanMore()+IsSolved() 循环测量，与 OMPL
//   setIntermediateSolutionCallback 口径不同（正式基准需统一）；
// - 预算末最优解 = 循环退出时 GetSolution() 的当前最优路径；固定迭代数下
//   逐位可复现（探针验证）。

#include "planner/sampling_search/planner.hpp"

#if defined(PLANNER_SAMPLING_SEARCH_HAS_KLAMPT)

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <KrisLibrary/Logger.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <KrisLibrary/planning/CSpace.h>
#include <KrisLibrary/planning/Path.h>

namespace planner::sampling_search {
namespace {

using Clock = std::chrono::steady_clock;

double seconds_since(const Clock::time_point& t0) {
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

std::string format_double(double value) {
  std::ostringstream out;
  out.precision(6);
  out << std::fixed << value;
  return out.str();
}

// 统一沿边检查的 EdgeChecker：IsVisible 直接调用夹具回调（步长由夹具决定）。
class FixtureEdgeChecker : public EdgeChecker {
 public:
  FixtureEdgeChecker(CSpace* space, const Config& a, const Config& b,
                     const Problem* problem, std::uint64_t* checks)
      : EdgeChecker(space, a, b), problem_(problem), checks_(checks) {}

  bool IsVisible() override {
    if (checks_ != nullptr) {
      ++(*checks_);
    }
    return problem_->edge_valid(
        Point2d{path->Start()(0), path->Start()(1)},
        Point2d{path->End()(0), path->End()(1)});
  }

  EdgePlannerPtr Copy() const override {
    return std::make_shared<FixtureEdgeChecker>(
        space, path->Start(), path->End(), problem_, checks_);
  }

  EdgePlannerPtr ReverseCopy() const override {
    return std::make_shared<FixtureEdgeChecker>(
        space, path->End(), path->Start(), problem_, checks_);
  }

 private:
  const Problem* problem_;
  std::uint64_t* checks_;
};

// 仅实现公共虚接口的二维连续空间。
class FixtureSpace : public CSpace {
 public:
  FixtureSpace(const Problem* problem, std::uint64_t* checks)
      : problem_(problem), checks_(checks) {}

  int NumDimensions() override { return 2; }

  void Sample(Config& x) override {
    x.resize(2);
    x(0) = Rand(problem_->bounds.min_x, problem_->bounds.max_x);
    x(1) = Rand(problem_->bounds.min_y, problem_->bounds.max_y);
  }

  bool IsFeasible(const Config& x) override {
    if (checks_ != nullptr) {
      ++(*checks_);
    }
    return problem_->point_valid(Point2d{x(0), x(1)});
  }

  EdgePlannerPtr PathChecker(const Config& a, const Config& b) override {
    return std::make_shared<FixtureEdgeChecker>(this, a, b, problem_, checks_);
  }

  // RRT* 连接半径依赖空间度量信息（AnyMotionPlanner.cpp:714-736）；
  // 缺少 diameter 时半径退化为 1（探针实测 0.5 s 无解）。
  void Properties(PropertyMap& map) override {
    CSpace::Properties(map);
    const double width = problem_->bounds.width();
    const double height = problem_->bounds.height();
    map.set("volume", width * height);
    map.set("diameter", std::sqrt(width * width + height * height));
    map.setArray("minimum", std::vector<Real>{problem_->bounds.min_x,
                                              problem_->bounds.min_y});
    map.setArray("maximum", std::vector<Real>{problem_->bounds.max_x,
                                              problem_->bounds.max_y});
  }

 private:
  const Problem* problem_;
  std::uint64_t* checks_;
};

class KlamptRrtStar : public Planner {
 public:
  const char* name() const override { return kBackendKlampt; }

  Result plan(const Problem& problem, CancelToken* cancel) override {
    const std::string invalid = validate_problem(problem);
    if (!invalid.empty()) {
      return invalid_result(invalid);
    }
    if (is_trivial_problem(problem)) {
      return trivial_result(problem, kBackendKlampt, kVariantRrtStar);
    }

    Result result;
    result.stats.backend = kBackendKlampt;
    result.stats.variant = kVariantRrtStar;
    result.stats.budget_s = problem.budget_s;

    std::uint64_t checks = 0;
    FixtureSpace space(&problem, &checks);

    Config start(2), goal(2);
    start(0) = problem.start.x_m;
    start(1) = problem.start.y_m;
    goal(0) = problem.goal.x_m;
    goal(1) = problem.goal.y_m;

    MotionPlannerFactory factory;
    factory.type = "rrt*";
    factory.bidirectional = false;  // 与 OMPL RRTstar（单向）对齐
    std::unique_ptr<MotionPlannerInterface> planner(
        factory.Create(&space, start, goal));
    if (!planner) {
      return invalid_result("klampt: MotionPlannerFactory failed to create rrt*");
    }

    Srand(static_cast<unsigned long>(problem.seed));

    const auto t0 = Clock::now();
    bool cancelled = false;
    bool solved_seen = false;
    double first_solution = -1.0;
    double best_length = std::numeric_limits<double>::infinity();
    std::uint64_t improvements = 0;
    std::uint64_t iterations = 0;

    while (seconds_since(t0) < problem.budget_s) {
      if (cancel != nullptr && cancel->cancelled()) {
        cancelled = true;
        break;
      }
      planner->PlanMore();
      ++iterations;
      if (planner->IsSolved()) {
        if (!solved_seen) {
          solved_seen = true;
          first_solution = seconds_since(t0);
        }
        if (problem.length_target > 0.0) {
          MilestonePath path;
          planner->GetSolution(path);
          const double length = path.Length();
          if (length < best_length) {
            if (std::isfinite(best_length)) {
              ++improvements;
            }
            best_length = length;
          }
          if (length <= problem.length_target) {
            break;
          }
        }
      }
    }

    result.stats.iterations = iterations;
    result.stats.edge_checks = checks;
    result.stats.first_solution_s = first_solution;
    result.stats.improvements = improvements;
    result.stats.nodes = static_cast<std::uint64_t>(planner->NumMilestones());
    result.stats.wall_s = seconds_since(t0);

    PropertyMap native_stats;
    planner->GetStats(native_stats);
    long native_edge_checks = 0;
    native_stats.get("numEdgeChecks", native_edge_checks);
    result.stats.native_params =
        "factory=rrt*(PRMStarPlanner rrg=true; RRT*/RRG* family);"
        "bidirectional=false;diameter=" +
        format_double(std::sqrt(problem.bounds.width() * problem.bounds.width() +
                                problem.bounds.height() * problem.bounds.height())) +
        ";volume=" + format_double(problem.bounds.width() * problem.bounds.height()) +
        ";point_locator=library default;native_numEdgeChecks=" +
        std::to_string(native_edge_checks) +
        ";first_solution=outer PlanMore loop;no upstream patch";

    if (cancelled) {
      result.status = Status::kCancelled;
      result.message = "cancelled; partial result discarded";
      return result;
    }
    if (!planner->IsSolved()) {
      result.status = Status::kTimeout;
      result.message =
          "budget exhausted without a solution (not an unreachability proof)";
      return result;
    }

    MilestonePath native_path;
    planner->GetSolution(native_path);
    if (native_path.NumMilestones() < 2) {
      result.status = Status::kTimeout;
      result.message = "klampt returned a degenerate solution path";
      return result;
    }
    std::vector<Point2d> path;
    path.reserve(static_cast<std::size_t>(native_path.NumMilestones()));
    for (int i = 0; i < native_path.NumMilestones(); ++i) {
      const Config& q = native_path.GetMilestone(i);
      path.push_back(Point2d{q(0), q(1)});
    }

    if (!path_edges_valid(problem, path)) {
      result.status = Status::kTimeout;
      result.message =
          "native solution rejected by unified edge re-check (see "
          "provenance.md)";
      return result;
    }

    result.status = Status::kSuccess;
    result.path = std::move(path);
    result.cost = polyline_length(result.path);
    return result;
  }
};

}  // namespace

bool klampt_rrtstar_available() { return true; }

std::unique_ptr<Planner> make_klampt_rrtstar() {
  return std::make_unique<KlamptRrtStar>();
}

}  // namespace planner::sampling_search

#endif  // PLANNER_SAMPLING_SEARCH_HAS_KLAMPT
