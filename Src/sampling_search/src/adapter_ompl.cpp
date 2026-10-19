// OMPL 1.6.0 RRT* / Informed RRT* 适配器（X-S4，连续空间组）。
//
// 上游：OMPL 1.6.0（BSD-3-Clause），复用 dependencies.lock.json 既有条目
// （commit e2994e58…，URL + SHA256 校验；不静默升级）。沿用 RS 阶段
// planner_rs_ompl_reference 的私有适配器模式：公共 API 不暴露 OMPL 类型。
//
// 适配面（探针 family_d_probe.md §2.3）：
// - ob::StateValidityChecker → 统一点有效性；
// - ob::MotionValidator      → 统一沿边有效性（显式步长参数 edge_check_step
//   写入结果元数据；夹具回调自身沿边采样，非仅端点）；
// - 预算/取消：timedPlannerTerminationCondition 与取消条件经
//   plannerOrTerminationCondition 组合；预算耗尽返回 kTimeout 且不误报不可达；
// - 种子：ompl::RNG::setSeed（统一输入的种子映射；不声称跨实现同种子同序列）；
// - 首次解：ProblemDefinition::setIntermediateSolutionCallback（首次回调计时）；
//   预算末最优解与首次解分开记录；
// - 长度目标：PathLengthOptimizationObjective::setCostThreshold，RRT* 在
//   isSatisfied(bestCost) 时提前结束（OMPL 1.6.0 原生语义）；
// - 输出：og::PathGeometric → 统一折线；返回前经统一沿边复检。
//
// 变体：RRTstar 与 InformedRRTstar 为同一库两个变体，Informed 单列
// （kBackendOmpl + kVariantInformedRrtStar），不充当独立来源。

#include "planner/sampling_search/planner.hpp"

#if defined(PLANNER_SAMPLING_SEARCH_HAS_OMPL)

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>

namespace planner::sampling_search {
namespace {

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Clock = std::chrono::steady_clock;

using OmplState = ob::RealVectorStateSpace::StateType;

double seconds_since(const Clock::time_point& t0) {
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

struct Counters {
  std::uint64_t point_checks{0};
  std::uint64_t edge_checks{0};
};

Point2d to_point(const ob::State* state) {
  const auto* s = state->as<OmplState>();
  return Point2d{s->values[0], s->values[1]};
}

class FixtureValidityChecker : public ob::StateValidityChecker {
 public:
  FixtureValidityChecker(const ob::SpaceInformationPtr& si,
                         const Problem* problem, Counters* counters)
      : ob::StateValidityChecker(si), problem_(problem), counters_(counters) {}

  bool isValid(const ob::State* state) const override {
    if (counters_ != nullptr) {
      ++counters_->point_checks;
    }
    return problem_->point_valid(to_point(state));
  }

 private:
  const Problem* problem_;
  Counters* counters_;
};

class FixtureMotionValidator : public ob::MotionValidator {
 public:
  FixtureMotionValidator(ob::SpaceInformation* si, const Problem* problem,
                         Counters* counters)
      : ob::MotionValidator(si), problem_(problem), counters_(counters) {}

  // 统一沿边回调（夹具按 edge_check_step 沿边采样，非仅端点）。
  bool checkMotion(const ob::State* s1, const ob::State* s2) const override {
    if (counters_ != nullptr) {
      ++counters_->edge_checks;
    }
    return problem_->edge_valid(to_point(s1), to_point(s2));
  }

  // OMPL 的部分调用方需要"最后有效状态"。统一回调是不透明布尔函数，
  // 因此在 [0,1] 上对 edge_valid(s1, interp(t)) 做二分，取最长有效前缀。
  bool checkMotion(const ob::State* s1, const ob::State* s2,
                   std::pair<ob::State*, double>& lastValid) const override {
    if (checkMotion(s1, s2)) {
      return true;
    }
    double low = 0.0;
    double high = 1.0;
    ob::State* probe = si_->allocState();
    for (int i = 0; i < 40; ++i) {
      const double mid = 0.5 * (low + high);
      si_->getStateSpace()->interpolate(s1, s2, mid, probe);
      if (counters_ != nullptr) {
        ++counters_->edge_checks;
      }
      if (problem_->edge_valid(to_point(s1), to_point(probe))) {
        low = mid;
      } else {
        high = mid;
      }
    }
    si_->getStateSpace()->interpolate(s1, s2, low, lastValid.first);
    lastValid.second = low;
    si_->freeState(probe);
    return false;
  }

 private:
  const Problem* problem_;
  Counters* counters_;
};

class OmplPlanner : public Planner {
 public:
  explicit OmplPlanner(bool informed) : informed_(informed) {}

  const char* name() const override { return kBackendOmpl; }

  Result plan(const Problem& problem, CancelToken* cancel) override {
    const std::string invalid = validate_problem(problem);
    if (!invalid.empty()) {
      return invalid_result(invalid);
    }
    if (is_trivial_problem(problem)) {
      return trivial_result(problem, kBackendOmpl,
                            informed_ ? kVariantInformedRrtStar
                                      : kVariantRrtStar);
    }

    Result result;
    result.stats.backend = kBackendOmpl;
    result.stats.variant = informed_ ? kVariantInformedRrtStar : kVariantRrtStar;
    result.stats.budget_s = problem.budget_s;

    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, problem.bounds.min_x);
    bounds.setHigh(0, problem.bounds.max_x);
    bounds.setLow(1, problem.bounds.min_y);
    bounds.setHigh(1, problem.bounds.max_y);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    const ob::SpaceInformationPtr si = ss.getSpaceInformation();
    Counters counters;
    ss.setStateValidityChecker(
        std::make_shared<FixtureValidityChecker>(si, &problem, &counters));
    si->setMotionValidator(
        std::make_shared<FixtureMotionValidator>(si.get(), &problem, &counters));

    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    start->values[0] = problem.start.x_m;
    start->values[1] = problem.start.y_m;
    goal->values[0] = problem.goal.x_m;
    goal->values[1] = problem.goal.y_m;
    ss.setStartAndGoalStates(start, goal);

    // 长度目标：PathLengthOptimizationObjective 的代价阈值；RRT* 在
    // opt_->isSatisfied(bestCost_) 时提前结束（OMPL 1.6.0 原生路径）。
    auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    if (problem.length_target > 0.0) {
      objective->setCostThreshold(ob::Cost(problem.length_target));
    }
    ss.getProblemDefinition()->setOptimizationObjective(objective);

    if (informed_) {
      ss.setPlanner(std::make_shared<og::InformedRRTstar>(si));
    } else {
      ss.setPlanner(std::make_shared<og::RRTstar>(si));
    }

    // 统一输入的种子编号映射。OMPL 1.6.0 的 RNG::setSeed(0) 会被忽略
    // （0 视为"不播种"），因此映射为 seed+1，保证每个统一种子都可复现；
    // 不声称与其他实现同种子同采样序列。
    ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(problem.seed + 1));

    double first_solution = -1.0;
    std::uint64_t improvements = 0;
    const auto t0 = Clock::now();
    ss.getProblemDefinition()->setIntermediateSolutionCallback(
        [&](const ob::Planner*, const std::vector<const ob::State*>&,
            const ob::Cost) {
          if (first_solution < 0.0) {
            first_solution = seconds_since(t0);
          } else {
            ++improvements;
          }
        });

    ob::PlannerTerminationCondition budget_tc =
        ob::timedPlannerTerminationCondition(problem.budget_s);
    ob::PlannerTerminationCondition cancel_tc([&]() {
      return cancel != nullptr && cancel->cancelled();
    });
    ob::PlannerTerminationCondition combined_tc =
        ob::plannerOrTerminationCondition(budget_tc, cancel_tc);

    const ob::PlannerStatus status = ss.solve(combined_tc);
    const double wall = seconds_since(t0);

    result.stats.edge_checks = counters.edge_checks;
    result.stats.first_solution_s = first_solution;
    result.stats.improvements = improvements;
    result.stats.wall_s = wall;
    if (auto* rrtstar = dynamic_cast<og::RRTstar*>(ss.getPlanner().get())) {
      result.stats.iterations =
          static_cast<std::uint64_t>(rrtstar->numIterations());
    }
    result.stats.native_params =
        "ompl=1.6.0;range=default;goal_bias=default;informed=" +
        std::string(informed_ ? "true" : "false") +
        ";edge_check_step=" + std::to_string(problem.edge_check_step) +
        ";termination=timed|cancel;seed=RNG::setSeed(seed+1; 0 maps to 1);"
        "rng_note=setSeed is a process-global singleton and is ignored after "
        "the first RNG use (upstream behaviour); reproducible seeding needs "
        "one planner per process;status=" +
        status.asString();

    if (cancel != nullptr && cancel->cancelled()) {
      result.status = Status::kCancelled;
      result.message = "cancelled; partial result discarded";
      return result;
    }
    // 只接受精确解：预算耗尽时 OMPL 可能上报"近似解"（末点未达目标），
    // 按统一语义视为未找到解（kTimeout），不返回无效/未达目标的路径。
    if (!ss.haveExactSolutionPath()) {
      result.status = Status::kTimeout;
      result.message =
          ss.haveSolutionPath()
              ? "only an approximate (non-exact) solution was found; discarded "
                "(not an unreachability proof)"
              : "budget exhausted without a solution (not an unreachability "
                "proof)";
      return result;
    }

    const og::PathGeometric& native = ss.getSolutionPath();
    std::vector<Point2d> path;
    path.reserve(native.getStateCount());
    for (std::size_t i = 0; i < native.getStateCount(); ++i) {
      path.push_back(to_point(native.getState(i)));
    }

    if (!path_edges_valid(problem, path)) {
      result.status = Status::kTimeout;
      result.message =
          "native solution rejected by unified edge re-check (see "
          "provenance.md)";
      return result;
    }
    // 端点安全网（精确解本应精确到达；防御性检查，不满足按无解处理）。
    const double start_gap =
        std::hypot(path.front().x_m - problem.start.x_m,
                   path.front().y_m - problem.start.y_m);
    const double goal_gap = std::hypot(path.back().x_m - problem.goal.x_m,
                                       path.back().y_m - problem.goal.y_m);
    if (start_gap > 1e-6 || goal_gap > 1e-6) {
      result.status = Status::kTimeout;
      result.message = "exact solution endpoints outside tolerance; discarded";
      return result;
    }

    result.status = Status::kSuccess;
    result.path = std::move(path);
    result.cost = polyline_length(result.path);
    return result;
  }

 private:
  bool informed_;
};

}  // namespace

bool ompl_rrtstar_available() { return true; }

bool ompl_informed_rrtstar_available() { return true; }

std::unique_ptr<Planner> make_ompl_rrtstar() {
  return std::make_unique<OmplPlanner>(false);
}

std::unique_ptr<Planner> make_ompl_informed_rrtstar() {
  return std::make_unique<OmplPlanner>(true);
}

}  // namespace planner::sampling_search

#endif  // PLANNER_SAMPLING_SEARCH_HAS_OMPL
