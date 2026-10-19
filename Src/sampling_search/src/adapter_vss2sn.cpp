// vss2sn/path_planning 栅格 RRT* 适配器（X-S4，栅格采样组）。
//
// 上游：commit d8f9edaeae064ab4665350bf820d1d5ebbbfcc45（BSD-3-Clause）。
// 使用最小抽取副本（third_party/vss2sn/）：rrt_star + planner + utils。
//
// 栅格采样组语义（候选登记 §4.2：状态离散，与连续空间组分开评分）：
// - 统一 Problem 的连续边界/起终点/点边回调映射为格点栅格；
// - 分辨率 = Problem::edge_check_step（格采样组的统一沿边步长即栅格分辨率），
//   要求边界为分辨率的整数倍且宽高相等（vss2sn 栅格为方阵 n×n）；
// - 格点 (i, j) 的世界坐标为 (min_x + i·res, min_y + j·res)；受阻格点覆盖
//   其封闭方格 [x−res/2, x+res/2] × [y−res/2, y+res/2]（vss2sn 原生障碍语义），
//   保守栅格化（9 点 + 边界边采样）保证障碍 ⊆ 受阻方格并集，从而"原生接受
//   ⇒ 统一沿边检查通过"；返回路径仍经统一复检，失败按无解处理。
//
// 适配层补齐的缺口（上游补丁，见 third_party/vss2sn/PATCHES.md）：
// 1. 种子：GenerateRandomNode 原为逐次 random_device，改为成员 mt19937 + SetSeed；
// 2. 预算/取消：Plan 主循环加入 stop predicate（墙钟预算/取消标志）；
// 3. 首次解时间：CheckGoalVisible 首次成功时回调（原生无中间解观测点）；
// 4. 迭代计数：记录尝试次数。
// 最近邻/重连/障碍判定等算法原语未改。
//
// 原生限制（登记于 provenance.md）：
// - 只支持方阵栅格；非方阵边界返回 kInvalidInput；
// - 起终点按最近格点吸附；对齐输入（测试口径）端点精确；
// - 预算耗尽无解报 kTimeout（不证明不可达）。

#include "planner/sampling_search/planner.hpp"

#if defined(PLANNER_SAMPLING_SEARCH_HAS_VSS2SN)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// 上游（vendored，BSD-3-Clause）头文件；include 路径由 CMake 指向
// third_party/vss2sn/include。
#include "path_planning/rrt_star.hpp"

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

// 格点 (i, j) 的封闭方格与障碍相交的保守测试（方格以格点为中心）。
bool lattice_point_blocked(const Problem& problem, int i, int j, double res,
                           std::uint64_t* checks) {
  const double cx = problem.bounds.min_x + static_cast<double>(i) * res;
  const double cy = problem.bounds.min_y + static_cast<double>(j) * res;
  const double half = 0.5 * res;
  for (double dx : {-half, 0.0, half}) {
    for (double dy : {-half, 0.0, half}) {
      ++(*checks);
      if (!problem.point_valid(Point2d{cx + dx, cy + dy})) {
        return true;
      }
    }
  }
  const Point2d corners[4] = {{cx - half, cy - half},
                              {cx + half, cy - half},
                              {cx + half, cy + half},
                              {cx - half, cy + half}};
  for (int k = 0; k < 4; ++k) {
    ++(*checks);
    if (!problem.edge_valid(corners[k], corners[(k + 1) % 4])) {
      return true;
    }
  }
  return false;
}

class Vss2snRrtStar : public Planner {
 public:
  const char* name() const override { return kBackendVss2sn; }

  Result plan(const Problem& problem, CancelToken* cancel) override {
    const std::string invalid = validate_problem(problem);
    if (!invalid.empty()) {
      return invalid_result(invalid);
    }
    if (is_trivial_problem(problem)) {
      return trivial_result(problem, kBackendVss2sn, kVariantRrtStar);
    }
    const double res = problem.edge_check_step;
    const double width = problem.bounds.width();
    const double height = problem.bounds.height();
    const int nx = static_cast<int>(std::llround(width / res)) + 1;
    const int ny = static_cast<int>(std::llround(height / res)) + 1;
    if (std::fabs(width - static_cast<double>(nx - 1) * res) > 1e-9 ||
        std::fabs(height - static_cast<double>(ny - 1) * res) > 1e-9) {
      return invalid_result(
          "vss2sn: bounds must be integral multiples of edge_check_step");
    }
    if (nx != ny) {
      return invalid_result(
          "vss2sn: native grid is square (n x n); bounds must be square");
    }
    if (nx < 2) {
      return invalid_result("vss2sn: grid must have at least 2 lattice points");
    }

    Result result;
    result.stats.backend = kBackendVss2sn;
    result.stats.variant = kVariantRrtStar;
    result.stats.budget_s = problem.budget_s;

    // 保守栅格化：grid[x][y]（vss2sn 的第一维为 x）。
    std::vector<std::vector<int>> grid(
        nx, std::vector<int>(ny, 0));
    std::uint64_t raster_checks = 0;
    for (int x = 0; x < nx; ++x) {
      for (int y = 0; y < ny; ++y) {
        if (lattice_point_blocked(problem, x, y, res, &raster_checks)) {
          grid[x][y] = 1;
        }
      }
    }
    const auto to_lattice = [&](const Point2d& p) {
      return Node(static_cast<int>(std::llround((p.x_m - problem.bounds.min_x) / res)),
                  static_cast<int>(std::llround((p.y_m - problem.bounds.min_y) / res)),
                  0, 0, 0, 0);
    };
    Node start_node = to_lattice(problem.start);
    Node goal_node = to_lattice(problem.goal);
    if (start_node.x_ < 0 || start_node.x_ >= nx || start_node.y_ < 0 ||
        start_node.y_ >= ny || goal_node.x_ < 0 || goal_node.x_ >= nx ||
        goal_node.y_ < 0 || goal_node.y_ >= ny) {
      return invalid_result("vss2sn: start/goal snap outside the grid");
    }
    start_node.id_ = nx * start_node.x_ + start_node.y_;
    start_node.pid_ = start_node.id_;
    goal_node.id_ = nx * goal_node.x_ + goal_node.y_;
    goal_node.pid_ = goal_node.id_;
    // 起点/终点格点若被保守栅格化判为受阻，则返回无效输入（与原生语义一致：
    // 障碍格点不能作为起点）。
    if (grid[start_node.x_][start_node.y_] != 0 ||
        grid[goal_node.x_][goal_node.y_] != 0) {
      return invalid_result(
          "vss2sn: start/goal lattice point blocked by conservative "
          "rasterization (move start/goal farther than half a lattice cell "
          "from obstacles)");
    }

    RRTStar rrt(grid);
    rrt.SetParams(2, 500);  // threshold=2 格；500×n² 上限（构造默认），预算先行
    rrt.SetSeed(static_cast<unsigned>(problem.seed & 0xffffffffu));

    const auto t0 = Clock::now();
    bool cancelled = false;
    double first_solution = -1.0;
    rrt.SetFirstSolutionCallback([&]() {
      if (first_solution < 0.0) {
        first_solution = seconds_since(t0);
      }
    });
    rrt.SetStopPredicate([&]() {
      if (cancel != nullptr && cancel->cancelled()) {
        cancelled = true;
        return true;
      }
      return seconds_since(t0) >= problem.budget_s;
    });

    const auto [found, native_path] = rrt.Plan(start_node, goal_node);

    result.stats.iterations = static_cast<std::uint64_t>(rrt.GetIterations());
    result.stats.first_solution_s = first_solution;
    result.stats.wall_s = seconds_since(t0);
    result.stats.native_params =
        "resolution=" + format_double(res) + ";grid=" + std::to_string(nx) +
        "x" + std::to_string(ny) + ";threshold=2lattice;max_iter_x_factor=500;"
        "raster=conservative(lattice-square 9pt+boundary);"
        "patches=seed+stop_predicate+first_solution+iterations;"
        "edge_checks=not instrumented natively";

    if (cancelled) {
      result.status = Status::kCancelled;
      result.message = "cancelled; partial result discarded";
      return result;
    }
    if (!found) {
      result.status = Status::kTimeout;
      result.message =
          rrt.GetStoppedEarly()
              ? "budget exhausted without a solution (not an unreachability "
                "proof)"
              : "native iteration cap exhausted without a solution (not an "
                "unreachability proof)";
      return result;
    }

    // 原生路径为 goal→…→start 顺序；转换为世界坐标后反转为 start→goal。
    std::vector<Point2d> path;
    path.reserve(native_path.size() + 2);
    for (auto it = native_path.rbegin(); it != native_path.rend(); ++it) {
      path.push_back(Point2d{problem.bounds.min_x + static_cast<double>(it->x_) * res,
                             problem.bounds.min_y + static_cast<double>(it->y_) * res});
    }
    // 非对齐输入：吸附端点后补接精确起终点（统一边有效时）。
    if ((path.front().x_m != problem.start.x_m || path.front().y_m != problem.start.y_m) &&
        problem.edge_valid(problem.start, path.front())) {
      path.insert(path.begin(), problem.start);
    }
    if ((path.back().x_m != problem.goal.x_m || path.back().y_m != problem.goal.y_m) &&
        problem.edge_valid(path.back(), problem.goal)) {
      path.push_back(problem.goal);
    }

    if (!path_edges_valid(problem, path)) {
      result.status = Status::kTimeout;
      result.message =
          "native solution rejected by unified edge re-check (conservative "
          "rasterization gap; see provenance.md)";
      return result;
    }

    result.status = Status::kSuccess;
    result.path = std::move(path);
    result.cost = polyline_length(result.path);
    return result;
  }
};

}  // namespace

bool vss2sn_rrtstar_available() { return true; }

std::unique_ptr<Planner> make_vss2sn_rrtstar() {
  return std::make_unique<Vss2snRrtStar>();
}

}  // namespace planner::sampling_search

#endif  // PLANNER_SAMPLING_SEARCH_HAS_VSS2SN
