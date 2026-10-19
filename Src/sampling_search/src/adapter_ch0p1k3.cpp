// Ch0p1k3/path-planning-algorithms-rrt-rrtstar 适配器（X-S4）。
//
// 上游：commit 4ae8c935f985097e50976e75220c27dac57ffa61（MIT）。
// 仓库含约 2.9 GB tests/ 数据，无法整仓 FetchContent；本目录使用探针验证过的
// 最小抽取副本（third_party/ch0p1k3/），仅算法核心 7 个 .cpp + 头文件。
//
// 适配面（对照探针报告 supplement_ch0p1k3_probe.md §6）：
// 1. 种子：上游 rrt.hpp 上 4 行种子钩子（setSeed；探针 seed_hook.patch 正式化，
//    见 third_party/ch0p1k3/PATCHES.md）；采样函数本身未改。
// 2. 预算/取消：驱动层镜像上游 RRTStarAlgorithm::launch 循环（process.cpp
//    692-810），每次尝试前检查 steady_clock 预算与取消标志；采样、最近邻、
//    steer、近邻、选父、重连、插入、改边全部调用上游原语，未重写核心。
// 3. 障碍映射：统一夹具 → 保守栅格化（单元封闭矩形与障碍相交即受阻）；
//    起点/终点所在单元强制解除（避免上游 Obstacle 构造对边界点的 exit()），
//    返回路径仍经统一沿边回调复检，复检失败按无解处理（记录消息）。
// 4. 目标域语义：上游在 eps 距离内视为到达；适配器在统一边有效时补接精确
//    终点，原生末点间距记录在消息/统计中。
// 5. 原生参数：stepsize = max(3 × edge_check_step, 1)、eps = max(edge_check_step, 1)、
//    gamma = 100（探针口径）；上游对 <1 的值会自行改写，故先夹取。
//
// 已知上游风险（不修补，登记于 provenance.md）：
// - Tree::used 去重表为死代码（tree.hpp/tree.cpp 从未写入）；
// - obstacleFree 半径搜索 len²+1 存在理论漏检窗口；
// - 上游 XML/Map/Algorithm 构造在输入错误时 exit()，故适配器先做完整预检。
//
// 本适配器属栅格采样组？否——Ch0p1k3 状态全程连续（探针 Gate 2），与 OMPL/
// Klamp't 同属连续空间组；栅格仅用于障碍表达。

#include "planner/sampling_search/planner.hpp"

#if defined(PLANNER_SAMPLING_SEARCH_HAS_CH0P1K3)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

// 上游（vendored，MIT）头文件；include 路径由 CMake 指向
// third_party/ch0p1k3/src。
#include <algorithm.hpp>
#include <constants.hpp>
#include <geometry.hpp>
#include <map.hpp>
#include <obstacle.hpp>
#include <rrt.hpp>
#include <rrtstar.hpp>
#include <tree.hpp>

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

// 单元 (j, i)（j 为 x 方向索引）的封闭矩形是否与障碍相交的保守测试：
// 9 个格点采样（角点/边中点/中心）+ 4 条边界边的统一沿边检查。
// 残余缺口：完全位于单元内部且不触碰任何采样点的细障碍可能被漏掉；
// 夹具自身的沿边采样检查具有同样的分辨率限制（见 provenance.md）。
bool cell_blocked(const Problem& problem, int j, int i,
                  std::uint64_t* checks) {
  const double x0 = problem.bounds.min_x + static_cast<double>(j);
  const double y0 = problem.bounds.min_y + static_cast<double>(i);
  const double x1 = x0 + 1.0;
  const double y1 = y0 + 1.0;
  for (double dx : {-0.5, 0.0, 0.5}) {
    for (double dy : {-0.5, 0.0, 0.5}) {
      ++(*checks);
      if (!problem.point_valid(Point2d{x0 + 0.5 + dx, y0 + 0.5 + dy})) {
        return true;
      }
    }
  }
  const Point2d corners[4] = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
  for (int k = 0; k < 4; ++k) {
    ++(*checks);
    if (!problem.edge_valid(corners[k], corners[(k + 1) % 4])) {
      return true;
    }
  }
  return false;
}

struct Grid {
  int width{0};
  int height{0};
  std::vector<std::vector<int>> cells;  // cells[i][j]，i 为行（y），j 为列（x）
  std::uint64_t checks{0};
};

// 保守栅格化：单元封闭矩形与障碍相交 ⇒ 受阻（1），否则自由（0）。
// 起点/终点所在（封闭矩形包含该点的）单元强制解除，避免上游 Obstacle
// 构造对落在受阻单元边界上的点调用 exit()。
Grid rasterize(const Problem& problem, std::uint64_t* checks) {
  Grid grid;
  grid.width = static_cast<int>(
      std::llround(problem.bounds.width()));  // 预检保证整数宽高
  grid.height = static_cast<int>(std::llround(problem.bounds.height()));
  grid.cells.assign(grid.height, std::vector<int>(grid.width, 0));
  for (int i = 0; i < grid.height; ++i) {
    for (int j = 0; j < grid.width; ++j) {
      if (cell_blocked(problem, j, i, checks)) {
        grid.cells[i][j] = 1;
      }
    }
  }
  // 强制解除包含起点/终点的单元（闭区间包含，最多 4 个单元）。
  for (const Point2d& p : {problem.start, problem.goal}) {
    const int j0 = static_cast<int>(std::floor(p.x_m - problem.bounds.min_x));
    const int i0 = static_cast<int>(std::floor(p.y_m - problem.bounds.min_y));
    for (int dj = -1; dj <= 0; ++dj) {
      for (int di = -1; di <= 0; ++di) {
        const int j = j0 + dj;
        const int i = i0 + di;
        if (j < 0 || j >= grid.width || i < 0 || i >= grid.height) {
          continue;
        }
        const double x0 = problem.bounds.min_x + j;
        const double y0 = problem.bounds.min_y + i;
        if (p.x_m >= x0 && p.x_m <= x0 + 1.0 && p.y_m >= y0 && p.y_m <= y0 + 1.0) {
          grid.cells[i][j] = 0;
        }
      }
    }
  }
  return grid;
}

// 生成上游 XML（内存字符串）；坐标平移使栅格原点为 (0,0)。
std::string build_xml(const Problem& problem, const Grid& grid, double step,
                      double eps, double gamma) {
  std::ostringstream out;
  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<root>\n  <map>\n";
  out << "    <height>" << grid.height << "</height>\n";
  out << "    <width>" << grid.width << "</width>\n";
  out.precision(17);
  out << "    <startx>" << (problem.start.x_m - problem.bounds.min_x)
      << "</startx>\n";
  out << "    <starty>" << (problem.start.y_m - problem.bounds.min_y)
      << "</starty>\n";
  out << "    <finishx>" << (problem.goal.x_m - problem.bounds.min_x)
      << "</finishx>\n";
  out << "    <finishy>" << (problem.goal.y_m - problem.bounds.min_y)
      << "</finishy>\n";
  out << "    <grid>\n";
  for (int i = 0; i < grid.height; ++i) {
    out << "      <row>";
    for (int j = 0; j < grid.width; ++j) {
      if (j) {
        out << ' ';
      }
      out << grid.cells[i][j];
    }
    out << "</row>\n";
  }
  out << "    </grid>\n  </map>\n  <algorithm>\n";
  out << "    <searchtype>rrtstar</searchtype>\n";
  out << "    <numberofiterations>1000000000</numberofiterations>\n";
  out << "    <stepsize>" << step << "</stepsize>\n";
  out << "    <eps>" << eps << "</eps>\n";
  out << "    <gamma>" << gamma << "</gamma>\n";
  out << "  </algorithm>\n</root>\n";
  return out.str();
}

// 把 XML 写入临时文件（上游 Map/Algorithm 只接受文件路径）。
class TempXmlFile {
 public:
  explicit TempXmlFile(const std::string& contents) {
    char tmpl[] = "/tmp/sampling_search_ch0p1k3_XXXXXX";
    const int fd = ::mkstemp(tmpl);
    if (fd >= 0) {
      path_ = tmpl;
      std::size_t written = 0;
      while (written < contents.size()) {
        const ssize_t n =
            ::write(fd, contents.data() + written, contents.size() - written);
        if (n <= 0) {
          break;
        }
        written += static_cast<std::size_t>(n);
      }
      ::close(fd);
      ok_ = written == contents.size();
    }
  }
  ~TempXmlFile() {
    if (!path_.empty()) {
      ::remove(path_.c_str());
    }
  }
  TempXmlFile(const TempXmlFile&) = delete;
  TempXmlFile& operator=(const TempXmlFile&) = delete;

  bool ok() const { return ok_; }
  const char* path() const { return path_.c_str(); }

 private:
  std::string path_;
  bool ok_{false};
};

class Ch0p1k3RrtStar : public Planner {
 public:
  const char* name() const override { return kBackendCh0p1k3; }

  Result plan(const Problem& problem, CancelToken* cancel) override {
    const std::string invalid = validate_problem(problem);
    if (!invalid.empty()) {
      return invalid_result(invalid);
    }
    if (is_trivial_problem(problem)) {
      return trivial_result(problem, kBackendCh0p1k3, kVariantRrtStar);
    }
    // Ch0p1k3 需要整数边界（Map width/height 为 size_t）。
    const double width = problem.bounds.width();
    const double height = problem.bounds.height();
    if (std::fabs(width - std::llround(width)) > 1e-9 ||
        std::fabs(height - std::llround(height)) > 1e-9) {
      return invalid_result(
          "ch0p1k3 requires integral bounds (width/height in whole units)");
    }
    if (std::llround(width) < 1 || std::llround(height) < 1) {
      return invalid_result("ch0p1k3 requires bounds of at least 1x1 units");
    }

    Result result;
    result.stats.backend = kBackendCh0p1k3;
    result.stats.variant = kVariantRrtStar;
    result.stats.budget_s = problem.budget_s;

    // 原生参数（探针口径；上游对 <1 的 stepsize/eps 会自行改写，先夹取）。
    const double step = std::max(3.0 * problem.edge_check_step, 1.0);
    const double eps = std::max(problem.edge_check_step, 1.0);
    const double gamma = 100.0;

    std::uint64_t raster_checks = 0;
    const Grid grid = rasterize(problem, &raster_checks);
    const std::string xml = build_xml(problem, grid, step, eps, gamma);
    TempXmlFile xml_file(xml);
    if (!xml_file.ok()) {
      return invalid_result("ch0p1k3: cannot write temporary scenario XML");
    }

    // 上游构造器在解析失败/起点终点在障碍上时 exit()；上面的栅格化已保证
    // 起点/终点不在受阻单元内，XML 由本适配器生成，故安全。
    Map map(xml_file.path());
    Algorithm algorithm(xml_file.path());
    RRTStar rrt(map, algorithm);
    rrt.setSeed(static_cast<unsigned>(problem.seed & 0xffffffffu));

    const Geometry::Point finish = rrt.getFinish();
    const double eps_sq = eps * eps;
    std::vector<Tree::Node*> goal_nodes;
    Tree::Node* best = nullptr;
    double first_solution = -1.0;
    double best_cost = std::numeric_limits<double>::infinity();
    std::uint64_t improvements = 0;
    std::uint64_t iterations = 0;
    std::uint64_t edge_checks = 0;
    bool cancelled = false;

    const auto t0 = Clock::now();
    for (;;) {
      if (cancel != nullptr && cancel->cancelled()) {
        cancelled = true;
        break;
      }
      if (seconds_since(t0) >= problem.budget_s) {
        break;
      }
      ++iterations;
      // 上游 RRTStarAlgorithm::launch 循环体（process.cpp:703-739），
      // 仅增加预算/取消检查与计数，算法原语全部来自上游。
      Geometry::Point x_rand = rrt.getRandomPoint();
      Tree::Node* x_nearest = rrt.getNearest(x_rand);
      Geometry::Point x_new = rrt.steer(x_nearest->point, x_rand);
      if (Geometry::euclideanMetric(x_nearest->point, x_new) < 0.01) {
        continue;
      }
      ++edge_checks;
      if (!rrt.obstacleFree(x_nearest->point, x_new)) {
        continue;
      }
      Tree::Node* x_min = x_nearest;
      std::vector<Tree::Node*> near;
      rrt.getNear(x_new, near);
      for (const auto& x : near) {
        if (Geometry::euclideanMetric(x->point, x_new) >= 0.01 &&
            rrt.obstacleFree(x->point, x_new) &&
            x->getDistance() + std::sqrt(Geometry::euclideanMetric(x->point, x_new)) <
                x_min->getDistance() +
                    std::sqrt(Geometry::euclideanMetric(x_min->point, x_new))) {
          x_min = x;
        }
      }
      Tree::Node* new_node = rrt.insertVertexAndEdge(x_min, x_new);
      if (new_node == nullptr) {
        continue;
      }
      for (const auto& x : near) {
        if (x == x_min) {
          continue;
        }
        if (Geometry::euclideanMetric(x->point, x_new) >= 0.01 &&
            rrt.obstacleFree(x->point, x_new) &&
            x->getDistance() >
                new_node->getDistance() +
                    std::sqrt(Geometry::euclideanMetric(x->point, new_node->point))) {
          rrt.changeEdge(x->parent, x, new_node);
        }
      }
      if (Geometry::euclideanMetric(new_node->point, finish) <= eps_sq) {
        goal_nodes.push_back(new_node);
        if (first_solution < 0.0) {
          first_solution = seconds_since(t0);
        }
        const double cost = new_node->getDistance();
        if (cost < best_cost) {
          if (std::isfinite(best_cost)) {
            ++improvements;
          }
          best_cost = cost;
          best = new_node;
        }
      }
    }
    // 预算末最优解：在所有目标域节点中取代价最小者。
    for (Tree::Node* node : goal_nodes) {
      if (best == nullptr || node->getDistance() < best->getDistance()) {
        best = node;
      }
    }

    result.stats.iterations = iterations;
    result.stats.edge_checks = edge_checks;
    result.stats.first_solution_s = first_solution;
    result.stats.improvements = improvements;
    result.stats.wall_s = seconds_since(t0);
    result.stats.native_params =
        "stepsize=" + format_double(step) + ";eps=" + format_double(eps) +
        ";gamma=" + format_double(gamma) + ";grid=" +
        std::to_string(grid.width) + "x" + std::to_string(grid.height) +
        ";raster=conservative(9pt+boundary);start_goal_cells_force_free=1;"
        "native_goal_domain=eps;seed_hook=4-line rrt.hpp patch";

    if (cancelled) {
      result.status = Status::kCancelled;
      result.message = "cancelled; partial result discarded";
      return result;
    }
    if (best == nullptr) {
      result.status = Status::kTimeout;
      result.message =
          "budget exhausted without a solution (not an unreachability proof)";
      return result;
    }

    std::vector<Geometry::Point> native;
    for (Tree::Node* node = best; node != nullptr; node = node->parent) {
      native.push_back(node->point);
    }
    std::reverse(native.begin(), native.end());
    const double native_gap = std::sqrt(Geometry::euclideanMetric(native.back(), finish));

    std::vector<Point2d> path;
    path.reserve(native.size() + 1);
    for (const Geometry::Point& p : native) {
      path.push_back(Point2d{p.x + problem.bounds.min_x, p.y + problem.bounds.min_y});
    }
    // 目标域语义：末点在 eps 内且统一边有效时补接精确终点。
    if (native_gap <= eps &&
        problem.edge_valid(path.back(), problem.goal)) {
      path.push_back(problem.goal);
    }

    // 统一复检：保守栅格化 + 强制解除起点/终点单元的残余风险在此拦截；
    // 复检失败按无解处理（丢弃原生结果，不返回无效边）。
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
    result.message = "native_goal_gap=" + format_double(native_gap);
    return result;
  }
};

}  // namespace

bool ch0p1k3_rrtstar_available() { return true; }

std::unique_ptr<Planner> make_ch0p1k3_rrtstar() {
  return std::make_unique<Ch0p1k3RrtStar>();
}

}  // namespace planner::sampling_search

#endif  // PLANNER_SAMPLING_SEARCH_HAS_CH0P1K3
