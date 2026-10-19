// jps3d（BSD-3-Clause，commit ef65e36a）栅格 A* 适配器。
//
// 普通 A*（use_jps=false）进入统一主组：适配补丁 P1（getSucc 禁对角穿角）、
// P2（unknown 预转换为 occupied）、P4（四邻域选择）；`eps` 固定 1（可采纳，
// 见 provenance.md）。JPS 为单列变体后端：保持原生八邻域与穿角语义，
// 不混入统一排名（计划 §4.1）。世界<->单元映射按探针 P4 结论直接构造单元
// 中心坐标（(cell + 0.5) * res），绕过上游 floatToInt 的 -0.5 约定偏差。

#include <cmath>
#include <string>
#include <vector>

#include "jps_collision/map_util.h"
#include "jps_planner/jps_planner/jps_planner.h"

#include "planner/grid_search/adapters/adapters.hpp"

#include "adapter_util.hpp"

namespace planner::grid_search {
namespace {

class Jps3dBackend final : public GridSearchBackend {
 public:
  Jps3dBackend(bool use_jps, std::string name)
      : use_jps_(use_jps), name_(std::move(name)) {}

  std::string name() const override { return name_; }

  GridSearchResult search(const GridSearchRequest& request) override {
    std::string reason;
    const common::Status status = detail::precheck(request, &reason);
    if (status != common::Status::kSuccess) {
      return detail::make_failure(status, reason);
    }
    if (use_jps_ && request.neighborhood != Neighborhood::kEight) {
      return detail::make_failure(
          common::Status::kInvalidInput,
          "jps3d JPS keeps its native 8-neighborhood; the four-neighborhood "
          "control group is not available for the JPS variant");
    }

    const int width = request.grid->width;
    const int height = request.grid->height;

    // GridView 三态 -> Tmap（free 0 / occupied 1）；unknown 预转换为 occupied
    // （上游 updateMap 把 unknown(-1) 当 free，补丁 P2）。
    JPS::Tmap tmap(static_cast<std::size_t>(width) *
                       static_cast<std::size_t>(height),
                   static_cast<signed char>(1));
    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        const std::size_t index = static_cast<std::size_t>(row) *
                                      static_cast<std::size_t>(width) +
                                  static_cast<std::size_t>(col);
        tmap[index] = request.grid->cells[index] == common::kCellFree
                          ? static_cast<signed char>(0)
                          : static_cast<signed char>(1);
      }
    }

    auto map_util = std::make_shared<JPS::OccMapUtil>();
    map_util->setMap(Vec2f(0.0, 0.0), Vec2i(width, height), tmap,
                     1.0);
    JPSPlanner2D planner(false);
    planner.setMapUtil(map_util);
    planner.updateMap();
    if (!use_jps_) {
      planner.setFourConnect(request.neighborhood == Neighborhood::kFour);
    }

    detail::BudgetTracker tracker(request.time_budget_s, request.cancelled);
    planner.setTerminationCheck([&tracker]() { return tracker.should_stop(); });

    const bool found = planner.plan(
        Vec2f(static_cast<double>(request.start.col) + 0.5,
                   static_cast<double>(request.start.row) + 0.5),
        Vec2f(static_cast<double>(request.goal.col) + 0.5,
                   static_cast<double>(request.goal.row) + 0.5),
        1.0 /*eps 固定 1：可采纳，不得宣称最短路时使用其他值*/,
        use_jps_);

    GridSearchStats stats;
    stats.upstream_status = "status_" + std::to_string(planner.status());
    stats.expanded_nodes =
        static_cast<std::int64_t>(planner.getCloseSet().size());
    stats.generated_nodes =
        static_cast<std::int64_t>(planner.getAllSet().size());
    stats.iterations = stats.expanded_nodes;
    stats.upstream_cost = std::numeric_limits<double>::quiet_NaN();

    if (!found) {
      if (tracker.should_stop()) {
        return detail::make_failure(tracker.stop_status(),
                                    tracker.stop_reason(), stats);
      }
      return detail::make_failure(
          common::Status::kNoPath,
          "no path found by jps3d " +
              std::string(use_jps_ ? "JPS" : "A*") +
              " (native status " + std::to_string(planner.status()) + ")",
          stats);
    }

    // 统一输出使用 getRawPath()（搜索原始路径，代价与搜索一致）；
    // getPath() 的 removeCornerPts/removeLinePts 后处理不纳入统一口径。
    // JPS 返回跳点路径（相邻点之间是直线段，正交或对角）；统一输出展开为
    // 单位步，使路径口径与统一契约一致（长度不变）。JPS 单列变体保持原生
    // 允许对角穿角语义，路径校验不强制禁穿角规则。
    const vec_Vec2f raw_path = planner.getRawPath();
    std::vector<GridCell> path;
    const auto to_cell = [](const Vec2f& point) {
      return GridCell{static_cast<int>(std::lround(point(1) - 0.5)),
                      static_cast<int>(std::lround(point(0) - 0.5))};
    };
    if (!raw_path.empty()) {
      path.push_back(to_cell(raw_path.front()));
      for (std::size_t index = 1; index < raw_path.size(); ++index) {
        const GridCell from = path.back();
        const GridCell to = to_cell(raw_path[index]);
        const int d_row = to.row - from.row;
        const int d_col = to.col - from.col;
        const int abs_row = d_row < 0 ? -d_row : d_row;
        const int abs_col = d_col < 0 ? -d_col : d_col;
        const int steps = abs_row > abs_col ? abs_row : abs_col;
        const bool straight =
            d_row == 0 || d_col == 0 || abs_row == abs_col;
        if (!straight || steps == 0) {
          // 不应出现：保留原样，由统一路径校验暴露异常。
          path.push_back(to);
          continue;
        }
        const int step_row = (d_row > 0) ? 1 : ((d_row < 0) ? -1 : 0);
        const int step_col = (d_col > 0) ? 1 : ((d_col < 0) ? -1 : 0);
        for (int step = 1; step <= steps; ++step) {
          path.push_back(GridCell{from.row + step_row * step,
                                  from.col + step_col * step});
        }
      }
    }
    return detail::make_path_result(request, std::move(path), stats,
                                    /*enforce_corner_rule=*/!use_jps_);
  }

 private:
  bool use_jps_{false};
  std::string name_;
};

}  // namespace

std::unique_ptr<GridSearchBackend> make_jps3d_astar_backend() {
  return std::make_unique<Jps3dBackend>(false, "jps3d_astar");
}

std::unique_ptr<GridSearchBackend> make_jps3d_jps_backend() {
  return std::make_unique<Jps3dBackend>(true, "jps3d_jps");
}

}  // namespace planner::grid_search
