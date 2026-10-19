// planner_grid_search 独立示例（X-S3）：在合成夹具上运行全部已启用后端，
// 打印状态/代价/展开节点，并用统一契约做自检（--self-check）。
//
// 构建（独立模式）：
//   cmake -S Src/grid_search -B <build-dir> && cmake --build <build-dir> -j4
//   <build-dir>/planner_grid_search_demo --self-check

#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "planner/grid_search/backend.hpp"
#include "planner/grid_search/distance_field.hpp"
#include "planner/grid_search/types.hpp"

namespace {

using planner::common::kCellFree;
using planner::common::kCellOccupied;
using planner::grid_search::GridCell;
using planner::grid_search::GridSearchRequest;
using planner::grid_search::GridSearchResult;
using planner::grid_search::Neighborhood;

planner::common::GridView make_demo_grid() {
  planner::common::GridView grid;
  grid.map_id = "demo";
  grid.map_version = "unit";
  grid.width = 24;
  grid.height = 24;
  grid.resolution_m = 0.5;
  grid.cells.assign(24 * 24, kCellFree);
  // 外围墙 + 中间横墙（缺口 3 格）构成必须绕行的通道。
  for (int index = 0; index < 24; ++index) {
    grid.cells[index] = kCellOccupied;                          // row 0
    grid.cells[23 * 24 + index] = kCellOccupied;                // row 23
    grid.cells[index * 24] = kCellOccupied;                     // col 0
    grid.cells[index * 24 + 23] = kCellOccupied;                // col 23
  }
  for (int col = 1; col <= 22; ++col) {
    if (col < 11 || col > 13) {
      grid.cells[12 * 24 + col] = kCellOccupied;
    }
  }
  return grid;
}

}  // namespace

int main(int argc, char** argv) {
  bool self_check = false;
  for (int index = 1; index < argc; ++index) {
    if (std::strcmp(argv[index], "--self-check") == 0) {
      self_check = true;
    }
  }

  const planner::common::GridView grid = make_demo_grid();
  GridSearchRequest request;
  request.grid = &grid;
  request.start = GridCell{2, 2};
  request.goal = GridCell{21, 21};

  const std::vector<std::string> backends =
      planner::grid_search::available_backends();
  std::cout << "grid_search demo: " << grid.width << "x" << grid.height
            << " resolution=" << grid.resolution_m << " m; backends="
            << backends.size() << "\n";
  std::cout << std::setw(24) << std::left << "backend" << std::setw(8)
            << "neigh" << std::setw(12) << "status" << std::setw(12) << "cost_m"
            << std::setw(10) << "expanded"
            << "path_cells\n";

  bool ok = true;
  double reference_cost_four = -1.0;
  double reference_cost_eight = -1.0;
  for (const std::string& name : backends) {
    std::unique_ptr<planner::grid_search::GridSearchBackend> backend =
        planner::grid_search::make_backend(name);
    if (backend == nullptr) {
      std::cerr << "cannot create backend " << name << "\n";
      ok = false;
      continue;
    }
    for (Neighborhood neighborhood :
         {Neighborhood::kFour, Neighborhood::kEight}) {
      request.neighborhood = neighborhood;
      const GridSearchResult result = backend->search(request);
      std::cout << std::setw(24) << std::left << name << std::setw(8)
                << planner::grid_search::to_string(neighborhood) << std::setw(12)
                << planner::grid_search::status_name(result.status)
                << std::setw(12) << std::fixed << std::setprecision(4)
                << result.cost_m << std::setw(10)
                << result.stats.expanded_nodes << result.path.size() << "\n";
      // JPS 单列变体保持原生穿角语义，路径校验不强制禁穿角规则，
      // 也不参与统一代价一致性；四邻域对照组不适用（原生仅八邻域）。
      const bool jps = name == "jps3d_jps";
      if (jps && neighborhood == Neighborhood::kFour) {
        if (result.status != planner::common::Status::kInvalidInput) {
          std::cerr << "expected invalid_input for the JPS four-neighborhood "
                       "control group\n";
          ok = false;
        }
        continue;
      }
      if (result.status != planner::common::Status::kSuccess) {
        ok = false;
        continue;
      }
      const std::string problem = planner::grid_search::validate_path(
          grid, request, result.path, /*enforce_corner_rule=*/!jps);
      if (!problem.empty()) {
        std::cerr << "invalid path from " << name << ": " << problem << "\n";
        ok = false;
      }
      if (!jps) {
        double& reference = neighborhood == Neighborhood::kFour
                                ? reference_cost_four
                                : reference_cost_eight;
        if (reference < 0.0) {
          reference = result.cost_m;
        } else if (!planner::grid_search::costs_match(result.cost_m,
                                                      reference)) {
          std::cerr << "cost mismatch: " << name << "=" << result.cost_m
                    << " vs " << reference << "\n";
          ok = false;
        }
      }
    }
  }

  for (const std::string& name : backends) {
    std::unique_ptr<planner::grid_search::GridSearchBackend> backend =
        planner::grid_search::make_backend(name);
    if (backend->supports_distance_field()) {
      planner::grid_search::DistanceFieldRequest field_request;
      field_request.grid = &grid;
      field_request.source = request.start;
      field_request.neighborhood = Neighborhood::kEight;
      const auto field = backend->distance_field(field_request);
      std::cout << "distance field (" << name << "): status="
                << planner::grid_search::status_name(field.status)
                << " unreachable=" << field.field.unreachable_count()
                << " max=" << field.field.max_reachable_m() << " m\n";
    }
  }

  if (self_check) {
    if (!ok) {
      std::cerr << "self-check FAILED\n";
      return 1;
    }
    std::cout << "self-check OK\n";
  }
  return ok ? 0 : 1;
}
