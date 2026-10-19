// 测试夹具加载实现。

#include "scenarios.hpp"

#include <fstream>
#include <limits>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "planner/grid_search/distance_field.hpp"

namespace planner::grid_search::test {
namespace {

nlohmann::json read_json(const std::string& path) {
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("cannot open fixture: " + path);
  }
  nlohmann::json document;
  stream >> document;
  return document;
}

double json_distance(const nlohmann::json& value) {
  if (value.is_null()) {
    return kUnreachableDistance;
  }
  return value.get<double>();
}

ExpectedNeighborhood parse_neighborhood(const nlohmann::json& node) {
  ExpectedNeighborhood expected;
  expected.reachable = node.at("reachable").get<bool>();
  if (expected.reachable) {
    expected.cost_m = node.at("cost_m").get<double>();
  }
  // 单列条目（eight_corner_cut_allowed）只带 reachable/cost_m。
  if (node.contains("unreachable_cells")) {
    expected.unreachable_cells =
        node.at("unreachable_cells").get<std::int64_t>();
  }
  if (node.contains("max_distance_m")) {
    expected.max_distance_m = node.at("max_distance_m").get<double>();
  }
  if (node.contains("distance_field_m")) {
    for (const auto& row : node.at("distance_field_m")) {
      for (const auto& value : row) {
        expected.distance_field_m.push_back(json_distance(value));
      }
    }
  }
  return expected;
}

}  // namespace

std::string fixture_dir() { return PLANNER_GRID_SEARCH_FIXTURE_DIR; }

std::vector<Scenario> load_scenarios() {
  const nlohmann::json document =
      read_json(fixture_dir() + "/scenarios.json");
  std::vector<Scenario> scenarios;
  for (const auto& item : document.at("scenarios")) {
    Scenario scenario;
    scenario.id = item.at("id").get<std::string>();
    scenario.grid.map_id = scenario.id;
    scenario.grid.map_version = "grid_search_scenarios_v1";
    scenario.grid.width = item.at("width").get<int>();
    scenario.grid.height = item.at("height").get<int>();
    scenario.grid.resolution_m = item.at("resolution_m").get<double>();
    scenario.grid.cells.reserve(static_cast<std::size_t>(scenario.grid.width) *
                                static_cast<std::size_t>(scenario.grid.height));
    for (const auto& row : item.at("cells")) {
      for (char cell : row.get<std::string>()) {
        switch (cell) {
          case '.':
            scenario.grid.cells.push_back(common::kCellFree);
            break;
          case '#':
            scenario.grid.cells.push_back(common::kCellOccupied);
            break;
          case '?':
            scenario.grid.cells.push_back(common::kCellUnknown);
            break;
          default:
            throw std::runtime_error("invalid fixture cell: " +
                                     std::string(1, cell));
        }
      }
    }
    scenario.start.row = item.at("start").at(0).get<int>();
    scenario.start.col = item.at("start").at(1).get<int>();
    scenario.goal.row = item.at("goal").at(0).get<int>();
    scenario.goal.col = item.at("goal").at(1).get<int>();
    scenario.distance_source.row =
        item.at("distance_source").at(0).get<int>();
    scenario.distance_source.col =
        item.at("distance_source").at(1).get<int>();
    scenarios.push_back(std::move(scenario));
  }
  return scenarios;
}

std::map<std::string, ExpectedScenario> load_expected() {
  const nlohmann::json document =
      read_json(fixture_dir() + "/expected_costs.json");
  std::map<std::string, ExpectedScenario> expected;
  for (auto iterator = document.at("scenarios").begin();
       iterator != document.at("scenarios").end(); ++iterator) {
    ExpectedScenario scenario;
    scenario.id = iterator.key();
    scenario.four = parse_neighborhood(iterator.value().at("neighborhoods").at("four"));
    scenario.eight =
        parse_neighborhood(iterator.value().at("neighborhoods").at("eight"));
    scenario.eight_corner_cut_allowed = parse_neighborhood(
        iterator.value().at("neighborhoods").at("eight_corner_cut_allowed"));
    expected.emplace(scenario.id, std::move(scenario));
  }
  return expected;
}

const ExpectedNeighborhood& expected_for(const ExpectedScenario& scenario,
                                         Neighborhood neighborhood,
                                         bool corner_cut_allowed) {
  if (neighborhood == Neighborhood::kFour) {
    return scenario.four;
  }
  return corner_cut_allowed ? scenario.eight_corner_cut_allowed
                            : scenario.eight;
}

GridSearchRequest make_request(const Scenario& scenario,
                               Neighborhood neighborhood) {
  GridSearchRequest request;
  request.grid = &scenario.grid;
  request.start = scenario.start;
  request.goal = scenario.goal;
  request.neighborhood = neighborhood;
  return request;
}

std::string context_of(const std::string& backend, const Scenario& scenario,
                       Neighborhood neighborhood) {
  return "backend=" + backend + " scenario=" + scenario.id +
         " neighborhood=" + to_string(neighborhood);
}

}  // namespace planner::grid_search::test
