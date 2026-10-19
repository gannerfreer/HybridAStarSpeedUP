#pragma once

// 后端接口与注册表。
//
// 每个来源一个后端（vss2sn 提供 Dijkstra 与 A* 两个后端角色）；JPS 为 jps3d
// 的单列变体后端（不充当独立来源、不混入统一排名，见计划 §4.1）。
//
// 工厂名称（稳定标识符，供基准/报告引用）：
//   vss2sn_dijkstra、vss2sn_astar、smac_2d_astar、jps3d_astar、
//   jps3d_jps、libmultirobot_astar、justinhj_astar（内存池开）、
//   justinhj_astar_nopool（内存池关）。
//
// 未启用的适配器开关（PLANNER_GRID_SEARCH_WITH_*，默认 ON）不编译对应源码，
// available_backends() 不返回该名称。

#include <memory>
#include <string>
#include <vector>

#include "planner/grid_search/distance_field.hpp"
#include "planner/grid_search/types.hpp"

namespace planner::grid_search {

class GridSearchBackend {
 public:
  virtual ~GridSearchBackend() = default;

  // 稳定后端名称（与注册表工厂名称一致）。
  virtual std::string name() const = 0;

  // 是否支持距离场角色。上游无距离场 API 且未提供适配实现时返回 false。
  virtual bool supports_distance_field() const { return false; }

  // 点到点搜索。输入非法时返回 kInvalidInput（适配层必须自行校验，
  // 不依赖上游的错误语义）。
  virtual GridSearchResult search(const GridSearchRequest& request) = 0;

  // 距离场。默认实现返回 kInvalidInput（后端不支持）。
  virtual DistanceFieldResult distance_field(const DistanceFieldRequest& request);
};

// 已编译启用的后端名称（确定性排序）。
std::vector<std::string> available_backends();

// 创建后端；名称未知或未启用时返回 nullptr。
std::unique_ptr<GridSearchBackend> make_backend(const std::string& name);

}  // namespace planner::grid_search
