// X-S4 测试夹具：轻量二维矩形障碍点/边有效性（与探针 family_d 场景一致），
// 以及统一 Problem 构造。库自身不内置障碍模型；夹具只在测试/示例侧。
#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <planner/sampling_search/types.hpp>
#include <planner/sampling_search/validity.hpp>

namespace planner::sampling_search::test {

struct Rect {
  double x0{0.0};
  double y0{0.0};
  double x1{0.0};
  double y1{0.0};

  bool contains(const Point2d& p) const {
    return p.x_m >= x0 && p.x_m <= x1 && p.y_m >= y0 && p.y_m <= y1;
  }
};

struct RectFixture {
  Bounds2d bounds{};
  std::vector<Rect> obstacles{};
  double step{1.0};

  bool point_valid(const Point2d& p) const {
    if (p.x_m < bounds.min_x || p.x_m > bounds.max_x || p.y_m < bounds.min_y ||
        p.y_m > bounds.max_y) {
      return false;
    }
    for (const Rect& r : obstacles) {
      if (r.contains(p)) {
        return false;
      }
    }
    return true;
  }

  // 统一 Problem：回调按值捕获夹具副本，避免夹具生命周期问题。
  Problem make_problem(const Point2d& start, const Point2d& goal,
                       double budget_s, std::uint64_t seed,
                       double length_target = 0.0) const {
    Problem problem;
    problem.bounds = bounds;
    problem.start = start;
    problem.goal = goal;
    problem.edge_check_step = step;
    problem.length_target = length_target;
    problem.budget_s = budget_s;
    problem.seed = seed;
    const RectFixture copy = *this;
    auto validity = std::make_shared<SampledEdgeValidity>(
        [copy](const Point2d& p) { return copy.point_valid(p); }, step);
    problem.point_valid = [validity](const Point2d& p) {
      return validity->is_point_valid(p);
    };
    problem.edge_valid = [validity](const Point2d& a, const Point2d& b) {
      return validity->is_edge_valid(a, b);
    };
    return problem;
  }
};

// 开阔场景：100×100，无障碍，起 (10,10)、终 (90,90)。
inline RectFixture open_fixture(double size = 100.0, double step = 1.0) {
  RectFixture fixture;
  fixture.bounds = Bounds2d{0.0, 0.0, size, size};
  fixture.step = step;
  return fixture;
}

// 探针 family_d 同款三墙场景：100×100，起 (10,10)、终 (90,90)。
inline RectFixture three_wall_fixture(double step = 1.0) {
  RectFixture fixture;
  fixture.bounds = Bounds2d{0.0, 0.0, 100.0, 100.0};
  fixture.obstacles = {
      Rect{30.0, 0.0, 45.0, 55.0},
      Rect{30.0, 70.0, 45.0, 100.0},
      Rect{60.0, 20.0, 75.0, 70.0},
  };
  fixture.step = step;
  return fixture;
}

// 小场景（多种子烟测）：50×50，单墙 + 一个方块，起 (5,5)、终 (45,45)。
inline RectFixture small_fixture(double step = 1.0) {
  RectFixture fixture;
  fixture.bounds = Bounds2d{0.0, 0.0, 50.0, 50.0};
  fixture.obstacles = {
      Rect{15.0, 0.0, 18.0, 30.0},
      Rect{30.0, 20.0, 38.0, 28.0},
  };
  fixture.step = step;
  return fixture;
}

// 不可达场景：终点位于 1 m 厚封闭矩形内（壁厚 ≥ 沿边步长，采样检查可命中）。
inline RectFixture unreachable_fixture(double step = 0.5) {
  RectFixture fixture;
  fixture.bounds = Bounds2d{0.0, 0.0, 100.0, 100.0};
  // 封闭方腔：外墙 [80,92]²，内腔 [81,91]²；终点 (86,86) 在内腔。
  fixture.obstacles = {
      Rect{80.0, 80.0, 92.0, 81.0},  // 下壁
      Rect{80.0, 91.0, 92.0, 92.0},  // 上壁
      Rect{80.0, 81.0, 81.0, 91.0},  // 左壁
      Rect{91.0, 81.0, 92.0, 91.0},  // 右壁
  };
  fixture.step = step;
  return fixture;
}

}  // namespace planner::sampling_search::test
