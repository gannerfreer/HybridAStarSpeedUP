#pragma once

// planner_rs GoogleTest 套件的共享辅助：冻结的基准车辆、独立的运动学 ODE
// 重构，以及确定性合成查询集（Docs/component_development_plan.md 7.1）。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "planner/rs/reeds_shepp.hpp"

namespace planner::rs::test {

// 冻结的基准车辆 (7.1)：轴距 2.8 m，宽 1.8 m，前/后悬 0.9/1.0 m，
// 最大转向角 0.5 rad，安全裕度 0.1 m。
inline common::VehicleModel benchmark_vehicle() {
  common::VehicleModel vehicle;
  vehicle.id = "v1_benchmark";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

// 显式指定最小转弯半径的同一车辆（最大转向角由半径推导）。
inline common::VehicleModel vehicle_with_radius(double radius_m) {
  common::VehicleModel vehicle = benchmark_vehicle();
  vehicle.max_steer_rad = std::atan(vehicle.wheelbase_m / radius_m);
  return vehicle;
}

inline common::Pose2d pose(double x, double y, double yaw) {
  return {x, y, yaw};
}

inline double yaw_error(double a, double b) {
  return std::fabs(common::normalize_angle_rad(a - b));
}

// 独立的终点重构：对每个分段用定步长 RK4 积分 dx/ds = gear*cos(yaw)、
// dy/ds = gear*sin(yaw)、dyaw/ds = gear*curvature。仅使用分段字段，
// 不依赖库的闭式组合，因此可交叉检验所报告的目标点。
struct Rk4Pose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline Rk4Pose rk4_integrate(const common::Pose2d& start, const RsPath& path,
                             double step_m = 1e-3) {
  Rk4Pose state{start.x_m, start.y_m, start.yaw_rad};
  for (const RsSegment& segment : path.segments) {
    const double gear = (segment.direction == common::Direction::kForward) ? 1.0 : -1.0;
    const double kappa = segment.curvature_inv_m;
    auto derivative = [gear, kappa](const Rk4Pose& p) {
      return Rk4Pose{gear * std::cos(p.yaw), gear * std::sin(p.yaw), gear * kappa};
    };
    const int steps = std::max(1, static_cast<int>(std::ceil(segment.length_m / step_m)));
    const double h = segment.length_m / static_cast<double>(steps);
    for (int i = 0; i < steps; ++i) {
      const Rk4Pose k1 = derivative(state);
      const Rk4Pose s2{state.x + 0.5 * h * k1.x, state.y + 0.5 * h * k1.y,
                       state.yaw + 0.5 * h * k1.yaw};
      const Rk4Pose k2 = derivative(s2);
      const Rk4Pose s3{state.x + 0.5 * h * k2.x, state.y + 0.5 * h * k2.y,
                       state.yaw + 0.5 * h * k2.yaw};
      const Rk4Pose k3 = derivative(s3);
      const Rk4Pose s4{state.x + h * k3.x, state.y + h * k3.y, state.yaw + h * k3.yaw};
      const Rk4Pose k4 = derivative(s4);
      state.x += h * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x) / 6.0;
      state.y += h * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y) / 6.0;
      state.yaw += h * (k1.yaw + 2.0 * k2.yaw + 2.0 * k3.yaw + k4.yaw) / 6.0;
    }
  }
  return state;
}

inline double endpoint_position_error(const RsPath& path) {
  const Rk4Pose end = rk4_integrate(path.start, path);
  return std::hypot(end.x - path.goal.x_m, end.y - path.goal.y_m);
}

inline double endpoint_yaw_error(const RsPath& path) {
  const Rk4Pose end = rk4_integrate(path.start, path);
  return yaw_error(end.yaw, path.goal.yaw_rad);
}

struct Query {
  std::string id;
  common::Pose2d start;
  common::Pose2d goal;
};

// 小型确定性 PRNG (splitmix64)；不存在 <random> 引擎/状态的不确定性。
class SplitMix64 {
 public:
  explicit SplitMix64(std::uint64_t seed) : state_(seed) {}
  std::uint64_t next() {
    state_ += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }
  // 均匀分布于 [0, 1)。
  double uniform() {
    return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0);
  }
  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

 private:
  std::uint64_t state_;
};

// 覆盖 6.2 六个类别的确定性合成查询集：相同位姿、直线、转弯
// （含最小半径临界）、倒车、尖点和角度环绕；外加固定种子生成的随机位姿。
inline std::vector<Query> deterministic_queries() {
  const common::VehicleModel vehicle = benchmark_vehicle();
  const double radius = min_turn_radius_m(vehicle);
  const double pi = common::kPi;
  std::vector<Query> queries;
  auto add = [&queries](const std::string& id, const common::Pose2d& start,
                        const common::Pose2d& goal) {
    queries.push_back({id, start, goal});
  };

  add("same_pose", pose(3.0, -2.0, 0.3), pose(3.0, -2.0, 0.3));
  add("straight_forward", pose(0.0, 0.0, 0.0), pose(12.0, 0.0, 0.0));
  add("straight_reverse", pose(0.0, 0.0, 0.0), pose(-7.0, 0.0, 0.0));
  add("turn_quarter_left", pose(0.0, 0.0, 0.0), pose(radius, radius, 0.5 * pi));
  add("turn_quarter_right", pose(0.0, 0.0, 0.0), pose(-radius, radius, -0.5 * pi));
  add("turn_min_radius_uturn", pose(0.0, 0.0, 0.0), pose(0.0, 2.0 * radius, pi));
  add("cusp_opposite_heading", pose(0.0, 0.0, 0.0), pose(0.0, 0.0, pi));
  add("reverse_parking", pose(4.0, 3.0, 0.35), pose(1.5, 0.4, -1.2));
  add("angle_wrap_same_pose", pose(0.0, 0.0, 350.0 * pi / 180.0),
      pose(0.0, 0.0, -10.0 * pi / 180.0));
  add("angle_wrap_forward", pose(0.0, 0.0, 0.0), pose(6.0, 1.5, -0.4));
  add("angle_wrap_full_turn", pose(0.0, 0.0, 0.0), pose(6.0, 1.5, 2.0 * pi - 0.4));
  add("far_goal", pose(-25.0, 8.0, 2.5), pose(22.0, -14.0, -1.1));

  SplitMix64 rng(0x5253313032303236ULL);  // 以十六进制风格拼出 "RS102026"
  for (int i = 0; i < 32; ++i) {
    const std::string id = "random_" + (i < 10 ? std::string("0") : std::string("")) +
                           std::to_string(i);
    add(id, pose(rng.uniform(-20.0, 20.0), rng.uniform(-20.0, 20.0),
                 rng.uniform(-pi, pi)),
        pose(rng.uniform(-20.0, 20.0), rng.uniform(-20.0, 20.0),
             rng.uniform(-pi, pi)));
  }
  return queries;
}

}  // namespace planner::rs::test
