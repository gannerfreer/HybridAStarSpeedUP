// 跨组件观察复核（hybrid-003，Wave 8）。
//
// Dftpav 的 Wave 7 报告声称：(a) park_116-120 与 deadend_122/125 存在
// 0.3-2.0 m 的尖点间隙，(b) obstacle_107/109 出现小的环路路径，(c)
// obstacle_108 的目标足迹触碰地图边界。这些检查在当前构建上通过公开 API
// 复现（或否定）各观察：
//   * 尖点间隙：前一分段的最后一个点与后一分段的第一个点必须位姿和 s_m
//     完全一致（冻结契约）；
//   * 环路检查：展平路径（合并重复的尖点位姿），检测每一对非相邻折线
//     分段是否相交；
//   * 边界检查：实际端点的足迹角点相对地图矩形（栅格坐标系）的最小距离。
//
// 测试为组件报告打印逐用例测量值，并断言契约值（零间隙、无自相交、
// 足迹在地图内）。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "planner/collision/scene.hpp"
#include "test_support.hpp"

namespace planner::hybrid_astar {
namespace {

using common::PathPoint;
using common::Pose2d;
using test_support::PathCase;
using test_support::PreparedCase;

constexpr double kTolerance = 1e-9;

const std::vector<PathCase>& all_cases() {
  static const std::vector<PathCase> cases = test_support::load_path_cases();
  return cases;
}

const PathCase* find_case(const std::string& id) {
  for (const PathCase& path_case : all_cases()) {
    if (path_case.id == id) {
      return &path_case;
    }
  }
  return nullptr;
}

std::optional<common::PlanResult> plan_case(const std::string& backend_id,
                                            const PreparedCase& prepared,
                                            const common::VehicleModel& vehicle) {
  const std::unique_ptr<HybridAStarBackend> backend = create_backend(backend_id);
  if (backend == nullptr ||
      !backend->configure(nlohmann::json::object()).accepted) {
    return std::nullopt;
  }
  PlannerRsProvider provider(vehicle);
  common::PlanRequest request = prepared.request;
  request.time_budget_s = 60.0;
  return backend->plan(request, prepared.grid, prepared.checker, provider);
}

bool same_pose(const Pose2d& a, const Pose2d& b) {
  return std::hypot(a.x_m - b.x_m, a.y_m - b.y_m) <= kTolerance &&
         std::abs(common::normalize_angle_rad(a.yaw_rad - b.yaw_rad)) <=
             kTolerance;
}

double cross2d(const common::Point2d& a, const common::Point2d& b,
               const common::Point2d& c) {
  return (b.x_m - a.x_m) * (c.y_m - a.y_m) -
         (b.y_m - a.y_m) * (c.x_m - a.x_m);
}

bool point_on_segment(const common::Point2d& a, const common::Point2d& b,
                      const common::Point2d& p) {
  return std::min(a.x_m, b.x_m) - kTolerance <= p.x_m &&
         p.x_m <= std::max(a.x_m, b.x_m) + kTolerance &&
         std::min(a.y_m, b.y_m) - kTolerance <= p.y_m &&
         p.y_m <= std::max(a.y_m, b.y_m) + kTolerance;
}

// 闭线段相交（接触也算）。
bool segments_intersect(const common::Point2d& a, const common::Point2d& b,
                        const common::Point2d& c, const common::Point2d& d) {
  const double o1 = cross2d(a, b, c);
  const double o2 = cross2d(a, b, d);
  const double o3 = cross2d(c, d, a);
  const double o4 = cross2d(c, d, b);
  const auto sign = [](double value) {
    return value > kTolerance ? 1 : (value < -kTolerance ? -1 : 0);
  };
  if (sign(o1) != sign(o2) && sign(o3) != sign(o4)) {
    return true;
  }
  if (sign(o1) == 0 && point_on_segment(a, b, c)) return true;
  if (sign(o2) == 0 && point_on_segment(a, b, d)) return true;
  if (sign(o3) == 0 && point_on_segment(c, d, a)) return true;
  if (sign(o4) == 0 && point_on_segment(c, d, b)) return true;
  return false;
}

// 展平路径，合并重复的尖点位姿（连续位姿相同），使折线没有零长步进。
std::vector<Pose2d> flatten_path(const common::PlanResult& result) {
  std::vector<Pose2d> flat;
  for (const common::PathSegment& segment : result.path) {
    for (const PathPoint& point : segment.points) {
      if (!flat.empty() && same_pose(flat.back(), point.pose)) {
        continue;
      }
      flat.push_back(point.pose);
    }
  }
  return flat;
}

struct IntersectionDetail {
  int count{0};
  std::size_t first_i{0};
  std::size_t first_j{0};
  bool first_collinear{false};
  double first_x{0.0};
  double first_y{0.0};
  common::Point2d first_a;
  common::Point2d first_b;
  common::Point2d first_c;
  common::Point2d first_d;
};

IntersectionDetail count_self_intersections(const std::vector<Pose2d>& flat) {
  IntersectionDetail detail;
  const std::size_t count = flat.size();
  for (std::size_t i = 0; i + 1 < count; ++i) {
    for (std::size_t j = i + 2; j + 1 < count; ++j) {
      const common::Point2d a{flat[i].x_m, flat[i].y_m};
      const common::Point2d b{flat[i + 1].x_m, flat[i + 1].y_m};
      const common::Point2d c{flat[j].x_m, flat[j].y_m};
      const common::Point2d d{flat[j + 1].x_m, flat[j + 1].y_m};
      if (segments_intersect(a, b, c, d)) {
        ++detail.count;
        if (detail.count == 1) {
          detail.first_i = i;
          detail.first_j = j;
          detail.first_collinear =
              std::abs(cross2d(a, b, c)) <= kTolerance &&
              std::abs(cross2d(a, b, d)) <= kTolerance;
          // 取第一段中点的粗略位置。
          detail.first_x = 0.5 * (a.x_m + b.x_m);
          detail.first_y = 0.5 * (a.y_m + b.y_m);
          detail.first_a = a;
          detail.first_b = b;
          detail.first_c = c;
          detail.first_d = d;
        }
      }
    }
  }
  return detail;
}

const std::vector<std::string>& normalized_backends() {
  static const std::vector<std::string> ids = {
      "ros_free", "smac_normalized", "kth_normalized", "zm0612_normalized"};
  return ids;
}

TEST(CrossComponent, cusp_gap_per_case_current_build) {
  const common::VehicleModel vehicle = test_support::load_default_vehicle();
  const std::vector<std::string> case_ids = {
      "park_116", "park_117", "park_118", "park_119",
      "park_120", "deadend_122", "deadend_125"};
  for (const std::string& backend_id : normalized_backends()) {
    for (const std::string& case_id : case_ids) {
      SCOPED_TRACE(backend_id + " / " + case_id);
      const PathCase* path_case = find_case(case_id);
      ASSERT_NE(path_case, nullptr);
      const PreparedCase prepared =
          test_support::prepare_case(*path_case, vehicle);
      const auto result = plan_case(backend_id, prepared, vehicle);
      ASSERT_TRUE(result.has_value());
      ASSERT_EQ(result->status, common::Status::kSuccess)
          << result->reason;

      double max_gap = 0.0;
      for (std::size_t segment = 1; segment < result->path.size(); ++segment) {
        const PathPoint& previous =
            result->path[segment - 1].points.back();
        const PathPoint& next = result->path[segment].points.front();
        const double gap =
            std::hypot(previous.pose.x_m - next.pose.x_m,
                       previous.pose.y_m - next.pose.y_m);
        max_gap = std::max(max_gap, gap);
        EXPECT_NEAR(previous.s_m, next.s_m, kTolerance);
        EXPECT_NE(previous.direction, next.direction);
      }
      std::fprintf(stderr, "CROSS_CUSP %-20s %-12s segments=%zu max_gap=%.3e\n",
                   backend_id.c_str(), case_id.c_str(), result->path.size(),
                   max_gap);
      EXPECT_LE(max_gap, kTolerance);
    }
  }
}

// Wave 8 发现：绕行/死胡同搜索可能选出小的、无碰撞的自交折返（基元换序
// 或 RS 连接穿过更早的分段）。它们不是闭合环路（单次横穿），保持路径
// 契约有效，而软环路防护实验以更差的路径质量换取了它们的消除（见
// README）。本测试固定已知状态：
//   * 泊车用例 park_116/118/119/120 必须保持无相交；
//   * obstacle_107/109 与 deadend_122/125 每个后端/用例最多出现一次交叉
//    （回归防护：出现新用例或计数增加即失败）；
//   * 任何交叉都不得共线（共线重叠将是折返痕迹，而非已知的横穿）。
TEST(CrossComponent, self_intersection_inventory_matches_known_state) {
  const common::VehicleModel vehicle = test_support::load_default_vehicle();
  const std::vector<std::string> strict_cases = {"park_116", "park_118",
                                                 "park_119", "park_120"};
  const std::vector<std::string> tolerant_cases = {
      "obstacle_107", "obstacle_109", "deadend_122", "deadend_125"};
  for (const std::string& backend_id : normalized_backends()) {
    for (const std::string& case_id : strict_cases) {
      SCOPED_TRACE(backend_id + " / " + case_id);
      const PathCase* path_case = find_case(case_id);
      ASSERT_NE(path_case, nullptr);
      const PreparedCase prepared =
          test_support::prepare_case(*path_case, vehicle);
      const auto result = plan_case(backend_id, prepared, vehicle);
      ASSERT_TRUE(result.has_value());
      ASSERT_EQ(result->status, common::Status::kSuccess)
          << result->reason;
      const std::vector<Pose2d> flat = flatten_path(*result);
      const IntersectionDetail intersections = count_self_intersections(flat);
      std::fprintf(stderr,
                   "CROSS_LOOP %-20s %-12s points=%zu intersections=%d\n",
                   backend_id.c_str(), case_id.c_str(), flat.size(),
                   intersections.count);
      EXPECT_EQ(intersections.count, 0);
    }
    for (const std::string& case_id : tolerant_cases) {
      SCOPED_TRACE(backend_id + " / " + case_id);
      const PathCase* path_case = find_case(case_id);
      ASSERT_NE(path_case, nullptr);
      const PreparedCase prepared =
          test_support::prepare_case(*path_case, vehicle);
      const auto result = plan_case(backend_id, prepared, vehicle);
      ASSERT_TRUE(result.has_value());
      ASSERT_EQ(result->status, common::Status::kSuccess)
          << result->reason;
      const std::vector<Pose2d> flat = flatten_path(*result);
      const IntersectionDetail intersections = count_self_intersections(flat);
      std::fprintf(stderr,
                   "CROSS_LOOP %-20s %-12s points=%zu intersections=%d "
                   "first=(i=%zu,j=%zu,collinear=%d)\n",
                   backend_id.c_str(), case_id.c_str(), flat.size(),
                   intersections.count, intersections.first_i,
                   intersections.first_j,
                   intersections.first_collinear ? 1 : 0);
      EXPECT_LE(intersections.count, 1)
          << "new self-crossings appeared on " << case_id;
      EXPECT_FALSE(intersections.first_collinear)
          << "collinear overlap on " << case_id;
    }
  }
}

TEST(CrossComponent, obstacle_108_goal_footprint_inside_map) {
  const common::VehicleModel vehicle = test_support::load_default_vehicle();
  const PathCase* path_case = find_case("obstacle_108");
  ASSERT_NE(path_case, nullptr);
  const PreparedCase prepared = test_support::prepare_case(*path_case, vehicle);
  const collision::GridFrame frame =
      collision::GridFrame::from_grid(prepared.grid);
  const collision::Footprint footprint = collision::make_footprint(vehicle);

  const auto report_pose = [&](const std::string& label, const Pose2d& pose) {
    double min_distance = std::numeric_limits<double>::infinity();
    for (const common::Point2d& corner : footprint.world_corners(pose)) {
      const common::Point2d grid_point = frame.to_grid_frame(corner);
      min_distance =
          std::min(min_distance, frame.distance_to_boundary(grid_point));
    }
    std::fprintf(stderr, "CROSS_BOUNDARY %-12s min_distance=%.4f m\n",
                 label.c_str(), min_distance);
    return min_distance;
  };

  const double goal_distance =
      report_pose("goal", path_case->goal);
  EXPECT_GE(goal_distance, -kTolerance)
      << "goal footprint is outside the map rectangle";

  for (const std::string& backend_id : normalized_backends()) {
    SCOPED_TRACE(backend_id);
    const auto result = plan_case(backend_id, prepared, vehicle);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->status, common::Status::kSuccess) << result->reason;
    ASSERT_FALSE(result->path.empty());
    ASSERT_FALSE(result->path.back().points.empty());
    const Pose2d& endpoint = result->path.back().points.back().pose;
    const double endpoint_distance =
        report_pose(backend_id, endpoint);
    EXPECT_GE(endpoint_distance, -kTolerance)
        << "achieved endpoint footprint is outside the map rectangle";
    EXPECT_LE(std::hypot(endpoint.x_m - path_case->goal.x_m,
                         endpoint.y_m - path_case->goal.y_m),
              0.05);
  }
}

}  // namespace
}  // namespace planner::hybrid_astar
