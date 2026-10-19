// 验收回归用例（100k 运行，种子 04/82）。
//
// 这两个位姿用例是首次验收运行
// （artifacts/collision_differential/acceptance_20261009T172336Z_1399037）中
// 仅有的漏检：
//   * cr-acc-s04-0347-pose
//   * cr-acc-s82-0375-pose
// 二者都是 exact_contact 构造，真实角点间隙略高于尺度感知的接触容差。独立参考
// 过去在包含测试中按每条边线分别施加容差，当 g*cos(45) <= tol 时会把凸角外侧
// 距离为 g 的点判为“在内”（最多宽容 sqrt(2) 倍），而生产分离轴测试使用对角线
// 间隙 g > tol，报告 clear。修复后两种实现采用相同的欧氏距离接触策略。
//
// 原始用例中零散的栅格内容距足迹 > 0.5 m（最近的非自由单元 0.5449 m /
// 0.5669 m），因此下面用原始地图尺寸的全自由栅格逐字复现决策相关输入。下次夹具
// 冻结时提升为 tests/fixtures/v2；完整输入文件保留在验收运行目录中。

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "planner/collision/collision_checker.hpp"
#include "planner/collision/geometry.hpp"
#include "planner/collision/reference.hpp"
#include "test_helpers.hpp"

namespace planner::collision {
namespace {

using test::default_vehicle;
using test::free_grid;
using test::paint_cells;

struct CornerContactCase {
  Pose2d pose;
  std::vector<Point2d> obstacle;
};

CornerContactCase seed_04_case() {
  CornerContactCase test_case;
  test_case.pose = Pose2d{7.9955896407020335, 11.036485910086743,
                          0.5175604102984566};
  test_case.obstacle = {Point2d{10.803137464927573, 13.785608536458508},
                        Point2d{11.189589472281545, 15.193798856754634},
                        Point2d{9.904376412123172, 15.54650201045285},
                        Point2d{9.5179244047692, 14.138311690156723}};
  return test_case;
}

CornerContactCase seed_82_case() {
  CornerContactCase test_case;
  test_case.pose = Pose2d{11.731752773173547, 10.709023774144452,
                          0.9092017411568696};
  test_case.obstacle = {Point2d{10.266925852490072, 10.455484640111592},
                        Point2d{8.622429560142583, 10.250843511843001},
                        Point2d{8.78824341792126, 8.918363200363416},
                        Point2d{10.432739710268748, 9.123004328632007}};
  return test_case;
}

std::vector<Point2d> footprint_polygon(const VehicleModel& vehicle,
                                       const Pose2d& pose) {
  const Footprint footprint = make_footprint(vehicle);
  const auto corners = footprint.world_corners(pose);
  return {corners[0], corners[1], corners[2], corners[3]};
}

// 沿（最近的足迹角点 -> 障碍物角点）方向移动障碍物，使真实角点间隙变为
// `target_gap`（负值表示侵入）。用于构造低于容差与重叠的对照用例。
std::vector<Point2d> obstacle_with_gap(const VehicleModel& vehicle,
                                       const CornerContactCase& test_case,
                                       double target_gap) {
  const std::vector<Point2d> footprint =
      footprint_polygon(vehicle, test_case.pose);
  Point2d nearest = footprint.front();
  double best = distance(nearest, test_case.obstacle.front());
  for (const Point2d& corner : footprint) {
    const double d = distance(corner, test_case.obstacle.front());
    if (d < best) {
      best = d;
      nearest = corner;
    }
  }
  const Point2d direction = test_case.obstacle.front() - nearest;
  const double gap = norm(direction);
  const Point2d shift = direction * (target_gap / gap - 1.0);
  std::vector<Point2d> moved;
  moved.reserve(test_case.obstacle.size());
  for (Point2d vertex : test_case.obstacle) {
    moved.push_back(vertex + shift);
  }
  return moved;
}

// 当构造确实是“角点间隙高于容差”的用例时返回真。
bool gap_above_tolerance(const VehicleModel& vehicle,
                         const CornerContactCase& test_case) {
  const std::vector<Point2d> footprint =
      footprint_polygon(vehicle, test_case.pose);
  const double distance_m =
      convex_polygon_distance(footprint, test_case.obstacle);
  const double scale =
      std::max(aabb_of(footprint).max_abs_coordinate(),
               aabb_of(test_case.obstacle).max_abs_coordinate());
  return distance_m > contact_tolerance(scale);
}

void expect_verdicts(Scene scene, const Pose2d& pose, bool colliding,
                     const std::string& context) {
  const CheckResult plain = check_pose(pose, default_vehicle(), scene);
  ASSERT_NE(plain.status, CheckStatus::kInvalidInput) << context;
  EXPECT_EQ(plain.collided(), colliding) << context << ": " << plain.detail;
  CheckOptions accelerated;
  accelerated.enable_acceleration = true;
  const CheckResult accel =
      check_pose(pose, default_vehicle(), scene, accelerated);
  EXPECT_EQ(accel.collided(), colliding) << context << ": " << accel.detail;
  const ReferenceResult reference =
      reference_check_pose(pose, default_vehicle(), scene);
  ASSERT_NE(reference.verdict, ReferenceVerdict::kInvalidInput) << context;
  EXPECT_EQ(reference.verdict == ReferenceVerdict::kCollision, colliding)
      << context << ": " << reference.detail;
}

TEST(AcceptanceRegression, seed04_corner_gap_above_tolerance_is_clear) {
  const VehicleModel vehicle = default_vehicle();
  const CornerContactCase test_case = seed_04_case();
  ASSERT_TRUE(gap_above_tolerance(vehicle, test_case));
  Scene scene = free_grid(24.0, 26.0);  // 原始地图尺寸，全部自由
  scene.obstacles.push_back(Obstacle::convex_polygon(
      "feature_corner", test_case.obstacle));
  expect_verdicts(scene, test_case.pose, false, "cr-acc-s04-0347-pose");
}

TEST(AcceptanceRegression, seed82_corner_gap_above_tolerance_is_clear) {
  const VehicleModel vehicle = default_vehicle();
  const CornerContactCase test_case = seed_82_case();
  ASSERT_TRUE(gap_above_tolerance(vehicle, test_case));
  Scene scene = free_grid(26.0, 25.0);  // 原始地图尺寸，全部自由
  scene.obstacles.push_back(Obstacle::convex_polygon(
      "feature_corner", test_case.obstacle));
  expect_verdicts(scene, test_case.pose, false, "cr-acc-s82-0375-pose");
}

TEST(AcceptanceRegression, corner_gap_below_tolerance_is_contact) {
  const VehicleModel vehicle = default_vehicle();
  const CornerContactCase base = seed_04_case();
  Scene scene = free_grid(24.0, 26.0);
  scene.obstacles.push_back(Obstacle::convex_polygon(
      "feature_corner", obstacle_with_gap(vehicle, base, 5e-12)));
  expect_verdicts(scene, base.pose, true, "5e-12 gap below tolerance");
}

TEST(AcceptanceRegression, corner_penetration_is_collision) {
  const VehicleModel vehicle = default_vehicle();
  const CornerContactCase base = seed_82_case();
  Scene scene = free_grid(26.0, 25.0);
  scene.obstacles.push_back(Obstacle::convex_polygon(
      "feature_corner", obstacle_with_gap(vehicle, base, -1e-11)));
  expect_verdicts(scene, base.pose, true, "-1e-11 corner penetration");
}

// 栅格单元角点变体：被占用单元的角点沿对角线靠近足迹角点。高于容差时两种
// 实现都判 clear；在半个容差时二者都报告接触。
TEST(AcceptanceRegression, blocked_cell_corner_gap_policy) {
  const VehicleModel vehicle = default_vehicle();
  const double root_half = std::sqrt(0.5);
  const auto pose_for_gap = [&](double gap) {
    const double offset = gap * root_half;
    return Pose2d{2.0 - offset, 2.0 - offset, 0.0};
  };
  const auto scene_with_cell = [] {
    Scene scene = free_grid(20.0, 20.0);
    paint_cells(scene.grid, 5.8, 3.0, 5.9, 3.1, 1);
    return scene;
  };
  // 车体坐标系中的足迹角点 (3.8, 1.0) 沿对角线恰好落在距单元角点 (5.8, 3.0)
  // g/sqrt(2) 处。
  expect_verdicts(scene_with_cell(), pose_for_gap(2e-11), false,
                  "cell corner gap above tolerance");
  expect_verdicts(scene_with_cell(), pose_for_gap(2e-12), true,
                  "cell corner gap below tolerance");
}

}  // namespace
}  // namespace planner::collision
