#include <gtest/gtest.h>

#include <vector>

#include "dftpav_test_support.hpp"

namespace planner::dftpav::test {
namespace {

OptimizeRequest reverse_request() {
  OptimizeRequest request;
  request.request_id = "test-reverse";
  request.map_id = "test_map";
  request.map_version = "v1";
  request.vehicle = default_vehicle();
  common::PathSegment forward = straight_segment(0.0, 0.0, 6.0, 0.0, 1.0);
  const std::vector<common::Point2d> backward_points = {
      {6.0, 0.0}, {5.0, 0.0}, {4.0, 0.0}, {3.0, 0.0}};
  common::PathSegment backward = make_segment(backward_points, common::Direction::kReverse);
  request.seed = {forward, backward};
  // 车道局部走廊（模拟走廊生成器的局部多边形）；单个大矩形会让
  // 时间最优优化器抄近路绕过尖点，而不是强制执行倒车动作。
  // 带小间隙的车道局部走廊：多边形在站点方向很紧，时间最优优化器
  // 无法让尖点远离初值（validation 按轨迹自身弧长关联走廊区间）。
  // 适配器的几何收紧为软约束残差提供余量。
  request.corridor = make_footprint_lane_corridor(request.seed, request.vehicle, 0.5, 0.15);
  request.corridor.map_id = request.map_id;
  request.corridor.map_version = request.map_version;
  request.limits = default_limits();
  // 合成尖点需要强锚定，才能让平滑后的轨迹保持在初值的站点结构上
  // （validation 按轨迹自身弧长关联走廊区间）。
  request.options.anchor_weight = 10000.0;
  return request;
}

}  // namespace

TEST(DftpavDirectionChange, StopsAtSwitchAndMeetsTolerances) {
  const OptimizeRequest request = reverse_request();
  const OptimizeResult result = optimize_trajectory(request);
  std::string attempt_text;
  for (std::size_t i = 0; i < result.stats.attempt_reasons.size(); ++i) {
    attempt_text += " attempt" + std::to_string(i) + "=" + result.stats.attempt_reasons[i];
  }
  ASSERT_TRUE(result.ok())
      << "reason=" << result.reason << " detail=" << result.detail
      << " attempts=" << result.stats.tightening_attempts << attempt_text
      << " seed_len=" << polyline_length(request.seed)
      << " traj_len=" << trajectory_length(result.trajectory)
      << " window="
      << trajectory_window_diag(result.trajectory,
                                result.validation.first_failure.point_index);
  ASSERT_EQ(result.stats.direction_changes, 1);
  ASSERT_EQ(result.stats.direction_switch_times_s.size(), 1u);
  const double switch_time = result.stats.direction_switch_times_s.front();

  // 时间严格递增与几何：起点 (0,0)、终点 (3,0)。
  ASSERT_GE(result.trajectory.size(), 3u);
  EXPECT_NEAR(result.trajectory.front().pose.x_m, 0.0, 0.05);
  EXPECT_NEAR(result.trajectory.back().pose.x_m, 3.0, 0.05);
  EXPECT_NEAR(result.trajectory.back().pose.y_m, 0.0, 0.05);
  EXPECT_NEAR(result.trajectory.back().pose.yaw_rad, 0.0, 0.02);

  // 换向采样点必须存在且处于（近乎）静止：
  // 第 7.1 节验收界限 |v| <= 1e-3 m/s。
  const common::TrajectoryPoint* switch_point = sample_at(result.trajectory, switch_time);
  ASSERT_NE(switch_point, nullptr) << "no sample at switch time " << switch_time;
  EXPECT_LE(std::abs(switch_point->velocity_mps), 1e-3);

  // 速度在换向点附近改变符号。
  double max_forward = 0.0;
  double max_backward = 0.0;
  std::vector<common::TrajectoryPoint> forward_part;
  std::vector<common::TrajectoryPoint> backward_part;
  for (const auto& point : result.trajectory) {
    if (point.t_s < switch_time - 1e-12) {
      forward_part.push_back(point);
      max_forward = std::max(max_forward, point.velocity_mps);
    } else if (point.t_s > switch_time + 1e-12) {
      backward_part.push_back(point);
      max_backward = std::max(max_backward, -point.velocity_mps);
    }
  }
  EXPECT_GT(max_forward, 0.1);
  EXPECT_GT(max_backward, 0.1);

  // 按第 7.1 节容差检查每个方向的动力学限制。
  const CheckResult forward_check =
      check_trajectory_semantics(forward_part, request.limits, 1e-6, 1);
  EXPECT_TRUE(forward_check.ok) << forward_check.reason << " at t=" << forward_check.value;
  const CheckResult backward_check =
      check_trajectory_semantics(backward_part, request.limits, 1e-6, -1);
  EXPECT_TRUE(backward_check.ok) << backward_check.reason << " at t=" << backward_check.value;

  // 独立的走廊/障碍物检查（此处无障碍物）。
  const CheckResult containment = check_corridor_and_obstacles(
      result.trajectory, request.vehicle, request.seed, request.corridor, {}, 0.0);
  EXPECT_TRUE(containment.ok) << containment.reason << " at t=" << containment.value;
}

TEST(DftpavDirectionChange, IsDeterministic) {
  const OptimizeRequest request = reverse_request();
  const OptimizeResult first = optimize_trajectory(request);
  const OptimizeResult second = optimize_trajectory(request);
  ASSERT_TRUE(first.ok()) << first.reason;
  ASSERT_TRUE(second.ok()) << second.reason;
  ASSERT_EQ(first.trajectory.size(), second.trajectory.size());
  for (std::size_t i = 0; i < first.trajectory.size(); ++i) {
    EXPECT_EQ(first.trajectory[i].t_s, second.trajectory[i].t_s);
    EXPECT_EQ(first.trajectory[i].velocity_mps, second.trajectory[i].velocity_mps);
    EXPECT_EQ(first.trajectory[i].pose.x_m, second.trajectory[i].pose.x_m);
  }
}

}  // namespace planner::dftpav::test
