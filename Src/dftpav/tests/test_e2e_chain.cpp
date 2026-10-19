// 端到端链路测试：初始路径 -> planner::corridor -> Dftpav
// 优化器 -> planner::validation。
//
// 成功链路使用共享的 v2 路径夹具：open_direct 与 narrow_corridor 使用
// Hybrid A* ros_free 后端的输出作为初始路径，死胡同倒车使用文档化的
// 局部车道走廊回退方案，中心线单障碍物夹具（obstacle_106/109/110）使用
// 预制的平滑迂回初值（这些用例中 Hybrid A* 后端自身的路径在组件报告中
// 被记为不稳定）。失败链路检验验证门槛（碰撞拒绝）、空走廊，以及低于
// 路径运动学要求的曲率限制。
//
// hybrid-004（小折返自交残余）核查：obstacle_107/109、deadend_122/125 的
// 原始 Hybrid A* 初值含小折返；`SmallFoldSeeds*` 用例按车道回退（0.8 m）
// 跑完整链路，验证 V 通过并锁定"折返保留在优化输出中"的已知状态
// （Dftpav 优化/平滑阶段不消除折返；见 Src/dftpav/README.md §4/§6）。
// hybrid-006：smac/zm0612 的死胡同贴墙种子原先被 V 以碰撞正确拒绝；
// 碰撞驱动的定向重试（见 optimizer.cpp 与 README §6）修复后这些组合也
// 全部 V 通过，折返清单不变。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "dftpav_e2e_test_support.hpp"

namespace planner::dftpav::e2e {
namespace {

void expect_switch_stops(const OptimizeResult& result) {
  for (std::size_t i = 0; i + 1 < result.trajectory.size(); ++i) {
    const double v0 = result.trajectory[i].velocity_mps;
    const double v1 = result.trajectory[i + 1].velocity_mps;
    if ((v0 > 0.0 && v1 < 0.0) || (v0 < 0.0 && v1 > 0.0)) {
      EXPECT_LE(std::min(std::abs(v0), std::abs(v1)), 1e-3)
          << "gear change between points " << i << " and " << i + 1
          << " does not pass through zero speed";
    }
  }
}

int count_gear_changes(const std::vector<common::TrajectoryPoint>& trajectory) {
  int changes = 0;
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
    if ((trajectory[i].velocity_mps > 0.0 && trajectory[i + 1].velocity_mps < 0.0) ||
        (trajectory[i].velocity_mps < 0.0 && trajectory[i + 1].velocity_mps > 0.0)) {
      ++changes;
    }
  }
  return changes;
}

void run_success_case(const std::string& id, bool expect_reverse,
                      bool force_lane_corridor = false,
                      double fallback_clearance_m = 0.15) {
  const common::VehicleModel vehicle = load_default_vehicle();
  const PathCase path_case = load_path_case(id);
  const ChainResult chain =
      run_chain(path_case, vehicle, {}, nullptr, force_lane_corridor, fallback_clearance_m);

  ASSERT_TRUE(chain.plan_ok) << id << ": hybrid A* failed: " << chain.plan.reason << " "
                             << chain.plan.statistics.dump();
  ASSERT_TRUE(chain.corridor_ok)
      << id << ": corridor failed: " << chain.corridor.reason << " " << chain.corridor.detail;
  const OptimizeResult& result = chain.optimize;
  ASSERT_TRUE(result.ok()) << id << ": " << result.reason << " | " << result.detail
                           << " | attempts=" << result.stats.tightening_attempts
                           << describe_attempts(result)
                           << " seed_len=" << polyline_length(chain.plan.path)
                           << " traj_len=" << trajectory_length(result.trajectory)
                           << " | " << describe_first_failure(result)
                           << " | " << describe_containment_context(chain, result)
                           << " |" << describe_path_junctions(chain.plan.path);
  EXPECT_TRUE(result.validation.valid);
  EXPECT_GT(result.validation.trajectory_points, 0u);
  EXPECT_GT(result.validation.corridor_samples, 0u);
  EXPECT_GT(result.validation.collision_pose_checks + result.validation.collision_motion_checks,
            0u);

  ASSERT_FALSE(result.trajectory.empty());
  EXPECT_NEAR(result.trajectory.back().pose.x_m, path_case.goal.x_m, 0.05);
  EXPECT_NEAR(result.trajectory.back().pose.y_m, path_case.goal.y_m, 0.05);
  EXPECT_NEAR(result.trajectory.back().pose.yaw_rad, path_case.goal.yaw_rad, 0.02);
  EXPECT_NEAR(result.trajectory.front().velocity_mps, 0.0, 1e-6);
  EXPECT_NEAR(result.trajectory.back().velocity_mps, 0.0, 1e-6);

  if (expect_reverse) {
    EXPECT_GE(count_gear_changes(result.trajectory), 1)
        << id << ": the fixture requires a direction change";
  }
  expect_switch_stops(result);
}

}  // namespace

TEST(DftpavE2eChain, OpenDirectSucceeds) {
  run_success_case("open_101", false);
}

TEST(DftpavE2eChain, NarrowCorridorBetweenObstaclesSucceeds) {
  // corridor_111：两道障碍墙之间 3 m 宽的通道；Hybrid A* 路径从中穿过，
  // 生成的走廊被直接消费，validation 接受优化后的轨迹。
  run_success_case("corridor_111", false);
}

TEST(DftpavE2eChain, ReverseParkingSucceeds) {
  // deadend_124：需要倒车动作（一次换挡）的死胡同停车位。生成的走廊在
  // 停车位墙壁处是刀刃式的（1 度航向抖动就会超出多边形数厘米），因此
  // 使用文档化的、带 0.8 m 间隙的局部车道走廊回退方案。
  run_success_case("deadend_124", true, true, 0.8);
}

TEST(DftpavE2eChain, IsDeterministic) {
  const common::VehicleModel vehicle = load_default_vehicle();
  const PathCase path_case = load_path_case("corridor_111");
  const ChainResult first = run_chain(path_case, vehicle);
  const ChainResult second = run_chain(path_case, vehicle);
  ASSERT_TRUE(first.plan_ok && second.plan_ok);
  ASSERT_TRUE(first.optimize.ok()) << first.optimize.reason << " "
                                   << describe_first_failure(first.optimize);
  ASSERT_TRUE(second.optimize.ok()) << second.optimize.reason;
  ASSERT_EQ(first.optimize.trajectory.size(), second.optimize.trajectory.size());
  for (std::size_t i = 0; i < first.optimize.trajectory.size(); ++i) {
    EXPECT_EQ(first.optimize.trajectory[i].t_s, second.optimize.trajectory[i].t_s);
    EXPECT_EQ(first.optimize.trajectory[i].pose.x_m, second.optimize.trajectory[i].pose.x_m);
    EXPECT_EQ(first.optimize.trajectory[i].pose.y_m, second.optimize.trajectory[i].pose.y_m);
    EXPECT_EQ(first.optimize.trajectory[i].velocity_mps,
              second.optimize.trajectory[i].velocity_mps);
  }
}

namespace {

// 中心线障碍物夹具的预制平滑迂回初值（圆角半径 5.5 m，
// 最大曲率 0.182 1/m <= 运动学上界 0.1951）。
std::vector<common::PathSegment> obstacle_detour_seed(const PathCase& path_case) {
  std::vector<common::Point2d> waypoints;
  if (path_case.id == "obstacle_106") {
    waypoints = {{2.0, 5.0}, {10.75, 9.0}, {17.0, 5.0}, {20.0, 5.0}};
  } else if (path_case.id == "obstacle_109") {
    waypoints = {{2.0, 5.0}, {10.0, 10.0}, {17.0, 5.0}, {20.0, 5.0}};
  } else {
    waypoints = {{2.0, 5.0}, {10.75, 11.0}, {18.0, 5.0}, {20.0, 5.0}};
  }
  return {test::make_fillet_segment(waypoints, 5.5, 0.1)};
}

// 紧贴顶点的变体：预制迂回擦过中心线障碍物（保守足迹间隙低于约 0.25 m），
// 因此走廊是刀刃式的，粗的 1 s 参数化会留下毫米级的包含残差。这些用例
// 检验精细参数化重试与节点空间可行性恢复（Wave 9，dftpav-002）。
std::vector<common::PathSegment> tight_obstacle_detour_seed(const PathCase& path_case) {
  std::vector<common::Point2d> waypoints;
  if (path_case.id == "obstacle_106") {
    waypoints = {{2.0, 5.0}, {10.75, 7.8}, {17.0, 5.0}, {20.0, 5.0}};
  } else {
    waypoints = {{2.0, 5.0}, {10.0, 9.0}, {17.0, 5.0}, {20.0, 5.0}};
  }
  return {test::make_fillet_segment(waypoints, 5.5, 0.1)};
}

void run_prefab_obstacle_case(const std::string& id,
                              const std::vector<common::PathSegment>& seed) {
  const common::VehicleModel vehicle = load_default_vehicle();
  const PathCase path_case = load_path_case(id);
  const ChainResult chain = run_prefab_chain(path_case, vehicle, seed);
  ASSERT_TRUE(chain.corridor_ok) << id << ": corridor failed: " << chain.corridor.reason
                                 << " " << chain.corridor.detail;
  const OptimizeResult& result = chain.optimize;
  ASSERT_TRUE(result.ok()) << id << ": " << result.reason << " | " << result.detail << " | "
                           << describe_first_failure(result);
  EXPECT_TRUE(result.validation.valid);
  EXPECT_GT(result.validation.corridor_samples, 0u);
  EXPECT_GT(result.validation.collision_pose_checks + result.validation.collision_motion_checks,
            0u);
  ASSERT_FALSE(result.trajectory.empty());
  EXPECT_NEAR(result.trajectory.back().pose.x_m, path_case.goal.x_m, 0.05);
  EXPECT_NEAR(result.trajectory.back().pose.y_m, path_case.goal.y_m, 0.05);
  EXPECT_NEAR(result.trajectory.back().pose.yaw_rad, path_case.goal.yaw_rad, 0.02);
  expect_switch_stops(result);
}

void run_prefab_obstacle_case(const std::string& id) {
  const PathCase path_case = load_path_case(id);
  run_prefab_obstacle_case(id, obstacle_detour_seed(path_case));
}

}  // namespace

TEST(DftpavE2eChain, CenterlineObstacleDetour106Succeeds) {
  run_prefab_obstacle_case("obstacle_106");
}

TEST(DftpavE2eChain, CenterlineObstacleDetour109Succeeds) {
  run_prefab_obstacle_case("obstacle_109");
}

TEST(DftpavE2eChain, CenterlineObstacleDetour110Succeeds) {
  run_prefab_obstacle_case("obstacle_110");
}

TEST(DftpavE2eChain, TightCenterlineObstacleDetour106Succeeds) {
  const PathCase path_case = load_path_case("obstacle_106");
  run_prefab_obstacle_case("obstacle_106", tight_obstacle_detour_seed(path_case));
}

TEST(DftpavE2eChain, TightCenterlineObstacleDetour109Succeeds) {
  const PathCase path_case = load_path_case("obstacle_109");
  run_prefab_obstacle_case("obstacle_109", tight_obstacle_detour_seed(path_case));
}

TEST(DftpavE2eChain, ValidationRejectsCollision) {
  // Hybrid A* 规划与走廊按无障碍物的情况构建；随后验证场景在直线路径上
  // 加入一个障碍物。优化器本身只知道走廊，因此轨迹会穿过障碍物，
  // 验证门槛必须拒绝它。
  const common::VehicleModel vehicle = load_default_vehicle();
  const PathCase path_case = load_path_case("open_101");
  collision::Scene scene = build_scene(path_case);
  scene.obstacles.push_back(collision::Obstacle::rectangle("blocker", 9.0, 3.0, 11.0, 7.0));

  const ChainResult chain = run_chain(path_case, vehicle, {}, &scene);
  ASSERT_TRUE(chain.plan_ok) << chain.plan.reason;
  ASSERT_TRUE(chain.corridor_ok) << chain.corridor.reason;
  const OptimizeResult& result = chain.optimize;
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "validation_collision")
      << result.reason << " | " << result.detail;
  EXPECT_EQ(result.status, common::Status::kValidationFailed);
  EXPECT_FALSE(result.validation.valid);
  EXPECT_EQ(result.validation.first_failure.constraint, validation::Constraint::kCollision);
  EXPECT_EQ(result.validation.reason, "collision");
}

TEST(DftpavE2eChain, EmptyCorridorIsRejected) {
  const common::VehicleModel vehicle = load_default_vehicle();
  const PathCase path_case = load_path_case("open_101");
  const ChainResult chain = run_chain(path_case, vehicle, [](OptimizeRequest* request) {
    request->corridor.polygons.clear();
  });
  ASSERT_TRUE(chain.plan_ok);
  const OptimizeResult& result = chain.optimize;
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "empty_corridor");
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_TRUE(result.trajectory.empty());
}

TEST(DftpavE2eChain, CurvatureLimitBelowRequirementFails) {
  // deadend_124 需要最小半径圆弧（kappa ~ 0.195 1/m）；0.02 1/m 的限制
  // 在运动学上不可行，绝不能报告为有效。
  const common::VehicleModel vehicle = load_default_vehicle();
  const PathCase path_case = load_path_case("deadend_124");
  const ChainResult chain =
      run_chain(path_case, vehicle,
                [](OptimizeRequest* request) {
                  request->limits.max_forward_curvature_inv_m = 0.02;
                  request->limits.max_backward_curvature_inv_m = 0.02;
                },
                nullptr, true, 0.8);
  ASSERT_TRUE(chain.plan_ok) << chain.plan.reason;
  ASSERT_TRUE(chain.corridor_ok) << chain.corridor.reason;
  const OptimizeResult& result = chain.optimize;
  EXPECT_FALSE(result.ok()) << "infeasible curvature limit must not succeed";
  EXPECT_FALSE(result.reason.empty());
  // 不可行的曲率限制要么以曲率限制本身的形式出现，要么表现为优化器在
  // 追逐它时扭曲出走廊；两者都是对所请求轨迹的验证拒绝。
  const bool curvature_or_giveup =
      result.reason == "validation_curvature_limit" ||
      result.reason == "provisional_curvature_limit" ||
      result.reason == "validation_corridor_containment" ||
      result.reason == "provisional_corridor_containment" ||
      result.reason == "cost_too_high" || result.reason == "solver_failed";
  EXPECT_TRUE(curvature_or_giveup) << "unexpected reason: " << result.reason << " | "
                                   << result.detail << " | " << describe_first_failure(result);
}

namespace {

// hybrid-004（小折返自交残余）的 Dftpav 消费核查。
//
// hybrid-003 的自交清单登记：下列 normalized 后端/用例组合的 Hybrid A*
// 初值包含 ≥1 处非共线横切（"前进-倒车-再前进"小折返），其余组合为 0。
// 本组用例按既有链路（run_chain：Hybrid A* -> 走廊 -> Dftpav -> V）逐组合
// 复核该前提，并记录 Dftpav 优化/平滑输出中的自交清单。
//
// 已验证结论（证据 run 见 Src/dftpav/README.md §4/§6）：
//   * V 是硬门禁且从未放宽；下列组合全部要求 V 通过；
//   * Dftpav 优化输出保留同样的折返（显著非共线横切 ≥1），只是横切段
//     长度从初值的 0.35–2.0 m 缩小到约 0.07–0.13 m；换向尖点处另有亚
//     厘米"原地微环"（significant 计数之外）。hybrid-004 因此仍未闭环：
//     Dftpav 优化/平滑阶段不消除这类小折返；
//   * 若未来优化器消除了折返，本用例会失败，提示关闭 hybrid-004 并更新
//     README 中的消费结论。
bool expects_folding_seed(const std::string& backend_id, const std::string& case_id) {
  if (backend_id == "ros_free") {
    return case_id == "obstacle_107" || case_id == "obstacle_109" ||
           case_id == "deadend_122" || case_id == "deadend_125";
  }
  if (backend_id == "smac_normalized" || backend_id == "zm0612_normalized") {
    return case_id == "obstacle_107" || case_id == "obstacle_109" ||
           case_id == "deadend_122";
  }
  if (backend_id == "kth_normalized") {
    return case_id == "deadend_122" || case_id == "deadend_125";
  }
  return false;
}

// 用文档化的局部车道走廊回退（0.8 m 间隙，与 deadend_124 一致）运行完整
// 链路：生成走廊在折返/刀口处是刀刃式的，原始 Hybrid A* 初值会留下毫米级
// 包含残差；车道回退让 V 的硬门禁检查在可用配置上进行。
void run_fold_case(const std::string& case_id, const std::string& backend_id) {
  SCOPED_TRACE(backend_id + " / " + case_id);
  const common::VehicleModel vehicle = load_default_vehicle();
  const PathCase path_case = load_path_case(case_id);
  const ChainResult chain = run_chain(path_case, vehicle, {}, nullptr, true, 0.8, backend_id);

  ASSERT_TRUE(chain.plan_ok) << backend_id << "/" << case_id
                             << ": hybrid A* failed: " << chain.plan.reason << " "
                             << chain.plan.statistics.dump();
  ASSERT_TRUE(chain.corridor_ok) << backend_id << "/" << case_id
                                 << ": corridor failed: " << chain.corridor.reason << " "
                                 << chain.corridor.detail;
  const OptimizeResult& result = chain.optimize;
  ASSERT_TRUE(result.ok()) << backend_id << "/" << case_id << ": " << result.reason << " | "
                           << result.detail << " | attempts=" << result.stats.tightening_attempts
                           << describe_attempts(result)
                           << " seed_len=" << polyline_length(chain.plan.path)
                           << " traj_len=" << trajectory_length(result.trajectory)
                           << " | " << describe_first_failure(result)
                           << " | " << describe_containment_context(chain, result)
                           << " |" << describe_path_junctions(chain.plan.path);
  EXPECT_TRUE(result.validation.valid);
  EXPECT_GT(result.validation.trajectory_points, 0u);
  EXPECT_GT(result.validation.corridor_samples, 0u);
  ASSERT_FALSE(result.trajectory.empty());
  EXPECT_NEAR(result.trajectory.back().pose.x_m, path_case.goal.x_m, 0.05);
  EXPECT_NEAR(result.trajectory.back().pose.y_m, path_case.goal.y_m, 0.05);
  EXPECT_NEAR(result.trajectory.back().pose.yaw_rad, path_case.goal.yaw_rad, 0.02);

  const std::vector<common::Point2d> seed_polyline = flatten_seed_polyline(chain.plan.path);
  const SelfIntersectionCount seed = count_self_intersections(seed_polyline);
  const std::vector<common::Point2d> optimized_polyline =
      trajectory_polyline(result.trajectory);
  const SelfIntersectionCount optimized = count_self_intersections(optimized_polyline);
  std::fprintf(stderr,
               "FOLD_ABSORB %-20s %-12s seed=total%d(nc%d,sig%d,li%.3f,lj%.3f) "
               "traj=total%d(nc%d,sig%d,li%.3f,lj%.3f) fallback=%d\n",
               backend_id.c_str(), case_id.c_str(), seed.total, seed.noncollinear,
               seed.significant_noncollinear, seed.first_len_i, seed.first_len_j,
               optimized.total, optimized.noncollinear, optimized.significant_noncollinear,
               optimized.first_len_i, optimized.first_len_j,
               chain.corridor_fallback ? 1 : 0);

  if (expects_folding_seed(backend_id, case_id)) {
    EXPECT_GE(seed.noncollinear, 1)
        << backend_id << "/" << case_id
        << ": hybrid-003 inventory expects a folding seed here; seed state drifted";
    EXPECT_GE(optimized.significant_noncollinear, 1)
        << backend_id << "/" << case_id
        << ": the hybrid-004 fold is gone from the optimized output; if this is a real "
           "improvement, close hybrid-004 and update the READMEs";
  } else {
    EXPECT_EQ(seed.noncollinear, 0)
        << backend_id << "/" << case_id << ": new folding seed outside the hybrid-003 inventory";
    EXPECT_EQ(optimized.significant_noncollinear, 0)
        << backend_id << "/" << case_id << ": new significant self-crossing appeared";
  }
  EXPECT_EQ(optimized.collinear, 0)
      << backend_id << "/" << case_id << ": Dftpav output contains a collinear overlap";
}

}  // namespace

// smac_normalized 的已知状态（hybrid-006 修复后）：障碍用例与
// deadend_122/125 经车道回退链路全部 V 通过且折返状态保持；deadend_125 的
// 贴墙种子由碰撞驱动的定向重试（细分段 + 曲率收紧，见 dftpav README §6）
// 收敛到 V 接受的结果。
TEST(DftpavE2eChain, SmallFoldSeedsSmacKnownState) {
  for (const std::string& case_id :
       {"obstacle_107", "obstacle_109", "deadend_122", "deadend_125"}) {
    run_fold_case(case_id, "smac_normalized");
  }
}

// zm0612_normalized 的已知状态（hybrid-006 修复后）：障碍用例与
// deadend_122/125 经车道回退链路全部 V 通过；deadend_122 的贴墙种子由
// 细分段 + 2x 锚定、deadend_125 由细分段 + 可行性权重/约束采样收紧
// （见 dftpav README §6）收敛到 V 接受的结果。
TEST(DftpavE2eChain, SmallFoldSeedsZm0612KnownState) {
  for (const std::string& case_id :
       {"obstacle_107", "obstacle_109", "deadend_122", "deadend_125"}) {
    run_fold_case(case_id, "zm0612_normalized");
  }
}

// hybrid-006 修复的确定性：走满碰撞驱动重试阶梯（zm0612/deadend_125，
// 4 次尝试）的链路对相同输入逐位可复现。
TEST(DftpavE2eChain, CollisionRetryIsDeterministic) {
  const common::VehicleModel vehicle = load_default_vehicle();
  const PathCase path_case = load_path_case("deadend_125");
  const ChainResult first =
      run_chain(path_case, vehicle, {}, nullptr, true, 0.8, "zm0612_normalized");
  const ChainResult second =
      run_chain(path_case, vehicle, {}, nullptr, true, 0.8, "zm0612_normalized");
  ASSERT_TRUE(first.optimize.ok()) << first.optimize.reason << " "
                                   << describe_first_failure(first.optimize);
  ASSERT_TRUE(second.optimize.ok()) << second.optimize.reason;
  EXPECT_EQ(first.optimize.stats.tightening_attempts,
            second.optimize.stats.tightening_attempts);
  ASSERT_EQ(first.optimize.trajectory.size(), second.optimize.trajectory.size());
  for (std::size_t i = 0; i < first.optimize.trajectory.size(); ++i) {
    EXPECT_EQ(first.optimize.trajectory[i].t_s, second.optimize.trajectory[i].t_s);
    EXPECT_EQ(first.optimize.trajectory[i].pose.x_m,
              second.optimize.trajectory[i].pose.x_m);
    EXPECT_EQ(first.optimize.trajectory[i].pose.y_m,
              second.optimize.trajectory[i].pose.y_m);
    EXPECT_EQ(first.optimize.trajectory[i].velocity_mps,
              second.optimize.trajectory[i].velocity_mps);
  }
}

// hybrid-004 主链核查：既有 e2e 后端（ros_free）× 四个场景。全部组合 V 通过
// （硬门禁），但优化输出保留折返（显著非共线横切），即 Dftpav 优化/平滑
// 阶段未消除 hybrid-004 的小折返。
TEST(DftpavE2eChain, SmallFoldSeedsRosFreePassValidationButRemainInOutput) {
  for (const std::string& case_id :
       {"obstacle_107", "obstacle_109", "deadend_122", "deadend_125"}) {
    run_fold_case(case_id, "ros_free");
  }
}

// 同上，默认后端 kth_normalized（其初值仅在 deadend_122/125 含折返）。
TEST(DftpavE2eChain, SmallFoldSeedsKthPassValidationButRemainInOutput) {
  for (const std::string& case_id :
       {"obstacle_107", "obstacle_109", "deadend_122", "deadend_125"}) {
    run_fold_case(case_id, "kth_normalized");
  }
}

}  // namespace planner::dftpav::e2e
