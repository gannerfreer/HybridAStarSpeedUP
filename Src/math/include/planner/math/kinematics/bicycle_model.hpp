/**
 * @file bicycle_model.hpp
 * @brief 后轴中心自行车模型的前轮转角/曲率/最小转弯半径换算
 */

#pragma once

namespace planner::math::kinematics {

/**
 * @brief 由前轮转角计算后轴中心轨迹曲率
 * @param[in] steer_rad 前轮转向角（弧度，左转为正）
 * @param[in] wheelbase_m 车辆轴距（米）
 * @return 有符号曲率（1/m）
 */
double curvature_from_steer(double steer_rad, double wheelbase_m) noexcept;

/**
 * @brief 由后轴中心轨迹曲率反算所需前轮转角
 * @param[in] curvature_per_m 有符号曲率（1/m）
 * @param[in] wheelbase_m 车辆轴距（米）
 * @return 前轮转向角（弧度）
 */
double steer_from_curvature(double curvature_per_m, double wheelbase_m) noexcept;

/**
 * @brief 由轴距与最大前轮转角计算后轴中心最小转弯半径
 * @param[in] wheelbase_m 车辆轴距（米）
 * @param[in] max_steer_rad 最大前轮转向角（弧度）
 * @return 后轴中心最小转弯半径（米）：wheelbase / tan(max_steer)
 */
double min_turn_radius_m(double wheelbase_m, double max_steer_rad) noexcept;

}  // namespace planner::math::kinematics
