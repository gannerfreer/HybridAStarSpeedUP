/**
 * @file scenario_config.hpp
 * @brief 场景配置的纯数据结构定义（不依赖任何配置解析库）。
 */

#pragma once

#include <string>
#include <vector>

#include "planner/math/pose2d.hpp"

namespace planner::common {

/**
 * @brief 场景中的凸多边形障碍物配置。
 */
struct ObstacleConfig {
  std::string id;                       ///< 障碍物唯一标识符。
  std::vector<math::Point2d> vertices;  ///< 逆时针或顺时针排列的凸多边形顶点（米）。
};

/**
 * @brief 栅格地图与起终点位姿组成的场景配置。
 */
struct ScenarioConfig {
  int width{0};                           ///< 栅格列数（沿局部 X 轴方向）。
  int height{0};                          ///< 栅格行数（沿局部 Y 轴方向）。
  double resolution_m{0.0};               ///< 正方形栅格单元边长分辨率（米）。
  math::Pose2d origin;                    ///< 栅格原点位姿。
  math::Pose2d start;                     ///< 规划起始位姿（后轴中心）。
  math::Pose2d goal;                      ///< 规划目标位姿（后轴中心）。
  std::vector<ObstacleConfig> obstacles;  ///< 场景障碍物列表。
};

/**
 * @brief 校验场景配置的栅格尺寸、分辨率、位姿有限性与障碍物凸多边形合法性。
 * @param[in] scenario 待校验的场景配置。
 * @param[out] reason 失败时的诊断说明文本指针，可为 nullptr。
 * @return 若场景配置全部合法则返回 true。
 */
[[nodiscard]] bool scenario_config_valid(const ScenarioConfig& scenario,
                                         std::string* reason = nullptr);

}  // namespace planner::common
