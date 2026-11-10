/**
 * @file polygon2d.hpp
 * @brief 二维凸多边形基元、分离轴定理 (SAT) 及精确欧氏距离查询算法。
 * @details 核心几何规范与约定：
 *          - 所有几何图形均视为闭集（边界接触或相切在碰撞判定中均归类为相交）。
 *          - 凸多边形障碍物顶点统一遵循逆时针 (CCW) 环绕序。
 *          - 数值计算采用保守策略：尺度相关的微小数值容差确保浮点舍入误差绝不漏判接触碰撞。
 */

#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "planner/math/aabb2d.hpp"
#include "planner/math/vec2d.hpp"

namespace planner::math {

/**
 * @name 凸多边形与分离轴定理 (SAT)
 * @{
 */

/**
 * @brief 计算多边形有向面积的两倍（逆时针顶点序为正，顺时针为负）。
 * @param[in] polygon 多边形顶点数组。
 * @return 有向面积的两倍（m^2）。
 */
double signed_area_twice(const std::vector<Point2d>& polygon) noexcept;

/**
 * @brief 校验多边形是否为严格凸多边形。
 * @param[in] polygon 顶点序列。
 * @param[out] reason 校验失败时的可选原因说明文本。
 * @return 若顶点数 >= 3、坐标有限、非共线退化且具备凸性则返回 true。
 */
bool convex_polygon_valid(const std::vector<Point2d>& polygon,
                          std::string* reason = nullptr);

/**
 * @brief 检查多边形顶点序列是否为严格逆时针 (CCW) 排列。
 * @param[in] polygon 顶点序列。
 * @return 若为逆时针返回 true，顺时针或退化则返回 false。
 */
bool is_counter_clockwise(const std::vector<Point2d>& polygon) noexcept;

/**
 * @brief 原地确保多边形顶点序列按逆时针方向排列（若为顺时针则翻转）。
 * @param[in,out] polygon 待调整的多边形。
 */
void normalize_counter_clockwise(std::vector<Point2d>& polygon);

/**
 * @brief 根据轴对齐矩形边界构造逆时针 4 顶点凸多边形。
 * @param[in] x_min X 轴下界。
 * @param[in] y_min Y 轴下界。
 * @param[in] x_max X 轴上界。
 * @param[in] y_max Y 轴上界。
 * @param[out] reason 构造失败时的诊断文本。
 * @return 包含 4 个 CCW 顶点的多边形。
 */
std::vector<Point2d> rectangle_polygon(double x_min, double y_min,
                                       double x_max, double y_max,
                                       std::string* reason = nullptr);

/**
 * @brief 采用单调链 (Monotone Chain / Andrew) 算法计算二维点集的逆时针凸包。
 * @param[in] points 输入点云。
 * @return 逆时针排列的凸包多边形顶点序列。
 */
std::vector<Point2d> convex_hull(std::vector<Point2d> points);

/**
 * @brief 基于分离轴定理 (SAT) 判定两闭凸多边形在指定容差下是否相交或接触。
 * @param[in] a 第一个凸多边形。
 * @param[in] b 第二个凸多边形。
 * @param[in] tol 接触判定距离容差（米）。
 * @return 若在容差范围内存在交集或接触则返回 true。
 */
bool convex_polygons_intersect(const std::vector<Point2d>& a,
                               const std::vector<Point2d>& b,
                               double tol = 0.0) noexcept;

/**
 * @brief 基于分离轴定理 (SAT) 判定凸多边形与 AABB 在指定容差下是否相交或接触。
 * @param[in] polygon 凸多边形。
 * @param[in] box 轴对齐包围盒。
 * @param[in] tol 接触判定距离容差（米）。
 * @return 若在容差范围内相交或接触则返回 true。
 */
bool convex_polygon_intersects_aabb(const std::vector<Point2d>& polygon,
                                    const Aabb2d& box,
                                    double tol = 0.0) noexcept;
/// @}

/**
 * @name 精确几何欧氏距离计算
 * @{
 */

/**
 * @brief 计算点到闭凸多边形的最小欧氏距离（点在内部或边界上时为 0）。
 * @param[in] p 待测点。
 * @param[in] polygon 凸多边形。
 * @return 最小欧氏距离（米）。
 */
double point_polygon_distance(const Point2d& p,
                              const std::vector<Point2d>& polygon) noexcept;

/**
 * @brief 计算点到闭凸多边形的最小欧氏距离的平方。
 * @param[in] p 待测点。
 * @param[in] polygon 凸多边形。
 * @return 平方欧氏距离（m^2）。
 */
double point_polygon_distance_sq(const Point2d& p,
                                 const std::vector<Point2d>& polygon) noexcept;

/**
 * @brief 计算两个闭凸多边形之间的最小欧氏距离（相交或接触时返回 0）。
 * @param[in] a 第一个凸多边形。
 * @param[in] b 第二个凸多边形。
 * @return 最小欧氏距离（米）。
 */
double convex_polygon_distance(const std::vector<Point2d>& a,
                               const std::vector<Point2d>& b) noexcept;

/**
 * @brief 计算两个闭凸多边形之间的最小欧氏距离的平方。
 * @param[in] a 第一个凸多边形。
 * @param[in] b 第二个凸多边形。
 * @return 平方距离（m^2）。
 */
double convex_polygon_distance_sq(const std::vector<Point2d>& a,
                                  const std::vector<Point2d>& b) noexcept;

/**
 * @brief 计算凸多边形与闭 AABB 之间的最小欧氏距离（相交或接触时返回 0）。
 * @param[in] polygon 凸多边形。
 * @param[in] box 目标 AABB。
 * @return 最小欧氏距离（米）。
 */
double polygon_aabb_distance(const std::vector<Point2d>& polygon,
                             const Aabb2d& box) noexcept;

/**
 * @brief 计算凸多边形与闭 AABB 之间的最小欧氏距离的平方。
 * @param[in] polygon 凸多边形。
 * @param[in] box 目标 AABB。
 * @return 平方距离（m^2）。
 */
double polygon_aabb_distance_sq(const std::vector<Point2d>& polygon,
                                const Aabb2d& box) noexcept;
/// @}

// 零动态分配的固定容量几何入口；n 必须不大于 8。
std::size_t convex_hull_small(std::array<Point2d, 8> points, std::size_t n,
                              std::array<Point2d, 8>& out) noexcept;

}  // namespace planner::math
