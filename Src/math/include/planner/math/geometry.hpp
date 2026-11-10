/**
 * @file geometry.hpp
 * @brief 二维计算几何基元、分离轴定理 (SAT) 及精确欧氏距离查询算法。
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

#include "planner/math/pose2d.hpp"
#include "planner/math/vec2d.hpp"

namespace planner::math {

/**
 * @name 二维基础向量代数
 * @{
 */

/**
 * @brief 计算二维向量叉积标量（对应三维叉积的 Z 分量）：\f$ a_x b_y - a_y b_x \f$。
 * @param[in] a 第一个二维向量。
 * @param[in] b 第二个二维向量。
 * @return 叉积标量值。
 */
double cross(const Point2d& a, const Point2d& b) noexcept;

/**
 * @brief 计算二维向量点积标量：\f$ a_x b_x + a_y b_y \f$。
 * @param[in] a 第一个二维向量。
 * @param[in] b 第二个二维向量。
 * @return 点积标量值。
 */
double dot(const Point2d& a, const Point2d& b) noexcept;

Point2d operator+(const Point2d& a, const Point2d& b) noexcept;
Point2d operator-(const Point2d& a, const Point2d& b) noexcept;
Point2d operator*(const Point2d& a, double s) noexcept;

/**
 * @brief 计算二维点/向量的欧氏范数（模长）。
 * @param[in] a 输入二维向量。
 * @return 向量模长。
 */
double norm(const Point2d& a) noexcept;

/**
 * @brief 计算两二维点之间的欧氏距离。
 * @param[in] a 第一个点。
 * @param[in] b 第二个点。
 * @return 空间距离（米）。
 */
double distance(const Point2d& a, const Point2d& b) noexcept;

/**
 * @brief 根据车体位姿，将车体局部坐标系下的点变换至世界坐标系。
 * @param[in] pose 车体在世界系下的位姿。
 * @param[in] body_point 车体局部坐标点。
 * @return 变换后的世界坐标系点。
 */
Point2d transform_point(const Pose2d& pose, const Point2d& body_point) noexcept;

/**
 * @brief 将世界坐标系下的点反算至车体局部坐标系。
 * @param[in] pose 车体在世界系下的位姿。
 * @param[in] world_point 世界坐标点。
 * @return 车体局部坐标系下的点。
 */
Point2d to_body_frame(const Pose2d& pose, const Point2d& world_point) noexcept;
/// @}

/**
 * @name 尺度相关数值容差
 * @{
 */

/**
 * @brief 计算与几何尺度自适应匹配的接触判断数值容差。
 * @details 当两几何形状的最小欧氏距离小于等于 contact_tolerance(scale) 时，判定为接触碰撞。
 * @param[in] scale_m 几何坐标绝对值尺度（米）。
 * @return 接触判断的自适应容差（米，约为 1e-12 * scale 级别）。
 */
double contact_tolerance(double scale_m) noexcept;
/// @}

/**
 * @name 二维轴对齐包围盒 (AABB)
 * @{
 */

/**
 * @brief 二维轴对齐矩形包围盒。
 */
struct Aabb2d {
  double min_x{0.0};  ///< X 轴坐标下界（米）
  double min_y{0.0};  ///< Y 轴坐标下界（米）
  double max_x{0.0};  ///< X 轴坐标上界（米）
  double max_y{0.0};  ///< Y 轴坐标上界（米）

  /**
   * @brief 判定点 p 是否位于闭 AABB 内部或其边界上
   * @param[in] p 待测点
   * @return 若点在盒内或边上返回 true
   */
  bool contains(const Point2d& p) const noexcept;

  /**
   * @brief 判定两个闭 AABB 是否相交或接触（边界相碰判定为相交）
   * @param[in] other 另一个 AABB。
   * @return 若相交或接触返回 true。
   */
  bool intersects(const Aabb2d& other) const noexcept;

  /**
   * @brief 四周对称向外扩展包围盒边界。
   * @param[in] margin 向外扩充的外扩距离（米）
   */
  void expand(double margin) noexcept;

  /**
   * @brief 扩展本包围盒以紧密包络新增的点 p
   * @param[in] p 待合并的点
   */
  void merge_point(const Point2d& p) noexcept;

  /**
   * @brief 计算包围盒各角点坐标的最大绝对值尺度
   * @return 最大绝对值坐标（米）
   */
  double max_abs_coordinate() const noexcept;
};

/**
 * @brief 计算包含点集的紧凑二维轴对齐包围盒
 * @param[in] points 点集数组
 * @return 最小外包 Aabb2d
 */
Aabb2d aabb_of(const std::vector<Point2d>& points) noexcept;

/**
 * @brief 计算两个闭 AABB 之间的最小欧氏距离（重叠或接触时返回 0）
 * @param[in] a 第一个 AABB
 * @param[in] b 第二个 AABB
 * @return 最小欧氏距离（米）
 */
double aabb_distance(const Aabb2d& a, const Aabb2d& b) noexcept;

/**
 * @brief 平方距离域下的两 AABB 快速可达性粗筛。
 * @details 数学上完全等价于 `aabb_distance(a, b) <= reach`，但省去开方计算开销。
 * @param[in] a 第一个 AABB。
 * @param[in] b 第二个 AABB。
 * @param[in] reach 判定距离阈值（米）。
 * @return 若间距小于等于 reach 则返回 true。
 */
bool aabb_within(const Aabb2d& a, const Aabb2d& b, double reach) noexcept;

/**
 * @brief 计算点到闭 AABB 的最小欧氏距离（点在盒内或边界上时为 0）。
 * @param[in] p 待测点。
 * @param[in] box 目标 AABB。
 * @return 最小欧氏距离（米）。
 */
double point_aabb_distance(const Point2d& p, const Aabb2d& box) noexcept;

/**
 * @brief 计算点到闭 AABB 的最小欧氏距离的平方。
 * @param[in] p 待测点。
 * @param[in] box 目标 AABB。
 * @return 平方距离（m^2）。
 */
double point_aabb_distance_sq(const Point2d& p, const Aabb2d& box) noexcept;
/// @}

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
 * @brief 计算点到线段 [a, b] 的最小欧氏距离。
 * @param[in] p 待测点。
 * @param[in] a 线段起点。
 * @param[in] b 线段终点。
 * @return 最小欧氏距离（米）。
 */
double point_segment_distance(const Point2d& p, const Point2d& a,
                              const Point2d& b) noexcept;

/**
 * @brief 计算点到线段 [a, b] 的最小欧氏距离的平方。
 * @param[in] p 待测点。
 * @param[in] a 线段起点。
 * @param[in] b 线段终点。
 * @return 平方距离（m^2）。
 */
double point_segment_distance_sq(const Point2d& p, const Point2d& a,
                                 const Point2d& b) noexcept;

/**
 * @brief 计算两线段 [a0, a1] 与 [b0, b1] 之间的最小欧氏距离。
 * @param[in] a0 第一条线段起点。
 * @param[in] a1 第一条线段终点。
 * @param[in] b0 第二条线段起点。
 * @param[in] b1 第二条线段终点。
 * @return 最小欧氏距离（米）。
 */
double segment_segment_distance(const Point2d& a0, const Point2d& a1,
                                const Point2d& b0, const Point2d& b1) noexcept;

/**
 * @brief 计算两线段之间的最小欧氏距离的平方。
 * @param[in] a0 第一条线段起点。
 * @param[in] a1 第一条线段终点。
 * @param[in] b0 第二条线段起点。
 * @param[in] b1 第二条线段终点。
 * @return 平方距离（m^2）。
 */
double segment_segment_distance_sq(const Point2d& a0, const Point2d& a1,
                                   const Point2d& b0,
                                   const Point2d& b1) noexcept;

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
Aabb2d aabb_of_raw(const Point2d* points, std::size_t n) noexcept;
std::size_t convex_hull_small(std::array<Point2d, 8> points, std::size_t n,
                              std::array<Point2d, 8>& out) noexcept;
}  // namespace planner::math
