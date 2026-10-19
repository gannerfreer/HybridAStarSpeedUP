#pragma once

// 安全走廊组件的小型 2D 凸几何辅助函数。
//
// 约定：
//   * 除函数名另有说明外，多边形顶点序列均按逆时针（CCW）顺序。
//   * 半平面为 {p : dot(normal, p) <= offset}，法向量为单位长度。
//   * 谓词接受以米为单位的绝对容差；数据为 1 量级（米），因此绝对容差
//     是合适的。
//
// 这些辅助函数只依赖 C++17 标准库和 planner::common::Point2d，由生成器、
// 走廊验证器与组件测试共用；验证器用它们对生成的多边形做独立几何检查。

#include <vector>

#include "planner/common/types.hpp"

namespace planner::corridor {

// 方向、包含与分离谓词的绝对容差。
inline constexpr double kGeometryEps = 1e-9;

// 半平面 {p : dot(normal, p) <= offset}；`normal` 应为单位长度。
struct HalfPlane {
  common::Point2d normal;
  double offset{0.0};
};

double dot(const common::Point2d& a, const common::Point2d& b);
double cross(const common::Point2d& a, const common::Point2d& b);
common::Point2d add(const common::Point2d& a, const common::Point2d& b);
common::Point2d subtract(const common::Point2d& a, const common::Point2d& b);
common::Point2d multiply(const common::Point2d& a, double scalar);
double norm(const common::Point2d& a);
double distance(const common::Point2d& a, const common::Point2d& b);
bool is_finite(const common::Point2d& p);

// 闭合环的鞋带公式有符号面积；逆时针顺序时为正。
double polygon_signed_area(const std::vector<common::Point2d>& polygon);

// 当 `polygon` 有限、严格凸、逆时针、至少三个顶点、面积为正且没有
// 重复/零长度边时返回 true。
bool is_convex_ccw_polygon(const std::vector<common::Point2d>& polygon,
                           double eps = kGeometryEps);

// 输入点集的凸包（Andrew 单调链）。返回无共线点、无重复点的严格凸 CCW
// 多边形；输入退化（共线、为空或仅一个点）时返回少于三个点。
std::vector<common::Point2d> convex_hull(std::vector<common::Point2d> points,
                                         double eps = kGeometryEps);

// 点在凸多边形内的测试；在 `eps` 范围内边界视为内部。
bool point_in_convex_polygon_ccw(const common::Point2d& point,
                                 const std::vector<common::Point2d>& polygon_ccw,
                                 double eps = kGeometryEps);

// 两个凸多边形距离的保守下界：为正值表示沿某一支撑轴至少相隔该间隙；
// 为零或负表示多边形接触或重叠。输入退化时返回 -infinity，调用方据此
// 视为相交。
double convex_polygons_gap(const std::vector<common::Point2d>& a_ccw,
                           const std::vector<common::Point2d>& b_ccw);

// 当 `inner_ccw` 的每个顶点都在 `outer_ccw` 内（`eps` 内允许落在边界）
// 时返回 true；对凸的 `inner_ccw` 足够。
bool convex_polygon_contains(const std::vector<common::Point2d>& outer_ccw,
                             const std::vector<common::Point2d>& inner_ccw,
                             double eps = kGeometryEps);

// 两个凸 CCW 多边形的交集面积；不重叠时为 0。
double convex_polygon_intersection_area(const std::vector<common::Point2d>& a_ccw,
                                        const std::vector<common::Point2d>& b_ccw);

// 凸 CCW 多边形的最小卡尺宽度；输入退化时为 +infinity。包含宽度为 W 的
// 矩形的凸多边形，其最小宽度至少为 W。
double convex_polygon_min_width(const std::vector<common::Point2d>& polygon_ccw);

// 凸 CCW 多边形的外向半平面（每条非退化边对应一个）。
std::vector<HalfPlane> polygon_half_planes(const std::vector<common::Point2d>& polygon_ccw);

// 半平面求交，从必须包含结果的边界多边形开始裁剪。返回严格凸 CCW
// 多边形；交集为空或退化时返回 false。
bool intersect_half_planes(const std::vector<HalfPlane>& half_planes,
                           const std::vector<common::Point2d>& bounds_ccw,
                           std::vector<common::Point2d>* out);

// 将凸 CCW 多边形的每条边向外偏移 `offset_m`。结果包含输入与半径为
// `offset_m` 的圆盘的闵可夫斯基和。输入退化或结果为空时返回 false。
bool offset_convex_polygon(const std::vector<common::Point2d>& polygon_ccw,
                           double offset_m, std::vector<common::Point2d>* out);

}  // namespace planner::corridor
