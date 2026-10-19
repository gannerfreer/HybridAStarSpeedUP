#pragma once

// 碰撞组件使用的小型 2D 几何工具箱。
//
// 约定要点（见 Docs/component_development_plan.md §6.1/§6.2）：
//   * 所有形状都是闭集：接触（相切）计为相交；
//   * 凸多边形障碍物按逆时针存储；
//   * 数值判定一律保守：比较时在“视为重叠”的方向上使用极小的尺度相关
//     容差，使舍入只会增加误报，绝不会掩盖接触。
//
// 本头文件不依赖 ROS 和 Lanelet2，只依赖 planner/common 类型。

#include <string>
#include <vector>

#include "planner/common/types.hpp"

namespace planner::collision {

using planner::common::Direction;
using planner::common::Point2d;
using planner::common::Pose2d;

// ---------------------------------------------------------------------------
// 基础向量代数
// ---------------------------------------------------------------------------

double cross(const Point2d& a, const Point2d& b) noexcept;
double dot(const Point2d& a, const Point2d& b) noexcept;

Point2d operator+(const Point2d& a, const Point2d& b) noexcept;
Point2d operator-(const Point2d& a, const Point2d& b) noexcept;
Point2d operator*(const Point2d& a, double s) noexcept;

double norm(const Point2d& a) noexcept;
double distance(const Point2d& a, const Point2d& b) noexcept;

// 将车体坐标系中的点按 pose.yaw 旋转并平移到世界坐标。
Point2d transform_point(const Pose2d& pose, const Point2d& body_point) noexcept;

// 逆变换：世界点表示在给定位姿的车体坐标系中。
Point2d to_body_frame(const Pose2d& pose, const Point2d& world_point) noexcept;

// ---------------------------------------------------------------------------
// 容差
// ---------------------------------------------------------------------------

// 尺度相关数值容差：低于该容差的两个形状视为接触。该公式是组件约定的
// 组成部分：生产检查器与独立参考实现使用同一策略，使接触样本得到一致
// 分类。`scale_m` 是具体输入的坐标量级（米）；结果最大为 1e-12 * scale。
//
// 接触策略：两形状接触当且仅当其最小*欧氏*距离 <=
// contact_tolerance(scale)。两种实现都显式计算该距离；容差并不按分离轴
// 或边线分别施加，因此角点附近的构型会得到一致的分类。
double contact_tolerance(double scale_m) noexcept;

// ---------------------------------------------------------------------------
// 轴对齐包围盒
// ---------------------------------------------------------------------------

struct Aabb2d {
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};

  bool contains(const Point2d& p) const noexcept;
  bool intersects(const Aabb2d& other) const noexcept;
  void expand(double margin) noexcept;
  void merge_point(const Point2d& p) noexcept;
  double max_abs_coordinate() const noexcept;
};

Aabb2d aabb_of(const std::vector<Point2d>& points) noexcept;

// 两个轴对齐包围盒之间的欧氏距离（相交时为 0）。
double aabb_distance(const Aabb2d& a, const Aabb2d& b) noexcept;

// 点到闭轴对齐包围盒的距离（在内部/边界上为 0）。
double point_aabb_distance(const Point2d& p, const Aabb2d& box) noexcept;

// ---------------------------------------------------------------------------
// 凸多边形
// ---------------------------------------------------------------------------

// 带符号面积的两倍（逆时针顺序为正）。
double signed_area_twice(const std::vector<Point2d>& polygon) noexcept;

// 当多边形具有 >= 3 个两两不同的顶点、坐标有限、面积非零且为凸时，
// 可作为障碍物使用（允许共线顶点）。`reason` 可选地接收简短诊断信息。
bool convex_polygon_valid(const std::vector<Point2d>& polygon,
                          std::string* reason = nullptr);

// 多边形已是逆时针时返回 true；退化多边形返回 false。
bool is_counter_clockwise(const std::vector<Point2d>& polygon) noexcept;

// 原地操作：多边形为顺时针时反转顶点顺序。
void normalize_counter_clockwise(std::vector<Point2d>& polygon);

// 由矩形构造的凸多边形，逆时针。
std::vector<Point2d> rectangle_polygon(double x_min, double y_min,
                                       double x_max, double y_max,
                                       std::string* reason = nullptr);

// 点集的凸包（单调链）。退化输入返回共线点；少于 3 个非共线点时得到
// 退化多边形，会被 `convex_polygon_valid` 拒绝。
std::vector<Point2d> convex_hull(std::vector<Point2d> points);

// 两个凸多边形的闭集相交测试（分离轴）。`tol` 范围内的接触计为相交。
bool convex_polygons_intersect(const std::vector<Point2d>& a,
                               const std::vector<Point2d>& b,
                               double tol = 0.0) noexcept;

// 凸多边形与轴对齐包围盒的闭集相交测试（分离轴：包围盒法线加多边形
// 边法线）。
bool convex_polygon_intersects_aabb(const std::vector<Point2d>& polygon,
                                    const Aabb2d& box,
                                    double tol = 0.0) noexcept;

// 点到闭凸多边形的距离（在内部/边界上为 0）。退化（零面积）多边形按
// 线段/点集处理。
double point_polygon_distance(const Point2d& p,
                              const std::vector<Point2d>& polygon) noexcept;

double segment_segment_distance(const Point2d& a0, const Point2d& a1,
                                const Point2d& b0, const Point2d& b1) noexcept;

// 两个闭凸多边形之间的距离（相交或相切时为 0）。适用于不相交的凸多边
// 形；退化多边形按其顶点/边集合处理。
double convex_polygon_distance(const std::vector<Point2d>& a,
                               const std::vector<Point2d>& b) noexcept;

}  // namespace planner::collision
