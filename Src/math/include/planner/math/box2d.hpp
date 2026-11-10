/**
 * @file box2d.hpp
 * @brief 带航向角的二维有向包围盒 (OBB) 及其相交/距离查询。
 * @details 所有几何查询按闭集语义处理：边界接触或相切均视为相交。
 *          构造时缓存航向角正余弦与半长/半宽，避免热路径重复三角函数计算。
 */

#pragma once

#include <array>
#include <cmath>

#include "planner/math/aabb2d.hpp"
#include "planner/math/vec2d.hpp"

namespace planner::math {

/**
 * @brief 二维有向包围盒（中心、全长、全宽与航向角）。
 */
class Box2d {
 public:
  /**
   * @brief 构造零尺寸、航向角为 0 的包围盒。
   */
  Box2d() = default;

  /**
   * @brief 由几何参数构造包围盒并缓存派生的正余弦与半轴长度。
   * @param[in] center 包围盒几何中心。
   * @param[in] length_m 沿局部 X 轴的完整长度（米）。
   * @param[in] width_m 沿局部 Y 轴的完整宽度（米）。
   * @param[in] heading_rad 局部 X 轴相对世界 X 轴的航向角（弧度）。
   */
  Box2d(const Vec2d& center, double length_m, double width_m,
        double heading_rad) noexcept;

  /**
   * @brief 由几何参数与已预计算的正余弦构造包围盒，避免重复三角函数计算。
   * @param[in] center 包围盒几何中心。
   * @param[in] length_m 沿局部 X 轴的完整长度（米）。
   * @param[in] width_m 沿局部 Y 轴的完整宽度（米）。
   * @param[in] heading_rad 局部 X 轴相对世界 X 轴的航向角（弧度）。
   * @param[in] cos_heading 航向角余弦缓存。
   * @param[in] sin_heading 航向角正弦缓存。
   */
  Box2d(const Vec2d& center, double length_m, double width_m,
        double heading_rad, double cos_heading,
        double sin_heading) noexcept
      : center_(center),
        length_m_(length_m),
        width_m_(width_m),
        heading_rad_(heading_rad),
        cos_heading_(cos_heading),
        sin_heading_(sin_heading),
        half_length_m_(0.5 * length_m),
        half_width_m_(0.5 * width_m),
        corner_origin_(center),
        local_corners_{{{-half_length_m_, -half_width_m_},
                        {half_length_m_, -half_width_m_},
                        {half_length_m_, half_width_m_},
                        {-half_length_m_, half_width_m_}}} {}

  /**
   * @brief 由几何参数、预计算正余弦及显式局部角点构造包围盒。
   * @details 局部角点按给定顺序相对 corner_origin 存放，供需要逐位保持原始
   *          坐标变换顺序的调用方（如车辆足迹）使用；length/width/center
   *          仍描述包围盒的几何语义。
   * @param[in] center 包围盒几何中心。
   * @param[in] length_m 沿局部 X 轴的完整长度（米）。
   * @param[in] width_m 沿局部 Y 轴的完整宽度（米）。
   * @param[in] heading_rad 局部 X 轴相对世界 X 轴的航向角（弧度）。
   * @param[in] cos_heading 航向角余弦缓存。
   * @param[in] sin_heading 航向角正弦缓存。
   * @param[in] corner_origin 局部角点坐标系原点在世界系下的位置。
   * @param[in] local_corners 相对 corner_origin 的逆时针局部角点。
   */
  Box2d(const Vec2d& center, double length_m, double width_m,
        double heading_rad, double cos_heading, double sin_heading,
        const Vec2d& corner_origin,
        const std::array<Point2d, 4>& local_corners) noexcept
      : center_(center),
        length_m_(length_m),
        width_m_(width_m),
        heading_rad_(heading_rad),
        cos_heading_(cos_heading),
        sin_heading_(sin_heading),
        half_length_m_(0.5 * length_m),
        half_width_m_(0.5 * width_m),
        corner_origin_(corner_origin),
        local_corners_(local_corners) {}

  /**
   * @brief 获取包围盒几何中心。
   * @return 中心点坐标。
   */
  const Vec2d& center() const noexcept { return center_; }

  /**
   * @brief 获取沿局部 X 轴的完整长度。
   * @return 长度（米）。
   */
  double length_m() const noexcept { return length_m_; }

  /**
   * @brief 获取沿局部 Y 轴的完整宽度。
   * @return 宽度（米）。
   */
  double width_m() const noexcept { return width_m_; }

  /**
   * @brief 获取航向角。
   * @return 航向角（弧度）。
   */
  double heading_rad() const noexcept { return heading_rad_; }

  /**
   * @brief 计算按逆时针排列的 4 个世界系角点：
   *        (-hl,-hw), (+hl,-hw), (+hl,+hw), (-hl,+hw)。
   * @return 包含 4 个世界系角点的定长数组。
   */
  std::array<Point2d, 4> corners() const noexcept {
    const double c = cos_heading_;
    const double s = sin_heading_;
    std::array<Point2d, 4> out;
    for (std::size_t i = 0; i < 4; ++i) {
      out[i] = Point2d{
          corner_origin_.x_m + c * local_corners_[i].x_m -
              s * local_corners_[i].y_m,
          corner_origin_.y_m + s * local_corners_[i].x_m +
              c * local_corners_[i].y_m};
    }
    return out;
  }

  /**
   * @brief 计算包围盒的紧密轴对齐包围盒。
   * @return 对应 AABB。
   */
  Aabb2d aabb() const noexcept {
    const std::array<Point2d, 4> c = corners();
    return aabb_of_raw(c.data(), c.size());
  }

  /**
   * @brief 判定点是否位于包围盒内部或边界上（可附加距离容差）。
   * @param[in] p 待测点。
   * @param[in] tol 接触判定距离容差（米）。
   * @return 若点在盒内、边界上或容差范围内返回 true。
   */
  [[nodiscard]] bool contains(const Point2d& p,
                              double tol = 0.0) const noexcept {
    const double dx = p.x_m - center_.x_m;
    const double dy = p.y_m - center_.y_m;
    const double local_x = cos_heading_ * dx + sin_heading_ * dy;
    const double local_y = -sin_heading_ * dx + cos_heading_ * dy;
    return std::abs(local_x) <= half_length_m_ + tol &&
           std::abs(local_y) <= half_width_m_ + tol;
  }

  /**
   * @brief 判定包围盒与闭 AABB 是否相交或接触。
   * @param[in] box 目标 AABB。
   * @param[in] tol 接触判定距离容差（米）。
   * @return 若相交或接触返回 true。
   */
  [[nodiscard]] bool intersects(const Aabb2d& box,
                                double tol = 0.0) const noexcept;

  /**
   * @brief 判定两个有向包围盒是否相交或接触。
   * @param[in] other 另一个包围盒。
   * @param[in] tol 接触判定距离容差（米）。
   * @return 若相交或接触返回 true。
   */
  [[nodiscard]] bool intersects(const Box2d& other,
                                double tol = 0.0) const noexcept;

  /**
   * @brief 计算点到闭包围盒的最小欧氏距离平方（点在内部或边界上时为 0）。
   * @param[in] p 待测点。
   * @return 平方欧氏距离（m^2）。
   */
  double distance_sq_to(const Point2d& p) const noexcept;

 private:
  Vec2d center_{};              ///< 包围盒几何中心。
  double length_m_{0.0};        ///< 沿局部 X 轴的完整长度（米）。
  double width_m_{0.0};         ///< 沿局部 Y 轴的完整宽度（米）。
  double heading_rad_{0.0};     ///< 航向角（弧度）。
  double cos_heading_{1.0};     ///< 航向角余弦缓存。
  double sin_heading_{0.0};     ///< 航向角正弦缓存。
  double half_length_m_{0.0};   ///< 半长缓存（米）。
  double half_width_m_{0.0};    ///< 半宽缓存（米）。
  Vec2d corner_origin_{};       ///< 局部角点坐标系原点。
  std::array<Point2d, 4> local_corners_{{{0.0, 0.0},
                                         {0.0, 0.0},
                                         {0.0, 0.0},
                                         {0.0, 0.0}}};  ///< 逆时针局部角点。
};

}  // namespace planner::math
