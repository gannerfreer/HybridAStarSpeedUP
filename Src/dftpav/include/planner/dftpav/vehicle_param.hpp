#pragma once

// ===========================================================================
// PROVENANCE / 来源说明
// Adapted from ZJU-FAST-Lab/Dftpav (GPL-3.0), see Src/dftpav/PROVENANCE.md:
//   * src/Sim/core/common/inc/common/basics/semantics.h  (class VehicleParam)
// Pinned upstream commit: dependencies.lock.json entry `dftpav`.
//
// Porting changes: the upstream `common::VehicleParam` class is reduced to the
// accessors used by the extracted optimizer core and moved into the
// `planner::dftpav` namespace; no ROS, no `common/basics/*` headers.
// `max_steering_angle_` keeps the upstream unit (degrees) to avoid silently
// changing the semantics of ported code; the adapter converts from radians.
// ===========================================================================

#include <cmath>

namespace planner::dftpav {

// 抽取出的 Dftpav 优化器使用的车辆几何（"T" 形）。
//
// 优化器使用的整车外壳是宽 `width()`、长 `length()` 的矩形，中心位于后轴
// 前方 `d_cr()` 处（MINCO/Bezier 轨迹的平坦输出是后轴中心）。适配器传入的
// 宽/长已包含配置的外壳余量。
class VehicleParam {
 public:
  double width() const { return width_; }
  double length() const { return length_; }
  double wheel_base() const { return wheel_base_; }
  double front_suspension() const { return front_suspension_; }
  double rear_suspension() const { return rear_suspension_; }
  double max_steering_angle() const { return max_steering_angle_; }
  double d_cr() const { return d_cr_; }

  void set_width(double val) { width_ = val; }
  void set_length(double val) { length_ = val; }
  void set_wheel_base(double val) { wheel_base_ = val; }
  void set_front_suspension(double val) { front_suspension_ = val; }
  void set_rear_suspension(double val) { rear_suspension_ = val; }
  void set_max_steering_angle(double val) { max_steering_angle_ = val; }
  void set_d_cr(double val) { d_cr_ = val; }

 private:
  // 默认值刻意与上游类默认值一致；适配器始终用公开的 VehicleModel 覆盖它们。
  double width_ = 1.90;
  double length_ = 4.88;
  double wheel_base_ = 2.85;
  double front_suspension_ = 0.93;
  double rear_suspension_ = 1.10;
  double max_steering_angle_ = 45.0;  // 度，上游约定
  double d_cr_ = 1.015;  // 几何中心到后轴的距离 [m]
};

}  // namespace planner::dftpav
