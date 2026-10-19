// planner_common 类型的单元测试，不涉及 JSON 编解码器。

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "planner/common/errors.hpp"
#include "planner/common/types.hpp"

namespace planner::common {
namespace {

TEST(NormalizeAngleRad, leaves_canonical_values_unchanged) {
  EXPECT_EQ(normalize_angle_rad(0.0), 0.0);
  EXPECT_EQ(normalize_angle_rad(0.5), 0.5);
  EXPECT_EQ(normalize_angle_rad(-0.5), -0.5);
  EXPECT_EQ(normalize_angle_rad(-kPi), -kPi);
  EXPECT_EQ(normalize_angle_rad(kPi / 2.0), kPi / 2.0);
}

TEST(NormalizeAngleRad, wraps_positive_half_turn_to_lower_bound) {
  EXPECT_EQ(normalize_angle_rad(kPi), -kPi);
}

TEST(NormalizeAngleRad, wraps_angles_outside_the_range) {
  EXPECT_NEAR(normalize_angle_rad(1.5 * kPi), -0.5 * kPi, 1e-12);
  EXPECT_NEAR(normalize_angle_rad(2.5 * kPi), 0.5 * kPi, 1e-12);
  EXPECT_NEAR(normalize_angle_rad(-1.5 * kPi), 0.5 * kPi, 1e-12);
  EXPECT_NEAR(normalize_angle_rad(-2.5 * kPi), -0.5 * kPi, 1e-12);
  EXPECT_NEAR(normalize_angle_rad(4.0 * kPi), 0.0, 1e-12);
}

TEST(NormalizeAngleRad, result_stays_inside_half_open_range) {
  const double inputs[] = {kPi, -kPi, 3.0 * kPi, -3.0 * kPi, 100.0 * kPi, -100.0 * kPi, 1e6, -1e6, 0.25};
  for (double input : inputs) {
    const double wrapped = normalize_angle_rad(input);
    EXPECT_GE(wrapped, -kPi) << "input: " << input;
    EXPECT_LT(wrapped, kPi) << "input: " << input;
  }
}

TEST(VehicleModel, derives_minimum_turn_radius_from_wheelbase_and_steer) {
  VehicleModel vehicle;
  vehicle.id = "test";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  const double expected = vehicle.wheelbase_m / std::tan(vehicle.max_steer_rad);
  EXPECT_DOUBLE_EQ(vehicle.min_turn_radius_m(), expected);
  EXPECT_GT(vehicle.min_turn_radius_m(), vehicle.wheelbase_m);
}

TEST(GridView, indexes_cells_row_major_and_checks_bounds) {
  GridView grid;
  grid.map_id = "grid";
  grid.map_version = "v1";
  grid.width = 3;
  grid.height = 2;
  grid.resolution_m = 0.5;
  grid.origin = Pose2d{1.0, 2.0, 0.0};
  grid.cells = {0, 1, 2, 0, 0, 1};

  EXPECT_TRUE(grid.in_bounds(0, 0));
  EXPECT_TRUE(grid.in_bounds(1, 2));
  EXPECT_FALSE(grid.in_bounds(2, 0));
  EXPECT_FALSE(grid.in_bounds(0, 3));
  EXPECT_FALSE(grid.in_bounds(-1, 0));

  EXPECT_EQ(grid.cell(0, 1), kCellOccupied);
  EXPECT_EQ(grid.cell(0, 2), kCellUnknown);
  EXPECT_EQ(grid.cell(1, 2), kCellOccupied);
  EXPECT_THROW(grid.cell(2, 0), std::out_of_range);
  EXPECT_THROW(grid.cell(0, -1), std::out_of_range);
}

TEST(Enums, map_to_canonical_strings) {
  EXPECT_EQ(to_string(Direction::kForward), "forward");
  EXPECT_EQ(to_string(Direction::kReverse), "reverse");
  EXPECT_EQ(to_string(Status::kSuccess), "success");
  EXPECT_EQ(to_string(Status::kInvalidInput), "invalid_input");
  EXPECT_EQ(to_string(Status::kNoPath), "no_path");
  EXPECT_EQ(to_string(Status::kTimeout), "timeout");
  EXPECT_EQ(to_string(Status::kCancelled), "cancelled");
  EXPECT_EQ(to_string(Status::kOptimizationFailed), "optimization_failed");
  EXPECT_EQ(to_string(Status::kValidationFailed), "validation_failed");
}

TEST(ErrorCodes, map_to_stable_names) {
  EXPECT_STREQ(to_string(ErrorCode::kParseError), "parse_error");
  EXPECT_STREQ(to_string(ErrorCode::kInvalidDocument), "invalid_document");
  EXPECT_STREQ(to_string(ErrorCode::kMissingField), "missing_field");
  EXPECT_STREQ(to_string(ErrorCode::kTypeError), "type_error");
  EXPECT_STREQ(to_string(ErrorCode::kUnsupportedSchemaVersion), "unsupported_schema_version");
  EXPECT_STREQ(to_string(ErrorCode::kUnknownType), "unknown_type");
  EXPECT_STREQ(to_string(ErrorCode::kInvalidValue), "invalid_value");
  EXPECT_STREQ(to_string(ErrorCode::kUnknownEnumValue), "unknown_enum_value");
  EXPECT_STREQ(to_string(ErrorCode::kNonFiniteValue), "non_finite_value");
}

TEST(ValueEquality, compares_every_field) {
  EXPECT_EQ((Pose2d{1.0, 2.0, 0.5}), (Pose2d{1.0, 2.0, 0.5}));
  EXPECT_TRUE((Pose2d{1.0, 2.0, 0.5}) == (Pose2d{1.0, 2.0, 0.5}));
  EXPECT_FALSE((Pose2d{1.0, 2.0, 0.5}) == (Pose2d{1.0, 2.0, 0.25}));
  EXPECT_FALSE((Pose2d{1.0, 2.0, 0.5}) == (Pose2d{1.0, 1.0, 0.5}));
  EXPECT_FALSE((Point2d{1.0, 2.0}) == (Point2d{2.0, 1.0}));
}

TEST(VehicleModelEquality, compares_all_parameters) {
  VehicleModel lhs;
  lhs.id = "vehicle";
  lhs.wheelbase_m = 2.8;
  lhs.width_m = 1.8;
  lhs.front_overhang_m = 0.9;
  lhs.rear_overhang_m = 1.0;
  lhs.max_steer_rad = 0.5;
  lhs.safety_margin_m = 0.1;
  VehicleModel rhs = lhs;
  EXPECT_TRUE(lhs == rhs);
  rhs.safety_margin_m = 0.2;
  EXPECT_FALSE(lhs == rhs);
}

}  // namespace
}  // namespace planner::common
