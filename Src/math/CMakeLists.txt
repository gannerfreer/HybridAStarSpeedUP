add_library(planner_math
  src/vec2d.cpp
  src/pose2d.cpp
  src/segment2d.cpp
  src/aabb2d.cpp
  src/box2d.cpp
  src/polygon2d.cpp
  src/angle.cpp
  src/kinematics/bicycle_model.cpp)
add_library(planner::math ALIAS planner_math)
target_include_directories(planner_math PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
