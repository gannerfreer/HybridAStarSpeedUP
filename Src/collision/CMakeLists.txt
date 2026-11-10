add_library(planner_collision
  src/scene.cpp
  src/collision_checker.cpp)
add_library(planner::collision ALIAS planner_collision)
target_link_libraries(planner_collision PUBLIC planner::math planner::common)
target_include_directories(planner_collision PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
