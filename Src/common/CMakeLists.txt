add_library(planner_common
  src/status.cpp
  src/path.cpp
  src/vehicle.cpp
  src/grid_view.cpp
  src/planner_config.cpp)
add_library(planner::common ALIAS planner_common)
target_compile_features(planner_common PUBLIC cxx_std_17)
target_link_libraries(planner_common PUBLIC planner::math)
target_include_directories(planner_common PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
