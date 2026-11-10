add_library(planner_rs src/reeds_shepp.cpp src/path_sampling.cpp)
add_library(planner::rs ALIAS planner_rs)
target_link_libraries(planner_rs PUBLIC planner::math planner::common)
target_include_directories(planner_rs PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
