add_library(planner_math src/geometry.cpp src/angle.cpp)
add_library(planner::math ALIAS planner_math)
target_link_libraries(planner_math PUBLIC planner::common)
target_include_directories(planner_math PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
