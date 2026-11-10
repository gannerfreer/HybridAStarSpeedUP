add_library(planner_hybrid_astar
  src/hybrid_astar.cpp
  src/user_astar.cpp
  src/grid_potential.cpp)
add_library(planner::hybrid_astar ALIAS planner_hybrid_astar)
target_link_libraries(planner_hybrid_astar PUBLIC planner::collision planner::rs planner::common planner::math)
target_include_directories(planner_hybrid_astar PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
