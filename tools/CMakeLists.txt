add_executable(plan_demo plan_demo/main.cpp)
target_link_libraries(plan_demo
  PRIVATE planner::config planner::collision planner::hybrid_astar)
planner_set_warnings(plan_demo)
