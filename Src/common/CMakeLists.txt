add_library(planner_common src/types.cpp)
add_library(planner::common ALIAS planner_common)
target_compile_features(planner_common PUBLIC cxx_std_17)
target_include_directories(planner_common PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
