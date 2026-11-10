add_library(planner_config
  src/toml_config_loader.cpp)
add_library(planner::config ALIAS planner_config)
target_link_libraries(planner_config
  PUBLIC planner::common
  PRIVATE tomlplusplus::tomlplusplus)
target_include_directories(planner_config PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
