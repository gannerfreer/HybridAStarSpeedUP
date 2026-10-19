# planner_grid：通用栅格数学与校验（X-A1-17）。
#
# 依赖边界（冻结计划 §2.1）：只依赖 planner::common（GridView 类型按冻结
# 补充暂留 common）；不暴露 Lanelet2/上游类型；不运行搜索。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_GRID=ON，默认 ON）。

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  option(PLANNER_BUILD_TESTS "Build tests and register them with CTest" ON)
endif()

if(NOT TARGET planner::common)
  message(FATAL_ERROR
    "planner_grid requires the planner::common target. Configure through the "
    "root CMakeLists.txt (PLANNER_BUILD_COMMON=ON).")
endif()

add_library(planner_grid
  src/grid.cpp)

add_library(planner::grid ALIAS planner_grid)

target_compile_features(planner_grid PUBLIC cxx_std_17)
target_include_directories(planner_grid PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_grid PUBLIC planner::common)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_grid PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 测试（GoogleTest + CTest，标签 "grid"）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_TESTS)
  if(NOT TARGET GTest::gtest_main)
    message(FATAL_ERROR
      "PLANNER_BUILD_TESTS=ON but GTest::gtest_main is not available. "
      "Configure through the root CMakeLists.txt, which fetches googletest "
      "from dependencies.lock.json.")
  endif()
  add_subdirectory(tests)
endif()
