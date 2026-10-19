# planner_collision：面向规划核心的矩形足迹碰撞检测
# （Doc/component_development_plan.md §6.1/§6.2/§7.1）。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_COLLISION=ON）。
# 下面的回退只是让该文件更易于被其他父项目使用；根项目提供
# PLANNER_ROOT_DIR，并从 dependencies.lock.json 获取
# nlohmann_json / GoogleTest。

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  option(PLANNER_BUILD_TESTS "Build tests and register them with CTest" ON)
endif()
if(NOT DEFINED PLANNER_BUILD_EXAMPLES)
  option(PLANNER_BUILD_EXAMPLES "Build component examples" ON)
endif()
if(NOT DEFINED PLANNER_BUILD_BENCHMARKS)
  option(PLANNER_BUILD_BENCHMARKS "Build component benchmarks" ON)
endif()

add_library(planner_collision
  src/geometry.cpp
  src/scene.cpp
  src/collision_checker.cpp
  src/reference.cpp
  src/differential.cpp
  src/differential_strata.cpp)

add_library(planner::collision ALIAS planner_collision)

target_compile_features(planner_collision PUBLIC cxx_std_17)
target_include_directories(planner_collision PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_collision PUBLIC planner::common)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_collision PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 命令行示例：场景检查器和随机差分运行器（差分运行器同时支撑
# tests/CMakeLists.txt 中的 CTest 集成用例）。PLANNER_BUILD_EXAMPLES
# 默认 ON；OFF 时相关 CTest 用例一并跳过。
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_collision_check_scene examples/check_scene.cpp)
  target_link_libraries(planner_collision_check_scene PRIVATE planner_collision)
  target_compile_options(planner_collision_check_scene PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(planner_collision_differential examples/random_differential.cpp)
  target_link_libraries(planner_collision_differential PRIVATE planner_collision)
  target_compile_options(planner_collision_differential PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 固定批次微基准测试二进制（PLANNER_BUILD_BENCHMARKS 默认 ON）。
if(PLANNER_BUILD_BENCHMARKS)
  add_executable(planner_collision_bench examples/collision_bench.cpp)
  target_link_libraries(planner_collision_bench PRIVATE planner_collision)
  target_compile_options(planner_collision_bench PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(PLANNER_BUILD_TESTS)
  if(NOT TARGET GTest::gtest_main)
    message(FATAL_ERROR
      "PLANNER_BUILD_TESTS=ON but GTest::gtest_main is not available. "
      "Configure through the root CMakeLists.txt, which fetches googletest "
      "from dependencies.lock.json.")
  endif()
  add_subdirectory(tests)
endif()
