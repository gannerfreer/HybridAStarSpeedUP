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

# 判空金字塔层数（块边长 1、2、4…）。0 表示关闭该加速；默认 2 由实验
# （A1/A2/A3）选定。仅影响本组件库的编译定义。
#
# 注意：这里不能用 option()——CMake 的 BOOL cache 会把数值 2/3 归一化为
# OFF，只有 1 会变成 ON。层数是计数而非布尔量，故用 STRING cache。
set(PLANNER_COLLISION_PREFIX_LEVELS "2" CACHE STRING
  "Number of prefix-sum emptiness pyramid levels (block sizes 1,2,4,...; 0 disables)")
set_property(CACHE PLANNER_COLLISION_PREFIX_LEVELS PROPERTY STRINGS 0 1 2 3 4)
if(PLANNER_COLLISION_PREFIX_LEVELS GREATER 0)
  target_compile_definitions(planner_collision PRIVATE
    PLANNER_COLLISION_PREFIX_LEVELS=${PLANNER_COLLISION_PREFIX_LEVELS})
endif()

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
