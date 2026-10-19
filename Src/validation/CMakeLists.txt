# planner_validation：优化后轨迹验证 V
# （Docs/component_development_plan.md 第 2、6.2 和 7.1 节）。
#
# 复用 planner::collision（位姿 + 重建运动的静态碰撞检测）和
# planner::corridor（整车走廊包含性）；验证本身独立于任何优化器。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_VALIDATION=ON）。下面的依赖回退
# 会在本目录以 PLANNER_BUILD_VALIDATION=ON 配置、而被复用组件的自身选项为 OFF
# 时启用这些可选组件，使 `task_runner.py build --component validation` 能在
# 全新构建目录中运行，且无需改动共享文件。

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  option(PLANNER_BUILD_TESTS "Build tests and register them with CTest" ON)
endif()
if(NOT DEFINED PLANNER_BUILD_EXAMPLES)
  option(PLANNER_BUILD_EXAMPLES "Build component examples" ON)
endif()

if(NOT TARGET planner::collision)
  set(PLANNER_BUILD_COLLISION ON CACHE BOOL "enabled as a planner_validation dependency" FORCE)
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/collision" "${CMAKE_BINARY_DIR}/Src/collision")
endif()
if(NOT TARGET planner::corridor)
  set(PLANNER_BUILD_CORRIDOR ON CACHE BOOL "enabled as a planner_validation dependency" FORCE)
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/corridor" "${CMAKE_BINARY_DIR}/Src/corridor")
endif()

add_library(planner_validation
  src/validation.cpp)

add_library(planner::validation ALIAS planner_validation)

target_compile_features(planner_validation PUBLIC cxx_std_17)
target_include_directories(planner_validation PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_validation PUBLIC
  planner::common
  planner::collision
  planner::corridor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_validation PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 命令行示例：读取一个预构建的验证请求 JSON 文档，运行验证器，
# 并把判定结果打印为 JSON 结论（PLANNER_BUILD_EXAMPLES 默认 ON）。
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_validation_example examples/validate_trajectory.cpp)
  target_link_libraries(planner_validation_example PRIVATE planner_validation)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_validation_example PRIVATE -Wall -Wextra -Wpedantic)
  endif()
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
