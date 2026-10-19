# planner_dftpav：隔离的实验性 Dftpav 轨迹优化器后端。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_DFTPAV=ON；
# components.json 条目还会启用 PLANNER_BUILD_CORRIDOR）。下面的回退只是让该
# 文件更易于独立使用；根项目提供 PLANNER_ROOT_DIR 和 GTest::gtest_main。
#
# Provenance and license:
#   The optimizer core under src/upstream/ is adapted from ZJU-FAST-Lab/Dftpav
#   (GPL-3.0) pinned at commit 87f63ad2f67506e090b36eac38f5d247c51a330e (the
#   exact commit/archive SHA256 recorded in dependencies.lock.json entry
#   `dftpav`). The upstream archive is NOT fetched at build time: only the
#   ported, ROS-free files live in this repository and no upstream CMake target
#   is consumed. See PROVENANCE.md for the per-file porting notes and
#   LICENSE-GPL-3.0.txt for the full license text.
#
# 依赖：
#   * planner::common、planner::corridor（公开的走廊/格式）以及
#     planner::validation + planner::collision（验证关卡及其消费的场景类型）；
#   * planner::hybrid_astar（仅测试/示例的端到端链路，库本身不用）；
#   * Eigen3（仅头文件，系统包；上游核心需要它）。
#     锁定参考：dependencies.lock.json 的 `eigen` 条目（3.4.0，曲线库对照后端消费）；
#     本组件保留系统 Eigen3（3.3/3.4 系列头文件）作为构建提供者，离线复现按锁
#     条目版本要求。没有可用的 Eigen3 时，构建会以清晰的消息失败。

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  option(PLANNER_BUILD_TESTS "Build tests and register them with CTest" ON)
endif()
if(NOT DEFINED PLANNER_BUILD_EXAMPLES)
  option(PLANNER_BUILD_EXAMPLES "Build component examples" ON)
endif()

# X-A1-06：示例/端到端测试的 Hybrid A* 链路独立开关（默认 ON，保持既有
# 行为）。OFF 时库、单元测试与示例都不引入 planner::hybrid_astar。
option(PLANNER_DFTPAV_WITH_E2E
  "Build the Hybrid A* end-to-end example/tests of the Dftpav backend" ON)

if(NOT TARGET planner::common)
  message(FATAL_ERROR
    "planner_dftpav requires the planner::common target. Configure through "
    "the root CMakeLists.txt (PLANNER_BUILD_COMMON=ON).")
endif()
if(NOT TARGET planner::corridor)
  message(FATAL_ERROR
    "planner_dftpav requires the planner::corridor target (the public Corridor "
    "type is consumed directly; components.json enables PLANNER_BUILD_CORRIDOR "
    "for this component).")
endif()

# 验证关卡（planner::validation）是公开结果契约的硬依赖：
# 轨迹只有在 validate_trajectory 接受后才有效。该回退仿照
# planner_validation 自身的依赖模式，使 `task_runner.py build --component dftpav`
# 能在全新构建目录中运行。
if(NOT TARGET planner::validation)
  if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/validation/CMakeLists.txt")
    message(FATAL_ERROR
      "planner_dftpav requires Src/validation (planner::validation), but "
      "${PLANNER_ROOT_DIR}/Src/validation/CMakeLists.txt does not exist.")
  endif()
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/validation"
    "${CMAKE_BINARY_DIR}/Src/validation")
endif()

# Hybrid A* 仅被测试/示例的端到端链路使用（库本身以普通 PathSegment 序列
# 作为输入）。X-A1-06：由 PLANNER_DFTPAV_WITH_E2E（默认 ON）控制；OFF 时
# 不引入该子目录，也不链接 planner::hybrid_astar。
if(PLANNER_DFTPAV_WITH_E2E AND NOT TARGET planner::hybrid_astar)
  if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/hybrid_astar/CMakeLists.txt")
    message(FATAL_ERROR
      "planner_dftpav tests/example require Src/hybrid_astar "
      "(planner::hybrid_astar), but its CMakeLists.txt does not exist.")
  endif()
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/hybrid_astar"
    "${CMAKE_BINARY_DIR}/Src/hybrid_astar")
endif()

# 系统 Eigen3（仅头文件）。移植后的上游核心基于 Eigen 3.3/3.4 编写，
# 不需要任何其他上游依赖。X-A1-07 依赖登记：本组件以系统 Eigen3 为构建
# 提供者（锁文件没有系统包条目）；dependencies.lock.json 的 `eigen` 条目
# （3.4.0）仅作版本参考，供曲线库对照后端消费。离线复现按锁条目版本要求。
find_package(Eigen3 QUIET)

# ---------------------------------------------------------------------------
# planner_dftpav 库。
# ---------------------------------------------------------------------------
add_library(planner_dftpav
  src/optimizer.cpp
  src/upstream/traj_optimizer.cpp)

add_library(planner::dftpav ALIAS planner_dftpav)

target_compile_features(planner_dftpav PUBLIC cxx_std_17)
target_include_directories(planner_dftpav
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/upstream")
target_link_libraries(planner_dftpav PUBLIC
  planner::common
  planner::corridor
  planner::collision
  planner::validation)

if(TARGET Eigen3::Eigen)
  target_link_libraries(planner_dftpav PRIVATE Eigen3::Eigen)
elseif(EIGEN3_INCLUDE_DIRS)
  target_include_directories(planner_dftpav PRIVATE ${EIGEN3_INCLUDE_DIRS})
else()
  message(FATAL_ERROR
    "planner_dftpav needs Eigen3 headers (upstream Dftpav core depends on "
    "Eigen). Install libeigen3-dev or provide Eigen3_DIR.")
endif()
if(EIGEN3_INCLUDE_DIR AND NOT EIGEN3_INCLUDE_DIRS)
  target_include_directories(planner_dftpav PRIVATE "${EIGEN3_INCLUDE_DIR}")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # 我们自己的适配器无警告；移植后的上游编译单元尽量与源文件保持逐字节接近，
  # 因而带有其历史警告，仅对该文件抑制这些警告。
  target_compile_options(planner_dftpav PRIVATE -Wall -Wextra -Wpedantic)
  set_source_files_properties(src/upstream/traj_optimizer.cpp
    PROPERTIES COMPILE_OPTIONS "-w")
endif()

# ---------------------------------------------------------------------------
# 命令行示例（端到端链路；PLANNER_DFTPAV_WITH_E2E + PLANNER_BUILD_EXAMPLES
# 默认 ON）。当 E2E 链路开启时还会注册为带 `dftpav` 标签的 CTest 自检。
# ---------------------------------------------------------------------------
if(PLANNER_DFTPAV_WITH_E2E AND PLANNER_BUILD_EXAMPLES)
  add_executable(planner_dftpav_example examples/dftpav_optimize_example.cpp)
  target_link_libraries(planner_dftpav_example PRIVATE
    planner_dftpav
    planner::hybrid_astar)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_dftpav_example PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  if(PLANNER_BUILD_TESTS)
    add_test(NAME planner_dftpav_example_self_check
      COMMAND planner_dftpav_example --self-check)
    set_tests_properties(planner_dftpav_example_self_check PROPERTIES LABELS "dftpav")
  endif()
endif()

# ---------------------------------------------------------------------------
# 测试（CTest 标签 "dftpav"）。
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
