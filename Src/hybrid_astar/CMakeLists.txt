# planner_hybrid_astar：Hybrid A* 适配器框架和第一个归一化后端
# （Wave 4，Docs/component_development_plan.md §6.2/§6.3）。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_HYBRID_ASTAR=ON）。
# 注册条目（tools/components.json）只启用本组件，因此当依赖组件的目标尚不可用
# 时会在此处引入它们（当各自的 PLANNER_BUILD_<ID> 选项为 ON 时，根项目会先
# 添加它们）。不获取任何上游（Smac / KTH / zm0612）代码：归一化后端不依赖
# ROS，使用 planner::rs 和 planner::collision。

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

# ---------------------------------------------------------------------------
# 依赖组件：planner::rs（RS 距离 + 解析连接）和
# planner::collision（整车位姿/运动/路径检查）。
# ---------------------------------------------------------------------------
if(NOT TARGET planner::rs)
  if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/rs/CMakeLists.txt")
    message(FATAL_ERROR
      "planner_hybrid_astar requires Src/rs (planner::rs), but "
      "${PLANNER_ROOT_DIR}/Src/rs/CMakeLists.txt does not exist.")
  endif()
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/rs" "${CMAKE_BINARY_DIR}/Src/rs")
endif()
if(NOT TARGET planner::collision)
  if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/collision/CMakeLists.txt")
    message(FATAL_ERROR
      "planner_hybrid_astar requires Src/collision (planner::collision), but "
      "${PLANNER_ROOT_DIR}/Src/collision/CMakeLists.txt does not exist.")
  endif()
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/collision"
    "${CMAKE_BINARY_DIR}/Src/collision")
endif()

add_library(planner_hybrid_astar
  src/backend.cpp
  src/grid_potential.cpp
  src/ros_free_backend.cpp
  src/rs_provider.cpp
  src/user_astar_backend.cpp)

# ---------------------------------------------------------------------------
# 用户自研上游（vendored；来源与改写见 upstream/user_astar/PROVENANCE.md）。
# 仅编译搜索核心源；上游代码不参与本组件的告警门槛。
# ---------------------------------------------------------------------------
add_library(planner_user_astar_upstream STATIC
  upstream/user_astar/src/hybrid_astar.cpp
  upstream/user_astar/src/node2d.cpp)
target_compile_features(planner_user_astar_upstream PUBLIC cxx_std_17)
target_include_directories(planner_user_astar_upstream PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/upstream/user_astar/include>")

add_library(planner::hybrid_astar ALIAS planner_hybrid_astar)

target_compile_features(planner_hybrid_astar PUBLIC cxx_std_17)
target_include_directories(planner_hybrid_astar PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_hybrid_astar PUBLIC
  planner::common
  planner::rs
  planner::collision)
target_link_libraries(planner_hybrid_astar PRIVATE planner_user_astar_upstream)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_hybrid_astar PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 命令行示例：固定墙壁场景，打印规划摘要/统计信息；
# --self-check 注册到 CTest（标签 hybrid_astar）。PLANNER_BUILD_EXAMPLES
# 默认 ON；OFF 时示例与自检测试都不构建。
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_hybrid_astar_example examples/plan_scene_example.cpp)
  target_link_libraries(planner_hybrid_astar_example PRIVATE planner_hybrid_astar)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_hybrid_astar_example
      PRIVATE -Wall -Wextra -Wpedantic)
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

# ---------------------------------------------------------------------------
# 测试和基准测试使用的夹具（v2 修订集）。
# ---------------------------------------------------------------------------
set(PLANNER_HYBRID_ASTAR_FIXTURES_DIR "${PLANNER_ROOT_DIR}/tests/fixtures/v2"
  CACHE PATH "Versioned path fixtures consumed by tests and benchmarks")
if(NOT EXISTS "${PLANNER_HYBRID_ASTAR_FIXTURES_DIR}/paths/cases.json")
  message(FATAL_ERROR
    "Path fixtures are missing at "
    "${PLANNER_HYBRID_ASTAR_FIXTURES_DIR}/paths/cases.json")
endif()

# ---------------------------------------------------------------------------
# 基准测试：planner_hybrid_astar_bench --suite v1 --output-dir <dir>
# （tools/components.json 的 benchmark_suites 条目；指标契约见
# tools/README.md §7.2）。tests/ 目录位于包含路径中，
# 供共享夹具加载器（fixture_scene.hpp）使用。PLANNER_BUILD_BENCHMARKS
# 默认 ON；OFF 时基准与冒烟用例都不构建。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_BENCHMARKS)
  find_package(Threads REQUIRED)
  add_executable(planner_hybrid_astar_bench
    benchmarks/hybrid_astar_bench.cpp)
  target_link_libraries(planner_hybrid_astar_bench PRIVATE planner_hybrid_astar
    Threads::Threads)
  target_include_directories(planner_hybrid_astar_bench PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/tests")
  target_compile_definitions(planner_hybrid_astar_bench PRIVATE
    "PLANNER_HYBRID_ASTAR_FIXTURES_DIR=\"${PLANNER_HYBRID_ASTAR_FIXTURES_DIR}\"")
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_hybrid_astar_bench
      PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  if(PLANNER_BUILD_TESTS)
    # 基准测试契约的冒烟自检（两个场景、两次试验）。
    add_test(NAME planner_hybrid_astar_bench_smoke
      COMMAND planner_hybrid_astar_bench --suite v1 --smoke
        --output-dir "${CMAKE_CURRENT_BINARY_DIR}/bench_smoke")
    set_tests_properties(planner_hybrid_astar_bench_smoke
      PROPERTIES LABELS "hybrid_astar")

    # v3 停车套件的冒烟自检（3 个场景 × 4 个 normalized 后端，冷启动单次；
    # 夹具 v8 与论文车辆参数）。
    add_test(NAME planner_hybrid_astar_bench_parking_smoke
      COMMAND planner_hybrid_astar_bench --suite v3 --smoke --cold-only
        --fixtures-dir "${PLANNER_ROOT_DIR}/tests/fixtures/v8"
        --vehicle parking_bench.json
        --output-dir "${CMAKE_CURRENT_BINARY_DIR}/bench_smoke_v3")
    set_tests_properties(planner_hybrid_astar_bench_parking_smoke
      PROPERTIES LABELS "hybrid_astar")

    # v4 ParkBench 套件的冒烟自检（3 个场景 × 4 个后端，冷启动单次；
    # 夹具 v9 与 ParkBench 论文车辆参数）。
    add_test(NAME planner_hybrid_astar_bench_parkbench_smoke
      COMMAND planner_hybrid_astar_bench --suite v4 --smoke --cold-only
        --fixtures-dir "${PLANNER_ROOT_DIR}/tests/fixtures/v9"
        --vehicle parkbench_vehicle.json
        --output-dir "${CMAKE_CURRENT_BINARY_DIR}/bench_smoke_v4")
    set_tests_properties(planner_hybrid_astar_bench_parkbench_smoke
      PROPERTIES LABELS "hybrid_astar")
  endif()
endif()
