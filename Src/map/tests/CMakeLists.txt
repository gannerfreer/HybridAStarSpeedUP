# planner_map GoogleTest 测试套件。所有测试都带有 `map` 标签，
# 因此 `tools/task_runner.py test --component map` 可通过 ctest -L map 选中它们。

include(GoogleTest)

set(PLANNER_MAP_TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")
if(NOT EXISTS "${PLANNER_MAP_TEST_DATA_DIR}/mapping_example.osm")
  message(FATAL_ERROR
    "map test data is missing at ${PLANNER_MAP_TEST_DATA_DIR} "
    "(mapping_example.osm must exist; see tests/data/README.md).")
endif()

add_executable(planner_map_tests
  test_import.cpp
  test_projection.cpp
  test_query.cpp
  test_crop.cpp
  test_snapshot.cpp)

target_link_libraries(planner_map_tests PRIVATE planner_map GTest::gtest_main)
target_compile_definitions(planner_map_tests PRIVATE
  "PLANNER_MAP_TEST_DATA_DIR=\"${PLANNER_MAP_TEST_DATA_DIR}\"")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_map_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_map_tests
  PROPERTIES LABELS "map"
  DISCOVERY_TIMEOUT 60)

# HTTP 服务测试：在临时端口上启动真实的 planner_map_service 二进制，
# 并用 httplib 客户端查询每个端点。仅在 PLANNER_MAP_WITH_SERVICE=ON
# （默认）时构建：OFF 时没有服务二进制，也不链接 cpp-httplib（X-A1-08）。
if(PLANNER_MAP_WITH_SERVICE)
  add_executable(planner_map_service_tests test_service.cpp)
  target_link_libraries(planner_map_service_tests
    PRIVATE planner_map GTest::gtest_main httplib::httplib)
  target_compile_definitions(planner_map_service_tests PRIVATE
    "PLANNER_MAP_TEST_DATA_DIR=\"${PLANNER_MAP_TEST_DATA_DIR}\""
    "PLANNER_MAP_SERVICE_PATH=\"$<TARGET_FILE:planner_map_service>\"")
  add_dependencies(planner_map_service_tests planner_map_service)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_map_service_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  gtest_discover_tests(planner_map_service_tests
    PROPERTIES LABELS "map"
    DISCOVERY_TIMEOUT 60)

  # X-M1：栅格/场景预览端点的服务测试（真实二进制；协议 §3.2/§3.3/§8 语义、
  # v6 P1–P11 服务侧期望值）。仅在 scene 目标可用时构建（scene 关闭的缩减
  # 配置下两个端点返回 500，由组件 CMake 显式警告）。
  if(TARGET planner::scene)
    add_executable(planner_map_grid_service_tests test_service_grid.cpp)
    target_link_libraries(planner_map_grid_service_tests
      PRIVATE planner_map GTest::gtest_main httplib::httplib)
    target_compile_definitions(planner_map_grid_service_tests PRIVATE
      "PLANNER_MAP_TEST_DATA_DIR=\"${PLANNER_MAP_TEST_DATA_DIR}\""
      "PLANNER_MAP_SERVICE_PATH=\"$<TARGET_FILE:planner_map_service>\"")
    add_dependencies(planner_map_grid_service_tests planner_map_service)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(planner_map_grid_service_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    gtest_discover_tests(planner_map_grid_service_tests
      PROPERTIES LABELS "map"
      DISCOVERY_TIMEOUT 60)
  endif()
endif()

# 基准测试契约冒烟测试：确定性的小型合成地图，批次数量极小；
# 通过测试入口检验生成、加载、查询、裁剪、路由以及
# benchmark_metrics.csv / 图表 / 摘要输出。
# spec 优先级：v5（large 已物化并带 §9 归档）→ v4（规格自洽）→ v3 → v2 → v1（旧夹具回退）。
set(PLANNER_MAP_BENCH_SPEC "")
if(EXISTS "${PLANNER_ROOT_DIR}/tests/fixtures/v5/benchmarks/map/specs.json")
  set(PLANNER_MAP_BENCH_SPEC "${PLANNER_ROOT_DIR}/tests/fixtures/v5/benchmarks/map/specs.json")
elseif(EXISTS "${PLANNER_ROOT_DIR}/tests/fixtures/v4/benchmarks/map/specs.json")
  set(PLANNER_MAP_BENCH_SPEC "${PLANNER_ROOT_DIR}/tests/fixtures/v4/benchmarks/map/specs.json")
elseif(EXISTS "${PLANNER_ROOT_DIR}/tests/fixtures/v3/benchmarks/map/specs.json")
  set(PLANNER_MAP_BENCH_SPEC "${PLANNER_ROOT_DIR}/tests/fixtures/v3/benchmarks/map/specs.json")
elseif(EXISTS "${PLANNER_ROOT_DIR}/tests/fixtures/v2/benchmarks/map/specs.json")
  set(PLANNER_MAP_BENCH_SPEC "${PLANNER_ROOT_DIR}/tests/fixtures/v2/benchmarks/map/specs.json")
elseif(EXISTS "${PLANNER_ROOT_DIR}/tests/fixtures/v1/benchmarks/map/specs.json")
  set(PLANNER_MAP_BENCH_SPEC "${PLANNER_ROOT_DIR}/tests/fixtures/v1/benchmarks/map/specs.json")
endif()

set(_map_bench_smoke_args
  --suite smoke
  --output-dir "${CMAKE_CURRENT_BINARY_DIR}/bench_smoke"
  --scales small
  --warmup-batches 1
  --measured-batches 2
  --queries-per-batch 100
  --windows-per-batch 10
  --routes-per-batch 10
  --load-batches 2)
if(PLANNER_MAP_BENCH_SPEC)
  list(APPEND _map_bench_smoke_args --spec "${PLANNER_MAP_BENCH_SPEC}")
endif()

# 基准冒烟测试仅在 PLANNER_BUILD_BENCHMARKS=ON（默认）时注册，
# 与 planner_map_bench 目标的存在性保持一致。
if(PLANNER_BUILD_BENCHMARKS)
  add_test(NAME map_bench_smoke
    COMMAND planner_map_bench ${_map_bench_smoke_args})
  set_tests_properties(map_bench_smoke PROPERTIES
    LABELS "map"
    TIMEOUT 600
    WORKING_DIRECTORY "${PLANNER_ROOT_DIR}")
endif()

# 可行驶域栅格导出测试（W5 服务接入）。独立目标、纯追加：不改动上面的
# 既有测试目标与 bench smoke 注册。
add_executable(planner_map_driveable_grid_tests
  test_driveable_grid.cpp)
target_link_libraries(planner_map_driveable_grid_tests PRIVATE planner_map GTest::gtest_main)
target_compile_definitions(planner_map_driveable_grid_tests PRIVATE
  "PLANNER_MAP_TEST_DATA_DIR=\"${PLANNER_MAP_TEST_DATA_DIR}\"")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_map_driveable_grid_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_map_driveable_grid_tests
  PROPERTIES LABELS "map"
  DISCOVERY_TIMEOUT 60)
