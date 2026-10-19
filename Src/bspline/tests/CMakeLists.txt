# planner_bspline GoogleTest 测试套件。注册时使用 `bspline` CTest 标签，
# 以便 `test --component bspline`（ctest -L bspline）能够选中它们。

include(GoogleTest)

set(PLANNER_BSPLINE_BENCH_INPUTS
    "${PLANNER_ROOT_DIR}/tests/fixtures/v1/benchmarks/bspline/inputs.json")
if(NOT EXISTS "${PLANNER_BSPLINE_BENCH_INPUTS}")
  message(FATAL_ERROR
    "B-spline benchmark fixture is missing at ${PLANNER_BSPLINE_BENCH_INPUTS}; "
    "it is part of the v1 contract.")
endif()

# EGO 对照套件仅在 PLANNER_BSPLINE_WITH_EGO=ON（默认）时编译进测试可执行
# 文件并链接 GPL-3.0 隔离库；OFF 时主库测试保持完整（X-A1-03）。
set(PLANNER_BSPLINE_TEST_SOURCES
  test_construction.cpp
  test_evaluate.cpp
  test_continuity.cpp
  test_sample.cpp
  test_arc_length.cpp
  test_backends.cpp
  test_fixture_inputs.cpp)

if(TARGET planner_bspline_ego)
  list(APPEND PLANNER_BSPLINE_TEST_SOURCES test_backend_ego.cpp)
endif()

add_executable(planner_bspline_tests ${PLANNER_BSPLINE_TEST_SOURCES})

target_link_libraries(planner_bspline_tests PRIVATE
  planner_bspline
  GTest::gtest_main)

if(TARGET planner_bspline_ego)
  target_link_libraries(planner_bspline_tests PRIVATE planner_bspline_ego)
endif()

target_compile_definitions(planner_bspline_tests PRIVATE
  "PLANNER_BSPLINE_BENCH_INPUTS=\"${PLANNER_BSPLINE_BENCH_INPUTS}\"")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_bspline_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_bspline_tests
  PROPERTIES LABELS "bspline"
  DISCOVERY_TIMEOUT 30)
