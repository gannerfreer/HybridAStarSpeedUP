# planner_polynomial GoogleTest 测试套件，注册为带 `polynomial` 标签的
# CTest 测试（tools/README.md 第 7.1 节要求标签与 components.json 的 id
# 一致）。
include(GoogleTest)

set(PLANNER_POLYNOMIAL_FIXTURES_DIR
  "${PLANNER_ROOT_DIR}/tests/fixtures/v1/benchmarks/polynomial")
if(NOT EXISTS "${PLANNER_POLYNOMIAL_FIXTURES_DIR}/inputs.json")
  message(FATAL_ERROR
    "Polynomial fixtures are missing at ${PLANNER_POLYNOMIAL_FIXTURES_DIR}; "
    "they are part of the v1 contract.")
endif()

# mav 对照套件仅在 PLANNER_POLYNOMIAL_WITH_MAV_COMPARE=ON（默认）时编译进
# 测试可执行文件并链接对照库；OFF 时主库测试保持完整（X-A1-05）。
set(PLANNER_POLYNOMIAL_TEST_SOURCES
  test_polynomial.cpp
  test_trajectory_segment.cpp
  test_piecewise_trajectory.cpp
  test_fixture_bench_inputs.cpp)

if(TARGET planner_polynomial_mav_compare)
  list(APPEND PLANNER_POLYNOMIAL_TEST_SOURCES test_mav_backend.cpp)
endif()

add_executable(planner_polynomial_tests ${PLANNER_POLYNOMIAL_TEST_SOURCES})

target_link_libraries(planner_polynomial_tests PRIVATE planner_polynomial
  GTest::gtest_main)

if(TARGET planner_polynomial_mav_compare)
  target_link_libraries(planner_polynomial_tests PRIVATE
    planner_polynomial_mav_compare)
endif()

# 夹具驱动的测试通过配置时固化的绝对路径读取冻结的 v1 文件；
# 测试从不依赖工作目录。
target_compile_definitions(planner_polynomial_tests PRIVATE
  "PLANNER_POLYNOMIAL_FIXTURES_DIR=\"${PLANNER_POLYNOMIAL_FIXTURES_DIR}\"")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_polynomial_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_polynomial_tests
  PROPERTIES LABELS "polynomial"
  DISCOVERY_TIMEOUT 30)
