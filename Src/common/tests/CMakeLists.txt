# planner_common GoogleTest 测试套件。由 gtest_discover_tests 注册为带
# `common` 标签（E0-1）的 CTest 测试。
include(GoogleTest)

set(PLANNER_COMMON_FIXTURES_DIR "${PLANNER_ROOT_DIR}/tests/fixtures/v1/serialization")
if(NOT EXISTS "${PLANNER_COMMON_FIXTURES_DIR}")
  message(FATAL_ERROR
    "Serialization fixtures are missing at ${PLANNER_COMMON_FIXTURES_DIR}; "
    "they are part of the planner_common contract.")
endif()

add_executable(planner_common_tests
  test_types.cpp
  test_serialization.cpp
  test_fixtures.cpp)

target_link_libraries(planner_common_tests PRIVATE planner_common GTest::gtest_main)

# 夹具驱动的测试通过配置时固化的绝对路径读取冻结的 v1 文件；
# 测试从不依赖工作目录。
target_compile_definitions(planner_common_tests PRIVATE
  "PLANNER_COMMON_FIXTURES_DIR=\"${PLANNER_COMMON_FIXTURES_DIR}\"")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_common_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_common_tests
  PROPERTIES LABELS "common"
  DISCOVERY_TIMEOUT 30)
