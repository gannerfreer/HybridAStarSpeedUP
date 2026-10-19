# planner_hybrid_astar GoogleTest 测试套件以及示例自检。
# 全部注册为带 `hybrid_astar` CTest 标签的测试。

include(GoogleTest)

# 父组件 CMakeLists 会设置夹具目录；保留一个回退以便独立使用。
if(NOT DEFINED PLANNER_HYBRID_ASTAR_FIXTURES_DIR)
  set(PLANNER_HYBRID_ASTAR_FIXTURES_DIR "${PLANNER_ROOT_DIR}/tests/fixtures/v2")
endif()
if(NOT EXISTS "${PLANNER_HYBRID_ASTAR_FIXTURES_DIR}/paths/cases.json")
  message(FATAL_ERROR
    "Path fixtures are missing at "
    "${PLANNER_HYBRID_ASTAR_FIXTURES_DIR}/paths/cases.json")
endif()

add_executable(planner_hybrid_astar_tests
  test_contract.cpp
  test_cross_component.cpp
  test_fixture_cases.cpp
  test_path_quality.cpp
  test_upstream_backends.cpp)

target_link_libraries(planner_hybrid_astar_tests PRIVATE
  planner_hybrid_astar
  GTest::gtest_main)

target_compile_definitions(planner_hybrid_astar_tests PRIVATE
  "PLANNER_HYBRID_ASTAR_FIXTURES_DIR=\"${PLANNER_HYBRID_ASTAR_FIXTURES_DIR}\"")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_hybrid_astar_tests
    PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_hybrid_astar_tests
  PROPERTIES LABELS "hybrid_astar"
  DISCOVERY_TIMEOUT 30)

# 示例自检仅在示例目标存在时注册（PLANNER_BUILD_EXAMPLES 默认 ON）。
if(PLANNER_BUILD_EXAMPLES)
  add_test(NAME planner_hybrid_astar_example_self_check
    COMMAND planner_hybrid_astar_example --self-check)
  set_tests_properties(planner_hybrid_astar_example_self_check
    PROPERTIES LABELS "hybrid_astar")
endif()
