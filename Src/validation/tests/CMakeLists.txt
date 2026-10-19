# planner_validation GoogleTest 测试套件。由 gtest_discover_tests 注册为带
# `validation` 标签的 CTest 测试；示例自检和随附的预构建轨迹文档作为两个额外
# 的带标签测试运行。
include(GoogleTest)

add_executable(planner_validation_tests
  test_valid_trajectories.cpp
  test_input_validation.cpp
  test_temporal_checks.cpp
  test_kinematic_limits.cpp
  test_endpoint_and_ordering.cpp
  test_corridor_containment.cpp
  test_collision_check.cpp)

target_link_libraries(planner_validation_tests
  PRIVATE planner_validation GTest::gtest_main)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_validation_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_validation_tests
  PROPERTIES LABELS "validation"
  DISCOVERY_TIMEOUT 30)

# 示例自检与预构建轨迹文档用例仅在示例目标存在时注册
# （PLANNER_BUILD_EXAMPLES 默认 ON）。
if(PLANNER_BUILD_EXAMPLES)
  add_test(NAME planner_validation_example_self_check
    COMMAND planner_validation_example --self-check)
  set_tests_properties(planner_validation_example_self_check PROPERTIES
    LABELS "validation")

  add_test(NAME planner_validation_example_valid_data
    COMMAND planner_validation_example
            "${CMAKE_CURRENT_SOURCE_DIR}/../examples/data/valid_open_road.json")
  set_tests_properties(planner_validation_example_valid_data PROPERTIES
    LABELS "validation")
endif()
