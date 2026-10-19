# planner_corridor GoogleTest 测试套件。由 gtest_discover_tests 注册为带
# `corridor` 标签的 CTest 测试；示例自检作为额外的一个带标签测试运行。
include(GoogleTest)

add_executable(planner_corridor_tests
  test_corridor_generation.cpp
  test_corridor_verification.cpp
  test_geometry.cpp)

target_link_libraries(planner_corridor_tests PRIVATE planner_corridor GTest::gtest_main)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_corridor_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_corridor_tests
  PROPERTIES LABELS "corridor"
  DISCOVERY_TIMEOUT 30)

# 示例自检仅在示例目标存在时注册（PLANNER_BUILD_EXAMPLES 默认 ON）。
if(PLANNER_BUILD_EXAMPLES)
  add_test(NAME planner_corridor_example_self_check
    COMMAND planner_corridor_example --self-check)
  set_tests_properties(planner_corridor_example_self_check PROPERTIES LABELS "corridor")
endif()
