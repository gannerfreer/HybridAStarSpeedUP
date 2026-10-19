# planner_grid GoogleTest 测试套件。所有测试带有 `grid` 标签，
# 因此 `tools/task_runner.py test --component grid` 可通过 ctest -L grid
# 选中它们。

include(GoogleTest)

add_executable(planner_grid_tests
  test_grid.cpp)

target_link_libraries(planner_grid_tests PRIVATE planner_grid GTest::gtest_main)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_grid_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_grid_tests
  PROPERTIES LABELS "grid"
  DISCOVERY_TIMEOUT 30)
