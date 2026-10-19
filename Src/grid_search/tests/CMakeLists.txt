# planner_grid_search 测试（CTest 标签 "grid_search"；正确性档，性能属 X-B1）。
#
# 夹具：tests/fixtures/{scenarios.json,expected_costs.json} 由独立 Python 参考
# tests/reference/grid_reference.py 确定性生成；*_fixture_check 测试负责复现
# 校验（需要 CPython 解释器；找不到解释器时跳过并打印说明，不静默通过）。

add_library(grid_search_test_support STATIC scenarios.cpp)
target_include_directories(grid_search_test_support PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(grid_search_test_support PUBLIC
  planner_grid_search GTest::gtest_main)
target_compile_features(grid_search_test_support PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(grid_search_test_support PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_compile_definitions(grid_search_test_support PUBLIC
  PLANNER_GRID_SEARCH_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")

set(_grid_search_tests
  test_neighborhood
  test_cost_vs_reference
  test_distance_field
  test_status_and_budget
  test_configs_and_single_list)

foreach(_test IN LISTS _grid_search_tests)
  add_executable(planner_grid_search_${_test} ${_test}.cpp)
  target_link_libraries(planner_grid_search_${_test}
    PRIVATE grid_search_test_support)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_grid_search_${_test}
      PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  add_test(NAME planner_grid_search_${_test}
    COMMAND planner_grid_search_${_test})
  set_tests_properties(planner_grid_search_${_test} PROPERTIES
    LABELS "grid_search")
endforeach()

# 独立 Python 参考复现校验（标准库；不依赖 gtest）。
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_test(NAME planner_grid_search_scenarios_fixture_check
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/reference/grid_reference.py"
      --check-scenarios "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/scenarios.json")
  set_tests_properties(planner_grid_search_scenarios_fixture_check PROPERTIES
    LABELS "grid_search")

  add_test(NAME planner_grid_search_expected_fixture_check
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/reference/grid_reference.py"
      --check-expected "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/expected_costs.json"
      --scenarios "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/scenarios.json")
  set_tests_properties(planner_grid_search_expected_fixture_check PROPERTIES
    LABELS "grid_search")
else()
  message(STATUS
    "Python3 interpreter not found: the grid_search fixture reproduction "
    "tests are not registered (record this in the report).")
endif()
