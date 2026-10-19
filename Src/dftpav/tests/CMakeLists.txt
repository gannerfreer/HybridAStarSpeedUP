# planner_dftpav GoogleTest 测试套件。由 gtest_discover_tests 注册为带
# `dftpav` 标签的 CTest 测试；示例自检作为额外的一个带标签测试运行。
#
# 端到端链路测试使用共享的 v2 路径夹具（Hybrid A* 用例 + 默认车辆）
# 以及 Hybrid A* 的 ros_free 后端。X-A1-06：仅在
# PLANNER_DFTPAV_WITH_E2E=ON（默认）时编译并链接 planner::hybrid_astar；
# OFF 时其余单元测试保持完整。

include(GoogleTest)

set(PLANNER_DFTPAV_FIXTURES_DIR "${PLANNER_ROOT_DIR}/tests/fixtures/v2")
if(NOT EXISTS "${PLANNER_DFTPAV_FIXTURES_DIR}/paths/cases.json")
  message(FATAL_ERROR
    "Path fixtures are missing at ${PLANNER_DFTPAV_FIXTURES_DIR}/paths/cases.json")
endif()

set(PLANNER_DFTPAV_TEST_SOURCES
  test_straight_line.cpp
  test_corridor_avoidance.cpp
  test_direction_change.cpp
  test_failures.cpp
  test_cancellation.cpp)

if(PLANNER_DFTPAV_WITH_E2E AND TARGET planner::hybrid_astar)
  list(APPEND PLANNER_DFTPAV_TEST_SOURCES test_e2e_chain.cpp)
endif()

add_executable(planner_dftpav_tests ${PLANNER_DFTPAV_TEST_SOURCES})

target_include_directories(planner_dftpav_tests PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(planner_dftpav_tests PRIVATE
  "PLANNER_DFTPAV_FIXTURES_DIR=\"${PLANNER_DFTPAV_FIXTURES_DIR}\"")
target_link_libraries(planner_dftpav_tests PRIVATE
  planner_dftpav GTest::gtest_main)

if(PLANNER_DFTPAV_WITH_E2E AND TARGET planner::hybrid_astar)
  target_link_libraries(planner_dftpav_tests PRIVATE planner::hybrid_astar)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_dftpav_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_dftpav_tests
  PROPERTIES LABELS "dftpav"
  DISCOVERY_TIMEOUT 30)
