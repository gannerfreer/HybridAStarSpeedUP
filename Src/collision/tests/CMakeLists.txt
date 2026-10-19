# planner_collision GoogleTest 测试套件和差分运行器集成测试。所有测试都带有
# `collision` 标签，因此 `tools/task_runner.py test --component collision`
# 能够选中它们。
include(GoogleTest)

set(PLANNER_COLLISION_FIXTURES_DIR "${PLANNER_ROOT_DIR}/tests/fixtures/v1")
if(NOT EXISTS "${PLANNER_COLLISION_FIXTURES_DIR}/collision/random_spec.json")
  message(FATAL_ERROR
    "Collision fixtures are missing at ${PLANNER_COLLISION_FIXTURES_DIR}/collision; "
    "they are part of the v1 test contract.")
endif()

add_executable(planner_collision_tests
  test_geometry.cpp
  test_check_pose.cpp
  test_check_motion.cpp
  test_check_path.cpp
  test_invalid_input.cpp
  test_reference.cpp
  test_acceleration_equivalence.cpp
  test_differential_smoke.cpp
  test_regression_acceptance.cpp)

target_link_libraries(planner_collision_tests PRIVATE planner_collision GTest::gtest_main)
target_compile_definitions(planner_collision_tests PRIVATE
  "PLANNER_COLLISION_FIXTURES_DIR=\"${PLANNER_COLLISION_FIXTURES_DIR}\"")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_collision_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_collision_tests
  PROPERTIES LABELS "collision"
  DISCOVERY_TIMEOUT 30)

# 随机差分运行器：快速冒烟集和完整开发回归（10,000 个可判定用例，
# 按规范校验，逐用例比较加速度等价性）。证据保存在
# artifacts/collision_differential/。差分运行器属于示例目标
# （PLANNER_BUILD_EXAMPLES，默认 ON）；OFF 时这些用例一并跳过。
if(PLANNER_BUILD_EXAMPLES)
  add_test(NAME collision_differential_smoke
    COMMAND planner_collision_differential --regime smoke --case-target 300
            --accel both --quiet
            --output-dir "${CMAKE_CURRENT_BINARY_DIR}/differential_smoke")
  set_tests_properties(collision_differential_smoke PROPERTIES
    LABELS "collision"
    TIMEOUT 600
    WORKING_DIRECTORY "${PLANNER_ROOT_DIR}")

  # 一个验收种子：检验 random_spec.json 的验收验证
  # （逐种子计数与整个 regime 的聚合值对比）以及验收分配。
  add_test(NAME collision_differential_acceptance_seed_smoke
    COMMAND planner_collision_differential --regime acceptance --seed 7
            --accel both --quiet
            --spec "${PLANNER_COLLISION_FIXTURES_DIR}/collision/random_spec.json"
            --output-dir "${CMAKE_CURRENT_BINARY_DIR}/acceptance_smoke")
  set_tests_properties(collision_differential_acceptance_seed_smoke PROPERTIES
    LABELS "collision"
    TIMEOUT 600
    WORKING_DIRECTORY "${PLANNER_ROOT_DIR}")

  add_test(NAME collision_random_differential_dev
    COMMAND planner_collision_differential --regime dev --accel both
            --spec "${PLANNER_COLLISION_FIXTURES_DIR}/collision/random_spec.json")
  set_tests_properties(collision_random_differential_dev PROPERTIES
    LABELS "collision"
    TIMEOUT 3600
    WORKING_DIRECTORY "${PLANNER_ROOT_DIR}")
endif()
