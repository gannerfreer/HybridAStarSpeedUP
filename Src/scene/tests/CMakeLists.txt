# planner_scene GoogleTest 测试套件。所有测试带有 `scene` 标签，
# 因此 `tools/task_runner.py test --component scene` 可通过 ctest -L scene
# 选中它们。夹具为只读的 v3 mapping_example 地图。

include(GoogleTest)

set(PLANNER_SCENE_TEST_MAP
  "${PLANNER_ROOT_DIR}/tests/fixtures/v3/map/mapping_example.osm")
if(NOT EXISTS "${PLANNER_SCENE_TEST_MAP}")
  message(FATAL_ERROR
    "Scene test map is missing at ${PLANNER_SCENE_TEST_MAP} "
    "(tests/fixtures/v3/map/mapping_example.osm).")
endif()

add_executable(planner_scene_tests
  test_scene.cpp)

target_link_libraries(planner_scene_tests PRIVATE planner_scene GTest::gtest_main)
target_compile_definitions(planner_scene_tests PRIVATE
  "PLANNER_SCENE_TEST_MAP_PATH=\"${PLANNER_SCENE_TEST_MAP}\""
  "PLANNER_SCENE_TEST_ORIGIN_LAT=\"49.00345654351\""
  "PLANNER_SCENE_TEST_ORIGIN_LON=\"8.42427590707\"")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_scene_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_scene_tests
  PROPERTIES LABELS "scene"
  DISCOVERY_TIMEOUT 60)
