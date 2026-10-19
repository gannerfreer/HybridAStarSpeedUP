# planner_service GoogleTest 测试套件。所有测试都带有 `service` 标签，
# 因此 `tools/task_runner.py test --component service` 可通过
# ctest -L service 选中它们。

include(GoogleTest)

set(PLANNER_SERVICE_TEST_MAP
  "${PLANNER_ROOT_DIR}/tests/fixtures/v3/map/mapping_example.osm")
if(NOT EXISTS "${PLANNER_SERVICE_TEST_MAP}")
  message(FATAL_ERROR
    "Service test map is missing at ${PLANNER_SERVICE_TEST_MAP} "
    "(tests/fixtures/v3/map/mapping_example.osm).")
endif()

set(PLANNER_SERVICE_TEST_VEHICLE
  "${PLANNER_ROOT_DIR}/tests/fixtures/v3/vehicles/default.json")
if(NOT EXISTS "${PLANNER_SERVICE_TEST_VEHICLE}")
  message(FATAL_ERROR
    "Service test vehicle is missing at ${PLANNER_SERVICE_TEST_VEHICLE}.")
endif()

# 合成小窗口地图（场景单元测试用；见 tests/data/window_map.osm）。
set(PLANNER_SERVICE_TEST_WINDOW_MAP
  "${CMAKE_CURRENT_SOURCE_DIR}/data/window_map.osm")
if(NOT EXISTS "${PLANNER_SERVICE_TEST_WINDOW_MAP}")
  message(FATAL_ERROR
    "Service window-map fixture is missing at ${PLANNER_SERVICE_TEST_WINDOW_MAP}.")
endif()

# ---------------------------------------------------------------------------
# 单元测试：请求解析器和本地场景构建器（不启动服务进程）。
# 场景测试使用真实 v3 地图（可行驶域）与合成小地图（窗口裁剪）。
# ---------------------------------------------------------------------------
add_executable(planner_service_tests
  test_parse.cpp
  test_scene.cpp
  test_snapshot_store.cpp
  test_scene_preview.cpp
  test_pipeline_cancel.cpp)
target_link_libraries(planner_service_tests PRIVATE planner_service GTest::gtest_main)
target_compile_definitions(planner_service_tests PRIVATE
  "PLANNER_SERVICE_TEST_VEHICLE_PATH=\"${PLANNER_SERVICE_TEST_VEHICLE}\""
  "PLANNER_SERVICE_TEST_MAP_PATH=\"${PLANNER_SERVICE_TEST_MAP}\""
  "PLANNER_SERVICE_TEST_WINDOW_MAP_PATH=\"${PLANNER_SERVICE_TEST_WINDOW_MAP}\""
  "PLANNER_SERVICE_TEST_ORIGIN_LAT=\"49.00345654351\""
  "PLANNER_SERVICE_TEST_ORIGIN_LON=\"8.42427590707\""
  "PLANNER_SERVICE_TEST_WINDOW_ORIGIN_LAT=\"49.0\""
  "PLANNER_SERVICE_TEST_WINDOW_ORIGIN_LON=\"8.4\"")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_service_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_service_tests
  PROPERTIES LABELS "service"
  DISCOVERY_TIMEOUT 60)

# ---------------------------------------------------------------------------
# 集成测试：在临时端口上启动真实服务二进制，
# 检验 HTTP 契约以及 WebSocket 事件流。
# 仅在 PLANNER_SERVICE_WITH_HTTP=ON（默认）时构建：OFF 时没有服务二进制，
# 也不链接 cpp-httplib（X-A1-14）。
# ---------------------------------------------------------------------------
if(PLANNER_SERVICE_WITH_HTTP)
  add_executable(planner_plan_service_tests
    test_service_http.cpp
    test_service_events.cpp)
  target_link_libraries(planner_plan_service_tests PRIVATE
    planner_service GTest::gtest_main httplib::httplib)
  target_compile_definitions(planner_plan_service_tests PRIVATE
    "PLANNER_PLAN_SERVICE_PATH=\"$<TARGET_FILE:planner_plan_service>\""
    "PLANNER_SERVICE_TEST_MAP_PATH=\"${PLANNER_SERVICE_TEST_MAP}\""
    "PLANNER_SERVICE_TEST_ORIGIN_LAT=\"49.00345654351\""
    "PLANNER_SERVICE_TEST_ORIGIN_LON=\"8.42427590707\"")
  add_dependencies(planner_plan_service_tests planner_plan_service)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_plan_service_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  gtest_discover_tests(planner_plan_service_tests
    PROPERTIES LABELS "service" TIMEOUT 900
    DISCOVERY_TIMEOUT 60)

  # X-M1：场景标识/输入快照集成测试；跨服务一致性用例同时启动地图服务
  # （planner_map_service），因此仅在地图服务目标可用时构建。
  if(TARGET planner_map_service)
    add_executable(planner_service_scene_tests
      test_service_scene.cpp)
    target_link_libraries(planner_service_scene_tests PRIVATE
      planner_service GTest::gtest_main httplib::httplib)
    target_compile_definitions(planner_service_scene_tests PRIVATE
      "PLANNER_PLAN_SERVICE_PATH=\"$<TARGET_FILE:planner_plan_service>\""
      "PLANNER_MAP_SERVICE_PATH=\"$<TARGET_FILE:planner_map_service>\""
      "PLANNER_SERVICE_TEST_MAP_PATH=\"${PLANNER_SERVICE_TEST_MAP}\""
      "PLANNER_SERVICE_TEST_ORIGIN_LAT=\"49.00345654351\""
      "PLANNER_SERVICE_TEST_ORIGIN_LON=\"8.42427590707\"")
    add_dependencies(planner_service_scene_tests planner_plan_service
      planner_map_service)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(planner_service_scene_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    gtest_discover_tests(planner_service_scene_tests
      PROPERTIES LABELS "service" TIMEOUT 900
      DISCOVERY_TIMEOUT 60)
  endif()
endif()
