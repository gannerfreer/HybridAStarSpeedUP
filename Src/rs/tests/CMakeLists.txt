# planner_rs GoogleTest 测试套件，由 gtest_discover_tests 注册为 CTest 标签
# "rs"。仅当可选的参考适配器存在时，OMPL 封装一致性套件才会编译进同一可执行
# 文件。

include(GoogleTest)

add_executable(planner_rs_tests
  test_distance_cases.cpp
  test_path_structure.cpp
  test_sampling.cpp
  test_transform_invariance.cpp
  test_invalid_input.cpp
  test_ompl_consistency.cpp)

target_link_libraries(planner_rs_tests PRIVATE planner_rs GTest::gtest_main)

if(TARGET planner_rs_ompl_reference)
  target_link_libraries(planner_rs_tests PRIVATE planner_rs_ompl_reference)
  target_include_directories(planner_rs_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_rs_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_rs_tests
  PROPERTIES LABELS "rs"
  DISCOVERY_TIMEOUT 60)
