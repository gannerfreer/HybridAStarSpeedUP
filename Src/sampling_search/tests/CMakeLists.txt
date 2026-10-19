# planner_sampling_search GoogleTest 测试套件，由 gtest_discover_tests 注册为
# CTest 标签 "sampling_search"。
#
# 契约测试在运行期遍历 available_backends()：未编译的适配器自动跳过，
# 不伪造通过；各适配器另有专项测试文件。

include(GoogleTest)

add_executable(planner_sampling_search_tests
  test_core.cpp
  test_contract.cpp
  test_ompl.cpp
  test_ch0p1k3.cpp
  test_klampt.cpp
  test_vss2sn.cpp)

target_link_libraries(planner_sampling_search_tests
  PRIVATE planner_sampling_search GTest::gtest_main)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_sampling_search_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

gtest_discover_tests(planner_sampling_search_tests
  PROPERTIES LABELS "sampling_search"
  DISCOVERY_TIMEOUT 60)
