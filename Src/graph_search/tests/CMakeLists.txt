# planner_graph_search GoogleTest 测试套件（CTest 标签 "graph_search"）。
#
# 每适配器一个可执行文件，链接对应适配器目标；未启用的适配器不构建测试。
# 共享夹具与套件在 planner_graph_search_test_support 中。

include(GoogleTest)

add_library(planner_graph_search_test_support STATIC fixtures.cpp)
add_library(planner::graph_search_test_support ALIAS
            planner_graph_search_test_support)
target_compile_features(planner_graph_search_test_support PUBLIC cxx_std_17)
target_include_directories(planner_graph_search_test_support PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(planner_graph_search_test_support PUBLIC
  planner_graph_search
  GTest::gtest_main)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_graph_search_test_support PRIVATE
    -Wall -Wextra -Wpedantic)
endif()

function(graph_search_register_test_executable NAME)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${NAME} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  gtest_discover_tests(${NAME}
    PROPERTIES LABELS "graph_search"
    DISCOVERY_TIMEOUT 30)
endfunction()

# 核心库测试（不依赖任何适配器）。
add_executable(planner_graph_search_tests_core test_core.cpp)
target_link_libraries(planner_graph_search_tests_core PRIVATE
  planner_graph_search_test_support)
graph_search_register_test_executable(planner_graph_search_tests_core)

# S-01 Boost Graph。
if(TARGET planner_graph_search_boost)
  add_executable(planner_graph_search_tests_boost test_boost.cpp)
  target_link_libraries(planner_graph_search_tests_boost PRIVATE
    planner_graph_search_test_support
    planner_graph_search_boost)
  graph_search_register_test_executable(planner_graph_search_tests_boost)
endif()

# S-02 libMultiRobotPlanning。
if(TARGET planner_graph_search_lmrp)
  add_executable(planner_graph_search_tests_lmrp test_lmrp.cpp)
  target_link_libraries(planner_graph_search_tests_lmrp PRIVATE
    planner_graph_search_test_support
    planner_graph_search_lmrp)
  graph_search_register_test_executable(planner_graph_search_tests_lmrp)
endif()

# S-03 justinhj（池开/关同一目标）。
if(TARGET planner_graph_search_justinhj)
  add_executable(planner_graph_search_tests_justinhj test_justinhj.cpp)
  target_link_libraries(planner_graph_search_tests_justinhj PRIVATE
    planner_graph_search_test_support
    planner_graph_search_justinhj)
  graph_search_register_test_executable(planner_graph_search_tests_justinhj)
endif()

# S-04 LEMON。
if(TARGET planner_graph_search_lemon)
  add_executable(planner_graph_search_tests_lemon test_lemon.cpp)
  target_link_libraries(planner_graph_search_tests_lemon PRIVATE
    planner_graph_search_test_support
    planner_graph_search_lemon)
  graph_search_register_test_executable(planner_graph_search_tests_lemon)
endif()

# S-05 igraph（GPL-2.0-or-later 隔离组；仅在该目标存在时构建）。
if(TARGET planner_graph_search_igraph)
  add_executable(planner_graph_search_tests_igraph test_igraph.cpp)
  target_link_libraries(planner_graph_search_tests_igraph PRIVATE
    planner_graph_search_test_support
    planner_graph_search_igraph)
  graph_search_register_test_executable(planner_graph_search_tests_igraph)
endif()
