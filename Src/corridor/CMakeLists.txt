# planner_corridor：安全走廊生成（第一版）。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_CORRIDOR=ON）。
# 下面的回退只是让该文件更易于被其他父项目使用；根项目总是提供
# PLANNER_ROOT_DIR，并从 dependencies.lock.json 获取
# nlohmann_json / GoogleTest。

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  option(PLANNER_BUILD_TESTS "Build tests and register them with CTest" ON)
endif()
if(NOT DEFINED PLANNER_BUILD_EXAMPLES)
  option(PLANNER_BUILD_EXAMPLES "Build component examples" ON)
endif()

add_library(planner_corridor
  src/geometry.cpp
  src/corridor_generator.cpp)

add_library(planner::corridor ALIAS planner_corridor)

target_compile_features(planner_corridor PUBLIC cxx_std_17)
target_include_directories(planner_corridor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_corridor PUBLIC planner::common)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_corridor PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 命令行示例：构建路径种子加障碍物，打印走廊文档，
# 或输出明确的失败诊断（PLANNER_BUILD_EXAMPLES 默认 ON）。
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_corridor_example examples/corridor_generator_example.cpp)
  target_link_libraries(planner_corridor_example PRIVATE planner_corridor)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_corridor_example PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

if(PLANNER_BUILD_TESTS)
  if(NOT TARGET GTest::gtest_main)
    message(FATAL_ERROR
      "PLANNER_BUILD_TESTS=ON but GTest::gtest_main is not available. "
      "Configure through the root CMakeLists.txt, which fetches googletest "
      "from dependencies.lock.json.")
  endif()
  add_subdirectory(tests)
endif()
