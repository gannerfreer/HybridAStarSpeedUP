# planner_common：第一版公开类型及其 JSON 编解码器（E0）。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_COMMON=ON）。
# 下面的回退只是让该文件在从其他父项目添加时更易于使用；
# 根项目总是提供 PLANNER_ROOT_DIR，并从 dependencies.lock.json 获取
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

add_library(planner_common
  src/types.cpp
  src/serialization.cpp)

add_library(planner::common ALIAS planner_common)

target_compile_features(planner_common PUBLIC cxx_std_17)
target_include_directories(planner_common PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_common PUBLIC nlohmann_json::nlohmann_json)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_common PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 命令行往返示例（自检 + 文件模式）。随组件一起构建（PLANNER_BUILD_EXAMPLES
# 默认 ON）；E0 阶段只记录它，不运行它。
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_common_roundtrip_example examples/roundtrip_example.cpp)
  target_link_libraries(planner_common_roundtrip_example PRIVATE planner_common)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_common_roundtrip_example PRIVATE -Wall -Wextra -Wpedantic)
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
