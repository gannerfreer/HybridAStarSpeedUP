# planner_scene：地图窗口、可行驶域栅格与用户障碍的场景组合（X-A1-18）。
#
# 职责与依赖边界（冻结计划 §2.1）：依赖 map/grid/collision 的公共接口；
# 不依赖规划器、HTTP 或显示。公开头文件（planner/scene/scene.hpp）使用
# map::Map 与 collision::Scene，因此 map/collision/common 为 PUBLIC，
# grid 仅为实现的数学/校验助手（PRIVATE）。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_SCENE=ON，默认 ON；场景需要
# PLANNER_BUILD_MAP=ON，根项目在 map 未启用时跳过它并给出说明）。

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  option(PLANNER_BUILD_TESTS "Build tests and register them with CTest" ON)
endif()

# ---------------------------------------------------------------------------
# 依赖组件：map（窗口与可行驶域导出）、grid（单元预算/坐标助手）、
# collision（障碍物与场景类型）。当依赖组件的自身选项为 OFF 时在此引入，
# 使 `task_runner.py build --component scene` 能在全新构建目录中运行。
# ---------------------------------------------------------------------------
if(NOT TARGET planner::map)
  if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/map/CMakeLists.txt")
    message(FATAL_ERROR
      "planner_scene requires Src/map (planner::map), but "
      "${PLANNER_ROOT_DIR}/Src/map/CMakeLists.txt does not exist.")
  endif()
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/map" "${CMAKE_BINARY_DIR}/Src/map")
endif()
if(NOT TARGET planner::grid)
  if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/grid/CMakeLists.txt")
    message(FATAL_ERROR
      "planner_scene requires Src/grid (planner::grid), but "
      "${PLANNER_ROOT_DIR}/Src/grid/CMakeLists.txt does not exist.")
  endif()
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/grid" "${CMAKE_BINARY_DIR}/Src/grid")
endif()
if(NOT TARGET planner::collision)
  if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/collision/CMakeLists.txt")
    message(FATAL_ERROR
      "planner_scene requires Src/collision (planner::collision), but "
      "${PLANNER_ROOT_DIR}/Src/collision/CMakeLists.txt does not exist.")
  endif()
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/collision"
    "${CMAKE_BINARY_DIR}/Src/collision")
endif()

add_library(planner_scene
  src/scene.cpp
  src/content_id.cpp
  src/snapshot.cpp)

add_library(planner::scene ALIAS planner_scene)

target_compile_features(planner_scene PUBLIC cxx_std_17)
target_include_directories(planner_scene PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_scene
  PUBLIC planner::common planner::map planner::collision
  PRIVATE planner::grid)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_scene PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 测试（GoogleTest + CTest，标签 "scene"）。测试只读 v3 地图夹具。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_TESTS)
  if(NOT TARGET GTest::gtest_main)
    message(FATAL_ERROR
      "PLANNER_BUILD_TESTS=ON but GTest::gtest_main is not available. "
      "Configure through the root CMakeLists.txt, which fetches googletest "
      "from dependencies.lock.json.")
  endif()
  add_subdirectory(tests)
endif()
