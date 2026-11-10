cmake_minimum_required(VERSION 3.16)
project(user_hybrid_astar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(PLANNER_BUILD_TESTS "Build the test suite" ON)
option(PLANNER_BUILD_BENCHMARKS "Build the benchmark harness" OFF)
option(PLANNER_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(PLANNER_ENABLE_CLANG_TIDY "Run clang-tidy while compiling" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

function(planner_set_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

if(PLANNER_ENABLE_SANITIZERS)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
  else()
    message(WARNING "PLANNER_ENABLE_SANITIZERS is only supported with GCC or Clang")
  endif()
endif()

if(PLANNER_ENABLE_CLANG_TIDY)
  find_program(PLANNER_CLANG_TIDY_EXECUTABLE NAMES clang-tidy)
  if(PLANNER_CLANG_TIDY_EXECUTABLE)
    set(CMAKE_CXX_CLANG_TIDY "${PLANNER_CLANG_TIDY_EXECUTABLE}")
  else()
    message(WARNING "PLANNER_ENABLE_CLANG_TIDY requested but clang-tidy was not found")
  endif()
endif()

add_subdirectory(Src/common)
add_subdirectory(Src/math)
add_subdirectory(Src/collision)
add_subdirectory(Src/rs)
add_subdirectory(Src/hybrid_astar)

include(CTest)
# 测试目标同时受 CTest 的 BUILD_TESTING 与本项目的 PLANNER_BUILD_TESTS 控制。
if(BUILD_TESTING AND PLANNER_BUILD_TESTS)
  # Conda 的 GTest 可能由更新版本的 libstdc++ 构建，与系统工具链 ABI 不兼容；
  # 优先使用编译器所在系统安装的 GTest。
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND CMAKE_IGNORE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  endif()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.14.0
      GIT_SHALLOW TRUE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()
  include(GoogleTest)

  function(planner_add_gtest target source library label timeout)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE ${library} GTest::gtest_main)
    gtest_discover_tests(${target} PROPERTIES LABELS "${label}" TIMEOUT ${timeout})
    planner_set_warnings(${target})
  endfunction()

  planner_add_gtest(math_test Src/math/tests/test_math.cpp planner::math unit 20)
  planner_add_gtest(collision_checker_test
    Src/collision/tests/test_collision_checker.cpp planner::collision unit 20)
  planner_add_gtest(reeds_shepp_test
    Src/rs/tests/test_reeds_shepp.cpp planner::rs unit 20)
  planner_add_gtest(grid_view_test
    Src/common/tests/test_grid_view.cpp planner::collision unit 20)
  planner_add_gtest(obstacle_detour_test
    Src/hybrid_astar/tests/test_obstacle_detour.cpp planner::hybrid_astar integration 20)
  planner_add_gtest(search_regression_test
    Src/hybrid_astar/tests/test_search_regression.cpp planner::hybrid_astar regression 20)
endif()

if(PLANNER_BUILD_BENCHMARKS)
  add_executable(planner_bench bench/bench_main.cpp)
  target_link_libraries(planner_bench PRIVATE planner::hybrid_astar)
  planner_set_warnings(planner_bench)
endif()

foreach(target planner_math planner_common planner_collision planner_rs planner_hybrid_astar)
  if(TARGET ${target})
    planner_set_warnings(${target})
  endif()
endforeach()
