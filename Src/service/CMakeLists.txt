# planner_service：W5 规划服务（HTTP/JSON + WebSocket 事件）以及
# 集成流水线 Hybrid A* -> corridor -> Dftpav -> validation。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_SERVICE=ON；
# components.json 的 `service` 条目还会启用 MAP/RS/COLLISION/HYBRID_ASTAR/
# CORRIDOR/VALIDATION/DFTPAV/SCENE/GRID）。下面的回退只是让该文件更易于从
# 根项目使用；根项目提供 PLANNER_ROOT_DIR、planner_lock_get() 和
# GTest::gtest_main。planner::scene 是库的硬依赖（场景委托），缺失时会按
# 回退列表引入。
#
# License/provenance note (GPL isolation): this component is the only
# integration consumer of the isolated experimental planner::dftpav backend
# (GPL-3.0 port of ZJU-FAST-Lab/Dftpav, see Src/dftpav/PROVENANCE.md). Dftpav
# stays behind the public optimizer interface as an experimental backend; its
# upstream attribution and license text remain inside Src/dftpav.
#
# 目标：
#   * planner_service       - 库（解析器、场景、流水线、作业管理器）；
#   * planner_plan_service  - 服务二进制（{build_dir}/Src/service/
#                             planner_plan_service，tools/components.json）。
# 测试带有 `service` CTest 标签。
#
# 可选构建目标（X-A1-14）：
#   * PLANNER_SERVICE_WITH_HTTP（默认 ON）控制 HTTP 服务二进制及其集成测试。
#     OFF 时库与单元测试不链接 cpp-httplib，也不获取该依赖（httplib 仅用于
#     src/main.cpp 与集成测试）。
#
# 场景组合已迁移到 planner::scene（X-A1-13）：库通过
# planner/service/scene.hpp 的委托入口消费它，公开入口与错误码不变。

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  option(PLANNER_BUILD_TESTS "Build tests and register them with CTest" ON)
endif()
if(NOT DEFINED PLANNER_BUILD_EXAMPLES)
  option(PLANNER_BUILD_EXAMPLES "Build component examples" ON)
endif()

option(PLANNER_SERVICE_WITH_HTTP
  "Build planner_plan_service (HTTP binary) and its integration tests" ON)
if(NOT DEFINED FETCHCONTENT_TRY_FIND_PACKAGE_MODE)
  set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
endif()

if(NOT TARGET planner::common)
  message(FATAL_ERROR
    "planner_service requires the planner::common target. Configure through "
    "the root CMakeLists.txt (PLANNER_BUILD_COMMON=ON).")
endif()

# ---------------------------------------------------------------------------
# 组件依赖。根项目会为 `service` 条目启用全部依赖；当根项目的锁定辅助函数
# 可用时，回退逻辑可保持独立的 add_subdirectory 正常工作。
# ---------------------------------------------------------------------------
set(_service_dependencies map rs collision hybrid_astar corridor validation dftpav grid scene)
foreach(_dep IN LISTS _service_dependencies)
  if(NOT TARGET planner::${_dep})
    if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/${_dep}/CMakeLists.txt")
      message(FATAL_ERROR
        "planner_service requires Src/${_dep} (planner::${_dep}), but "
        "${PLANNER_ROOT_DIR}/Src/${_dep}/CMakeLists.txt does not exist.")
    endif()
    add_subdirectory("${PLANNER_ROOT_DIR}/Src/${_dep}" "${CMAKE_BINARY_DIR}/Src/${_dep}")
  endif()
endforeach()

# ---------------------------------------------------------------------------
# 锁定的 cpp-httplib（依赖锁条目 `httplib`）。当 PLANNER_BUILD_MAP=ON 时，
# map 组件已经获取它；此处的防护用于应对未来的构建顺序变化。
# TLS 和压缩保持关闭（回环服务）。
# 仅在 PLANNER_SERVICE_WITH_HTTP=ON 时需要：planner_service 库与单元测试
# 不包含 httplib，OFF 时该依赖既不获取也不链接。
# ---------------------------------------------------------------------------
if(PLANNER_SERVICE_WITH_HTTP AND NOT TARGET httplib::httplib)
  if(NOT COMMAND planner_lock_get)
    message(FATAL_ERROR
      "planner_service needs cpp-httplib (httplib::httplib); build through "
      "the root CMakeLists.txt so dependencies.lock.json is available.")
  endif()
  include(FetchContent)
  if(DEFINED PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS)
    set(_service_timestamp_args ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})
  elseif(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    set(_service_timestamp_args DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  else()
    set(_service_timestamp_args "")
  endif()
  planner_lock_get(httplib _service_httplib_tag tag)
  planner_lock_get(httplib _service_httplib_commit commit)
  planner_lock_get(httplib _service_httplib_url source url)
  planner_lock_get_nullable(httplib _service_httplib_sha256 source sha256)
  message(STATUS "Dependency cpp-httplib: ${_service_httplib_tag} (${_service_httplib_commit})")
  if(_service_httplib_sha256)
    set(_service_httplib_hash_args URL_HASH "SHA256=${_service_httplib_sha256}")
  else()
    set(_service_httplib_hash_args "")
    message(WARNING
      "dependencies.lock.json has no sha256 for httplib (status pending); the "
      "download will not be hash-verified")
  endif()
  FetchContent_Declare(httplib
    URL "${_service_httplib_url}"
    ${_service_httplib_hash_args}
    ${_service_timestamp_args})
  set(HTTPLIB_COMPILE OFF CACHE BOOL "planner_service: header-only cpp-httplib" FORCE)
  set(HTTPLIB_TEST OFF CACHE BOOL "planner_service: cpp-httplib tests are not built" FORCE)
  set(HTTPLIB_INSTALL OFF CACHE BOOL "planner_service: cpp-httplib is not installed" FORCE)
  set(HTTPLIB_USE_OPENSSL_IF_AVAILABLE OFF CACHE BOOL "planner_service: no TLS" FORCE)
  set(HTTPLIB_USE_WOLFSSL_IF_AVAILABLE OFF CACHE BOOL "planner_service: no TLS" FORCE)
  set(HTTPLIB_USE_MBEDTLS_IF_AVAILABLE OFF CACHE BOOL "planner_service: no TLS" FORCE)
  set(HTTPLIB_USE_ZLIB_IF_AVAILABLE OFF CACHE BOOL "planner_service: no compression" FORCE)
  set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF CACHE BOOL "planner_service: no compression" FORCE)
  set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF CACHE BOOL "planner_service: no compression" FORCE)
  set(HTTPLIB_USE_NON_BLOCKING_GETADDRINFO OFF CACHE BOOL
      "planner_service: blocking getaddrinfo (no libanl dependency)" FORCE)
  FetchContent_MakeAvailable(httplib)
  if(NOT TARGET httplib::httplib)
    message(FATAL_ERROR "cpp-httplib was populated but did not define httplib::httplib")
  endif()
endif()

# ---------------------------------------------------------------------------
# planner_service：解析器、场景构建器、流水线和作业管理器。
# ---------------------------------------------------------------------------
add_library(planner_service
  src/types.cpp
  src/parse.cpp
  src/scene.cpp
  src/pipeline.cpp
  src/event_hub.cpp
  src/snapshot_store.cpp
  src/service.cpp)

add_library(planner::service ALIAS planner_service)

target_compile_features(planner_service PUBLIC cxx_std_17)
target_include_directories(planner_service PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_service PUBLIC
  planner::common
  planner::map
  planner::rs
  planner::collision
  planner::hybrid_astar
  planner::corridor
  planner::validation
  planner::dftpav
  planner::scene)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_service PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 服务二进制（tools/components.json 的 `service` 条目；X-A1-14 独立开关）。
# ---------------------------------------------------------------------------
if(PLANNER_SERVICE_WITH_HTTP)
  add_executable(planner_plan_service src/main.cpp)
  target_link_libraries(planner_plan_service PRIVATE
    planner_service
    httplib::httplib)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_plan_service PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

# ---------------------------------------------------------------------------
# 独立示例（当 v3 地图夹具可用时，还会注册为带 `service` 标签的 CTest 自检）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_plan_service_example examples/plan_example.cpp)
  target_link_libraries(planner_plan_service_example PRIVATE planner_service)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_plan_service_example PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  if(PLANNER_BUILD_TESTS)
    set(_service_example_map
      "${PLANNER_ROOT_DIR}/tests/fixtures/v3/map/mapping_example.osm")
    if(EXISTS "${_service_example_map}")
      add_test(NAME planner_plan_service_example_self_check
        COMMAND planner_plan_service_example
          --map "${_service_example_map}"
          --origin-lat 49.00345654351
          --origin-lon 8.42427590707
          --self-check)
      set_tests_properties(planner_plan_service_example_self_check PROPERTIES
        LABELS "service"
        TIMEOUT 300)
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# 测试（CTest 标签 "service"）。
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
