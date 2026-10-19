# planner_map：Lanelet2 导入、坐标、查询与裁剪（Wave 4：M1-M3）。
#
# 范围：不依赖 ROS，C++17。公开 API（planner/map/*.hpp）从不暴露 Lanelet2
# 类型；上游库被隔离在 pimpl 之后。
#
# 构建适配（计划第 3.6 节）：官方 Lanelet2 仓库用 Catkin/mrt_cmake_modules 构建，
# 不会导出普通 CMake 目标。因此本组件通过下面定义的私有 `planner_lanelet2`
# 静态库编译一部分精选的上游源码：
#   * FetchContent 从 dependencies.lock.json 填充锁定的 1.2.3 归档
#     （`lanelet2` 条目，URL + URL_HASH）。上游仓库根目录没有 CMakeLists.txt，
#     因此 FetchContent_MakeAvailable() 只下载/解压，绝不运行上游构建脚本；
#     不会对任何上游文件打补丁。
#   * 编译的模块：lanelet2_core、lanelet2_io（OSM + 二进制处理器）、
#     lanelet2_projection、lanelet2_traffic_rules、lanelet2_routing 和
#     lanelet2_validation（不含 CLI 入口）。没有 Python 绑定，
#     没有 ROS，没有示例。
#   * Boost（filesystem、serialization）、Eigen3、pugixml 和 GeographicLib
#     是仅有的外部依赖。pugixml 与 GeographicLib 已纳入
#     dependencies.lock.json（`pugixml` / `geographiclib` 条目），组件优先
#     使用锁条目；下面的组件本地 pin 仅作为回退保留（X-A1-11 修正注释）。
#     lanelet2 锁条目是必需的。
#
# 可选构建目标（X-A1-08 / X-A1-16）：
#   * PLANNER_MAP_WITH_SERVICE（默认 ON）控制 planner_map_service（W2 HTTP
#     服务）及其测试；OFF 时不获取/链接 cpp-httplib。
#   * PLANNER_BUILD_EXAMPLES / PLANNER_BUILD_BENCHMARKS（根选项，默认 ON）
#     分别控制 planner_map_info 与 planner_map_bench。
#
# 系统构建提供者（X-A1-12）：Boost >= 1.65 与系统 Eigen3 >= 3.3 通过
# find_package 消费，dependencies.lock.json 没有 Boost 条目；锁条目 `eigen`
# 仅服务于曲线库对照后端，此处不混用。
#
# tools/components.json 请求的组件目标是 `planner_map`
# （库，链接 planner::common + planner::grid + 内置的 Lanelet2）。测试带有
# `map` CTest 标签；示例 CLI 是 `planner_map_info`。

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  option(PLANNER_BUILD_TESTS "Build tests and register them with CTest" ON)
endif()
if(NOT DEFINED PLANNER_BUILD_EXAMPLES)
  option(PLANNER_BUILD_EXAMPLES "Build component examples" ON)
endif()
if(NOT DEFINED PLANNER_BUILD_BENCHMARKS)
  option(PLANNER_BUILD_BENCHMARKS "Build component benchmarks" ON)
endif()
if(NOT DEFINED FETCHCONTENT_TRY_FIND_PACKAGE_MODE)
  set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
endif()

option(PLANNER_MAP_WITH_SERVICE
  "Build planner_map_service (W2 HTTP map service) and its tests" ON)

if(NOT TARGET planner::common)
  message(FATAL_ERROR
    "planner_map requires the planner::common target. Configure through the "
    "root CMakeLists.txt (PLANNER_BUILD_COMMON=ON).")
endif()

# planner_grid 提供栅格窗口尺寸/预算助手（export_driveable_grid 复用；
# 冻结计划 §2.1 map 行）。网格组件自身选项为 OFF 时在此引入。
if(NOT TARGET planner::grid)
  if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/grid/CMakeLists.txt")
    message(FATAL_ERROR
      "planner_map requires Src/grid (planner::grid), but "
      "${PLANNER_ROOT_DIR}/Src/grid/CMakeLists.txt does not exist.")
  endif()
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/grid" "${CMAKE_BINARY_DIR}/Src/grid")
endif()
if(NOT COMMAND planner_lock_get)
  message(FATAL_ERROR
    "planner_lock_get() is not defined; the map component must be added from "
    "the root CMakeLists.txt, which reads dependencies.lock.json.")
endif()

include(FetchContent)

if(DEFINED PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS)
  set(_map_timestamp_args ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})
elseif(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
  set(_map_timestamp_args DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
else()
  set(_map_timestamp_args "")
endif()

# ---------------------------------------------------------------------------
# 依赖锁定辅助函数。
# ---------------------------------------------------------------------------

# 当 dependencies.lock.json 已包含 `dependencies.<dep>.source.url` 时，
# 在父作用域设置 _pm_lock_url / _pm_lock_sha / _pm_lock_tag；否则为空。
# 让组件优先采用协调者晋升的条目，并回退到下面的组件本地固定版本。
function(planner_map_try_lock DEP)
  set(_pm_lock_url "" PARENT_SCOPE)
  set(_pm_lock_sha "" PARENT_SCOPE)
  set(_pm_lock_tag "" PARENT_SCOPE)
  if(NOT DEFINED PLANNER_LOCK_JSON)
    return()
  endif()
  string(JSON _pm_type ERROR_VARIABLE _pm_err TYPE "${PLANNER_LOCK_JSON}" dependencies "${DEP}")
  if(_pm_err)
    return()
  endif()
  string(JSON _pm_url ERROR_VARIABLE _pm_err GET "${PLANNER_LOCK_JSON}" dependencies "${DEP}" source url)
  if(_pm_err)
    return()
  endif()
  string(JSON _pm_tag ERROR_VARIABLE _pm_err GET "${PLANNER_LOCK_JSON}" dependencies "${DEP}" tag)
  if(_pm_err)
    set(_pm_tag "unknown tag")
  endif()
  string(JSON _pm_sha_type ERROR_VARIABLE _pm_err TYPE "${PLANNER_LOCK_JSON}" dependencies "${DEP}" source sha256)
  if(_pm_err OR _pm_sha_type STREQUAL "NULL")
    set(_pm_sha "")
  else()
    string(JSON _pm_sha GET "${PLANNER_LOCK_JSON}" dependencies "${DEP}" source sha256)
  endif()
  set(_pm_lock_url "${_pm_url}" PARENT_SCOPE)
  set(_pm_lock_sha "${_pm_sha}" PARENT_SCOPE)
  set(_pm_lock_tag "${_pm_tag}" PARENT_SCOPE)
endfunction()

# planner_map_fetch_pinned(<name> <url> <sha256|null> <label> <warning-text>)
# 声明一个固定版本的 FetchContent 依赖。sha256 为空时跳过 URL_HASH 校验
# 并给出警告（与根项目策略相同）。
function(planner_map_fetch_pinned NAME URL SHA256 LABEL PENDING_WARNING)
  if(SHA256)
    FetchContent_Declare(${NAME}
      URL "${URL}"
      URL_HASH "SHA256=${SHA256}"
      ${_map_timestamp_args})
  else()
    FetchContent_Declare(${NAME}
      URL "${URL}"
      ${_map_timestamp_args})
    message(WARNING "${PENDING_WARNING}")
  endif()
  message(STATUS "Dependency ${LABEL}")
endfunction()

# ---------------------------------------------------------------------------
# 锁定的 Lanelet2 1.2.3（依赖锁条目 `lanelet2`）。
# ---------------------------------------------------------------------------
planner_lock_get(lanelet2 _lanelet2_tag tag)
planner_lock_get(lanelet2 _lanelet2_commit commit)
planner_lock_get(lanelet2 _lanelet2_url source url)
planner_lock_get_nullable(lanelet2 _lanelet2_sha256 source sha256)

if(_lanelet2_sha256)
  set(_lanelet2_hash_args URL_HASH "SHA256=${_lanelet2_sha256}")
else()
  set(_lanelet2_hash_args "")
  message(WARNING
    "dependencies.lock.json has no sha256 for lanelet2 (status pending); the "
    "download will not be hash-verified")
endif()

message(STATUS "Dependency Lanelet2: ${_lanelet2_tag} (${_lanelet2_commit})")

FetchContent_Declare(lanelet2
  URL "${_lanelet2_url}"
  ${_lanelet2_hash_args}
  ${_map_timestamp_args})

# 上游根目录没有 CMakeLists.txt：MakeAvailable() 只下载并解压。
# 此防护用于防范未来上游布局变化。
FetchContent_MakeAvailable(lanelet2)
if(NOT lanelet2_SOURCE_DIR OR NOT EXISTS "${lanelet2_SOURCE_DIR}/lanelet2_core/CMakeLists.txt")
  message(FATAL_ERROR
    "Lanelet2 source tree was not populated correctly (expected "
    "lanelet2_core/CMakeLists.txt under '${lanelet2_SOURCE_DIR}').")
endif()

# ---------------------------------------------------------------------------
# pugixml（Lanelet2 IO）。优先使用锁条目；否则使用组件固定版本。
# ---------------------------------------------------------------------------
planner_map_try_lock(pugixml)
if(_pm_lock_url)
  set(_pugixml_url "${_pm_lock_url}")
  set(_pugixml_sha "${_pm_lock_sha}")
  set(_pugixml_label "pugixml: ${_pm_lock_tag} (dependencies.lock.json)")
else()
  set(_pugixml_url "https://github.com/zeux/pugixml/releases/download/v1.16/pugixml-1.16.tar.gz")
  set(_pugixml_sha "4cee1ca4aad395170f4c7a07824f3bdd41f28316c6e1e1090a1425b278ec0b4b")
  set(_pugixml_label "pugixml: v1.16 (component pin fallback; the dependencies.lock.json entry is preferred)")
endif()
planner_map_fetch_pinned(pugixml "${_pugixml_url}" "${_pugixml_sha}" "${_pugixml_label}"
  "dependencies.lock.json has no entry for pugixml and the component pin has no sha256; the download will not be hash-verified")

set(PUGIXML_BUILD_TESTS OFF CACHE BOOL "planner_map: pugixml tests are not built" FORCE)
set(PUGIXML_INSTALL OFF CACHE BOOL "planner_map: pugixml is not installed" FORCE)
set(PUGIXML_USE_VERSIONED_LIBDIR OFF CACHE BOOL "planner_map: no versioned libdir" FORCE)
FetchContent_MakeAvailable(pugixml)
if(NOT TARGET pugixml::pugixml)
  message(FATAL_ERROR "pugixml was populated but did not define pugixml::pugixml")
endif()

# ---------------------------------------------------------------------------
# GeographicLib（Lanelet2 UTM 投影器）。优先使用锁条目；否则使用固定版本。
# ---------------------------------------------------------------------------
planner_map_try_lock(geographiclib)
if(_pm_lock_url)
  set(_gcl_url "${_pm_lock_url}")
  set(_gcl_sha "${_pm_lock_sha}")
  set(_gcl_label "GeographicLib: ${_pm_lock_tag} (dependencies.lock.json)")
else()
  set(_gcl_url "https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.5.tar.gz")
  set(_gcl_sha "347319e3dc5dd2d58116ff1ecb8c0ae11c9f095c655fe289992b3208bc7a32fc")
  set(_gcl_label "GeographicLib: v2.5 (component pin fallback; the dependencies.lock.json entry is preferred)")
endif()
planner_map_fetch_pinned(geographiclib "${_gcl_url}" "${_gcl_sha}" "${_gcl_label}"
  "dependencies.lock.json has no entry for geographiclib and the component pin has no sha256; the download will not be hash-verified")

# 静态链接使测试/示例二进制在构建树中自包含。在添加依赖目录之前设置；
# 其他 planner 组件不会参考 BUILD_SHARED_LIBS。
set(BUILD_SHARED_LIBS OFF CACHE BOOL "planner_map: build dependencies statically" FORCE)
set(BUILD_DOCUMENTATION OFF CACHE BOOL "planner_map: no GeographicLib documentation" FORCE)
FetchContent_MakeAvailable(geographiclib)
if(NOT TARGET GeographicLib::GeographicLib)
  message(FATAL_ERROR "GeographicLib was populated but did not define GeographicLib::GeographicLib")
endif()

# ---------------------------------------------------------------------------
# 锁定的 cpp-httplib（依赖锁条目 `httplib`）：用于本地 W2 地图服务的
# 仅头文件 HTTP 服务器/客户端。TLS 和压缩关闭；
# 服务仅限回环。仅在 PLANNER_MAP_WITH_SERVICE=ON 时获取（X-A1-08）；
# planner_map 库与单元测试不包含 httplib。
# ---------------------------------------------------------------------------
if(PLANNER_MAP_WITH_SERVICE)
  planner_lock_get(httplib _httplib_tag tag)
  planner_lock_get(httplib _httplib_commit commit)
  planner_lock_get(httplib _httplib_url source url)
  planner_lock_get_nullable(httplib _httplib_sha256 source sha256)

  message(STATUS "Dependency cpp-httplib: ${_httplib_tag} (${_httplib_commit})")
  if(_httplib_sha256)
    set(_httplib_hash_args URL_HASH "SHA256=${_httplib_sha256}")
  else()
    set(_httplib_hash_args "")
    message(WARNING
      "dependencies.lock.json has no sha256 for httplib (status pending); the "
      "download will not be hash-verified")
  endif()

  FetchContent_Declare(httplib
    URL "${_httplib_url}"
    ${_httplib_hash_args}
    ${_map_timestamp_args})

  set(HTTPLIB_COMPILE OFF CACHE BOOL "planner_map: header-only cpp-httplib" FORCE)
  set(HTTPLIB_TEST OFF CACHE BOOL "planner_map: cpp-httplib tests are not built" FORCE)
  set(HTTPLIB_INSTALL OFF CACHE BOOL "planner_map: cpp-httplib is not installed" FORCE)
  set(HTTPLIB_USE_OPENSSL_IF_AVAILABLE OFF CACHE BOOL "planner_map: no TLS" FORCE)
  set(HTTPLIB_USE_WOLFSSL_IF_AVAILABLE OFF CACHE BOOL "planner_map: no TLS" FORCE)
  set(HTTPLIB_USE_MBEDTLS_IF_AVAILABLE OFF CACHE BOOL "planner_map: no TLS" FORCE)
  set(HTTPLIB_USE_ZLIB_IF_AVAILABLE OFF CACHE BOOL "planner_map: no compression" FORCE)
  set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF CACHE BOOL "planner_map: no compression" FORCE)
  set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF CACHE BOOL "planner_map: no compression" FORCE)
  # 异步 getaddrinfo 路径需要 glibc 的 libanl（getaddrinfo_a），
  # 它不在默认链接命令中；回环服务并不需要它。
  set(HTTPLIB_USE_NON_BLOCKING_GETADDRINFO OFF CACHE BOOL
      "planner_map: blocking getaddrinfo (no libanl dependency)" FORCE)

  FetchContent_MakeAvailable(httplib)
  if(NOT TARGET httplib::httplib)
    message(FATAL_ERROR "cpp-httplib was populated but did not define httplib::httplib")
  endif()
endif()

# ---------------------------------------------------------------------------
# 私有的内置 Lanelet2 目标（由我们编译的上游源码）。
# ---------------------------------------------------------------------------
set(_lanelet2_lib_sources
  "${lanelet2_SOURCE_DIR}/lanelet2_core/src/Attribute.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_core/src/BasicRegulatoryElements.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_core/src/Lanelet.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_core/src/LaneletMap.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_core/src/LaneletSequence.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_core/src/LineStringGeometry.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_core/src/PolygonTriangulationGeometry.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_core/src/RegulatoryElement.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_core/src/RegulatoryElementGeometry.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_io/src/BinHandler.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_io/src/Factory.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_io/src/Io.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_io/src/OsmFile.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_io/src/OsmHandlerLoad.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_io/src/OsmHandlerWrite.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_projection/src/Geocentric.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_projection/src/LocalCartesian.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_projection/src/UTM.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_traffic_rules/src/GenericTrafficRules.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_traffic_rules/src/GermanTrafficRules.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_traffic_rules/src/TrafficRulesFactory.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_routing/src/LaneletPath.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_routing/src/RouteBuilder.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_routing/src/Route.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_routing/src/RoutingCost.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_routing/src/RoutingGraphBuilder.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_routing/src/RoutingGraph.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_validation/src/BasicValidator.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_validation/src/Validation.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_validation/src/ValidatorFactory.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_validation/src/validators/CheckTags.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_validation/src/validators/CurvatureTooBig.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_validation/src/validators/DuplicatedPoints.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_validation/src/validators/PointsTooClose.cpp"
  "${lanelet2_SOURCE_DIR}/lanelet2_validation/src/validators/RoutingGraphIsValid.cpp")

foreach(_src IN LISTS _lanelet2_lib_sources)
  if(NOT EXISTS "${_src}")
    message(FATAL_ERROR
      "Vendored Lanelet2 source is missing from the locked archive: ${_src}. "
      "The lock file tag no longer matches the source lists in Src/map/CMakeLists.txt.")
  endif()
endforeach()

add_library(planner_lanelet2 STATIC ${_lanelet2_lib_sources})
set_target_properties(planner_lanelet2 PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(planner_lanelet2 PUBLIC cxx_std_17)
target_include_directories(planner_lanelet2 SYSTEM PUBLIC
  "${lanelet2_SOURCE_DIR}/lanelet2_core/include"
  "${lanelet2_SOURCE_DIR}/lanelet2_io/include"
  "${lanelet2_SOURCE_DIR}/lanelet2_projection/include"
  "${lanelet2_SOURCE_DIR}/lanelet2_traffic_rules/include"
  "${lanelet2_SOURCE_DIR}/lanelet2_routing/include"
  "${lanelet2_SOURCE_DIR}/lanelet2_validation/include")

find_package(Boost 1.65 REQUIRED COMPONENTS filesystem serialization)
find_package(Eigen3 3.3 REQUIRED)

# lanelet2 公开头文件会引入 Boost 和 Eigen 头文件，
# 因此在编译 planner_map 时两个头文件接口都必须可见。
target_link_libraries(planner_lanelet2
  PUBLIC
    Boost::boost
    Eigen3::Eigen
  PRIVATE
    Boost::filesystem
    Boost::serialization
    pugixml::pugixml
    GeographicLib::GeographicLib)

# ---------------------------------------------------------------------------
# planner_map：公开组件库。
# ---------------------------------------------------------------------------
add_library(planner_map
  src/sha256.cpp
  src/map.cpp)

add_library(planner::map ALIAS planner_map)

target_compile_features(planner_map PUBLIC cxx_std_17)
target_include_directories(planner_map PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

# 内置的注册表（OSM 解析器、交通规则、监管要素、验证检查）由其他未引用的
# 目标文件中的静态初始化器填充。普通的静态归档链接会丢弃它们，
# 因此对每个使用方强制整档链接（CMake >= 3.24）。
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
  target_link_libraries(planner_map
    PUBLIC planner::common
    PRIVATE planner::grid
            "$<LINK_LIBRARY:WHOLE_ARCHIVE,planner_lanelet2>")
else()
  target_link_libraries(planner_map
    PUBLIC planner::common
    PRIVATE planner::grid
            planner_lanelet2)
  message(WARNING
    "CMake ${CMAKE_VERSION} cannot whole-archive planner_lanelet2; upstream "
    "registries (OSM parser, traffic rules, regulatory elements) may be "
    "missing at runtime. Use CMake >= 3.24.")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_map PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 命令行示例：导入统计、投影和窗口查询。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_map_info examples/map_info.cpp)
  target_link_libraries(planner_map_info PRIVATE planner_map)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_map_info PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

# ---------------------------------------------------------------------------
# 微基准测试二进制（tools/README.md 第 7.2 节输出契约）。
# 源码位于 benchmarks/ 下，可以像其他 planner 基准测试一样包含组件内部
# 头文件（../src/sha256.hpp）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_BENCHMARKS)
  add_executable(planner_map_bench benchmarks/map_bench.cpp)
  target_link_libraries(planner_map_bench PRIVATE planner_map)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_map_bench PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

# ---------------------------------------------------------------------------
# 本地 HTTP 地图/查询服务（W2；tools/components.json 的 `service` 条目）。
# X-A1-08：独立开关（默认 ON）；OFF 时不构建二进制，也不获取 httplib。
# X-M1：栅格/场景预览端点复用 planner::scene（传递链接 collision），因此
# 在 PLANNER_BUILD_SCENE=ON 时确保 scene 目标可用并定义
# PLANNER_MAP_SERVICE_WITH_SCENE；不依赖 Hybrid/Dftpav/RS。scene 关闭时仍
# 构建服务（既有端点不变），两个栅格端点返回 500 并给出 CMake 警告。
# ---------------------------------------------------------------------------
if(PLANNER_MAP_WITH_SERVICE)
  if(NOT DEFINED PLANNER_BUILD_SCENE)
    set(PLANNER_BUILD_SCENE ON)
  endif()
  if(PLANNER_BUILD_SCENE AND NOT TARGET planner::scene AND
     EXISTS "${PLANNER_ROOT_DIR}/Src/scene/CMakeLists.txt")
    add_subdirectory("${PLANNER_ROOT_DIR}/Src/scene" "${CMAKE_BINARY_DIR}/Src/scene")
  endif()

  add_executable(planner_map_service src/service_main.cpp)
  target_link_libraries(planner_map_service PRIVATE planner_map httplib::httplib
    planner::grid)
  if(TARGET planner::scene)
    target_link_libraries(planner_map_service PRIVATE planner::scene)
    target_compile_definitions(planner_map_service PRIVATE
      PLANNER_MAP_SERVICE_WITH_SCENE=1)
  else()
    message(WARNING
      "planner_map_service is built without planner::scene (PLANNER_BUILD_SCENE=OFF); "
      "the /map/grid/driveable and /map/scene/preview endpoints return 500.")
  endif()
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_map_service PRIVATE -Wall -Wextra -Wpedantic)
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
