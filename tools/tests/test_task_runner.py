"""task_runner 单元测试：组件注册表、命令构造、上限收敛、门禁、fixtures 子命令。

不运行 cmake/ctest/编译器；只验证命令构造与拒绝逻辑。
"""

import argparse
import csv
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path
from unittest import mock

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import clock as clock_mod
from tools import registry as registry_mod
from tools import rundir
from tools import runtime
from tools import task_runner


class ComponentsTest(unittest.TestCase):
    def test_registry_contains_tools_and_common(self):
        components = task_runner.load_components(_REPO_ROOT)
        self.assertIn("tools", components["components"])
        self.assertIn("common", components["components"])
        common = components["components"]["common"]
        self.assertEqual(common["cmake"]["target"], "planner_common")
        self.assertEqual(common["cmake"]["options"]["PLANNER_BUILD_COMMON"], "ON")
        self.assertEqual(common["test"]["runner"], "ctest")
        self.assertEqual(common["test"]["label"], "common")
        tools = components["components"]["tools"]
        self.assertEqual(tools["test"]["runner"], "unittest")
        self.assertEqual(tools["test"]["start_dir"], "tools/tests")

    def test_unknown_component_rejected(self):
        components = task_runner.load_components(_REPO_ROOT)
        with self.assertRaises(task_runner.ConfigError):
            task_runner.resolve_component(components, "does_not_exist")

    def test_normalize_parallel_clamps_to_four(self):
        self.assertEqual(task_runner.normalize_parallel(2), 2)
        self.assertEqual(task_runner.normalize_parallel(8), 4)
        with self.assertRaises(task_runner.ConfigError):
            task_runner.normalize_parallel(0)

    def test_normalize_workers_bounds(self):
        self.assertEqual(task_runner.normalize_workers(4, 4), 4)
        with self.assertRaises(task_runner.ConfigError):
            task_runner.normalize_workers(5, 4)
        with self.assertRaises(task_runner.ConfigError):
            task_runner.normalize_workers(0, 4)


class CommandCompositionTest(unittest.TestCase):
    def setUp(self):
        self.components = task_runner.load_components(_REPO_ROOT)
        self.common = self.components["components"]["common"]
        self.tools = self.components["components"]["tools"]
        self.tmp = tempfile.TemporaryDirectory()
        self.layout = rundir.RunLayout(Path(self.tmp.name) / "run").ensure_dirs()

    def tearDown(self):
        self.tmp.cleanup()

    def test_build_commands(self):
        build_dir = Path("/tmp/planner-build/release")
        commands = task_runner.compose_build_commands(
            Path(_REPO_ROOT), build_dir, "release", 4, self.common)
        self.assertEqual(len(commands), 2)
        configure = commands[0]
        self.assertEqual(configure[0], "cmake")
        self.assertIn("-S", configure)
        self.assertIn("-B", configure)
        self.assertIn("-DCMAKE_BUILD_TYPE=Release", configure)
        self.assertIn("-DPLANNER_BUILD_COMMON=ON", configure)
        self.assertEqual(commands[1], ["cmake", "--build", str(build_dir),
                                       "--parallel", "4"])

    def test_build_debug_profile(self):
        commands = task_runner.compose_build_commands(
            Path(_REPO_ROOT), Path("/tmp/b"), "debug", 1, self.common)
        self.assertIn("-DCMAKE_BUILD_TYPE=Debug", commands[0])

    def test_ctest_command_with_junit(self):
        commands = task_runner.compose_test_commands(
            Path(_REPO_ROOT), Path("/tmp/b"), "common", self.common["test"],
            2, self.layout, True)
        command = commands[0]
        self.assertEqual(command[0], "ctest")
        self.assertIn("-L", command)
        self.assertIn("common", command)
        self.assertIn("--parallel", command)
        self.assertIn("2", command)
        self.assertIn("--output-junit", command)
        self.assertIn(str(self.layout.junit), command)

    def test_ctest_command_without_junit_support(self):
        commands = task_runner.compose_test_commands(
            Path(_REPO_ROOT), Path("/tmp/b"), "common", self.common["test"],
            1, self.layout, False)
        self.assertNotIn("--output-junit", commands[0])

    def test_unittest_command_uses_stdlib_runner(self):
        commands = task_runner.compose_test_commands(
            Path(_REPO_ROOT), Path("/tmp/b"), "tools", self.tools["test"],
            1, self.layout, True)
        command = commands[0]
        self.assertEqual(command[0], sys.executable)
        self.assertEqual(command[1], str(Path(_REPO_ROOT) / "tools" / "junit_runner.py"))
        self.assertIn("--start-dir", command)
        self.assertIn(str(Path(_REPO_ROOT) / "tools" / "tests"), command)
        self.assertIn("--junit", command)
        self.assertIn(str(self.layout.junit), command)

    def test_template_command_substitution(self):
        command = task_runner.compose_template_command(
            ["{build_dir}/bin/server", "--run-dir", "{run_dir}", "{repo_root}"],
            "/repo", "/build", "/run")
        self.assertEqual(command, ["/build/bin/server", "--run-dir", "/run", "/repo"])
        with self.assertRaises(task_runner.ConfigError):
            task_runner.compose_template_command(["{unknown}"], "/repo", "/b", "/r")

    def test_exit_code_from_returncode(self):
        self.assertEqual(task_runner.exit_code_from_returncode(0), 0)
        self.assertEqual(task_runner.exit_code_from_returncode(3), 3)
        self.assertEqual(task_runner.exit_code_from_returncode(-15), 143)

    def test_build_dir_default(self):
        args = argparse.Namespace(build_dir=None)
        self.assertEqual(task_runner.build_dir_for(args, Path("/repo"), "release"),
                         Path("/repo") / "build" / "release")

    def test_thread_env_overrides(self):
        env = task_runner.build_child_env(["OMP_NUM_THREADS=8"], False)
        self.assertEqual(env["OMP_NUM_THREADS"], "8")
        self.assertEqual(env["OPENBLAS_NUM_THREADS"], "1")
        env2 = task_runner.build_child_env([], True)
        self.assertEqual(env2.get("MKL_NUM_THREADS"),
                         os.environ.get("MKL_NUM_THREADS"),
                         "no-default 时不得注入默认值")
        with self.assertRaises(task_runner.ConfigError):
            task_runner.build_child_env(["bad"], False)

    def test_build_rejects_python_component(self):
        args = argparse.Namespace(component="tools", profile="release",
                                  parallel=1, build_dir=None)
        code = task_runner.cmd_build(args, Path(_REPO_ROOT), self.components)
        self.assertEqual(code, runtime.EXIT_DATA)


class Wave3ContractTest(unittest.TestCase):
    WAVE3 = {
        "rs": ("PLANNER_BUILD_RS", "planner_rs", "rs", "planner_rs_bench"),
        "collision": ("PLANNER_BUILD_COLLISION", "planner_collision", "collision", "planner_collision_bench"),
        "bspline": ("PLANNER_BUILD_BSPLINE", "planner_bspline", "bspline", "planner_bspline_bench"),
        "polynomial": ("PLANNER_BUILD_POLYNOMIAL", "planner_polynomial",
                       "polynomial", "planner_polynomial_bench"),
        "corridor": ("PLANNER_BUILD_CORRIDOR", "planner_corridor", "corridor", None),
        "hybrid_astar": ("PLANNER_BUILD_HYBRID_ASTAR", "planner_hybrid_astar",
                         "hybrid_astar", "planner_hybrid_astar_bench"),
        "validation": ("PLANNER_BUILD_VALIDATION", "planner_validation", "validation", None),
        "map": ("PLANNER_BUILD_MAP", "planner_map", "map", "planner_map_bench"),
        "dftpav": ("PLANNER_BUILD_DFTPAV", "planner_dftpav", "dftpav", None),
    }

    def test_wave3_components_registered(self):
        components = task_runner.load_components(_REPO_ROOT)
        for name, (option, target, label, bench) in self.WAVE3.items():
            entry = components["components"].get(name)
            self.assertIsNotNone(entry, "缺少组件条目: %s" % name)
            self.assertEqual(entry["cmake"]["options"][option], "ON")
            self.assertEqual(entry["cmake"]["target"], target)
            self.assertEqual(entry["test"], {"runner": "ctest", "label": label})
            suites = entry.get("benchmark_suites") or {}
            if bench is None:
                self.assertEqual(suites, {})
            else:
                command = suites["v1"]["command"]
                # 基准命令允许包装器前缀（hybrid_astar 使用 taskset -c 0-7
                # 绑定 P 核）；二进制路径必须出现在命令中。
                expected_binary = "{build_dir}/Src/%s/%s" % (name, bench)
                self.assertIn(expected_binary, command)
                self.assertIn("--suite", command)
                self.assertIn("v1", command)
                self.assertIn("--output-dir", command)
                self.assertIn("{run_dir}/outputs", command)

    def test_dftpav_pulls_prerequisites(self):
        components = task_runner.load_components(_REPO_ROOT)
        options = components["components"]["dftpav"]["cmake"]["options"]
        for key in ("PLANNER_BUILD_DFTPAV", "PLANNER_BUILD_CORRIDOR",
                    "PLANNER_BUILD_VALIDATION", "PLANNER_BUILD_HYBRID_ASTAR",
                    "PLANNER_BUILD_COLLISION"):
            self.assertEqual(options[key], "ON", "dftpav 缺少 %s" % key)

    def test_web_npm_entry_registered(self):
        components = task_runner.load_components(_REPO_ROOT)
        web = components["components"]["web"]
        self.assertEqual(web["kind"], "npm")
        self.assertEqual(web["dir"], "Src/web")
        self.assertEqual(web["install"][:2], ["npm", "install"])
        self.assertEqual(web["install_ci"][:2], ["npm", "ci"])
        self.assertEqual(web["build"], ["npm", "run", "build"])
        self.assertEqual(web["test"], ["npm", "run", "test"])
        self.assertEqual(web["service"]["workers"], 1)
        self.assertIn("{run_dir}", web["service"]["command"])

    def test_plan_service_entry_registered(self):
        components = task_runner.load_components(_REPO_ROOT)
        entry = components["components"].get("service")
        self.assertIsNotNone(entry, "缺少组件条目: service")
        self.assertEqual(entry["kind"], "cmake")
        options = entry["cmake"]["options"]
        for key in ("PLANNER_BUILD_SERVICE", "PLANNER_BUILD_MAP", "PLANNER_BUILD_RS",
                    "PLANNER_BUILD_COLLISION", "PLANNER_BUILD_HYBRID_ASTAR",
                    "PLANNER_BUILD_CORRIDOR", "PLANNER_BUILD_VALIDATION",
                    "PLANNER_BUILD_DFTPAV"):
            self.assertEqual(options[key], "ON", "service 缺少 %s" % key)
        self.assertEqual(entry["cmake"]["target"], "planner_plan_service")
        self.assertEqual(entry["test"], {"runner": "ctest", "label": "service"})
        self.assertEqual(entry.get("benchmark_suites") or {}, {})
        command = entry["service"]["command"]
        self.assertEqual(command[0], "{build_dir}/Src/service/planner_plan_service")
        self.assertIn("--map", command)
        self.assertIn("--origin-lat", command)
        self.assertIn("--origin-lon", command)
        self.assertIn("--port", command)
        self.assertIn("8766", command)
        self.assertIn("--run-dir", command)
        self.assertIn("{run_dir}", command)
        self.assertEqual(entry["service"]["workers"], 1)

    def test_normalize_retry_of(self):
        self.assertIsNone(task_runner.normalize_retry_of(None))
        self.assertIsNone(task_runner.normalize_retry_of(""))
        valid = "20261009T151800491894Z_tools_00c5bc64"
        self.assertEqual(task_runner.normalize_retry_of(valid), valid)
        for bad in ("foo", "20261009T151800491894Z_tools", valid[:-1], "x" * 40,
                    "20261009T151800491894Z_tools_00C5BC64"):
            with self.assertRaises(task_runner.ConfigError):
                task_runner.normalize_retry_of(bad)

    def test_main_rejects_invalid_retry_of_before_dispatch(self):
        code = task_runner.main(["test", "--component", "tools",
                                 "--retry-of", "not-a-run-id"])
        self.assertEqual(code, runtime.EXIT_DATA)

    def test_base_manifest_records_retry_of(self):
        args = argparse.Namespace(backend=None, seed=None, allow_unmonitored=True,
                                  retry_of="20261009T151800491894Z_tools_00c5bc64")
        manifest = task_runner.base_manifest(
            args, "test", "test:tools", "20260101T000000000000Z_tools_deadbeef",
            "tools", Path(_REPO_ROOT), "release", 1, clock_mod.FakeClock())
        self.assertEqual(manifest["retry_of"],
                         "20261009T151800491894Z_tools_00c5bc64")
        args.retry_of = None
        manifest = task_runner.base_manifest(
            args, "test", "test:tools", "20260101T000000000000Z_tools_deadbeef",
            "tools", Path(_REPO_ROOT), "release", 1, clock_mod.FakeClock())
        self.assertIsNone(manifest["retry_of"])

    def test_inject_context_env(self):
        base = {"PATH": "/usr/bin", "OMP_NUM_THREADS": "1"}
        env = task_runner.inject_context_env(
            base, "run-1", "/runs/run-1", "test:rs", "rs", "release", "/build/release")
        self.assertEqual(base, {"PATH": "/usr/bin", "OMP_NUM_THREADS": "1"})
        self.assertEqual(env["PLANNER_RUN_ID"], "run-1")
        self.assertEqual(env["PLANNER_RUN_DIR"], "/runs/run-1")
        self.assertEqual(env["PLANNER_TASK_ID"], "test:rs")
        self.assertEqual(env["PLANNER_COMPONENT"], "rs")
        self.assertEqual(env["PLANNER_PROFILE"], "release")
        self.assertEqual(env["PLANNER_BUILD_DIR"], "/build/release")

    def test_merge_benchmark_metrics(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = rundir.RunLayout(Path(tmp) / "run").ensure_dirs()
            columns = list(rundir.METRIC_COLUMNS) + ["note"]
            source = layout.outputs / task_runner.BENCHMARK_METRICS_NAME
            with open(str(source), "w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(columns)
                writer.writerow(["run-x", "case_a", "0", "cold", "0", "success",
                                 "", "", "10.5", "5.0", "1024"]
                                + [""] * 6 + ["true", "first trial"])
                writer.writerow(["run-x", "case_b", "1", "cold", "1", "timeout",
                                 "budget"]
                                + [""] * 10 + ["false", "second trial"])
            result = task_runner.merge_benchmark_metrics(layout)
            self.assertEqual(result, {"merged": 2, "missing": False})
            rows = rundir.read_csv_rows(layout.metrics)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["case_id"], "case_a")
            self.assertEqual(rows[0]["wall_ms"], "10.5")
            self.assertEqual(rows[0]["valid"], "true")
            self.assertEqual(rows[1]["status"], "timeout")
            self.assertEqual(rows[1]["failure_reason"], "budget")
            self.assertNotIn("note", rows[0])

    def test_merge_benchmark_metrics_missing_is_nonblocking(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = rundir.RunLayout(Path(tmp) / "run").ensure_dirs()
            result = task_runner.merge_benchmark_metrics(layout)
            self.assertEqual(result, {"merged": 0, "missing": True})


class ExpansionRegistryTest(unittest.TestCase):
    """X-A1：grid/scene 组件条目、可选构建开关、全局示例/基准开关；
    X-S wiring 后 planned_components 占位已提升为正式条目（只读注册表与
    根 CMake 文本，不运行构建）。"""

    def test_grid_and_scene_registered(self):
        components = task_runner.load_components(_REPO_ROOT)
        grid = components["components"].get("grid")
        self.assertIsNotNone(grid, "缺少组件条目: grid")
        self.assertEqual(grid["cmake"]["options"]["PLANNER_BUILD_GRID"], "ON")
        self.assertEqual(grid["cmake"]["target"], "planner_grid")
        self.assertEqual(grid["test"], {"runner": "ctest", "label": "grid"})
        scene = components["components"].get("scene")
        self.assertIsNotNone(scene, "缺少组件条目: scene")
        options = scene["cmake"]["options"]
        for key in ("PLANNER_BUILD_SCENE", "PLANNER_BUILD_MAP",
                    "PLANNER_BUILD_GRID", "PLANNER_BUILD_COLLISION"):
            self.assertEqual(options[key], "ON", "scene 缺少 %s" % key)
        self.assertEqual(scene["cmake"]["target"], "planner_scene")
        self.assertEqual(scene["test"], {"runner": "ctest", "label": "scene"})

    def test_optional_switches_registered_with_defaults(self):
        components = task_runner.load_components(_REPO_ROOT)
        expected = {
            "rs": "PLANNER_RS_WITH_OMPL",
            "bspline": "PLANNER_BSPLINE_WITH_EGO",
            "polynomial": "PLANNER_POLYNOMIAL_WITH_MAV_COMPARE",
            "dftpav": "PLANNER_DFTPAV_WITH_E2E",
            "map": "PLANNER_MAP_WITH_SERVICE",
            "service": "PLANNER_SERVICE_WITH_HTTP",
        }
        for name, key in expected.items():
            options = components["components"][name]["cmake"]["options"]
            self.assertEqual(options[key], "ON", "%s 缺少 %s" % (name, key))

    def test_map_entry_pulls_grid_scene_collision(self):
        components = task_runner.load_components(_REPO_ROOT)
        options = components["components"]["map"]["cmake"]["options"]
        for key in ("PLANNER_BUILD_GRID", "PLANNER_BUILD_SCENE",
                    "PLANNER_BUILD_COLLISION", "PLANNER_MAP_WITH_SERVICE"):
            self.assertEqual(options[key], "ON", "map 缺少 %s" % key)

    def test_search_components_promoted_and_placeholders_removed(self):
        components = task_runner.load_components(_REPO_ROOT)
        # X-S wiring：三个搜索组件由 X-A1-19 占位提升为正式条目；
        # planned_components 占位登记删除，避免两处登记漂移。
        self.assertNotIn("planned_components", components)
        for name in ("graph_search", "grid_search", "sampling_search"):
            self.assertIn(name, components["components"])

    def test_root_global_switches_defined(self):
        text = (Path(_REPO_ROOT) / "CMakeLists.txt").read_text(encoding="utf-8")
        for name in ("PLANNER_BUILD_EXAMPLES", "PLANNER_BUILD_BENCHMARKS",
                     "PLANNER_BUILD_GRID", "PLANNER_BUILD_SCENE"):
            self.assertIn("option(%s" % name, text)

    def test_web_has_no_cmake_build_option(self):
        components = task_runner.load_components(_REPO_ROOT)
        web = components["components"]["web"]
        self.assertEqual(web["kind"], "npm")
        self.assertNotIn("cmake", web)
        text = (Path(_REPO_ROOT) / "CMakeLists.txt").read_text(encoding="utf-8")
        # 选项与注册都必须已移除；说明性注释允许提及该名称（X-A1-15）。
        self.assertNotIn("option(PLANNER_BUILD_WEB", text)
        self.assertNotIn("planner_add_optional_component(PLANNER_BUILD_WEB", text)

    def test_build_configure_passes_grid_options(self):
        components = task_runner.load_components(_REPO_ROOT)
        grid = components["components"]["grid"]
        commands = task_runner.compose_build_commands(
            Path(_REPO_ROOT), Path("/tmp/planner-build/grid"), "release", 4, grid)
        self.assertIn("-DPLANNER_BUILD_GRID=ON", commands[0])
        self.assertEqual(commands[1], ["cmake", "--build",
                                       "/tmp/planner-build/grid",
                                       "--parallel", "4"])


class SearchComponentsContractTest(unittest.TestCase):
    """X-S wiring：graph_search/grid_search/sampling_search 注册表条目契约
    （只读注册表与根 CMake 文本，不运行构建）。"""

    def _entry(self, name):
        components = task_runner.load_components(_REPO_ROOT)
        entry = components["components"].get(name)
        self.assertIsNotNone(entry, "缺少组件条目: %s" % name)
        return entry

    def test_graph_search_entry_registered(self):
        entry = self._entry("graph_search")
        self.assertEqual(entry["kind"], "cmake")
        options = entry["cmake"]["options"]
        for key in ("PLANNER_BUILD_GRAPH_SEARCH", "PLANNER_BUILD_COMMON",
                    "PLANNER_GRAPH_SEARCH_WITH_BOOST",
                    "PLANNER_GRAPH_SEARCH_WITH_LMRP",
                    "PLANNER_GRAPH_SEARCH_WITH_JUSTINHJ",
                    "PLANNER_GRAPH_SEARCH_WITH_LEMON",
                    "PLANNER_GRAPH_SEARCH_WITH_IGRAPH"):
            self.assertEqual(options[key], "ON", "graph_search 缺少 %s" % key)
        self.assertEqual(entry["cmake"]["target"], "planner_graph_search")
        self.assertEqual(entry["test"]["runner"], "ctest")
        self.assertEqual(entry["test"]["label"], "graph_search")
        self.assertEqual(entry["test"]["workers"], 1)
        suites = entry["benchmark_suites"]
        self.assertEqual(sorted(suites), ["generate", "v1"])
        self.assertEqual(suites["v1"]["command"][:2], [
            "{build_dir}/Src/graph_search/benchmarks/planner_graph_search_bench",
            "--suite"])
        self.assertIn("{run_dir}/outputs", suites["v1"]["command"])
        self.assertEqual(suites["generate"]["command"][:2], [
            "{build_dir}/Src/graph_search/benchmarks/planner_graph_search_bench",
            "--generate-only"])

    def test_grid_search_entry_registered(self):
        entry = self._entry("grid_search")
        self.assertEqual(entry["kind"], "cmake")
        options = entry["cmake"]["options"]
        for key in ("PLANNER_BUILD_GRID_SEARCH", "PLANNER_BUILD_COMMON",
                    "PLANNER_GRID_SEARCH_WITH_VSS2SN",
                    "PLANNER_GRID_SEARCH_WITH_SMAC2D",
                    "PLANNER_GRID_SEARCH_WITH_JPS3D",
                    "PLANNER_GRID_SEARCH_WITH_LIBMULTIROBOT",
                    "PLANNER_GRID_SEARCH_WITH_JUSTINHJ"):
            self.assertEqual(options[key], "ON", "grid_search 缺少 %s" % key)
        self.assertEqual(entry["cmake"]["target"], "planner_grid_search")
        self.assertEqual(entry["test"]["runner"], "ctest")
        self.assertEqual(entry["test"]["label"], "grid_search")
        self.assertEqual(entry["test"]["workers"], 1)
        suites = entry["benchmark_suites"]
        self.assertEqual(sorted(suites), ["capacity", "v1"])
        for name, mode in (("v1", "formal"), ("capacity", "capacity")):
            command = suites[name]["command"]
            self.assertEqual(command[:2], [
                "bash", "{repo_root}/Src/grid_search/benchmarks/run_suite.sh"])
            self.assertIn("--mode", command)
            self.assertIn(mode, command)
            self.assertIn("{run_dir}/outputs", command)

    def test_sampling_search_entry_registered(self):
        entry = self._entry("sampling_search")
        self.assertEqual(entry["kind"], "cmake")
        options = entry["cmake"]["options"]
        for key in ("PLANNER_BUILD_SAMPLING_SEARCH", "PLANNER_BUILD_COMMON",
                    "PLANNER_SAMPLING_SEARCH_WITH_OMPL",
                    "PLANNER_SAMPLING_SEARCH_WITH_CH0P1K3",
                    "PLANNER_SAMPLING_SEARCH_WITH_KLAMPT",
                    "PLANNER_SAMPLING_SEARCH_WITH_VSS2SN"):
            self.assertEqual(options[key], "ON", "sampling_search 缺少 %s" % key)
        # ai-winter（GPL-3.0）适配延期：不列入条目，保持组件默认 OFF。
        self.assertNotIn("PLANNER_SAMPLING_SEARCH_WITH_AIWINTER", options)
        self.assertEqual(entry["cmake"]["target"], "planner_sampling_search")
        self.assertEqual(entry["test"]["runner"], "ctest")
        self.assertEqual(entry["test"]["label"], "sampling_search")
        self.assertEqual(entry["test"]["workers"], 1)
        suites = entry["benchmark_suites"]
        self.assertEqual(sorted(suites), ["v1"])
        self.assertEqual(suites["v1"]["command"][:2], [
            "{build_dir}/Src/sampling_search/benchmarks/planner_sampling_search_bench",
            "--suite"])
        self.assertIn("{run_dir}/outputs", suites["v1"]["command"])

    def test_search_build_commands_pass_component_options(self):
        components = task_runner.load_components(_REPO_ROOT)
        expected = {
            "graph_search": "PLANNER_GRAPH_SEARCH_WITH_IGRAPH",
            "grid_search": "PLANNER_GRID_SEARCH_WITH_JPS3D",
            "sampling_search": "PLANNER_SAMPLING_SEARCH_WITH_KLAMPT",
        }
        for name, switch in expected.items():
            commands = task_runner.compose_build_commands(
                Path(_REPO_ROOT), Path("/tmp/planner-build/search"), "release",
                4, components["components"][name])
            configure = commands[0]
            self.assertIn("-DPLANNER_BUILD_COMMON=ON", configure)
            self.assertIn("-D%s=ON" % switch, configure)
            self.assertEqual(commands[1], ["cmake", "--build",
                                           "/tmp/planner-build/search",
                                           "--parallel", "4"])

    def test_web_grid_display_benchmark_suite_registered(self):
        components = task_runner.load_components(_REPO_ROOT)
        suite = components["components"]["web"]["benchmark_suites"].get(
            "grid_display")
        self.assertIsNotNone(suite, "web 缺少 grid_display 基准套件")
        command = suite["command"]
        self.assertEqual(command[0], "python3")
        self.assertEqual(command[1], "Src/web/scripts/grid_display_bench.py")
        self.assertIn("--output-dir", command)
        self.assertIn("{run_dir}/outputs", command)
        script = Path(_REPO_ROOT) / "Src/web/scripts/grid_display_bench.py"
        self.assertTrue(script.is_file(), "显示基准脚本不存在: %s" % script)

    def test_root_cmake_wires_search_components(self):
        text = (Path(_REPO_ROOT) / "CMakeLists.txt").read_text(encoding="utf-8")
        for name in ("PLANNER_BUILD_GRAPH_SEARCH", "PLANNER_BUILD_GRID_SEARCH",
                     "PLANNER_BUILD_SAMPLING_SEARCH"):
            self.assertIn("option(%s" % name, text)
            self.assertIn("planner_add_optional_component(%s" % name, text)


class ExecCommandTest(unittest.TestCase):
    """tools-003：exec 一次性任务的成功/失败/名额/封存/参数校验。

    全部在临时仓库根中执行（隔离 runtime 与 runs）；allow_unmonitored=True
    只作用于该临时目录，不触碰真实监控。
    """

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self.tmp.name) / "repo"
        self.repo.mkdir()
        self.components = task_runner.load_components(_REPO_ROOT)

    def tearDown(self):
        self.tmp.cleanup()

    def _args(self, command, workers=1, component="adhoc"):
        return argparse.Namespace(
            component=component, workers=workers, profile="release",
            exec_command=command, allow_unmonitored=True, retry_of=None,
            queue_timeout=30.0, build_dir=None, backend=None, seed=None,
            input=[], thread_env=[], no_default_thread_env=False,
        )

    def _run_dirs(self):
        runs = self.repo / "artifacts" / "runs"
        return sorted(runs.iterdir()) if runs.is_dir() else []

    def test_exec_parser_captures_command(self):
        args = task_runner.build_parser().parse_args(
            ["exec", "--component", "adhoc", "--workers", "1", "--",
             "sha256sum", "dependencies.lock.json"])
        self.assertEqual(args.command, "exec")
        self.assertEqual(task_runner.normalize_exec_command(args.exec_command),
                         ["sha256sum", "dependencies.lock.json"])

    def test_normalize_exec_command(self):
        self.assertEqual(task_runner.normalize_exec_command(["--", "echo", "x"]),
                         ["echo", "x"])
        self.assertEqual(task_runner.normalize_exec_command(["echo"]), ["echo"])
        with self.assertRaises(task_runner.ConfigError):
            task_runner.normalize_exec_command([])
        with self.assertRaises(task_runner.ConfigError):
            task_runner.normalize_exec_command(["--"])

    def test_exec_placeholder_expansion(self):
        build_dir = self.repo / "build" / "release"
        run_dir = self.repo / "artifacts" / "runs" / "r1"
        expanded = task_runner.expand_exec_placeholders(
            ["{repo_root}/tool", "--out", "{run_dir}/outputs",
             "--profile", "{profile}", "{build_dir}"],
            self.repo, build_dir, run_dir, profile="release")
        self.assertEqual(expanded[0], str(self.repo / "tool"))
        self.assertEqual(expanded[2], str(run_dir / "outputs"))
        self.assertEqual(expanded[4], "release")
        self.assertEqual(expanded[5], str(build_dir))

    def test_exec_empty_command_rejected_without_run_dir(self):
        code = task_runner.cmd_exec(self._args([]), self.repo, self.components)
        self.assertEqual(code, runtime.EXIT_DATA)
        self.assertEqual(self._run_dirs(), [])

    def test_exec_success_registers_seals_and_releases(self):
        args = self._args([sys.executable, "-c", "print('hello-exec')"])
        code = task_runner.cmd_exec(args, self.repo, self.components)
        self.assertEqual(code, 0)
        run_dirs = self._run_dirs()
        self.assertEqual(len(run_dirs), 1)
        manifest = runtime.read_json(run_dirs[0] / "manifest.json")
        self.assertEqual(manifest["status"], "success")
        self.assertEqual(manifest["exit_code"], 0)
        self.assertEqual(manifest["task_kind"], "exec")
        self.assertEqual(manifest["component"], "adhoc")
        self.assertEqual(manifest["worker_count"], 1)
        self.assertIsNotNone(manifest["artifact_sha256"])
        self.assertIn("hello-exec",
                      (run_dirs[0] / "stdout.log").read_text(encoding="utf-8"))
        config = runtime.read_json(run_dirs[0] / "config.json")
        self.assertFalse(config["component_registered"])
        verification = rundir.verify_run_dir(run_dirs[0])
        self.assertTrue(verification["ok"], verification["problems"])
        metrics = rundir.read_csv_rows(run_dirs[0] / "metrics.csv")
        self.assertEqual(metrics[0]["status"], "success")
        registry = registry_mod.Registry(runtime.runtime_dir(self.repo))
        self.assertEqual(registry.tasks(), [])

    def test_exec_nonzero_exit_forwarded_and_sealed(self):
        args = self._args([sys.executable, "-c",
                           "import sys; sys.exit(3)"])
        code = task_runner.cmd_exec(args, self.repo, self.components)
        self.assertEqual(code, 3)
        run_dirs = self._run_dirs()
        self.assertEqual(len(run_dirs), 1)
        manifest = runtime.read_json(run_dirs[0] / "manifest.json")
        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(manifest["exit_code"], 3)
        self.assertIsNotNone(manifest["artifact_sha256"])
        self.assertTrue(rundir.verify_run_dir(run_dirs[0])["ok"])

    def test_exec_registered_component_is_recorded(self):
        args = self._args([sys.executable, "-c", "print('ok')"],
                          component="tools")
        code = task_runner.cmd_exec(args, self.repo, self.components)
        self.assertEqual(code, 0)
        config = runtime.read_json(self._run_dirs()[0] / "config.json")
        self.assertTrue(config["component_registered"])
        self.assertEqual(config["component_spec"]["kind"], "python")

    # -- tools-004：输入哈希 -------------------------------------------------

    def test_exec_records_input_sha256(self):
        payload = self.repo / "fixture.txt"
        payload.write_text("fixture-data", encoding="utf-8")
        digest = runtime.sha256_file(payload)
        args = self._args([sys.executable, "-c", "print('ok')"])
        args.input = ["fixture.txt"]
        code = task_runner.cmd_exec(args, self.repo, self.components)
        self.assertEqual(code, 0)
        run_dirs = self._run_dirs()
        manifest = runtime.read_json(run_dirs[0] / "manifest.json")
        self.assertEqual(manifest["input_sha256"],
                         {"inputs/fixture.txt": digest})
        self.assertTrue((run_dirs[0] / "inputs" / "fixture.txt").is_file())
        self.assertEqual(
            runtime.read_json(run_dirs[0] / "inputs" / "input_sha256.json"),
            {"inputs/fixture.txt": digest})

    def test_exec_without_input_keeps_empty_object(self):
        args = self._args([sys.executable, "-c", "print('ok')"])
        code = task_runner.cmd_exec(args, self.repo, self.components)
        self.assertEqual(code, 0)
        run_dirs = self._run_dirs()
        manifest = runtime.read_json(run_dirs[0] / "manifest.json")
        self.assertEqual(manifest["input_sha256"], {})
        self.assertFalse((run_dirs[0] / "inputs" / "input_sha256.json").exists())

    def test_exec_missing_input_rejected_with_evidence(self):
        args = self._args([sys.executable, "-c", "print('ok')"])
        args.input = ["missing-fixture.bin"]
        code = task_runner.cmd_exec(args, self.repo, self.components)
        self.assertEqual(code, runtime.EXIT_DATA)
        run_dirs = self._run_dirs()
        self.assertEqual(len(run_dirs), 1)
        manifest = runtime.read_json(run_dirs[0] / "manifest.json")
        self.assertEqual(manifest["status"], "failed")
        self.assertIn("invalid_config", manifest["failure_reason"])
        self.assertEqual(manifest["input_sha256"], {})
        self.assertIsNotNone(manifest["artifact_sha256"])


class NpmComponentTest(unittest.TestCase):
    """npm 组件类型：注册表解析、命令构造、install/build/test 顺序、
    退出码、名额/编译锁登记、JUnit 收集。全部在临时仓库根中执行。"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self.tmp.name) / "repo"
        self.repo.mkdir()
        (self.repo / "web").mkdir()

    def tearDown(self):
        self.tmp.cleanup()

    def _common(self, **overrides):
        values = {
            "profile": "release", "build_dir": None, "allow_unmonitored": True,
            "retry_of": None, "queue_timeout": 30.0, "backend": None, "seed": None,
            "input": [], "thread_env": [], "no_default_thread_env": False,
        }
        values.update(overrides)
        return values

    def _build_args(self, **overrides):
        return argparse.Namespace(component="web", parallel=4,
                                  **self._common(**overrides))

    def _test_args(self, **overrides):
        return argparse.Namespace(component="web", workers=1,
                                  **self._common(**overrides))

    def _run_args(self, **overrides):
        return argparse.Namespace(component="web", workers=None,
                                  **self._common(**overrides))

    def _components(self, entry_overrides=None):
        entry = {
            "kind": "npm",
            "dir": "web",
            "install": [sys.executable, "-c", "print('install-step')"],
            "install_ci": [sys.executable, "-c", "print('ci-step')"],
            "build": [sys.executable, "-c", "print('build-step')"],
            "test": [sys.executable, "-c", "print('test-step')"],
            "service": {"command": [sys.executable, "-c", "print('serve-step')"],
                        "workers": 1},
        }
        if entry_overrides:
            entry.update(entry_overrides)
        return {"worker_cap": 4, "components": {"web": entry}}

    def _run_dirs(self):
        runs = self.repo / "artifacts" / "runs"
        return sorted(runs.iterdir()) if runs.is_dir() else []

    # -- 命令构造 ----------------------------------------------------------

    def test_npm_build_sequence_without_node_modules(self):
        entry = self._components()["components"]["web"]
        commands = task_runner.compose_npm_build_commands(
            entry, self.repo / "web", self.repo, Path("/b"), Path("/r"))
        self.assertEqual(len(commands), 2)
        self.assertEqual(commands[0][:2], [sys.executable, "-c"])
        self.assertEqual(commands[1][:2], [sys.executable, "-c"])
        self.assertIn("install-step", commands[0][2])
        self.assertIn("build-step", commands[1][2])

    def test_npm_build_prefers_ci_with_lockfile(self):
        entry = self._components()["components"]["web"]
        (self.repo / "web" / "package-lock.json").write_text("{}", encoding="utf-8")
        commands = task_runner.compose_npm_build_commands(
            entry, self.repo / "web", self.repo, Path("/b"), Path("/r"))
        self.assertIn("ci-step", commands[0][2])

    def test_npm_build_skips_install_when_node_modules_exists(self):
        entry = self._components()["components"]["web"]
        (self.repo / "web" / "node_modules").mkdir()
        commands = task_runner.compose_npm_build_commands(
            entry, self.repo / "web", self.repo, Path("/b"), Path("/r"))
        self.assertEqual(len(commands), 1)
        self.assertIn("build-step", commands[0][2])

    def test_npm_build_requires_build_command(self):
        with self.assertRaises(task_runner.ConfigError):
            task_runner.compose_npm_build_commands(
                {"kind": "npm"}, self.repo / "web", self.repo, Path("/b"), Path("/r"))

    def test_npm_placeholders_include_component_dir(self):
        expanded = task_runner.expand_component_command(
            ["{dir}/tool", "{component_dir}/x", "{run_dir}", "{repo_root}"],
            self.repo, Path("/b"), Path("/r"), "release", self.repo / "web")
        self.assertEqual(expanded[0], str(self.repo / "web" / "tool"))
        self.assertEqual(expanded[1], str(self.repo / "web" / "x"))
        self.assertEqual(expanded[2], "/r")
        self.assertEqual(expanded[3], str(self.repo))

    # -- 入口接线（mock 捕获 spec） ----------------------------------------

    def test_npm_build_spec_holds_lock_and_worker_slot(self):
        components = self._components()
        with mock.patch.object(task_runner, "execute_task", return_value=0) as mocked:
            code = task_runner.cmd_build(self._build_args(), self.repo, components)
        self.assertEqual(code, 0)
        spec = mocked.call_args[0][4]
        self.assertTrue(spec["compile_lock"])
        self.assertEqual(spec["workers"], 1)
        self.assertEqual(spec["cwd"], str(self.repo / "web"))

    def test_npm_test_spec_has_junit_source(self):
        components = self._components({"test_junit": "reports/junit.xml"})
        with mock.patch.object(task_runner, "execute_task", return_value=0) as mocked:
            code = task_runner.cmd_test(self._test_args(), self.repo, components)
        self.assertEqual(code, 0)
        spec = mocked.call_args[0][4]
        self.assertEqual(spec["workers"], 1)
        self.assertFalse(spec["compile_lock"])
        self.assertEqual(spec["junit_source"],
                         self.repo / "web" / "reports" / "junit.xml")

    def test_npm_run_spec_uses_service_workers_and_cwd(self):
        components = self._components()
        with mock.patch.object(task_runner, "execute_task", return_value=0) as mocked:
            code = task_runner.cmd_run(self._run_args(), self.repo, components)
        self.assertEqual(code, 0)
        spec = mocked.call_args[0][4]
        self.assertEqual(spec["workers"], 1)
        self.assertEqual(spec["cwd"], str(self.repo / "web"))

    # -- 集成：真实执行与封存 ----------------------------------------------

    def test_npm_build_install_then_build_and_seal(self):
        components = self._components()
        code = task_runner.cmd_build(self._build_args(), self.repo, components)
        self.assertEqual(code, 0)
        run_dirs = self._run_dirs()
        self.assertEqual(len(run_dirs), 1)
        stdout = (run_dirs[0] / "stdout.log").read_text(encoding="utf-8")
        self.assertLess(stdout.index("install-step"), stdout.index("build-step"))
        manifest = runtime.read_json(run_dirs[0] / "manifest.json")
        self.assertEqual(manifest["status"], "success")
        self.assertEqual(manifest["exit_code"], 0)
        self.assertEqual(len(manifest["steps"]), 2)
        self.assertIsNotNone(manifest["artifact_sha256"])
        self.assertTrue(rundir.verify_run_dir(run_dirs[0])["ok"])
        registry = registry_mod.Registry(runtime.runtime_dir(self.repo))
        self.assertEqual(registry.tasks(), [])

    def test_npm_build_nonzero_exit_forwarded(self):
        components = self._components({
            "build": [sys.executable, "-c", "import sys; sys.exit(4)"]})
        code = task_runner.cmd_build(self._build_args(), self.repo, components)
        self.assertEqual(code, 4)
        manifest = runtime.read_json(self._run_dirs()[0] / "manifest.json")
        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(manifest["exit_code"], 4)
        self.assertIsNotNone(manifest["artifact_sha256"])

    def test_npm_test_collects_junit(self):
        script = ("import pathlib, sys\n"
                  "p = pathlib.Path(sys.argv[1])\n"
                  "p.parent.mkdir(parents=True, exist_ok=True)\n"
                  "p.write_text('<testsuite name=\"web\"/>')\n")
        components = self._components({
            "test": [sys.executable, "-c", script, "{dir}/reports/junit.xml"],
            "test_junit": "reports/junit.xml",
        })
        code = task_runner.cmd_test(self._test_args(), self.repo, components)
        self.assertEqual(code, 0)
        run_dirs = self._run_dirs()
        junit = run_dirs[0] / "junit.xml"
        self.assertTrue(junit.is_file())
        self.assertIn("testsuite", junit.read_text(encoding="utf-8"))
        events = rundir.read_events(run_dirs[0] / "events.jsonl")
        self.assertTrue(any(item["event"] == "junit_collected" for item in events))

    def test_npm_test_missing_junit_recorded(self):
        components = self._components({"test_junit": "reports/junit.xml"})
        code = task_runner.cmd_test(self._test_args(), self.repo, components)
        self.assertEqual(code, 0)
        run_dirs = self._run_dirs()
        self.assertFalse((run_dirs[0] / "junit.xml").exists())
        events = rundir.read_events(run_dirs[0] / "events.jsonl")
        self.assertTrue(any(item["event"] == "junit_missing" for item in events))

    def test_npm_test_direct_junit_in_run_dir_kept(self):
        script = ("import pathlib, sys\n"
                  "pathlib.Path(sys.argv[1]).write_text('<testsuite/>')\n")
        components = self._components({
            "test": [sys.executable, "-c", script, "{run_dir}/junit.xml"]})
        code = task_runner.cmd_test(self._test_args(), self.repo, components)
        self.assertEqual(code, 0)
        run_dirs = self._run_dirs()
        self.assertTrue((run_dirs[0] / "junit.xml").is_file())
        self.assertTrue(rundir.verify_run_dir(run_dirs[0])["ok"])

    def test_npm_run_executes_service_command_and_seals(self):
        components = self._components()
        code = task_runner.cmd_run(self._run_args(), self.repo, components)
        self.assertEqual(code, 0)
        run_dirs = self._run_dirs()
        stdout = (run_dirs[0] / "stdout.log").read_text(encoding="utf-8")
        self.assertIn("serve-step", stdout)
        self.assertIsNotNone(
            runtime.read_json(run_dirs[0] / "manifest.json")["artifact_sha256"])

    def test_npm_component_dir_missing_rejected(self):
        components = self._components({"dir": "Src/web"})
        code = task_runner.cmd_build(self._build_args(), self.repo, components)
        self.assertEqual(code, runtime.EXIT_DATA)
        self.assertEqual(self._run_dirs(), [])


class ForceRebuildTest(unittest.TestCase):
    """tools-006：--force-rebuild 选项、touch 行为、默认不触发。"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self.tmp.name) / "repo"
        self.repo.mkdir()
        self.src = self.repo / "Src" / "fake"
        self.src.mkdir(parents=True)
        self.source = self.src / "a.cpp"
        self.source.write_text("int main() { return 0; }\n", encoding="utf-8")
        self.components = {"worker_cap": 4, "components": {"fake": {
            "kind": "cmake",
            "cmake": {"options": {}, "target": "planner_fake"},
            "test": {"runner": "ctest", "label": "fake"},
            "benchmark_suites": {},
        }}}

    def tearDown(self):
        self.tmp.cleanup()

    def _args(self, force, component="fake"):
        return argparse.Namespace(
            component=component, profile="release", parallel=1, build_dir=None,
            force_rebuild=force, allow_unmonitored=True, retry_of=None,
            queue_timeout=30.0, backend=None, seed=None, input=[],
            thread_env=[], no_default_thread_env=False)

    def test_parser_flag_default_off_and_on(self):
        parser = task_runner.build_parser()
        default = parser.parse_args(["build", "--component", "rs"])
        self.assertFalse(default.force_rebuild)
        flagged = parser.parse_args(
            ["build", "--component", "rs", "--force-rebuild"])
        self.assertTrue(flagged.force_rebuild)

    def test_touch_refreshes_mtime_without_content_change(self):
        old = 1000000000
        os.utime(str(self.source), (old, old))
        before = self.source.read_bytes()
        count = task_runner.touch_component_sources(self.src)
        self.assertEqual(count, 1)
        self.assertGreater(self.source.stat().st_mtime, old + 1000)
        self.assertEqual(self.source.read_bytes(), before)

    def test_cmd_build_prepare_wired_only_when_requested(self):
        with mock.patch.object(task_runner, "execute_task", return_value=0) as mocked:
            task_runner.cmd_build(self._args(False), self.repo, self.components)
        spec_off = mocked.call_args[0][4]
        self.assertIsNone(spec_off.get("prepare"))
        with mock.patch.object(task_runner, "execute_task", return_value=0) as mocked:
            task_runner.cmd_build(self._args(True), self.repo, self.components)
        spec_on = mocked.call_args[0][4]
        prepare = spec_on.get("prepare")
        self.assertIsNotNone(prepare)
        os.utime(str(self.source), (1000000000, 1000000000))
        result = prepare(None, Path("/b"))
        self.assertEqual(result["action"], "force_rebuild")
        self.assertEqual(result["touched_files"], 1)
        self.assertGreater(self.source.stat().st_mtime, 1000001000)

    def test_cmd_build_force_rebuild_missing_source_dir_rejected(self):
        components = {"worker_cap": 4, "components": {"ghost": {
            "kind": "cmake",
            "cmake": {"options": {}, "target": "planner_ghost"},
            "test": {"runner": "ctest", "label": "ghost"},
            "benchmark_suites": {},
        }}}
        code = task_runner.cmd_build(self._args(True, component="ghost"),
                                     self.repo, components)
        self.assertEqual(code, runtime.EXIT_DATA)


class SnapshotCurationTest(unittest.TestCase):
    """tools-007：未跟踪源快照策展（前端/脚本类型收录，派生目录排除）。"""

    def test_filter_accepts_web_and_script_types(self):
        rels = [
            "web/src/app.ts", "web/src/main.tsx", "web/src/module.js",
            "web/lib/util.mjs", "web/lib/legacy.cjs", "web/index.html",
            "web/styles/app.css", "web/scripts/build.sh",
            "web/conf/app.yaml", "web/conf/app.yml", "web/Cargo.toml",
            "tools/x.py", "Src/common/CMakeLists.txt", "tests/fixtures/a.json",
        ]
        self.assertEqual(task_runner.filter_untracked_candidates(rels), rels)

    def test_filter_rejects_derived_directories(self):
        rels = [
            "node_modules/pkg/index.js", "dist/bundle.js", "reports/junit.xml",
            "build/gen.js", "artifacts/runs/x/app.ts", "web/coverage/c.js",
            "web/__pycache__/mod.py", "web/.cache/c.js",
            "web/node_modules/.bin/x", "sub/dist/deep/file.ts",
        ]
        self.assertEqual(task_runner.filter_untracked_candidates(rels), [])

    @unittest.skipUnless(shutil.which("git"), "需要 git 可执行文件")
    def test_capture_snapshot_packs_web_sources_and_excludes_dirs(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "repo"
            repo.mkdir()
            subprocess.run(["git", "init", "-q"], cwd=str(repo), check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            (repo / "tracked.txt").write_text("tracked", encoding="utf-8")
            subprocess.run(["git", "add", "tracked.txt"], cwd=str(repo),
                           check=True, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
            included = ["web/src/app.ts", "web/src/main.tsx", "web/index.html",
                        "web/styles/app.css", "web/scripts/build.sh",
                        "web/conf/app.yaml"]
            excluded = ["node_modules/pkg/index.js", "dist/bundle.js",
                        "reports/junit.xml", "build/gen.js"]
            for rel in included + excluded:
                path = repo / rel
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("x", encoding="utf-8")
            layout = rundir.RunLayout(Path(tmp) / "run").ensure_dirs()
            result = task_runner.capture_source_snapshot(repo, layout)
            self.assertTrue(result["untracked_archive"])
            with tarfile.open(str(layout.child("source_untracked.tar.gz"))) as archive:
                names = set(archive.getnames())
            for rel in included:
                self.assertIn(rel, names)
            for rel in excluded:
                self.assertNotIn(rel, names)
            self.assertNotIn("tracked.txt", names)


class GateTest(unittest.TestCase):
    def test_heartbeat_gate_states(self):
        with tempfile.TemporaryDirectory() as tmp:
            rt = os.path.join(tmp, "planner-runtime")
            clk = clock_mod.FakeClock(1000.0)
            self.assertEqual(registry_mod.heartbeat_state(rt, clk), "monitor_missing")
            runtime.atomic_write_json(registry_mod.heartbeat_path(rt), {
                "status": "running", "updated_monotonic": 1000.0})
            self.assertIsNone(registry_mod.heartbeat_state(rt, clk))
            clk.advance(6.001)
            self.assertEqual(registry_mod.heartbeat_state(rt, clk), "monitor_lost")

    def test_dispatch_state_default_not_paused(self):
        with tempfile.TemporaryDirectory() as tmp:
            reg = registry_mod.Registry(tmp)
            state = reg.read_dispatch_state()
            self.assertFalse(state["paused"])


class FixturesCommandTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self.tmp.name) / "repo"
        fixture_root = self.repo / "tests" / "fixtures" / "v1"
        fixture_root.mkdir(parents=True)
        (fixture_root / "a.json").write_text("{}\n", encoding="utf-8")

    def tearDown(self):
        self.tmp.cleanup()

    def test_freeze_then_verify(self):
        code = task_runner.cmd_fixtures(
            argparse.Namespace(action="freeze", version="v1"), self.repo)
        self.assertEqual(code, runtime.EXIT_OK)
        manifest = self.repo / "tests" / "fixtures" / "v1" / "fixture_manifest.sha256.json"
        self.assertTrue(manifest.is_file())
        code = task_runner.cmd_fixtures(
            argparse.Namespace(action="verify", version="v1"), self.repo)
        self.assertEqual(code, runtime.EXIT_OK)

    def test_verify_rejects_tamper(self):
        task_runner.cmd_fixtures(
            argparse.Namespace(action="freeze", version="v1"), self.repo)
        target = self.repo / "tests" / "fixtures" / "v1" / "a.json"
        target.write_text('{"changed": true}\n', encoding="utf-8")
        code = task_runner.cmd_fixtures(
            argparse.Namespace(action="verify", version="v1"), self.repo)
        self.assertEqual(code, runtime.EXIT_DATA)


if __name__ == "__main__":
    unittest.main()
