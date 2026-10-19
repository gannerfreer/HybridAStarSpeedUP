# tools/junit_runner.py

- 作用：标准库 unittest 发现 + JUnit XML 生成器（无第三方依赖）；`task_runner.py test --component tools` 通过它复现 `python3 -m unittest discover -s tools/tests -p 'test_*.py'` 的发现逻辑并产出 `junit.xml`。
- 入口/用法：

```sh
python3 tools/junit_runner.py --start-dir tools/tests --pattern 'test_*.py' \
  --junit <run_dir>/junit.xml [--repo-root PATH]
```

| 参数 | 说明 | 默认 |
| --- | --- | --- |
| `--start-dir` | 测试发现起始目录（必填；同时作为 `top_level_dir`） | — |
| `--pattern` | 测试文件名模式 | `test_*.py` |
| `--junit` | JUnit XML 输出路径（必填；自动创建父目录） | — |
| `--repo-root` | 仓库根，用于导入 `tools` 包 | 由脚本位置推导 |

- 输入/输出：读 `--start-dir` 下匹配 `--pattern` 的测试模块；写 `--junit` XML（`testsuites`/`testsuite`/`testcase`，含 classname/name/time 与 failure/error/skipped 子节点、UTC timestamp）；用例输出打到 stderr（verbosity=2）。发现阶段异常也会写一份带 `DiscoveryError` 的 XML。
- 依赖：CPython 3.8+ 标准库（`unittest`、`xml.etree.ElementTree`）；不导入内部模块。
- 调用关系：被 `tools/task_runner.py`（`test.runner=unittest` 的组件，当前为 `tools`）以 `sys.executable` 子进程调用，XML 写入运行目录的 `junit.xml`；发现范围与 `python3 -m unittest discover -s tools/tests -p 'test_*.py'` 等价（README §3）。`rundir.parse_junit_counts` 解析其计数，`report_results.py` 在报告“正确性”一节引用。
- 证据与约定：退出码 0 = 全部通过且至少运行 1 个用例；1 = 存在失败/错误、未发现用例或发现失败；2 = 参数错误（argparse）。子测试（subTest）失败/错误记到父用例；expected failure 记 skipped，unexpected success 记 failure。XML 用 `xml.etree.ElementTree` 生成（UTF-8，`tests`/`failures`/`errors`/`skipped`/`time` 属性），不伪造、不缺失——未发现用例也如实报 1。
- 源码与测试：`tools/junit_runner.py`；`tools/tests/test_junit_runner.py`。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
