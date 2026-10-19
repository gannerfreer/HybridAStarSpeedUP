# tools/task_runner.py

- 作用：E0 统一任务入口，编排 `build` / `test` / `benchmark` / `run` / `exec` / `fixtures` 六类子命令，负责登记名额、监控门禁、运行目录与 §9 证据封存，并转发子进程退出码。
- 入口/用法：`python3 tools/task_runner.py <子命令> [选项]`；子命令必需，`--component` 除 `fixtures` 外均需（`exec` 默认 `adhoc`）。
- 子命令参数：

| 子命令 / 参数 | 说明 | 默认 |
| --- | --- | --- |
| `build --component` | 组件 id；`--profile {release,debug}`；`--parallel`（>4 警告降级）；`--force-rebuild` | `release` / `4` / 关闭 |
| `test --component` | 组件 id；`--workers`（预占名额，ctest 并行度）；`--profile` | `1` / `release` |
| `benchmark --component` | 组件 id；`--suite`（未给时取注册表 `default_suite` 或 `v1`） | `None` |
| `run --component` | 组件 id；`--workers`（未给时取 `service.workers`） | `None`→`1` |
| `exec --component` | 组件标签（未注册也允许，仅记录）；`--workers`；`-- <command> [args...]` | `adhoc` / `1` |
| `fixtures <freeze\|verify>` | `--version` | `v1` |

- 通用参数：

| 参数 | 说明 | 默认 |
| --- | --- | --- |
| `--allow-unmonitored` | 引导期跳过心跳门禁（会写入 config/manifest，正式实验禁用） | 关闭 |
| `--retry-of <run_id>` | 重试来源；格式非法在派发前以 65 拒绝 | `None` |
| `--queue-timeout` | 等待名额/编译锁/独占窗口的最长秒数；0=无限 | `3600.0` |
| `--build-dir` | 覆盖构建目录 | `build/<profile>` |
| `--backend` / `--seed` | 记录标签 / 随机种子 | `None` |
| `--input <path>` | 复现输入，可重复；≤10 MiB 复制、>10 MiB 记外部引用、目录打包 tar.gz | 空 |
| `--thread-env KEY=VALUE` | 覆盖 OMP/OPENBLAS/MKL=1 默认线程环境，可重复 | 空 |
| `--no-default-thread-env` | 不注入默认线程环境 | 关闭 |

- 输入/输出：读 `tools/components.json`、`dependencies.lock.json`、git 状态；写 `artifacts/runs/<run_id>/`（manifest/config、metrics/resources、events、日志、junit、inputs、outputs、summary、artifacts.sha256.json）；登记与锁位于 `planner-runtime/`；子进程输出同时落盘与回显；结束打印 `run_id=… status=… exit_code=… run_dir=…`。
- exec 约定：命令支持 `{repo_root}`/`{build_dir}`/`{run_dir}`/`{profile}` 占位符，未知花括号按字面保留；命令输出约定写 `{run_dir}/outputs`；`benchmark` 结束后把 `outputs/benchmark_metrics.csv` 并入 `metrics.csv` 再追加 `task_summary` 行。
- 依赖：标准库（argparse、selectors、subprocess、resource、tarfile 等）；内部 `tools/clock.py`、`tools/fixtures.py`、`tools/registry.py`、`tools/rundir.py`、`tools/runtime.py`；外部命令 cmake/ctest/npm/git；unittest 组件经 `tools/junit_runner.py` 运行。
- 调用关系：顶层入口，由人工或监控 agent 直接调用；调用上述内部模块，不修改其他模块；`test --component tools` 经子进程调用 `junit_runner.py`。
- 证据与约定：
  - run_id 形如 `YYYYMMDDTHHMMSSffffffZ_<component>_<8位小写十六进制>`；先建运行目录再登记；封存顺序见 `rundir.py`；
  - 心跳缺失/停止/超 6 秒拒绝派发（`--allow-unmonitored` 仅引导期自检）；cmake `build` 只持全局编译锁不占名额，npm `build` 占 `build_workers`（默认 1）；
  - 子进程在独立进程组运行并注入 `PLANNER_RUN_ID`、`PLANNER_RUN_DIR`、`PLANNER_TASK_ID`、`PLANNER_COMPONENT`、`PLANNER_PROFILE`、`PLANNER_BUILD_DIR`；
  - 退出码 0=成功、非零原样转发（信号死亡 128+signal）、65=配置/数据错误、69=资源不可用、70=内部错误、75=暂时无法派发、130=中断。
- 源码与测试：`tools/task_runner.py`；`tools/tests/test_task_runner.py`。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
