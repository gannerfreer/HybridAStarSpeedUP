# tools/fixtures.py

- 作用：版本化夹具目录 `tests/fixtures/<version>/` 的 freeze/verify（哈希清单生成与校验），是运行证据中夹具版本与哈希的记录依据。
- 入口/用法：模块无独立 CLI，经统一入口调用：

```sh
python3 tools/task_runner.py fixtures freeze --version v1
python3 tools/task_runner.py fixtures verify --version v1
```

- 输入/输出：
  - `freeze(fixture_root)`：生成 `<fixture_root>/fixture_manifest.sha256.json`（原子写 JSON），为按 POSIX 相对路径字典序排列的数组，每项 `{path, size_bytes, sha256}`；**不包含清单自身**。
  - `verify(fixture_root)`：逐项校验存在性、大小与 SHA256，并**拒绝清单外额外文件**；返回 `{ok, problems, checked, file_count}`。
- 依赖：CPython 3.8+ 标准库；内部模块 `tools/runtime.py`（`list_relative_files`、`sha256_file`、`atomic_write_json`、`read_json`）。
- 调用关系：被 `tools/task_runner.py` 的 `fixtures` 子命令调用（freeze 打印条目数；verify 失败打印问题并以 65 退出）；v6 栅格夹具的清单由 `freeze --version v6` 重新生成，`grid_fixtures.py` 的逐字节比较显式排除清单文件。
- 证据与约定：
  - verify 还检查清单结构：必须为 JSON 数组、条目含 `path/size_bytes/sha256`、禁止清单自身条目、路径必须为安全的相对路径（非绝对、不含 `..`）、路径不重复、整体按相对路径字典序排列。
  - 夹具版本冻结：v1–v5 保持不动；新增版本目录由对应生成器产出后再 freeze。
  - 失败语义：freeze 遇夹具目录不存在抛 `FixtureError`；verify 不抛异常，问题列表返回给调用方。
  - `freeze` 幂等：相同目录内容重复执行生成相同清单（测试覆盖）；清单哈希供 `manifest.input_sha256` 与报告“复现信息”引用（README §7.3）。
- 退出码（经入口）：0 成功；65 夹具目录/清单错误（freeze 失败或 verify 发现问题）。
- 源码与测试：`tools/fixtures.py`；`tools/tests/test_fixtures.py`。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
