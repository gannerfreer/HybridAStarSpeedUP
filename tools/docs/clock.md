# tools/clock.py

- 作用：时钟抽象与时间格式工具——正式入口使用真实 `Clock`，单元测试注入 `FakeClock`；对外时间字符串统一为 UTC ISO 8601 毫秒精度。
- 入口/用法：无 CLI。接口：

| 成员 | 说明 |
| --- | --- |
| `utc_iso(dt)` | 先 `astimezone(UTC)` 再转 `YYYY-MM-DDTHH:mm:ss.sssZ`（毫秒截断） |
| `parse_utc_iso(text)` | 解析 UTC 字符串，接受结尾 `Z`/`z` 或 `+00:00`；非字符串/非法格式抛 ValueError |
| `run_id_timestamp(dt)` | run ID 时间戳 `YYYYMMDDTHHMMSSffffffZ`（微秒精度） |
| `Clock` | 真实时钟：`utc_now()` / `monotonic()` / `sleep(seconds)`（非正数不睡） |
| `FakeClock(monotonic=1000.0, utc=None)` | 测试时钟：固定起点（默认 `2026-01-01T00:00:00Z`），`sleep` 即 `advance(seconds)`，回拨抛 ValueError |

- 关键细节：`FakeClock` 继承 `Clock`，`sleep`/`advance` 同步推进单调时间与 UTC 时间；`utc_iso` 与 `run_id_timestamp` 是纯格式函数，可独立用于测试；模块导出 `UTC = timezone.utc`。
- 用途示例（内部）：run ID 由 `rundir.new_run_id` 经 `run_id_timestamp` 生成；事件/心跳/manifest 时间由 `runtime.now_iso` 经 `utc_iso` 生成。
- 输入/输出：无文件读写；输出时间字符串与 `datetime`，供登记、事件、清单与心跳使用。
- 依赖：标准库 `time`、`datetime`（含 `timezone`）；无内部模块依赖（依赖树叶子）。
- 调用关系：被 `runtime.py`、`registry.py`、`rundir.py`、`task_runner.py`、`resource_monitor.py`、`report_results.py`、`sample_report.py` 及全部 `tools/tests/` 使用。
- 证据与约定：时间戳格式是 run ID、`events.jsonl`、心跳与 manifest 的公共约定；`FakeClock` 只允许在测试进程内构造，正式入口不暴露任何模拟开关。
- 状态说明：`Clock` 无实例状态、可安全复用；`FakeClock` 的可变状态仅存在于测试实例内。
- 源码与测试：`tools/clock.py`；`tools/tests/test_clock.py`。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
