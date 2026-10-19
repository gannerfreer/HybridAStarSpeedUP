# tools/grid_fixtures.py

- 作用：v6 栅格夹具确定性生成器（X-W1 交付）：v5 全量复制 + `grids/` 新增/修订（手算 G1–G8、协议用例 P1–P13、显示档 256²/512²/1024² 与 `display_spec`），并对落盘结果做独立复算自检。
- 入口/用法：

```sh
python3 tools/grid_fixtures.py generate --version v6 [--output-dir DIR] [--force]
python3 tools/grid_fixtures.py check    --version v6     # 亦支持 --check（等价 check）
python3 tools/grid_fixtures.py validate --version v6
```

| 参数 | 说明 | 默认 |
| --- | --- | --- |
| `command` | `generate` / `check` / `validate`（缺省由 `--check` 决定，否则 generate） | — |
| `--check` | 等价 check：临时目录重建后与仓库 v6 逐字节比较 | 关闭 |
| `--version` | 夹具版本（当前仅支持 v6） | `v6` |
| `--output-dir` | 生成目标目录 | `tests/fixtures/<version>` |
| `--force` | 目标非空时覆盖（保留清单文件，删除不再预期的文件） | 关闭 |

- 输入/输出：只读 `tests/fixtures/v5`（全量复制来源，绝不写入）与 `tests/fixtures/v3/map/mapping_example.osm` + `.snapshot.json`（P7/P11/P13 地图身份）；写 `tests/fixtures/v6`（或 `--output-dir`）：`grids/` 全部文档、修订后的 `README.md`/`index.json`、v5 其余文件逐字节复制；不写 `fixture_manifest.sha256.json`（由 `fixtures freeze --version v6` 重新生成）。`generate` 默认生成后立即自检（逐字节比较 + 独立复算），失败即报错。
- 依赖：CPython 3.8+ 标准库（`base64`、`hashlib`、`json`、`math`、`tempfile` 等）；无内部模块依赖。
- 调用关系：独立 CLI；与 `tools/fixtures.py` 配合（后者负责清单 freeze/verify，本工具的比较显式排除清单文件）。
- 证据与约定：
  - 显示档由 `splitmix64_thresholds_v1` 确定性生成：`index = seed + r*width + c`（行主序，r=0 为底行），`h % 100` 三态阈值 `<82 free / <94 occupied / 其余 unknown`；seed 20261012/20261013/20261014；`display_spec.json` 的文件 sha256 与 spot_checks（h/v/state）生成时回填并强制复验（generate/check/validate 均复算）。
  - G1–G8 与 P1–P13 的期望值在生成时独立复算核对（不抄录文档数值）；协议 §5.3 的 scene_id 规范序列化冻结示例在每条命令开始时自检。
  - 已知计划文本偏差：G5 `obs-d` 三角形计划底边 y=2.5 与第 2 行单元中心共线，按闭合边界语义会多占 `(2,0)`；生成器以计划期望 cells（8 格）为准，底边上移 0.1 m 为 y=2.6（见源码注释与夹具 notes）。
  - `check` 在临时目录重建并与仓库 v6 逐字节比较（不含清单），不改动仓库；`validate` 直接对仓库目录做逐字节比较 + `revalidate_grids` 独立复算。
- 退出码：0 成功；1 自检/校验失败或版本不支持（`FixtureError` 与比较问题）。
- 源码与测试：`tools/grid_fixtures.py`；`tools/tests/test_grid_fixtures.py`。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
