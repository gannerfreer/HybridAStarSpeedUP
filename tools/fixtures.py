"""版本化夹具（``tests/fixtures/<version>/``）的 freeze/verify。

- ``freeze``：生成 ``fixture_manifest.sha256.json``，为按 POSIX 相对路径
  字典序排列的数组，每项 ``{path, size_bytes, sha256}``；不包含清单自身。
- ``verify``：逐项校验存在性、大小与 SHA256，并拒绝清单外额外文件。
- 本模块只使用 CPython 3.8+ 标准库。
"""

from pathlib import Path

from tools import runtime

MANIFEST_NAME = "fixture_manifest.sha256.json"


class FixtureError(RuntimeError):
    pass


def manifest_path(fixture_root):
    return Path(fixture_root) / MANIFEST_NAME


def collect(fixture_root):
    root = Path(fixture_root)
    items = []
    for full in runtime.list_relative_files(root):
        rel = full.relative_to(root).as_posix()
        if rel == MANIFEST_NAME:
            continue
        items.append({
            "path": rel,
            "size_bytes": full.stat().st_size,
            "sha256": runtime.sha256_file(full),
        })
    items.sort(key=lambda item: item["path"])
    return items


def freeze(fixture_root):
    """生成夹具清单；返回写入的条目列表。"""
    root = Path(fixture_root)
    if not root.is_dir():
        raise FixtureError("夹具目录不存在: %s" % root)
    items = collect(root)
    runtime.atomic_write_json(manifest_path(root), items)
    return items


def _safe_relative(rel):
    path = Path(rel)
    if path.is_absolute():
        return False
    return ".." not in path.parts


def verify(fixture_root):
    """校验夹具清单；返回 {ok, problems, checked, file_count}。"""
    root = Path(fixture_root)
    problems = []
    if not root.is_dir():
        return {"ok": False, "problems": ["夹具目录不存在: %s" % root],
                "checked": 0, "file_count": 0}
    try:
        raw = runtime.read_json(manifest_path(root), default=None, strict=True)
    except ValueError as exc:
        raw = None
        problems.append("清单不是有效 JSON: %s" % exc)
    if not isinstance(raw, list):
        problems.append("清单缺失或不是数组: %s" % manifest_path(root))
        return {"ok": False, "problems": problems, "checked": 0, "file_count": 0}

    listed = {}
    paths = []
    for item in raw:
        if not isinstance(item, dict):
            problems.append("清单项不是对象: %r" % (item,))
            continue
        missing = [key for key in ("path", "size_bytes", "sha256") if key not in item]
        if missing:
            problems.append("清单项缺少字段 %s: %r" % (",".join(missing), item))
            continue
        rel = str(item["path"])
        paths.append(rel)
        if rel == MANIFEST_NAME:
            problems.append("清单不得包含自身: %s" % rel)
            continue
        if not _safe_relative(rel):
            problems.append("清单路径非法: %s" % rel)
            continue
        if rel in listed:
            problems.append("清单路径重复: %s" % rel)
            continue
        listed[rel] = item
    if paths != sorted(paths):
        problems.append("清单未按相对路径字典序排列")

    for rel, item in listed.items():
        full = root / rel
        if not full.is_file():
            problems.append("清单文件缺失: %s" % rel)
            continue
        size = full.stat().st_size
        try:
            expected_size = int(item["size_bytes"])
        except (TypeError, ValueError):
            problems.append("size_bytes 非法: %s" % rel)
            continue
        if expected_size != size:
            problems.append("size_bytes 不一致: %s（清单 %d / 实际 %d）"
                            % (rel, expected_size, size))
        actual = runtime.sha256_file(full)
        if item["sha256"] != actual:
            problems.append("sha256 不一致（疑似篡改或缺失更新）: %s" % rel)

    on_disk = set()
    for full in runtime.list_relative_files(root):
        rel = full.relative_to(root).as_posix()
        if rel == MANIFEST_NAME:
            continue
        on_disk.add(rel)
    for rel in sorted(on_disk - set(listed)):
        problems.append("存在清单外额外文件: %s" % rel)

    return {"ok": not problems, "problems": problems,
            "checked": len(listed), "file_count": len(on_disk)}
