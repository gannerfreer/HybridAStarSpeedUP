#!/usr/bin/env python3
"""位级等价核验：比较两个导出目录树中同名 plan_result JSON。

用法: compare_exports.py <dir-base> <dir-cand>
规则:
  * status/reason 必须相同 -> 任一处 state flip 即失败;
  * 每段点数、每点 (x,y,yaw,s,curvature,direction) 逐位 (bitwise) 比较;
  * 位级不一致的数值回退检查 abs diff <= 1e-9，超出即失败;
  * path_length_m / endpoint_* 同样逐位 + 1e-9 兜底;
  * 统计量 (expanded_nodes 等) 记录差异但只作为信息输出。
退出码: 0=全部通过, 1=有失败项, 2=缺文件。
"""
import json
import struct
import sys
from pathlib import Path


def bits(x):
    return struct.unpack("<Q", struct.pack("<d", float(x)))[0]


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    base_root, cand_root = Path(sys.argv[1]), Path(sys.argv[2])

    rels = sorted(p.relative_to(base_root) for p in base_root.rglob("*.json"))
    fails, info = [], []
    n_points = 0
    n_bit_diff = 0
    max_abs = 0.0
    for rel in rels:
        fb, fc = base_root / rel, cand_root / rel
        if not fc.exists():
            fails.append(f"{rel}: missing in candidate")
            continue
        b = json.loads(fb.read_text())
        c = json.loads(fc.read_text())
        tag = str(rel.parent)
        if b.get("status") != c.get("status") or b.get("reason") != c.get("reason"):
            fails.append(f"{tag}: STATUS FLIP {b.get('status')}/{b.get('reason')} -> "
                         f"{c.get('status')}/{c.get('reason')}")
            continue
        bp = [pt for seg in b.get("path", []) for pt in seg.get("points", [])]
        cp = [pt for seg in c.get("path", []) for pt in seg.get("points", [])]
        if len(b.get("path", [])) != len(c.get("path", [])) or len(bp) != len(cp):
            fails.append(f"{tag}: point count {len(bp)} -> {len(cp)}")
            continue
        case_max = 0.0
        for i, (pb, pc) in enumerate(zip(bp, cp)):
            vals = [(pb["pose"]["x_m"], pc["pose"]["x_m"]),
                    (pb["pose"]["y_m"], pc["pose"]["y_m"]),
                    (pb["pose"]["yaw_rad"], pc["pose"]["yaw_rad"]),
                    (pb["s_m"], pc["s_m"]),
                    (pb["curvature_inv_m"], pc["curvature_inv_m"])]
            if pb.get("direction") != pc.get("direction"):
                fails.append(f"{tag}: point {i} direction {pb.get('direction')} -> {pc.get('direction')}")
            for vb, vc in vals:
                n_points += 1
                d = abs(float(vb) - float(vc))
                case_max = max(case_max, d)
                max_abs = max(max_abs, d)
                if bits(vb) != bits(vc):
                    n_bit_diff += 1
                    if d > 1e-9:
                        fails.append(f"{tag}: point {i} |diff|={d:.3e} > 1e-9 ({vb} vs {vc})")
        sb, sc = b.get("statistics", {}), c.get("statistics", {})
        for key in ("path_length_m", "endpoint_position_error_m", "endpoint_yaw_error_rad"):
            if key in sb and key in sc:
                d = abs(float(sb[key]) - float(sc[key]))
                max_abs = max(max_abs, d)
                if bits(sb[key]) != bits(sc[key]):
                    n_bit_diff += 1
                    if d > 1e-9:
                        fails.append(f"{tag}: stat {key} |diff|={d:.3e} > 1e-9")
        for key in ("expanded_nodes", "closed_states", "collision_checks", "rs_distance_calls",
                    "generated_nodes", "motion_rejections", "path_length_m", "total_time_ms"):
            if sb.get(key) != sc.get(key):
                info.append(f"{tag}: stat {key}: {sb.get(key)} -> {sc.get(key)}")
        print(f"[ok] {tag}: status={b.get('status')} points={len(bp)} "
              f"max|d|={case_max:.3e} path_len={sb.get('path_length_m')}")

    print(f"\nfiles={len(rels)} point-values={n_points} bitwise-diffs={n_bit_diff} max|d|={max_abs:.3e}")
    for line in info:
        print("info:", line)
    if fails:
        print("\nFAIL:")
        for line in fails:
            print(" ", line)
        return 1
    print("PASS: 状态翻转 0，所有数值位级一致（或 <= 1e-9）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
