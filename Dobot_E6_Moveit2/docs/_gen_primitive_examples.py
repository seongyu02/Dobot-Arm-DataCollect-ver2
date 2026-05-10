# -*- coding: utf-8 -*-
"""Generate PRIMITIVE_SEGMENTATION_EXAMPLES.md from 5 sample episodes."""
import csv
import json
import math
import statistics as st
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / "vla_dataset_random_pose"
OUT = Path(__file__).resolve().parent / "PRIMITIVE_SEGMENTATION_EXAMPLES.md"

CLASS = {}
with open(ROOT / "_episode_direction_classification.csv", encoding="utf-8") as f:
    for r in csv.DictReader(f):
        CLASS[int(r["episode"])] = r

LOW_Z = 112.0
MIN_SEG_FRAMES = 10  # 15Hz 기준 667ms 미만 저점은 노이즈로 제거
INIT_MOTION = 3.0
INIT_HOLD_CAP = 25


def read_rows(fp):
    rows = []
    with open(fp, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                rows.append(
                    {
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "z": float(row["z"]),
                        "g1": int(float(row.get("gripper_tooldo1", 0))),
                        "img": row.get("image_path", ""),
                    }
                )
            except Exception:
                pass
    return rows


def low_z_segs(rows):
    segs, on, s = [], False, 0
    for i, r in enumerate(rows):
        low = r["z"] <= LOW_Z
        if low and not on:
            on, s = True, i
        if not low and on:
            segs.append((s, i - 1))
            on = False
    if on:
        segs.append((s, len(rows) - 1))
    return [(a, b) for a, b in segs if b - a + 1 >= MIN_SEG_FRAMES]


def first_motion(rows):
    if not rows:
        return 0
    x0, y0, z0 = rows[0]["x"], rows[0]["y"], rows[0]["z"]
    for i in range(1, len(rows)):
        if math.hypot(rows[i]["x"] - x0, rows[i]["y"] - y0) > INIT_MOTION or abs(
            rows[i]["z"] - z0
        ) > INIT_MOTION:
            return i
    return min(INIT_HOLD_CAP, len(rows) - 1)


def zone_label(z):
    if z == "L":
        return "left"
    if z == "R":
        return "right"
    if z == "M":
        return "middle"
    return z


def segment_five(ep):
    rows = read_rows(ROOT / str(ep) / "robot_data.csv")
    n = len(rows)
    segs = low_z_segs(rows)
    cr = CLASS[ep]
    cls = cr["class"]
    tz = zone_label(cr["place_zone"])

    if len(segs) >= 2:
        p0, p1 = segs[0]
        q0, q1 = segs[1]
        reach_e = p0 - 1
        trans = (p1 + 1, q0 - 1)
        if trans[0] > trans[1]:
            trans = (trans[1], trans[1])
        segs_out = {
            "reach_to_object": (0, reach_e) if reach_e >= 0 else (0, 0),
            "grasp_object": (p0, p1),
            "transport_to_target": trans,
            "release_object": (q0, q1),
            "return_to_init": (q1 + 1, n - 1),
        }
    elif len(segs) == 1:
        p0, p1 = segs[0]
        segs_out = {
            "reach_to_object": (0, max(0, p0 - 1)),
            "grasp_object": (p0, p1),
            "transport_to_target": (p1 + 1, n - 1),
            "release_object": (-1, -1),
            "return_to_init": (-1, -1),
        }
    else:
        segs_out = {
            "reach_to_object": (0, n - 1),
            "grasp_object": (-1, -1),
            "transport_to_target": (-1, -1),
            "release_object": (-1, -1),
            "return_to_init": (-1, -1),
        }
    return rows, segs_out, cls, tz, len(segs)


def main():
    examples = [
        (2, "left_to_right"),
        (1, "right_to_left"),
        (4, "to_middle"),
        (5, "M_to_L"),
        (23, "M_to_R"),
    ]
    lines = []
    lines.append("# Primitive 분할 예시 (5 에피소드)")
    lines.append("")
    lines.append(
        "목표 primitive: `reach_to_object` → `grasp_object` → `transport_to_target` → "
        "`release_object` → `return_to_init`."
    )
    lines.append("에피소드마다 총 프레임 수가 다르므로 구간은 **에피소드별**로만 비교한다.")
    lines.append("")
    lines.append("## 자동 1차 분할 규칙")
    lines.append("- `z <= 112` mm 가 연속 2프레임 이상인 구간 = 저점.")
    lines.append("- 첫 저점 → `grasp_object`, 둘째 저점 → `release_object`.")
    lines.append("- 첫 저점 이전 → `reach_to_object`, 두 저점 사이 → `transport_to_target`, 둘째 저점 이후 → `return_to_init`.")
    lines.append("- `target_zone` 은 메타의 `place_zone` (L/R/M)을 `left`/`right`/`middle` 로 표기.")
    lines.append("")

    for ep, reason in examples:
        rows, segs, cls, tz, nlow = segment_five(ep)
        n = len(rows)
        lines.append(f"## Episode {ep} (`{cls}` — {reason})")
        lines.append(f"- 총 프레임: **{n}** (0 … {n-1})")
        lines.append(f"- 저점 구간 수: {nlow}")
        lines.append(f"- `target_zone` (place 기준): **{tz}**")
        lines.append("")
        lines.append("| primitive | start | end | n_frames | image_start | image_end |")
        lines.append("|-----------|------:|----:|---------:|-------------|-----------|")
        for name in [
            "reach_to_object",
            "grasp_object",
            "transport_to_target",
            "release_object",
            "return_to_init",
        ]:
            a, b = segs[name]
            if a < 0 or b < 0:
                lines.append(f"| {name} | — | — | 0 | — | — |")
                continue
            img0, img1 = rows[a]["img"], rows[b]["img"]
            lines.append(f"| {name} | {a} | {b} | {b - a + 1} | {img0} | {img1} |")
        obj = {
            "episode": ep,
            "high_level_task": "pick_and_place",
            "object": "red_block",
            "target_zone": tz,
            "episode_class": cls,
            "segments": [],
        }
        for name in [
            "reach_to_object",
            "grasp_object",
            "transport_to_target",
            "release_object",
            "return_to_init",
        ]:
            a, b = segs[name]
            if a >= 0 and b >= 0:
                obj["segments"].append({"primitive": name, "start": a, "end": b})
        lines.append("")
        lines.append("```json")
        lines.append(json.dumps(obj, ensure_ascii=False, indent=2))
        lines.append("```")
        lines.append("")

    lines.append("---")
    lines.append("")
    lines.append(
        "전체 정의는 `PRIMITIVE_SEGMENTATION_SPEC.md` 와 동일한 좌표·Z 규칙을 따른다."
    )

    OUT.write_text("\n".join(lines), encoding="utf-8")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
