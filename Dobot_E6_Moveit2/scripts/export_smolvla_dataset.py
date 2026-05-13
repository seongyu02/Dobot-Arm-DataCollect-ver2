#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Export `vla_dataset_smolvla` episodes into a LeRobot-like dataset layout.

Output layout:
- <output>/data/chunk-XXX/episode_XXXXXX.parquet
- <output>/videos/chunk-XXX/observation.images.top/episode_XXXXXX.mp4
- <output>/videos/chunk-XXX/observation.images.side/episode_XXXXXX.mp4
- <output>/meta/info.json
- <output>/meta/stats.json
- <output>/meta/episodes.jsonl
- <output>/meta/tasks.jsonl
"""

import argparse
import csv
import json
import math
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import pyarrow as pa
import pyarrow.parquet as pq

JOINT_COLUMNS = [f"j{i}" for i in range(1, 7)]
TCP_COLUMNS = ["x", "y", "z", "rx", "ry", "rz"]
STATE_DIM = 13
ACTION_DIM = 13


@dataclass
class FrameRecord:
    frame_index: int
    timestamp: float
    state: List[float]
    image_top: Optional[Path]
    image_side: Optional[Path]


@dataclass
class EpisodePayload:
    episode_dir: Path
    episode_name: str
    instruction: str
    task_index: int
    frames: List[FrameRecord]
    meta: Dict
    quality: Dict


def _parse_float(value: Optional[str], default: float = 0.0) -> float:
    try:
        return float(value or default)
    except Exception:
        return default


def _parse_float_list(row: Dict[str, str], keys: Sequence[str]) -> List[float]:
    return [_parse_float(row.get(key, "0"), 0.0) for key in keys]


def _iter_episode_dirs(dataset_root: Path) -> Iterable[Path]:
    if not dataset_root.exists():
        return []
    dirs = [p for p in dataset_root.iterdir() if p.is_dir()]

    def _sort_key(path: Path) -> Tuple[int, int, str]:
        if path.name.isdigit():
            return (0, int(path.name), path.name)
        return (1, 0, path.name)

    return sorted(dirs, key=_sort_key)


def _load_json(path: Path) -> Dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _choose_instruction(meta: Dict) -> str:
    candidates = [
        meta.get("instruction_auto"),
        meta.get("instruction"),
        meta.get("task_instruction"),
        meta.get("task_name"),
    ]
    for text in candidates:
        if isinstance(text, str) and text.strip():
            return text.strip()
    return "Pick and place the target object."


def _resolve_image_path(episode_dir: Path, row: Dict[str, str], camera_key: str) -> Optional[Path]:
    row_value = (row.get(f"image_path_{camera_key}") or "").strip()
    if not row_value:
        if camera_key == "OBS_IMAGE_1":
            row_value = (row.get("image_path_hik") or row.get("image_path") or "").strip()
        elif camera_key == "OBS_IMAGE_2":
            row_value = (row.get("image_path_zed") or "").strip()
    if not row_value:
        return None

    value_path = Path(row_value)
    candidates: List[Path] = []
    if value_path.is_absolute():
        candidates.append(value_path)
    else:
        candidates.extend(
            [
                episode_dir / "images" / row_value,
                episode_dir / row_value,
                episode_dir / "images" / camera_key / row_value,
            ]
        )

    for path in candidates:
        if path.exists():
            return path.resolve()
    return None


def _load_episode_payload(episode_dir: Path, task_index: int) -> Optional[EpisodePayload]:
    csv_path = episode_dir / "robot_data.csv"
    if not csv_path.exists():
        return None

    meta = _load_json(episode_dir / "episode_meta.json")
    instruction = _choose_instruction(meta)

    frames: List[FrameRecord] = []
    with csv_path.open("r", encoding="utf-8", newline="") as in_f:
        reader = csv.DictReader(in_f)
        for row in reader:
            joints = _parse_float_list(row, JOINT_COLUMNS)
            tcp = _parse_float_list(row, TCP_COLUMNS)
            gripper = _parse_float(row.get("gripper_tooldo1", "0"), 0.0)
            frame = FrameRecord(
                frame_index=int(_parse_float(row.get("frame_id", "0"), 0.0)),
                timestamp=_parse_float(row.get("timestamp", "0"), 0.0),
                state=joints + tcp + [gripper],
                image_top=_resolve_image_path(episode_dir, row, "OBS_IMAGE_1"),
                image_side=_resolve_image_path(episode_dir, row, "OBS_IMAGE_2"),
            )
            frames.append(frame)

    if not frames:
        return None

    expected_obs1 = len(list((episode_dir / "images" / "OBS_IMAGE_1").glob("*.jpg")))
    expected_obs2 = len(list((episode_dir / "images" / "OBS_IMAGE_2").glob("*.jpg")))
    quality = {
        "csv_rows": len(frames),
        "obs1_images": expected_obs1,
        "obs2_images": expected_obs2,
        "obs1_missing_in_rows": sum(1 for f in frames if f.image_top is None),
        "obs2_missing_in_rows": sum(1 for f in frames if f.image_side is None),
    }
    return EpisodePayload(
        episode_dir=episode_dir,
        episode_name=episode_dir.name,
        instruction=instruction,
        task_index=task_index,
        frames=frames,
        meta=meta,
        quality=quality,
    )


def _build_actions(states: Sequence[List[float]], last_action_policy: str) -> List[List[float]]:
    if not states:
        return []

    actions: List[List[float]] = []
    for idx in range(len(states) - 1):
        cur = states[idx]
        nxt = states[idx + 1]
        delta_joint = [nxt[i] - cur[i] for i in range(6)]
        delta_tcp = [nxt[i] - cur[i] for i in range(6, 12)]
        gripper_next = [nxt[12]]
        actions.append(delta_joint + delta_tcp + gripper_next)

    if not actions:
        actions.append([0.0] * ACTION_DIM)
        return actions

    if last_action_policy == "zero":
        actions.append([0.0] * ACTION_DIM)
    else:
        actions.append(actions[-1][:])
    return actions


def _chunk_name(episode_index: int, episodes_per_chunk: int) -> str:
    chunk_idx = episode_index // episodes_per_chunk
    return f"chunk-{chunk_idx:03d}"


def _write_episode_parquet(
    payload: EpisodePayload,
    episode_index: int,
    data_root: Path,
    episodes_per_chunk: int,
    last_action_policy: str,
) -> Tuple[Path, int]:
    chunk = _chunk_name(episode_index, episodes_per_chunk)
    chunk_dir = data_root / chunk
    chunk_dir.mkdir(parents=True, exist_ok=True)
    parquet_path = chunk_dir / f"episode_{episode_index:06d}.parquet"

    states = [f.state for f in payload.frames]
    actions = _build_actions(states, last_action_policy)
    table = pa.table(
        {
            "frame_index": [f.frame_index for f in payload.frames],
            "timestamp": [f.timestamp for f in payload.frames],
            "observation.state": states,
            "action": actions,
            "task_index": [payload.task_index] * len(payload.frames),
            "episode_index": [episode_index] * len(payload.frames),
        },
        schema=pa.schema(
            [
                pa.field("frame_index", pa.int32()),
                pa.field("timestamp", pa.float64()),
                pa.field("observation.state", pa.list_(pa.float32(), STATE_DIM)),
                pa.field("action", pa.list_(pa.float32(), ACTION_DIM)),
                pa.field("task_index", pa.int32()),
                pa.field("episode_index", pa.int32()),
            ]
        ),
    )
    pq.write_table(table, parquet_path)
    return parquet_path, len(payload.frames)


def _encode_episode_video(image_paths: Sequence[Optional[Path]], out_path: Path, fps: int) -> Dict[str, int]:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    valid_paths = [p for p in image_paths if p is not None and p.exists()]
    if not valid_paths:
        return {"frames_input": 0, "frames_written": 0}

    first = cv2.imread(str(valid_paths[0]))
    if first is None:
        return {"frames_input": len(valid_paths), "frames_written": 0}
    height, width = first.shape[:2]
    fps = int(max(1, fps))

    # Prefer MP4-compatible codecs first; fallback to others only if needed.
    codec_candidates = ["mp4v", "avc1", "H264"]
    for codec in codec_candidates:
        fourcc = cv2.VideoWriter_fourcc(*codec)
        writer = cv2.VideoWriter(str(out_path), fourcc, float(fps), (width, height))
        if not writer.isOpened():
            writer.release()
            continue

        written = 0
        for image_path in valid_paths:
            frame = cv2.imread(str(image_path))
            if frame is None:
                continue
            if frame.shape[:2] != (height, width):
                frame = cv2.resize(frame, (width, height))
            writer.write(frame)
            written += 1
        writer.release()

        # Some OpenCV builds report success but leave an empty file.
        if written > 0 and out_path.exists() and out_path.stat().st_size > 0:
            return {"frames_input": len(valid_paths), "frames_written": written}
        try:
            out_path.unlink(missing_ok=True)
        except Exception:
            pass

    # Final fallback: ffmpeg image concat -> mp4 (if available in PATH).
    ffmpeg_bin = shutil.which("ffmpeg")
    if ffmpeg_bin:
        try:
            with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False, encoding="utf-8") as tf:
                list_path = Path(tf.name)
                for image_path in valid_paths:
                    escaped_path = str(image_path).replace("'", "'\\''")
                    tf.write("file '" + escaped_path + "'\n")

            cmd = [
                ffmpeg_bin,
                "-y",
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "concat",
                "-safe",
                "0",
                "-r",
                str(fps),
                "-i",
                str(list_path),
                "-pix_fmt",
                "yuv420p",
                str(out_path),
            ]
            proc = subprocess.run(cmd, check=False)
            if proc.returncode == 0 and out_path.exists() and out_path.stat().st_size > 0:
                return {"frames_input": len(valid_paths), "frames_written": len(valid_paths)}
        except Exception:
            pass
        finally:
            try:
                list_path.unlink(missing_ok=True)
            except Exception:
                pass

    return {"frames_input": len(valid_paths), "frames_written": 0}


def _write_json(path: Path, data: Dict):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def _write_jsonl(path: Path, rows: Sequence[Dict]):
    with path.open("w", encoding="utf-8") as out_f:
        for row in rows:
            out_f.write(json.dumps(row, ensure_ascii=False) + "\n")


def _update_stats(stats: Dict, values: Sequence[float], key: str):
    if key not in stats:
        stats[key] = {
            "count": 0,
            "sum": [0.0] * len(values),
            "sum_sq": [0.0] * len(values),
            "min": [float("inf")] * len(values),
            "max": [float("-inf")] * len(values),
        }
    blob = stats[key]
    blob["count"] += 1
    for idx, value in enumerate(values):
        blob["sum"][idx] += value
        blob["sum_sq"][idx] += value * value
        blob["min"][idx] = min(blob["min"][idx], value)
        blob["max"][idx] = max(blob["max"][idx], value)


def _finalize_stats(blob: Dict) -> Dict:
    count = max(blob["count"], 1)
    mean = [v / count for v in blob["sum"]]
    var = [max((blob["sum_sq"][i] / count) - mean[i] * mean[i], 0.0) for i in range(len(mean))]
    std = [math.sqrt(v) for v in var]
    return {
        "count": blob["count"],
        "mean": mean,
        "std": std,
        "min": blob["min"],
        "max": blob["max"],
    }


def export_dataset(
    dataset_root: Path,
    output_dir: Path,
    episodes_per_chunk: int,
    fps: int,
    last_action_policy: str,
    max_episodes: Optional[int],
) -> Dict:
    data_root = output_dir / "data"
    videos_root = output_dir / "videos"
    meta_root = output_dir / "meta"
    data_root.mkdir(parents=True, exist_ok=True)
    videos_root.mkdir(parents=True, exist_ok=True)
    meta_root.mkdir(parents=True, exist_ok=True)

    task_to_index: Dict[str, int] = {}
    task_rows: List[Dict] = []
    episodes_rows: List[Dict] = []
    quality_rows: List[Dict] = []
    stats_acc: Dict = {}

    episodes_exported = 0
    episodes_skipped = 0
    frames_exported = 0
    parquet_written = 0

    for idx, episode_dir in enumerate(_iter_episode_dirs(dataset_root)):
        if max_episodes is not None and idx >= max_episodes:
            break
        meta = _load_json(episode_dir / "episode_meta.json")
        instruction = _choose_instruction(meta)
        if instruction not in task_to_index:
            task_to_index[instruction] = len(task_to_index)
            task_rows.append({"task_index": task_to_index[instruction], "task": instruction})
        task_index = task_to_index[instruction]

        payload = _load_episode_payload(episode_dir, task_index)
        if payload is None:
            episodes_skipped += 1
            quality_rows.append(
                {
                    "episode_name": episode_dir.name,
                    "status": "skipped",
                    "reason": "missing robot_data.csv or no valid rows",
                }
            )
            continue

        episode_index = episodes_exported
        parquet_path, row_count = _write_episode_parquet(
            payload=payload,
            episode_index=episode_index,
            data_root=data_root,
            episodes_per_chunk=episodes_per_chunk,
            last_action_policy=last_action_policy,
        )
        parquet_written += 1
        frames_exported += row_count

        chunk_name = _chunk_name(episode_index, episodes_per_chunk)
        top_video_path = (
            videos_root
            / chunk_name
            / "observation.images.top"
            / f"episode_{episode_index:06d}.mp4"
        )
        side_video_path = (
            videos_root
            / chunk_name
            / "observation.images.side"
            / f"episode_{episode_index:06d}.mp4"
        )
        top_video_stats = _encode_episode_video([f.image_top for f in payload.frames], top_video_path, fps=fps)
        side_video_stats = _encode_episode_video([f.image_side for f in payload.frames], side_video_path, fps=fps)

        actions = _build_actions([f.state for f in payload.frames], last_action_policy)
        for frame in payload.frames:
            _update_stats(stats_acc, frame.state, "observation.state")
        for action in actions:
            _update_stats(stats_acc, action, "action")

        episodes_rows.append(
            {
                "episode_index": episode_index,
                "episode_id": payload.episode_name,
                "length": row_count,
                "task_index": payload.task_index,
            }
        )
        quality_rows.append(
            {
                "episode_name": payload.episode_name,
                "episode_index": episode_index,
                "status": "ok",
                "parquet_path": str(parquet_path.resolve()),
                "csv_rows": payload.quality["csv_rows"],
                "obs1_images": payload.quality["obs1_images"],
                "obs2_images": payload.quality["obs2_images"],
                "obs1_missing_in_rows": payload.quality["obs1_missing_in_rows"],
                "obs2_missing_in_rows": payload.quality["obs2_missing_in_rows"],
                "top_video_frames_written": top_video_stats["frames_written"],
                "side_video_frames_written": side_video_stats["frames_written"],
            }
        )
        episodes_exported += 1

    _write_jsonl(meta_root / "tasks.jsonl", sorted(task_rows, key=lambda x: x["task_index"]))
    _write_jsonl(meta_root / "episodes.jsonl", episodes_rows)

    stats_payload = {}
    if "observation.state" in stats_acc:
        stats_payload["observation.state"] = _finalize_stats(stats_acc["observation.state"])
    if "action" in stats_acc:
        stats_payload["action"] = _finalize_stats(stats_acc["action"])
    _write_json(meta_root / "stats.json", stats_payload)

    info_payload = {
        "dataset_name": output_dir.name,
        "dataset_root": str(dataset_root.resolve()),
        "output_root": str(output_dir.resolve()),
        "robot_type": "dobot_e6",
        "fps": fps,
        "episodes_exported": episodes_exported,
        "frames_exported": frames_exported,
        "feature_schema": {
            "frame_index": "int32",
            "timestamp": "float64",
            "observation.state": f"float32[{STATE_DIM}]",
            "action": f"float32[{ACTION_DIM}]",
            "task_index": "int32",
            "episode_index": "int32",
        },
        "camera_mapping": {
            "OBS_IMAGE_1": "observation.images.top",
            "OBS_IMAGE_2": "observation.images.side",
        },
        "path_templates": {
            "parquet": "data/chunk-{chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_top": "videos/chunk-{chunk:03d}/observation.images.top/episode_{episode_index:06d}.mp4",
            "video_side": "videos/chunk-{chunk:03d}/observation.images.side/episode_{episode_index:06d}.mp4",
        },
        "last_action_policy": last_action_policy,
        "episodes_per_chunk": episodes_per_chunk,
    }
    _write_json(meta_root / "info.json", info_payload)

    summary = {
        "dataset_root": str(dataset_root.resolve()),
        "output_root": str(output_dir.resolve()),
        "episodes_exported": episodes_exported,
        "episodes_skipped": episodes_skipped,
        "frames_exported": frames_exported,
        "parquet_written": parquet_written,
        "tasks_written": len(task_rows),
        "quality_rows": quality_rows,
        "artifacts": {
            "info_json": str((meta_root / "info.json").resolve()),
            "stats_json": str((meta_root / "stats.json").resolve()),
            "episodes_jsonl": str((meta_root / "episodes.jsonl").resolve()),
            "tasks_jsonl": str((meta_root / "tasks.jsonl").resolve()),
        },
    }
    _write_json(output_dir / "smolvla_export_summary.json", summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description="Export SmolVLA episodes to LeRobot-like dataset layout.")
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "vla_dataset_smolvla",
        help="Path to source episode directory.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "exports" / "lerobot" / "svla_so100_pickplace",
        help="Path to write LeRobot-style outputs.",
    )
    parser.add_argument(
        "--episodes-per-chunk",
        type=int,
        default=1000,
        help="Episodes per chunk directory.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=20,
        help="FPS for encoded episode videos.",
    )
    parser.add_argument(
        "--last-action-policy",
        type=str,
        choices=["repeat", "zero"],
        default="repeat",
        help="Policy for the last frame action.",
    )
    parser.add_argument(
        "--max-episodes",
        type=int,
        default=None,
        help="Optional cap for quick testing.",
    )
    args = parser.parse_args()

    summary = export_dataset(
        dataset_root=args.dataset_root,
        output_dir=args.output_dir,
        episodes_per_chunk=max(1, args.episodes_per_chunk),
        fps=max(1, args.fps),
        last_action_policy=args.last_action_policy,
        max_episodes=args.max_episodes,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
