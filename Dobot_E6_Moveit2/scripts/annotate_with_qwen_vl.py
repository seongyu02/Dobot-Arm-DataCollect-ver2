#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline batch annotation for vla_dataset_smolvla using Qwen2.5-VL-3B-Instruct.

This script samples representative frames from each episode and generates a
short action-oriented instruction. It stores:
- <episode_dir>/auto_annotation.json
- updated <episode_dir>/episode_meta.json with instruction_auto fields
"""

import argparse
import json
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


DEFAULT_PROMPT = (
    "Here is a current task description: {current_task}. "
    "Generate one short, clear, action-oriented sentence describing the robot arm behavior. "
    "Use a direct action verb (Pick, Place, Open, Push, etc.). Keep it concise."
)


def _iter_episode_dirs(dataset_root: Path) -> Iterable[Path]:
    if not dataset_root.exists():
        return []
    dirs = [p for p in dataset_root.iterdir() if p.is_dir() and p.name.isdigit()]
    return sorted(dirs, key=lambda p: int(p.name))


def _load_json(path: Path) -> Dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _save_json(path: Path, data: Dict):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def _sample_images(episode_dir: Path) -> List[Path]:
    obs1_dir = episode_dir / "images" / "OBS_IMAGE_1"
    obs2_dir = episode_dir / "images" / "OBS_IMAGE_2"
    candidates: List[Path] = []
    for d in (obs1_dir, obs2_dir):
        if d.exists():
            frames = sorted(d.glob("*.jpg"))
            if not frames:
                continue
            picks = [frames[0], frames[len(frames) // 2], frames[-1]]
            for p in picks:
                if p not in candidates:
                    candidates.append(p)
    return candidates[:4]


def _init_vlm_pipeline(model_id: str):
    try:
        from transformers import pipeline

        pipe = pipeline("image-to-text", model=model_id, device_map="auto")
        return pipe
    except Exception as exc:
        print(f"[WARN] Could not initialize VLM pipeline: {exc}")
        return None


def _generate_instruction(pipe, prompt: str, images: List[Path], fallback_task: str) -> Tuple[str, str]:
    if pipe is None or not images:
        base = fallback_task.strip() if fallback_task else "pick and place the target object"
        text = base if base[0].isupper() else base.capitalize()
        return text.rstrip("."), "fallback"
    try:
        result = pipe(str(images[0]), prompt=prompt, max_new_tokens=32)
        if isinstance(result, list) and result:
            item = result[0]
            text = item.get("generated_text", "").strip() if isinstance(item, dict) else str(item).strip()
            if text:
                return text.split("\n")[0].strip(), "qwen_vl"
    except Exception as exc:
        print(f"[WARN] VLM generation failed, using fallback: {exc}")
    base = fallback_task.strip() if fallback_task else "pick and place the target object"
    text = base if base and base[0].isupper() else base.capitalize()
    return text.rstrip("."), "fallback"


def annotate_dataset(dataset_root: Path, model_id: str, max_episodes: Optional[int]) -> Dict[str, int]:
    pipe = _init_vlm_pipeline(model_id)
    processed = 0
    updated = 0

    for idx, episode_dir in enumerate(_iter_episode_dirs(dataset_root)):
        if max_episodes is not None and idx >= max_episodes:
            break
        meta_path = episode_dir / "episode_meta.json"
        meta = _load_json(meta_path)
        current_task = str(meta.get("task_name", "pick and place"))
        images = _sample_images(episode_dir)
        prompt = DEFAULT_PROMPT.format(current_task=current_task)
        instruction, source = _generate_instruction(pipe, prompt, images, current_task)

        annotation = {
            "model_id": model_id,
            "source": source,
            "prompt": prompt,
            "sample_images": [str(p.resolve()) for p in images],
            "instruction_auto": instruction,
        }
        _save_json(episode_dir / "auto_annotation.json", annotation)

        meta["instruction_auto"] = instruction
        meta["instruction_source"] = source
        meta["instruction_model"] = model_id
        _save_json(meta_path, meta)

        processed += 1
        updated += 1
        print(f"[OK] Episode {episode_dir.name}: {instruction}")

    return {
        "episodes_processed": processed,
        "episodes_updated": updated,
    }


def main():
    parser = argparse.ArgumentParser(description="Annotate SmolVLA episodes with Qwen2.5-VL.")
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "vla_dataset_smolvla",
        help="Path to vla_dataset_smolvla directory.",
    )
    parser.add_argument(
        "--model-id",
        type=str,
        default="Qwen/Qwen2.5-VL-3B-Instruct",
        help="Hugging Face model ID.",
    )
    parser.add_argument(
        "--max-episodes",
        type=int,
        default=None,
        help="Optional cap for quick testing.",
    )
    args = parser.parse_args()

    summary = annotate_dataset(args.dataset_root, args.model_id, args.max_episodes)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
