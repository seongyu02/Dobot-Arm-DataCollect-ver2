#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Define Phase1 GUI — copied from pick_place_gui_random_pose pattern; random_pose unchanged."""
import json
import os
import sys
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QCheckBox, QComboBox, QGroupBox, QHBoxLayout, QLabel, QVBoxLayout
import pick_place_gui_new as base

# pick_place_gui_random_pose 와 동일: src -> Dobot_E6_Moveit2 -> TCP-IP-Python-V4
_SRC_DIR = os.path.dirname(os.path.abspath(__file__))
_WORKSPACE_ROOT = os.path.dirname(os.path.dirname(_SRC_DIR))
VLA_DATASET_DEFINE = os.path.join(_WORKSPACE_ROOT, "vla_dataset_define")
# 방향별 하위 폴더: vla_dataset_define/ab | vla_dataset_define/ba (에피소드 번호는 각각 독립)
_SUBDIR_A_TO_B = "ab"
_SUBDIR_B_TO_A = "ba"


def _dataset_root_for_direction(direction: str) -> str:
    sub = _SUBDIR_A_TO_B if direction == "A_to_B" else _SUBDIR_B_TO_A
    return os.path.join(VLA_DATASET_DEFINE, sub)


_FIXED_INIT_MM = (89.3715, -102.9836, 611.7122, 90.1244, 3.6761, 5.7400)
TEXT_PROMPT_A_TO_B = "pick up the red block from section A and place it in section B"
TEXT_PROMPT_B_TO_A = "pick up the red block from section B and place it in section A"


def _apply_fixed_init_to_base_module():
    base.INIT_X, base.INIT_Y, base.INIT_Z = _FIXED_INIT_MM[0], _FIXED_INIT_MM[1], _FIXED_INIT_MM[2]
    base.INIT_RX, base.INIT_RY, base.INIT_RZ = _FIXED_INIT_MM[3], _FIXED_INIT_MM[4], _FIXED_INIT_MM[5]


class PickPlaceGUIDefine(base.PickPlaceGUINew):
    def __init__(self):
        super().__init__()
        self._phase1_skip_episode_alternate = True
        try:
            self.setWindowTitle("Dobot E6 Pick & Place (Define / Phase 1) - Pose + Camera")
        except Exception:
            pass
        # 기본은 A_to_B 저장 경로; 스텝 시작 시 방향에 맞게 갱신됨
        self.vla_dataset_base = _dataset_root_for_direction("A_to_B")
        self.current_init_pose = (base.INIT_X, base.INIT_Y, base.INIT_Z, base.INIT_RX, base.INIT_RY, base.INIT_RZ)
        self._setup_phase1_widgets()

    def _setup_phase1_widgets(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Phase1 episode_direction:"))
        self.phase1_direction_combo = QComboBox()
        self.phase1_direction_combo.addItems(["A_to_B", "B_to_A"])
        self.phase1_direction_combo.setCurrentIndex(0)
        row.addWidget(self.phase1_direction_combo)
        row.addStretch()
        self.phase1_chain_ab_to_ba_cb = QCheckBox(
            "A→B 성공 후 B→A 자동 (pick=직전 place, 메타에 source_xy_origin)"
        )
        self.phase1_chain_ab_to_ba_cb.setChecked(False)
        col = QVBoxLayout()
        col.addLayout(row)
        col.addWidget(self.phase1_chain_ab_to_ba_cb)
        g = QGroupBox("Phase 1 (Define)")
        g.setLayout(col)
        central = self.centralWidget()
        right_l = central.layout().itemAt(1).layout()
        right_l.insertWidget(4, g)

    def _apply_fixed_init_pose(self):
        _apply_fixed_init_to_base_module()
        self.current_init_pose = (base.INIT_X, base.INIT_Y, base.INIT_Z, base.INIT_RX, base.INIT_RY, base.INIT_RZ)
        try:
            self.log(f"[Define] fixed INIT: X={base.INIT_X:.1f} Y={base.INIT_Y:.1f} Z={base.INIT_Z:.1f} mm")
        except Exception:
            pass

    def _current_phase1_direction(self):
        return self.phase1_direction_combo.currentText()

    def _set_dataset_base_for_direction(self, direction: str):
        """에피소드 저장 시 사용할 루트: vla_dataset_define/ab 또는 .../ba."""
        self.vla_dataset_base = _dataset_root_for_direction(direction)
        try:
            self.log(f"[Define] dataset root for this episode: {self.vla_dataset_base}")
        except Exception:
            pass

    def _stop_20hz_recording_and_save(self, step_success):
        save_dir = self.record_save_dir
        sw = self.step_worker
        meta_extra = None
        if step_success and sw and getattr(sw, "phase1_episode_direction", None) and save_dir:
            d = sw.phase1_episode_direction
            prompt = TEXT_PROMPT_A_TO_B if d == "A_to_B" else TEXT_PROMPT_B_TO_A
            meta_extra = {
                "phase": "phase1",
                "dataset_root_relative": f"vla_dataset_define/{_SUBDIR_A_TO_B if d == 'A_to_B' else _SUBDIR_B_TO_A}",
                "episode_direction": d,
                "pick_section": "A" if d == "A_to_B" else "B",
                "place_section": "B" if d == "A_to_B" else "A",
                "object_region": "A_center_small" if d == "A_to_B" else "B_center_small",
                "target_region": "B_center_small" if d == "A_to_B" else "A_center_small",
                "init_type": "fixed",
                "init_gate_passed": True,
                "approach_type": "top_down_pregrasp_vertical",
                "episode_type": "single_pick_place",
                "prompt": prompt,
                "sync_policy": base.SYNC_POLICY_STRING,
                "pick_xy_mm_metadata_only": [float(getattr(sw, "episode_pick_x", 0)), float(getattr(sw, "episode_pick_y", 0))],
                "place_xy_mm_metadata_only": [float(sw.place_x) if sw.place_x is not None else None, float(sw.place_y) if sw.place_y is not None else None],
                "note_pick_place_xy": "metadata for collection/analysis; model input is separate decision.",
                "source_xy_origin": getattr(sw, "phase1_pick_xy_origin", None),
                "target_xy_origin": getattr(sw, "phase1_place_xy_origin", None),
            }
            o = meta_extra.get("source_xy_origin")
            if d == "B_to_A" and o == "previous_episode_place":
                meta_extra["chained_from_previous_episode"] = True
                meta_extra["previous_ab_place_xy_mm"] = [
                    float(sw.pick_x) if getattr(sw, "pick_x", None) is not None else None,
                    float(sw.pick_y) if getattr(sw, "pick_y", None) is not None else None,
                ]
        super()._stop_20hz_recording_and_save(step_success)
        if meta_extra and save_dir and os.path.isdir(save_dir):
            try:
                path = os.path.join(save_dir, "episode_metadata.json")
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(meta_extra, f, indent=2, ensure_ascii=False)
                self.log(f"[Define] episode_metadata.json -> {path}")
            except Exception as e:
                self.log(f"[Define] episode_metadata.json error: {e}")

    def on_pick_place_step_finished(self, success):
        if not getattr(self, "_phase1_skip_episode_alternate", False):
            return super().on_pick_place_step_finished(success)
        do_record = self.record_20hz_cb.isChecked()
        save_ok = False
        sw = self.step_worker
        if do_record:
            try:
                save_ok = bool(success) and (not self.abort_current_episode)
                if self.abort_current_episode:
                    self.log("Stopped: episode folder not saved.")
                self._stop_20hz_recording_and_save(save_ok)
            except Exception as e:
                self.log(f"20Hz save error: {e}")
                self.recording = False
                self.record_timer.stop()
                self.recorded_data.clear()
                self.record_save_dir = None
        self.abort_current_episode = False
        if self.auto_collect_target > 0:
            self.auto_collect_done += 1
            self.log(f"Auto collect: {self.auto_collect_done}/{self.auto_collect_target} (Define Phase1)")
            if self.auto_collect_done >= self.auto_collect_target:
                self._end_auto_collect()
                return
            QTimer.singleShot(300, self._run_next_auto_step)
        else:
            chain = (
                getattr(self, "phase1_chain_ab_to_ba_cb", None)
                and self.phase1_chain_ab_to_ba_cb.isChecked()
                and do_record
                and save_ok
                and success
                and sw
                and getattr(sw, "phase1_episode_direction", None) == "A_to_B"
                and sw.place_x is not None
                and sw.place_y is not None
            )
            if chain:
                self.phase1_direction_combo.setCurrentIndex(1)
                self.log(
                    "[Define] Chaining B_to_A: fixed init + visible gate, pick from previous place_xy (metadata)"
                )
                px, py = float(sw.place_x), float(sw.place_y)
                QTimer.singleShot(400, lambda p=px, q=py: self._run_chained_b_to_a_step(p, q))
            else:
                self._end_step_ui(auto_collect_active=False)
                self.update_status()
                self.log("Pick-Place step done (Define Phase1).")

    def run_pick_place_step(self):
        if not self.robot or not self.robot.connected or not self.gripper:
            self.log("Connect robot first")
            return
        if self.step_worker and self.step_worker.isRunning():
            self.log("Step already running")
            return
        self._apply_fixed_init_pose()
        direction = self._current_phase1_direction()
        self._set_dataset_base_for_direction(direction)
        self.current_pick_section = "A" if direction == "A_to_B" else "B"
        self.last_place_x = None
        self.last_place_y = None
        self._begin_step_ui()
        do_record = self.record_20hz_cb.isChecked()
        if do_record:
            self.record_20hz_cb.setEnabled(False)
            self._ensure_camera_for_recording()
        self.step_worker = base.PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section=self.current_pick_section,
            pick_x=None, pick_y=None,
            camera=self.camera,
            fallback_initial_pose=_FIXED_INIT_MM,
            phase1_episode_direction=direction,
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        if do_record:
            self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()

    def _run_chained_b_to_a_step(self, place_x: float, place_y: float):
        """B_to_A after A_to_B: fixed init again, visible gate in worker; pick uses previous place_xy."""
        if not self.robot or not self.robot.connected or not self.gripper:
            self.log("[Define] Chain B_to_A: connect robot first")
            self._end_step_ui(auto_collect_active=False)
            return
        if self.step_worker and self.step_worker.isRunning():
            self.log("[Define] Chain B_to_A: step already running")
            return
        self._apply_fixed_init_pose()
        self.phase1_direction_combo.setCurrentIndex(1)
        self._set_dataset_base_for_direction("B_to_A")
        self.current_pick_section = "B"
        self.last_place_x = None
        self.last_place_y = None
        self._begin_step_ui()
        do_record = self.record_20hz_cb.isChecked()
        if do_record:
            self.record_20hz_cb.setEnabled(False)
            self._ensure_camera_for_recording()
        self.step_worker = base.PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section="B",
            pick_x=place_x,
            pick_y=place_y,
            camera=self.camera,
            fallback_initial_pose=_FIXED_INIT_MM,
            phase1_episode_direction="B_to_A",
            phase1_pick_xy_origin="previous_episode_place",
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        if do_record:
            self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()

    def run_auto_collect(self, n: int):
        if not self.robot or not self.robot.connected or not self.gripper:
            self.log("Connect robot first")
            return
        if self.step_worker and self.step_worker.isRunning():
            self.log("Step already running")
            return
        self._apply_fixed_init_pose()
        direction = self._current_phase1_direction()
        self._set_dataset_base_for_direction(direction)
        self.auto_collect_target = n
        self.auto_collect_done = 0
        self.current_pick_section = "A" if direction == "A_to_B" else "B"
        self.last_place_x = None
        self.last_place_y = None
        self._begin_step_ui()
        self.record_20hz_cb.setChecked(True)
        self.record_20hz_cb.setEnabled(False)
        self.log(f"Auto collect: {n} (Define Phase1, {direction})")
        self._ensure_camera_for_recording()
        self.step_worker = base.PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section=self.current_pick_section,
            pick_x=None, pick_y=None,
            camera=self.camera,
            fallback_initial_pose=_FIXED_INIT_MM,
            phase1_episode_direction=direction,
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()

    def _run_next_auto_step(self):
        if self.auto_collect_target <= 0 or self.auto_collect_done >= self.auto_collect_target:
            return
        if not self.robot or not self.robot.connected or not self.gripper:
            self._end_auto_collect()
            return
        self._apply_fixed_init_pose()
        direction = self._current_phase1_direction()
        self._set_dataset_base_for_direction(direction)
        self.current_pick_section = "A" if direction == "A_to_B" else "B"
        self.last_place_x = None
        self.last_place_y = None
        self.step_worker = base.PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section=self.current_pick_section,
            pick_x=None, pick_y=None,
            camera=self.camera,
            fallback_initial_pose=_FIXED_INIT_MM,
            phase1_episode_direction=direction,
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = PickPlaceGUIDefine()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
