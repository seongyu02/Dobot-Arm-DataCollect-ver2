#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pick & Place GUI - Random Initial Pose

- 기반 코드: pick_place_gui_new.PickPlaceGUINew
- Pick/Place, GRIPPER, OpenCV(카메라), A/B 섹션 로직은 모두 그대로 사용.
- 이 파일에서는 "초기 자세(INIT)"만 에피소드마다 랜덤 XYZ로 바꿔준다.
- 초기 자세 RPY는 INIT_SAFE_R* (수직에 가까운 고정값)을 사용한다.
- Pick/Place 이동 시에는 base의 섹션별 RPY(A/B 섹션, DYN_RX 보정, 7-8구간 접근 등)가 그대로 적용된다.
- 데이터 저장 루트만 vla_dataset_random_pose/ 로 변경.
"""

import os
import sys
import random

from PyQt5.QtWidgets import QApplication

import pick_place_gui_new as base


# 이 시나리오 전용 데이터셋 저장 경로
_CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
_WORKSPACE_ROOT = os.path.dirname(os.path.dirname(_CURRENT_DIR))
VLA_DATASET_RANDOM_POSE = os.path.join(_WORKSPACE_ROOT, "vla_dataset_random_pose")

# 초기 대기 자세용 고정 RPY — Travel RPY와 동일하게 설정.
# 이전값(-179.5275)은 Travel RX(+176.5°)와 ±180° 경계를 사이에 두고 반대편에 위치해,
# INIT → 이동 전환 시 관절이 ~356° 회전하며 충돌 알람을 유발했다.
# TRAVEL_RX와 일치시키면 RPY 변화가 0°이므로 경계 통과가 사라진다.
INIT_SAFE_RX = base.TRAVEL_RX   # 176.4624 (was -179.5275)
INIT_SAFE_RY = base.TRAVEL_RY   # -1.7726  (was -2.4369)
INIT_SAFE_RZ = base.TRAVEL_RZ   # 8.1319   (was 2.3663)


def generate_random_initial_pose():
    """
    랜덤 초기 대기 자세 생성.

    - XY: A/B 섹션 다각형 중 하나를 50:50으로 선택해서 내부 랜덤 좌표
    - Z: 150~300mm (카메라 가림 방지를 위해 작업판 위 낮은 구간에서 대기)
    - RPY: INIT_SAFE_R* 고정 (수직에 가까운 자세로 초기 대기)
    """
    if random.random() < 0.5:
        x, y = base.generate_random_point_in_section(base.A_SECTION_POINTS)
    else:
        x, y = base.generate_random_point_in_section(base.B_SECTION_POINTS)

    z = random.uniform(200.0, getattr(base, "Z_MOVE_MAX", 300.0))

    return x, y, z, INIT_SAFE_RX, INIT_SAFE_RY, INIT_SAFE_RZ


class RandomPosePickPlaceStepWorker(base.PickPlaceStepWorker):
    """기본 PickPlaceStepWorker와 동일한 동작.

    - 빨간 블록 감지, X- 탐색, A/B 섹션 로직 모두 base 구현 그대로 사용.
    - Pick/Place 이동 시 A/B 섹션별 RPY, DYN_RX 보정, 7-8구간 접근 RPY 등
      base의 회전값 로직이 그대로 적용된다.
    - RandomPose 시나리오에서 추가 동작이 필요할 때 이 클래스를 오버라이드해서 사용한다.
    """


class PickPlaceGUIRandomPose(base.PickPlaceGUINew):
    """
    base.PickPlaceGUINew 를 상속해서,
    - INIT_* 값을 에피소드마다 랜덤하게 재설정하고
    - 데이터셋 저장 루트를 vla_dataset_random_pose 로 바꾼 버전.
    나머지 Pick/Place 동작은 base 구현을 그대로 따른다.
    """

    def __init__(self):
        super().__init__()
        try:
            self.setWindowTitle("Dobot E6 Pick & Place (RandomPose INIT) - Pose + Camera")
        except Exception:
            pass
        self.vla_dataset_base = VLA_DATASET_RANDOM_POSE
        self.current_init_pose = (
            base.INIT_X,
            base.INIT_Y,
            base.INIT_Z,
            base.INIT_RX,
            base.INIT_RY,
            base.INIT_RZ,
        )

    def _set_new_random_init_pose(self):
        """INIT_* 모듈 변수를 새로운 랜덤 초기 대기 자세로 갱신.

        - XY는 A/B 섹션 랜덤, Z는 150~300mm, RPY는 INIT_SAFE_R* 고정
        - IK 체크를 통해 로봇이 실제로 도달 가능한 좌표만 사용
        - 30회 시도 후 모두 실패하면 마지막 후보를 그대로 사용
        """
        rx, ry, rz = INIT_SAFE_RX, INIT_SAFE_RY, INIT_SAFE_RZ
        ok = False
        msg = ""
        cx = cy = cz = 0.0
        for _ in range(30):
            cx, cy, cz, _, _, _ = generate_random_initial_pose()
            if self.robot:
                ok, msg = self.robot.check_ik_solution(cx, cy, cz, rx, ry, rz)
            else:
                ok, msg = True, "Robot not connected, skipping IK check"
            if ok:
                break
        else:
            try:
                self.log(f"[RandomPose] IK pre-check failed 30 times, using last candidate: {msg}")
            except Exception:
                pass

        base.INIT_X = cx
        base.INIT_Y = cy
        base.INIT_Z = cz
        base.INIT_RX = rx
        base.INIT_RY = ry
        base.INIT_RZ = rz

        self.current_init_pose = (cx, cy, cz, rx, ry, rz)
        try:
            self.log("[RandomPose] 랜덤 포즈로 초기화 (INIT randomized)")
            self.log(
                f"[RandomPose] INIT: "
                f"X={cx:.1f} Y={cy:.1f} Z={cz:.1f} mm, "
                f"RX={rx:.1f} RY={ry:.1f} RZ={rz:.1f} deg (IK ok={ok})"
            )
        except Exception:
            pass

    # 랜덤 INIT 첫 MovJ 실패 시 시도할 고정 대기 포즈 (x,y,z,rx,ry,rz)
    # Z=150~300mm 범위로 유지 (랜덤포즈와 동일 조건)
    _FIXED_INIT = (89.3715, -378.5400, 250.0000, INIT_SAFE_RX, INIT_SAFE_RY, INIT_SAFE_RZ)

    def _reset_init_to_fixed(self):
        """레거시 호환: 랜덤 INIT 사용으로 통일 (고정 초기화 제거)."""
        self._set_new_random_init_pose()

    def run_pick_place_step(self):
        """수동 한 스텝 실행.

        - 매번 새로운 랜덤 초기 자세로 INIT_* 설정 (첫 MovJ 실패 시 고정 포즈로 폴백)
        - Pick/Place 시퀀스는 RandomPosePickPlaceStepWorker를 통해 base와 동일하게 수행
        """
        if not self.robot or not self.robot.connected or not self.gripper:
            self.log("Connect robot first")
            return
        if self.step_worker and self.step_worker.isRunning():
            self.log("Step already running")
            return
        self._set_new_random_init_pose()
        self.current_pick_section = "A"
        self.last_place_x = None
        self.last_place_y = None
        self._begin_step_ui()
        do_record = self.record_20hz_cb.isChecked()
        if do_record:
            self.record_20hz_cb.setEnabled(False)
            self._ensure_camera_for_recording()
        self.step_worker = RandomPosePickPlaceStepWorker(
            self.robot,
            self.gripper,
            pick_section=self.current_pick_section,
            pick_x=None,
            pick_y=None,
            camera=self.camera,
            fallback_initial_pose=self._FIXED_INIT,
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        if do_record:
            self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()

    def run_auto_collect(self, n: int):
        """자동 수집 시작 (20Hz 기록 자동 적용).

        - 시작 시 랜덤 초기 자세 설정 후 Pick/Place 시퀀스 시작 (실패 시 고정 포즈 폴백)
        - 이후 각 에피소드마다 _run_next_auto_step에서 새로운 랜덤 초기 자세 재설정
        """
        if not self.robot or not self.robot.connected or not self.gripper:
            self.log("Connect robot first")
            return
        if self.step_worker and self.step_worker.isRunning():
            self.log("Step already running")
            return
        self._set_new_random_init_pose()
        self.auto_collect_target = n
        self.auto_collect_done = 0
        self.current_pick_section = "A"
        self.last_place_x = None
        self.last_place_y = None
        self._begin_step_ui()
        self.record_20hz_cb.setChecked(True)
        self.record_20hz_cb.setEnabled(False)
        self.log(f"자동 수집 시작: {n}개 (RandomPose)")
        self._ensure_camera_for_recording()
        self.step_worker = RandomPosePickPlaceStepWorker(
            self.robot,
            self.gripper,
            pick_section=self.current_pick_section,
            pick_x=None,
            pick_y=None,
            camera=self.camera,
            fallback_initial_pose=self._FIXED_INIT,
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()

    def _run_next_auto_step(self):
        """자동 수집에서 다음 에피소드 실행.

        - 매 에피소드마다 새로운 랜덤 초기 자세로 INIT_* 갱신 (실패 시 고정 포즈 폴백)
        - Pick 위치/섹션 교대, A/B 섹션 로직은 base와 동일
        """
        if self.auto_collect_target <= 0 or self.auto_collect_done >= self.auto_collect_target:
            return
        if not self.robot or not self.robot.connected or not self.gripper:
            self._end_auto_collect()
            return

        self._set_new_random_init_pose()

        pick_x = self.last_place_x if self.last_place_x is not None else None
        pick_y = self.last_place_y if self.last_place_y is not None else None

        self.step_worker = RandomPosePickPlaceStepWorker(
            self.robot,
            self.gripper,
            pick_section=self.current_pick_section,
            pick_x=pick_x,
            pick_y=pick_y,
            camera=self.camera,
            fallback_initial_pose=self._FIXED_INIT,
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = PickPlaceGUIRandomPose()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()