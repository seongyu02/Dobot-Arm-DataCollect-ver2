#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Workspace Random Pose Sampler

- Dobot E6를 A/B 섹션 근처 XY 그리드로 훑으면서
  · 각 포인트가 도달 가능한지(Reachable)
  를 체크하고 CSV로 저장하는 스크립트.

- 나중에 random pose 수집 시, 이 CSV에서 "안전한" 포인트만 뽑아서
  INIT 포즈로 사용하는 용도.

실행 방법 (터미널):

    python Dobot_E6_Moveit2/src/workspace_random_pose_sampler.py

주의:
- 로봇 작업 공간에 장애물이 없어야 합니다.
- 실행 전에 로봇을 TCP 모드로 전환해 두세요.
"""

import os
import sys
import csv
import time
from typing import Tuple

# 경로 설정 (프로젝트 루트 기준)
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(CURRENT_DIR)  # Dobot_E6_Moveit2
WORKSPACE_ROOT = os.path.dirname(PARENT_DIR)  # TCP-IP-Python-V4
sys.path.insert(0, CURRENT_DIR)
sys.path.insert(0, WORKSPACE_ROOT)

from dobot_e6_controller import DobotE6Controller  # type: ignore

# pick_place_gui_new 에서 좌표, INIT_* 값 재사용
try:
    from pick_place_gui_new import (  # type: ignore
        POS_1,
        POS_2,
        POS_3,
        POS_4,
        POS_5,
        POS_6,
        POS_7,
        POS_8,
        POS_9,
    )
except Exception:
    # 최소한의 fallback (아예 import 실패하는 경우 거의 없겠지만 방어용)
    POS_1 = POS_2 = POS_3 = POS_4 = POS_5 = POS_6 = POS_7 = POS_8 = POS_9 = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    INIT_X = 0.0
    INIT_Y = -350.0
    INIT_Z = 350.0
    INIT_RX = 180.0
    INIT_RY = 0.0
    INIT_RZ = 0.0


def _bounds_for_points(points) -> Tuple[float, float, float, float]:
    """주어진 점 집합의 XY 바운드 계산 (바깥 여유 없이)."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    return x_min, x_max, y_min, y_max


def _point_in_polygon(px: float, py: float, polygon) -> bool:
    """점 (px, py)가 다각형 polygon 내부에 있는지 여부 (ray casting)."""
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi + 1e-9) + xi):
            inside = not inside
        j = i
    return inside

def sample_workspace_grid():
    """
    A/B 섹션 주변 XY 그리드를 훑으면서:
    - IK/워크스페이스 체크
    - 실제 MovJ 시도
    를 CSV로 저장.
    """
    # 1) 로봇 연결
    robot = DobotE6Controller(ip="192.168.5.1")
    if not robot.connect():
        print("✗ Failed to connect robot.")
        return
    print("✓ Robot connected.")

    # 2) XY 그리드 범위 설정
    # - A섹션: 좌표 1~6만 사용 (다각형 내부만)
    # - B섹션: 좌표 6~9 사용 (다각형 내부만)
    a_points_1_to_6 = [POS_1, POS_2, POS_3, POS_4, POS_5, POS_6]
    b_points_6_to_9 = [POS_6, POS_7, POS_8, POS_9]

    a_x_min, a_x_max, a_y_min, a_y_max = _bounds_for_points(a_points_1_to_6)
    b_x_min, b_x_max, b_y_min, b_y_max = _bounds_for_points(b_points_6_to_9)

    # 실제로는 다각형 내부만 사용 (바운딩 박스 바깥 모서리 → 충돌/조인트 한계 위험)
    a_polygon = [(p[0], p[1]) for p in a_points_1_to_6]
    b_polygon = [(p[0], p[1]) for p in b_points_6_to_9]

    # 그리드 스텝 (너무 조밀하지 않게)
    STEP_X = 30.0
    STEP_Y = 30.0

    # Z/자세 설정: Z=101에서만 샘플 (픽/플레이스 높이)
    Z_SAMPLE = 101.0

    print(
        f"[GRID A] X: {a_x_min:.1f} ~ {a_x_max:.1f}, Y: {a_y_min:.1f} ~ {a_y_max:.1f}, Z: {Z_SAMPLE:.1f}"
    )
    print(
        f"[GRID B] X: {b_x_min:.1f} ~ {b_x_max:.1f}, Y: {b_y_min:.1f} ~ {b_y_max:.1f}, Z: {Z_SAMPLE:.1f}"
    )

    # 4) CSV 준비
    save_dir = os.path.join(WORKSPACE_ROOT, "vla_dataset_random_pose")
    os.makedirs(save_dir, exist_ok=True)
    csv_path = os.path.join(save_dir, "workspace_map.csv")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "section",
                "x_mm",
                "y_mm",
                "z_mm",
                "rx_deg",
                "ry_deg",
                "rz_deg",
                "reachable",
                "ik_ok",
                "ik_msg",
                "move_response",
            ]
        )

        # 5) 그리드 스캔 루프 (A섹션 → B섹션 순서)
        for section_name, x_min, x_max, y_min, y_max in (
            ("A", a_x_min, a_x_max, a_y_min, a_y_max),
            ("B", b_x_min, b_x_max, b_y_min, b_y_max),
        ):
            y = y_min
            while y <= y_max + 1e-6:
                x = x_min
                while x <= x_max + 1e-6:
                    # 우선 섹션 다각형 내부인지 확인 (바운딩 박스 모서리 영역 스킵)
                    if section_name == "A":
                        if not _point_in_polygon(x, y, a_polygon):
                            x += STEP_X
                            continue
                    else:
                        if not _point_in_polygon(x, y, b_polygon):
                            x += STEP_X
                            continue

                    # 섹션별 RPY 선택: A는 A섹션 기준 RPY, B는 B섹션 기준 RPY
                    from pick_place_gui_new import (  # type: ignore
                        A_SECTION_RX,
                        A_SECTION_RY,
                        A_SECTION_RZ,
                        B_SECTION_RX,
                        B_SECTION_RY,
                        B_SECTION_RZ,
                    )

                    if section_name == "A":
                        rx_s, ry_s, rz_s = (
                            float(A_SECTION_RX),
                            float(A_SECTION_RY),
                            float(A_SECTION_RZ),
                        )
                    else:
                        rx_s, ry_s, rz_s = (
                            float(B_SECTION_RX),
                            float(B_SECTION_RY),
                            float(B_SECTION_RZ),
                        )

                    # IK/워크스페이스 대략 체크
                    ik_ok, ik_msg = robot.check_ik_solution(
                        x, y, Z_SAMPLE, rx_s, ry_s, rz_s
                    )
                    if not ik_ok:
                        print(
                            f"[SKIP][{section_name}] ({x:.1f},{y:.1f}) IK pre-check fail: {ik_msg}"
                        )
                        writer.writerow(
                            [
                                section_name,
                                f"{x:.3f}",
                                f"{y:.3f}",
                                f"{Z_SAMPLE:.3f}",
                                f"{rx_s:.3f}",
                                f"{ry_s:.3f}",
                                f"{rz_s:.3f}",
                                0,
                                0,
                                ik_msg,
                                "",
                            ]
                        )
                        x += STEP_X
                        continue

                    # 실제 MovJ 시도
                    print(
                        f"[MOVE][{section_name}] ({x:.1f},{y:.1f},Z={Z_SAMPLE:.1f}) 시도 중..."
                    )
                    success = robot.move_j(
                        x,
                        y,
                        Z_SAMPLE,
                        rx_s,
                        ry_s,
                        rz_s,
                        velocity=20.0,
                        coordinate_mode=0,
                        use_waypoint=True,
                    )
                    if success:
                        # move_j 내부에서 에러 코드가 남아 있으면 last_move_response에 문자열이 남음
                        # (예: "0,{591},MovJ(...)" 등) → 이런 경우는 reachable=0 처리
                        if not robot.last_move_response:
                            robot.wait_for_motion_complete(timeout=20.0)

                    move_resp = robot.last_move_response or ""
                    reachable_flag = 1 if (success and not move_resp) else 0

                    writer.writerow(
                        [
                            section_name,
                            f"{x:.3f}",
                            f"{y:.3f}",
                            f"{Z_SAMPLE:.3f}",
                            f"{rx_s:.3f}",
                            f"{ry_s:.3f}",
                            f"{rz_s:.3f}",
                            reachable_flag,
                            1 if ik_ok else 0,
                            ik_msg,
                            move_resp,
                        ]
                    )

                    # 너무 빠르게 반복하지 않도록 약간 대기
                    time.sleep(0.1)
                    x += STEP_X
                y += STEP_Y

    print(f"\n✓ Workspace sampling complete. CSV saved to:\n  {csv_path}")
    robot.disconnect()


def main():
    sample_workspace_grid()


if __name__ == "__main__":
    main()

