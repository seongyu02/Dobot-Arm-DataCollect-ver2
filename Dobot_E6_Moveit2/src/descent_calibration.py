#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
130→101 하강 보정용 데이터 수집 (해결 방안 순서 1)

사용 방법:
  [수동] Teach pendant으로 각 위치에서 Z=101.7 자세 취한 뒤:
    python descent_calibration.py
  [자동 스캔] 대표점마다 130→101.7 하강 후 실제 포즈를 읽어 테이블 생성:
    python descent_calibration.py --scan
"""

import sys
import os
import time

_CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if _CURRENT_DIR not in sys.path:
    sys.path.insert(0, _CURRENT_DIR)

# 대표 측정점 (이름, x, y) — 130→101 수직 하강 시 해당 (x,y)에서의 rx, ry, rz 를 측정
DESCENT_CALIB_POINTS = [
    ("4번 (A섹션)", 220.21, -368.72),
    ("9번 (B섹션)", -15.38, -321.49),
    ("7번 (B섹션)", 84.97, -437.89),
    ("8번 (B섹션, 7-8 구간)", -27.02, -438.80),
    ("6번", 94.10, -311.54),
    ("1번 (A 뒤쪽)", 139.37, -435.31),
    ("5번 (A 앞쪽)", 221.63, -318.39),
]

# 측정값 기록용 테이블 (순서 1 완료 후 채움 또는 --scan으로 자동 채움)
# 형식: (x, y, rx, ry, rz) — Z=101.7에서 수직 하강 자세로 측정한 값
DESCENT_CALIB_TABLE = []

# 자동 스캔 시 사용할 하강 RPY/속도 (pick_place_gui_new와 동일)
RELEASE_Z = 101.7
DESCENT_MID_Z = 130.0
DESCENT_RX = 176.4624
DESCENT_RY = -1.7726
DESCENT_RZ = 8.1319
DESCENT_VELOCITY = 12.0   # 스캔 시에는 약간 보수적으로


def get_current_pose_for_calib(robot):
    """로봇 현재 포즈 [x,y,z,rx,ry,rz] 반환."""
    if robot is None:
        return None
    return robot.get_current_pose_from_feedback()


def run_auto_scan(robot, points=None, out_file=None):
    """
    각 대표점에서 130 → 101.7 하강 후 실제 (rx,ry,rz)를 읽어 테이블 생성.

    - points: [(name, x, y), ...] (기본값: DESCENT_CALIB_POINTS)
    - out_file: 결과를 저장할 .py 또는 .txt 경로 (None이면 DESCENT_CALIB_TABLE만 갱신)
    - 반환: [(x, y, rx, ry, rz), ...]
    """
    if points is None:
        points = DESCENT_CALIB_POINTS
    table = []
    for name, x, y in points:
        print(f"[스캔] {name} (x={x:.1f}, y={y:.1f}) ... ", end="", flush=True)
        # 1) (x, y, 130)으로 이동
        ok = robot.move_j(x, y, DESCENT_MID_Z, DESCENT_RX, DESCENT_RY, DESCENT_RZ,
                          coordinate_mode=0, velocity=18.0, use_waypoint=False)
        if not ok:
            print("실패 (130 이동)")
            continue
        robot.wait_for_motion_complete()
        time.sleep(0.3)
        # 2) (x, y, 101.7)로 하강
        ok = robot.move_j(x, y, RELEASE_Z, DESCENT_RX, DESCENT_RY, DESCENT_RZ,
                          coordinate_mode=0, velocity=DESCENT_VELOCITY, use_waypoint=False)
        if not ok:
            print("실패 (101.7 하강)")
            # 130으로 다시 올리기
            robot.move_j(x, y, DESCENT_MID_Z, DESCENT_RX, DESCENT_RY, DESCENT_RZ,
                         coordinate_mode=0, velocity=18.0, use_waypoint=False)
            continue
        robot.wait_for_motion_complete()
        time.sleep(0.2)
        # 3) 실제 포즈 읽기
        pose = get_current_pose_for_calib(robot)
        if pose is None or len(pose) < 6:
            print("실패 (포즈 읽기)")
            continue
        rx, ry, rz = float(pose[3]), float(pose[4]), float(pose[5])
        # 목표 (x,y) 사용, 실제 측정 rx,ry,rz 사용 (z는 101.7 근사)
        table.append((x, y, rx, ry, rz))
        print(f"OK  rx={rx:.2f} ry={ry:.2f} rz={rz:.2f}")
        # 4) 다음 점으로 가기 전 Z=200 정도로 올림
        robot.move_j(x, y, 200.0, DESCENT_RX, DESCENT_RY, DESCENT_RZ,
                     coordinate_mode=0, velocity=20.0, use_waypoint=False)
        robot.wait_for_motion_complete()
        time.sleep(0.2)

    if out_file:
        with open(out_file, "w", encoding="utf-8") as f:
            f.write("# 130→101 하강 보정 테이블 (자동 스캔)\n")
            f.write("DESCENT_CALIB_TABLE = [\n")
            for x, y, rx, ry, rz in table:
                f.write(f"    ({x:.4f}, {y:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}),\n")
            f.write("]\n")
        print(f"[저장] {out_file}")

    globals()["DESCENT_CALIB_TABLE"] = table
    return table


def run_once_print_pose():
    """로봇 연결 후 현재 포즈 한 번 읽어서 출력 (수동 측정 시 복사용)."""
    try:
        from dobot_e6_controller import DobotE6Controller
    except Exception:
        print("dobot_e6_controller import 실패. 경로 확인.")
        return
    robot = DobotE6Controller()
    if not robot.connect():
        print("로봇 연결 실패. IP/포트 확인.")
        return
    pose = get_current_pose_for_calib(robot)
    robot.disconnect()
    if pose is None or len(pose) < 6:
        print("포즈 읽기 실패.")
        return
    x, y, z, rx, ry, rz = pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
    print("--- 현재 포즈 (복사해서 DESCENT_CALIB_TABLE에 추가) ---")
    print(f"  ({x:.4f}, {y:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}),  # z={z:.2f}")
    print("---")


if __name__ == "__main__":
    if "--scan" in sys.argv:
        try:
            from dobot_e6_controller import DobotE6Controller
        except Exception:
            print("dobot_e6_controller import 실패.")
            sys.exit(1)
        robot = DobotE6Controller()
        if not robot.connect():
            print("로봇 연결 실패.")
            sys.exit(1)
        out = os.path.join(_CURRENT_DIR, "descent_calib_result.py")
        run_auto_scan(robot, out_file=out)
        robot.disconnect()
        print("자동 스캔 완료. 결과는 descent_calib_result.py 에서 확인 후 DESCENT_CALIB_TABLE 로 복사하세요.")
    else:
        print("대표 측정점 목록 (해당 위치에서 Z=101.7 수직 자세 취한 뒤 이 스크립트 실행):")
        for name, x, y in DESCENT_CALIB_POINTS:
            print(f"  {name}: x={x:.2f}, y={y:.2f}")
        print("\n자동 스캔:  python descent_calibration.py --scan\n")
        run_once_print_pose()
