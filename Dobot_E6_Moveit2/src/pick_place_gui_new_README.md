# pick_place_gui_new.py — 상세 스크립트 설명

**결론: 이 스크립트는 데이터 수집만 수행합니다. 학습 코드는 전혀 없습니다.**

---

## 1. 문서 상단 요약 (라인 1~14)

| 항목 | 설명 |
|------|------|
| **역할** | Pose 제어 + 카메라 On/Off + 1사이클 Pick-Place (초기→픽→플레이스→초기 복귀) |
| **Base** | `pick_place_gui_moveit.py` |
| **데이터 수집 전용** | `[데이터 수집 전용 — 학습 코드 없음]` 명시 |

### 네이밍 구분 (코드/문서에서의 의미)

| 이름 | 의미 | 스크립트에서 해당하는 부분 |
|------|------|----------------------------|
| **RoboData-Forge** | 로봇 데이터를 정교하게 만든다 | `vla_dataset/` 생성, `robot_data.csv` / `dataset.npy` / `metadata.txt` 저장 |
| **Action-Collector VLA** | Vision-Action 중심 데이터 수집기 | 20Hz 수집 (`RECORD_INTERVAL_MS=50`), 이미지 + 관절/TCP/그리퍼 |
| **Dobot-Behavior-Sync** | 로봇 움직임과 시각 데이터 동기화 | `frame_id` 기준으로 로봇 피드백 + 카메라 프레임 한 쌍으로 저장 |
| **Imitate-Flow** | 모방 학습용 에피소드 흐름 관리 | 초기→픽→플레이스→초기 한 사이클, **성공한 에피소드만** 저장 (마지막 프레임 INIT 검증) |

---

## 2. 스크립트 블록별 상세

### 2.1 인코딩 / 경로 / 임포트 (라인 16~54)

- Windows 콘솔 UTF-8 처리 (라인 24~33).
- `workspace_root` = `TCP-IP-Python-V4`, `current_dir` = `Dobot_E6_Moveit2/src`.
- **로봇**: `DobotE6Controller`, `SuctionGripper`.
- **카메라**: `HikRobotCamera` (선택, 없으면 `CAMERA_AVAILABLE=False`).

→ **학습 관련 import 없음** (torch, tensorflow, keras 등 없음).

---

### 2.2 카메라 스레드 (라인 63~82, CAMERA_AVAILABLE일 때)

- `CameraThread(QThread)`: 약 30fps로 `get_frame()` → `frame_ready` 시그널.
- **역할**: 영상 스트리밍 + 20Hz 기록 시 이미지 소스. 학습 없음.

---

### 2.3 좌표/상수 정의 (라인 84~173)

| 구분 | 상수 | 용도 |
|------|------|------|
| **POS_1~9** | 9개 TCP 좌표 (x,y,z,rx,ry,rz) | 픽/플레이스 영역 정의 |
| **A_SECTION_POINTS** | 1→2→3→4→5→6→7→1 다각형 | A섹션 (픽/플레이스 영역) |
| **B_SECTION_POINTS** | 6→7→8→9→6 사각형 | B섹션 |
| **RELEASE_Z, RELEASE_Z_TOLERANCE_MM** | 101mm, 1.5mm | 그리퍼 해제 높이/허용오차 |
| **INIT_X,Y,Z / INIT_RX,RY,RZ** | 홈(대기) 자세 | 에피소드 시작·종료 기준 (저장 검증에 사용) |
| **PICK_HOLD_TIME_LO/HI, PLACE_HOLD_TIME_LO/HI** | 진공 유지 시간(초) | 픽/플레이스 시 suction 시간 |
| **VLA_DATASET_BASE** | `workspace_root/vla_dataset` | **RoboData-Forge** 출력 디렉터리 |
| **RECORD_INTERVAL_MS** | 50 (20Hz) | **Action-Collector VLA** 수집 주기 |
| **INIT_POSE_TOLERANCE_MM/DEG** | 20mm, 8° | **Imitate-Flow**: 마지막 프레임이 INIT 근처인지 검증 (아니면 저장 안 함) |

---

### 2.4 유틸 함수 (라인 175~204)

- `point_in_polygon(px, py, polygon)`: 점이 다각형 내부인지 (Ray casting).
- `generate_random_point_in_section(section_points)`: A/B 섹션 내 랜덤 (x,y) 생성.

→ 픽/플레이스 위치 샘플링용. 데이터 수집 로직의 일부, 학습 없음.

---

### 2.5 PickPlaceStepWorker (라인 207~396) — **한 에피소드 실행**

**역할**: 한 스텝 = A섹션 Pick → B섹션 Place **또는** B섹션 Pick → A섹션 Place, 끝나면 초기 자세로 복귀.

- **시그널**
  - `finished(bool)`: 스텝 종료
  - `log_signal(str)`: 로그
  - `recording_begin_at_initial`: 초기 자세 1초 유지 후 발신 → **이때 20Hz 기록 시작** (Action-Collector VLA / Dobot-Behavior-Sync)
  - `episode_vacuum_durations(pick_hold, place_hold)`: vacuum 명령 유지 시간 → metadata 기록

- **주요 메서드**
  - `_move`, `_wait_until_z_reached`, `_release_at_safe_z`: 이동/대기/그리퍼 해제.
  - `_safe_return_home`, `_fail_and_go_home`: 실패 시 초기 복귀.
  - `_dist_point_to_line`, `_sample_waypoint_avoiding_line`: 직선 회피 웨이포인트 (궤적 다양화).

- **run() 흐름 (Imitate-Flow)**  
  1) 초기 위치 이동  
  2) 초기 자세 1초 유지 → `recording_begin_at_initial.emit()`  
  3) Pick 위치로 이동 (Z 랜덤 120~200)  
  4) Z=101 강하 → Gripper ON (pick_hold 시간)  
  5) Z 랜덤으로 리프트  
  6) Place 위치로 이동  
  7) Z=101 강하 → Z 도달 대기 → 2초 대기 → Gripper OFF (place_hold)  
  8) Z=200 리프트  
  9) 초기 위치 복귀 → `finished.emit(True)`  

→ **전부 제어·수집 흐름만 있고, 학습/추론 코드 없음.**

---

### 2.6 PickPlaceGUINew (라인 399~817) — GUI 및 20Hz 저장

- **연결/포즈/액션 패널**  
  로봇 연결, Target Pose (MoveJ), Home / Grip / Release / E-STOP.  
  → 수동 제어용.

- **Pick-Place Step 패널**
  - "Run Pick-Place Step (1개)": 1회 실행.
  - "20Hz 기록 (vla_dataset/1,2,3...)": 체크 시 에피소드마다 `vla_dataset/<N>/` 생성 및 기록.
  - 자동 수집: 1, 5, 10, 50번 (연속 에피소드).
  - "다음" 버튼: 자동 수집 시 다음 에피소드 수동 진행용.

- **20Hz 기록 (Dobot-Behavior-Sync + Action-Collector VLA)**
  - `_get_next_folder_number()`: `vla_dataset` 내 다음 번호 (1, 2, 3, ...).
  - `_start_20hz_recording()`: `recording_begin_at_initial` 수신 시 `vla_dataset/<N>/`, `images/` 생성, 50ms 타이머 시작.
  - `_on_record_tick()`: 매 틱마다  
    - 로봇 피드백: `QActual`, `ToolVectorActual`, `RobotMode`, 그리퍼 상태.  
    - 카메라 프레임 → `frame_000000.jpg`, `frame_000001.jpg`, ...  
    - 한 레코드: `frame_id`, `timestamp`, `image_path`, `joint_angles`, `tcp_pose`, `gripper_tooldo1/2`, `robot_mode`.  
  → **frame_id 하나에 이미지 한 장 + 로봇 상태 한 줄**로 동기화 (Dobot-Behavior-Sync).

- **저장 및 검증 (RoboData-Forge + Imitate-Flow)**
  - `_stop_20hz_recording_and_save(step_success)`:
    - `step_success == False` → 해당 에피소드 폴더 삭제, 저장 안 함.
    - 마지막 프레임 TCP가 INIT 근처(20mm, 8°)가 아니면 → 폴더 삭제, 저장 안 함. (**성공 에피소드만 저장**)
    - 저장 내용:
      - `robot_data.csv`: frame_id, timestamp, image_path, j1~j6, x,y,z,rx,ry,rz, gripper_tooldo1/2, robot_mode.
      - `dataset.npy`: 동일 데이터 리스트.
      - `metadata.txt`: 폴더번호, 날짜, 프레임 수, 20Hz, VacuumCommandPickDuration_s, VacuumCommandPlaceDuration_s.

→ **전부 디스크에 데이터 쓰는 로직만 있고, 학습/모델 코드 없음.**

---

### 2.7 카메라 시작/중지, 종료 (라인 764~817)

- `start_camera` / `stop_camera`: HikRobot 카메라 스트리밍.
- `closeEvent`: 카메라 정리, 로봇 연결 시 종료 확인.

---

## 3. 데이터 수집 vs 학습 — 최종 정리

| 구분 | 이 스크립트 |
|------|-------------|
| **데이터 수집** | 있음. 20Hz 이미지 + 관절/TCP/그리퍼, `vla_dataset/<N>/` 저장, 성공 에피소드만 저장. |
| **학습** | 없음. (train, learn, model, optimizer, loss, epoch 등 없음.) |

즉, **데이터 수집만 있고, 학습은 다른 코드/파이프라인에서 수행하는 구조**입니다.

---

## 4. 네이밍과 코드 매핑 요약

- **RoboData-Forge**: `vla_dataset/` 아래 `robot_data.csv`, `dataset.npy`, `metadata.txt` 생성.
- **Action-Collector VLA**: 20Hz 수집, Vision(이미지) + Action(관절/TCP/그리퍼).
- **Dobot-Behavior-Sync**: `frame_id` 기준으로 로봇 피드백과 카메라 프레임 한 쌍으로 저장.
- **Imitate-Flow**: 초기→픽→플레이스→초기 한 사이클, 마지막 프레임 INIT 검증 후 **성공 에피소드만** 저장.

이 문서는 `pick_place_gui_new.py`를 기준으로 작성되었습니다.
