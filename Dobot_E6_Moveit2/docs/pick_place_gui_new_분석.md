# pick_place_gui_new.py 상세 분석

## 1. 스크립트 역할 요약

- **이 스크립트는 학습 코드가 없습니다. 오직 데이터 수집(Data Collection)만 수행합니다.**
- Dobot E6 로봇 제어 + 픽앤플레이스 자동화 + **VLA용 20Hz 시각-동작 데이터 수집**까지 한 번에 처리하는 GUI 도구입니다.

---

## 2. 용도별 이름 구분 (프로젝트 내 역할)

| 이름 | 의미 | 이 스크립트와의 관계 |
|------|------|----------------------|
| **RoboData-Forge** | 로봇 데이터를 정교하게 만들어낸다 | 수집된 데이터가 `vla_dataset/`, `robot_data.csv`, `dataset.npy` 등으로 저장되며, 이후 학습 파이프라인에서 "정제된 로봇 데이터"로 사용됨. 이 스크립트가 그 **데이터를 만드는 도구**에 해당. |
| **Action-Collector VLA** | Vision–Action 중심 데이터 수집기 | **이 스크립트가 곧 Action-Collector VLA.** 20Hz로 이미지 + 관절/ TCP/ 그리퍼 상태를 함께 기록해 VLA( Vision-Language-Action ) 학습용 데이터를 수집함. |
| **Dobot-Behavior-Sync** | 로봇 움직임과 시각 데이터 동기화 수집 | **이 스크립트가 동기화를 담당.** 매 틱(50ms)마다 로봇 피드백(`ToolVectorActual`, `QActual`)과 카메라 프레임을 같은 `frame_id`로 묶어 저장함. |
| **Imitate-Flow** | 모방 학습용 데이터 흐름 관리 | 수집된 에피소드(초기자세→픽→플레이스→초기자세)가 **모방 학습(Imitation Learning)용 트레이젝토리**로 쓰이도록, 성공한 에피소드만 저장·검증하는 흐름을 이 스크립트가 관리함. |

→ **정리:** 이 파일은 **데이터 수집 전용**이며, 학습(train/fit/epoch 등)은 다른 레포/스크립트에서 수행합니다.

---

## 3. 스크립트 구조 (블록별 설명)

### 3.1 상단: 인코딩·경로·임포트 (1–56행)

- Windows에서 콘솔 UTF-8 출력 설정.
- `numpy`, `cv2`, `PyQt5` (GUI), `dobot_e6_controller`, `suction_gripper` 사용.
- `camera_viewer.HikRobotCamera`는 선택: 없으면 `CAMERA_AVAILABLE = False`, 카메라 없이도 로봇 제어·수집 가능.

### 3.2 좌표·상수 정의 (79–167행)

- **POS_1 ~ POS_9**: 워크스페이스 내 9개 고정 포인트 (x,y,z,rx,ry,rz).
- **A_SECTION_POINTS / B_SECTION_POINTS**: 픽/플레이스 영역을 다각형으로 정의. A는 1–7번, B는 6–9번으로 이루어진 영역.
- **RELEASE_Z = 101.0**: 그리퍼 해제 높이(mm). 이 높이까지 내려온 뒤 Place 시 그리퍼 OFF.
- **INIT_X,Y,Z / INIT_RX,RY,RZ**: 대기(홈) 자세. 에피소드 시작·종료는 이 자세로 통일.
- **Z_MOVE_MIN/MAX, RANDOM_Z_*, WAYPOINT_***: 이동 중 Z 높이, 웨이포인트 개수, 노이즈 등 궤적 다양화용.
- **VACUUM_DI_*, LEVEL2_***: 진공 센서(ToolDI)로 픽 성공 여부 확인·재시도(Level2 미세 이동) 관련.
- **RECORD_INTERVAL_MS = 50**: 20Hz 기록 주기.
- **VLA_DATASET_BASE**: `vla_dataset` 루트 경로.
- **INIT_POSE_TOLERANCE_MM/DEG**: 저장 시 "마지막 프레임이 초기 자세에 가까운지" 검증하는 허용 오차.

### 3.3 기하 유틸 (170–201행)

- `point_in_polygon`: Ray casting으로 점이 다각형 내부인지 판단.
- `generate_random_point_in_section`: A/B 섹션 내부에 랜덤 (x,y) 생성 → 픽/플레이스 위치 다양화.

### 3.4 PickPlaceStepWorker (204–396행) — 한 에피소드 실행

- **역할:** "초기자세 → A 또는 B에서 Pick → 반대 섹션 Place → 초기자세 복귀" 한 사이클을 **한 스레드에서 순차 실행**.
- **시그널:** `finished`, `log_signal`, `recording_begin_at_initial`, `episode_vacuum_durations`.
- **주요 메서드:**
  - `_move`: MoveJ로 (x,y,z,rx,ry,rz) 이동.
  - `_wait_until_z_reached`: 피드백으로 Z가 목표값(예: RELEASE_Z) 근처에 도달할 때까지 대기.
  - `_release_at_safe_z`: Z=101로 내린 뒤 그리퍼 해제.
  - `_safe_return_home` / `_fail_and_go_home`: 실패 시 그리퍼 해제 후 초기 자세로 복귀.
  - `_sample_waypoint_avoiding_line`: 직선에서 일정 거리 이상 떨어진 웨이포인트 샘플(현재 스크립트에서는 사용하지 않는 것으로 보임).
- **run() 흐름 요약:**
  1. 초기 위치(INIT)로 이동.
  2. 초기 자세에서 1초 대기 후 `recording_begin_at_initial` 발신 → **이 시점부터 20Hz 기록 시작**.
  3. Pick 영역으로 이동(Z=120~200 랜덤) → Z=101로 하강 → 그리퍼 ON (pick_hold 시간) → Z 다시 올림.
  4. Place 영역으로 이동 → Z=101 하강 → Z 도달 대기 → 2초 대기 후 그리퍼 OFF.
  5. Z=200으로 올린 뒤 초기 위치로 복귀.
  6. `finished.emit(True)` 및 `episode_vacuum_durations` 발신.

→ **학습 코드 없음.** 동작 시퀀스와 제어만 수행.

### 3.5 PickPlaceGUINew (399–817행) — 메인 GUI

- **연결:** IP/Port로 로봇 연결 → `DobotE6Controller` + `SuctionGripper` 생성.
- **Pose 패널:** X,Y,Z,RX,RY,RZ 입력 후 "Move to Pose (MoveJ)"로 단일 포인트 이동.
- **Actions:** Home, Grip, Release, E-STOP.
- **Pick-Place Step:**
  - "Run Pick-Place Step (1개)": 한 에피소드만 실행. "20Hz 기록" 체크 시 해당 에피소드만 `vla_dataset/(다음번호)`에 저장.
  - "자동 수집: 1, 5, 10, 50번": N개 에피소드 연속 수집. 매 에피소드 성공 시 저장 후 다음 에피소드 자동 시작.
  - "다음" 버튼: 자동 수집 중 한 에피소드 끝난 뒤 수동으로 다음 스텝 진행할 때(현재는 자동 진행이라 거의 미노출).
- **20Hz 기록 관련:**
  - `_get_next_folder_number`: `vla_dataset` 안에 1, 2, 3, … 다음 번호 부여.
  - `_ensure_camera_for_recording`: 기록 시 카메라 꺼져 있으면 자동 시작.
  - `_start_20hz_recording`: `recording_begin_at_initial` 수신 시 호출. `vla_dataset/(N)/`, `images/` 생성, 50ms 타이머로 `_on_record_tick` 주기 호출.
  - `_on_record_tick`: 로봇 피드백(joints, tcp_pose, robot_mode) + 그리퍼 상태 + 카메라 프레임을 한 레코드로 묶어 `recorded_data`에 append, 이미지는 `frame_XXXXXX.jpg` 저장.
  - `_stop_20hz_recording_and_save`: 스텝 성공 시에만 저장. **마지막 프레임이 INIT 근처인지 검사**하고, 통과하면 `robot_data.csv`, `dataset.npy`, `metadata.txt` 기록. 실패/초기 복귀 실패 시 해당 폴더 삭제 후 미저장.

→ **전부 제어 + 수집 + 저장 로직이며, 학습 루프는 없음.**

### 3.6 저장 데이터 형식 (vla_dataset/(N)/)

- **images/frame_000000.jpg, ...**: 20Hz로 캡처한 이미지. frame_000000 = 초기 자세 시점.
- **robot_data.csv**: frame_id, timestamp, image_path, j1~j6, x,y,z,rx,ry,rz, gripper_tooldo1/2, robot_mode.
- **dataset.npy**: 위와 동일한 레코드 리스트를 numpy로 저장.
- **metadata.txt**: 폴더 번호, 날짜, 총 프레임 수, 20Hz, Step Success, VacuumCommandPickDuration_s, VacuumCommandPlaceDuration_s.

---

## 4. 데이터 수집 vs 학습 — 최종 정리

| 구분 | 이 스크립트 (pick_place_gui_new.py) |
|------|-------------------------------------|
| **데이터 수집** | ✅ 로봇 제어 + 픽앤플레이스 + 20Hz 이미지·로봇 상태 기록 → `vla_dataset/` 저장 |
| **학습** | ❌ 없음 (train, fit, backward, optimizer, epoch 등 미사용) |

학습은 다른 프로젝트/스크립트에서 `vla_dataset`을 읽어 VLA 또는 Imitation Learning 모델을 훈련하는 구조로 두는 것이 맞습니다.

---

## 5. 자동화 시 자주 쓰는 부분 (체크리스트)

- 로봇 연결: `connect_robot()` → IP/Port.
- 1회 수집: "Run Pick-Place Step (1개)" + "20Hz 기록" 체크.
- N회 연속 수집: "자동 수집" 1/5/10/50번 버튼 (20Hz 자동 적용).
- 저장 조건: 스텝 성공 + **마지막 프레임이 초기 자세(INIT) 근처**일 때만 저장.
- 중단: "Stop" 또는 E-STOP; 실패 시 해당 에피소드 폴더는 삭제되고 저장되지 않음.

이 문서는 `pick_place_gui_new.py`를 **데이터 수집 전용(Action-Collector VLA / Dobot-Behavior-Sync)** 으로 구분하고, 학습은 포함하지 않음을 명시하기 위한 것입니다.
