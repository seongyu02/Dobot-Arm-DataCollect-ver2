# 데이터 수집 파이프라인 (디버깅용 명세)

자동 수집 시 **어떤 API를 쓰고, 어떤 순서로, 어떤 데이터가 어디에 저장되는지** 코드 기준으로 정리한 문서입니다.

---

## 1. 진입점 및 역할 분리

| 구분 | 파일 | 역할 |
|------|------|------|
| **실행 진입** | `pick_place_gui_random_pose.py` → `main()` | `PickPlaceGUIRandomPose` GUI 실행, 저장 루트는 `vla_dataset_random_pose/` |
| **Base GUI** | `pick_place_gui_new.py` | UI, 로봇/그리퍼/카메라 연결, 20Hz 기록 타이머, 저장 로직 |
| **Worker** | `pick_place_gui_new.py` → `PickPlaceStepWorker` (Random Pose는 `RandomPosePickPlaceStepWorker` 상속) | 한 에피소드: 초기 이동 → 1초 대기 → **녹화 시작 시그널** → Pick → Place → 초기 복귀 |
| **로봇 제어** | `dobot_e6_controller.py` | Dashboard(29999) + **Feedback(30004)** TCP 연결, 이동/피드백 |
| **로봇 통신** | `dobot_api.py` (프로젝트 루트) | `DobotApiDashboard`, **DobotApiFeedBack** (피드백 소켓 수신) |
| **그리퍼** | `suction_gripper.py` | DO(디지털 출력) 1번으로 흡착 ON/OFF, **상태는 `is_gripping` 플래그** |
| **카메라** | `Dobot_E6_Moveit2/src/camera_viewer.py` → `HikRobotCamera` | HIKRobot SDK, **get_frame() → (ret, frame)** RGB 640×480 |

---

## 2. 사용 API 요약

### 2.1 로봇 (Dobot E6)

- **Dashboard** (TCP `192.168.5.1:29999`)
  - `EnableRobot()` / `DisableRobot()`
  - `MovJ()`, `MovL()` 등 이동 명령
  - **`ToolDOInstant(index, 0|1)`** → 그리퍼용 DO 제어 (controller에서 `set_digital_output()`로 래핑)
- **Feedback** (TCP `192.168.5.1:30004`)
  - **`DobotApiFeedBack.feedBackData()`**
    - 소켓에서 **1440바이트** 수신 후 `np.frombuffer(..., dtype=MyType)` 파싱
    - 반환 구조체 필드 중 수집에 쓰는 것:
      - **`QActual`**: (6,) float64 — 관절 각도 6개
      - **`ToolVectorActual`**: (6,) float64 — TCP pose **x, y, z, rx, ry, rz**
      - **`RobotMode`**: uint64 — 로봇 상태
  - **블로킹**: `feedBackData()` 한 번 호출 = 소켓 `recv()` 한 번. 호출 간격이 곧 샘플링 간격.

### 2.2 그리퍼

- **물리 제어**: `DobotE6Controller.set_digital_output(do_index=1, True/False)` → Dashboard `ToolDOInstant(1, 0|1)`.
- **녹화 시 “그리퍼 상태”**: **센서가 아님**. `SuctionGripper.is_gripping` 플래그만 사용.
  - `grip()` 호출 시 `is_gripping = True`
  - `release()` 호출 시 `is_gripping = False`
  - 즉, **명령 기준**이며, 실제 흡착 여부는 별도 센서 없음.

### 2.3 카메라

- **HikRobotCamera** (MvImport SDK)
  - `init_camera()`: 장치 열기, 캘리브레이션 로드(선택)
  - **`get_frame()`**: `(bool success, np.ndarray frame)`
    - 내부: `MV_CC_GetOneFrameTimeout(..., 1000)` → Bayer 등 → **640×480 리사이즈** → undistort(캘리 있으면) → **RGB**
  - 수집 시: GUI에서 **RGB → BGR 변환 후** `cv2.imwrite(..., frame_bgr)` (JPEG 저장).

---

## 3. 자동 수집 흐름 (한 에피소드)

1. **사용자**: "자동 수집 N개" 버튼 클릭 → `run_auto_collect(n)` (Random Pose면 `pick_place_gui_random_pose.py`에서 오버라이드).
2. **초기 자세**: Random Pose인 경우 `_set_new_random_init_pose()`로 `base.INIT_X/Y/Z/RX/RY/RZ` 갱신.
3. **Worker 스레드 시작**: `PickPlaceStepWorker`(또는 `RandomPosePickPlaceStepWorker`) `.start()`.
4. **Worker 내부 순서** (`pick_place_gui_new.py`의 `PickPlaceStepWorker.run()`):
   - Pick/Place 목표 계산 (A/B 섹션, 랜덤 등).
   - **1) 초기 위치로 이동**: `_move(INIT_X, INIT_Y, INIT_Z, rx=INIT_RX, ry=INIT_RY, rz=INIT_RZ)`.
   - **1.2) 초기 자세에서 1초 대기**: `time.sleep(0.1)` × 10회.
   - **`recording_begin_at_initial.emit()`** → **여기서부터 20Hz 기록 시작** (아래 4절).
   - 2) Pick 위치로 이동, 2.5) 빨간 블록 X- 탐색(필요 시), 3) Pick 하강 → 그리퍼 ON → 4) 상승 → 5) Place 이동 → 6) Place 하강 → 그리퍼 OFF → 7) 상승 → 8) **초기 위치로 복귀**.
   - `episode_vacuum_durations.emit(pick_hold, place_hold)` → metadata용.
   - `finished.emit(True)`.
5. **GUI**: `on_pick_place_step_finished(True)` → **20Hz 중지 및 저장** `_stop_20hz_recording_and_save(True)`.
6. N개 미만이면 0.3초 후 `_run_next_auto_step()` → 다음 에피소드용 새 랜덤 초기 자세 설정 후 Worker 다시 시작 (반복).

---

## 4. 20Hz 기록 상세

### 4.1 시작 시점

- **시그널**: Worker가 `recording_begin_at_initial.emit()` 호출하는 시점 = **초기 자세 도달 후 1초 대기 직후**.
- **연결**: `step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)` (수동 스텝은 20Hz 체크 시에만 연결).

### 4.2 _start_20hz_recording()

- **저장 경로**: `self.vla_dataset_base` (Random Pose면 `vla_dataset_random_pose/`) 아래 **다음 번호 폴더** 생성 (예: `.../161`).
- **폴더 구조**:  
  `{vla_dataset_base}/{N}/`  
  `{vla_dataset_base}/{N}/images/`
- **상태 초기화**: `recorded_data = []`, `record_frame_count = 0`, `recording = True`.
- **타이머**: `QTimer(self).timeout → _on_record_tick`, **간격 50ms** (`RECORD_INTERVAL_MS = 50` → 20Hz).

### 4.3 _on_record_tick() — 매 50ms마다 호출

**순서 (한 틱 내):**

1. **로봇 피드백 1회**: `feed = self.robot.feed.feedBackData()`
   - **블로킹** 소켓 수신이므로, 이 호출이 지연되면 실제 샘플링 간격은 50ms보다 길어질 수 있음.
2. **추출**:
   - `joints = feed['QActual'][0].tolist()`  # 6개
   - `tcp_pose = feed['ToolVectorActual'][0].tolist()`  # x,y,z,rx,ry,rz
   - `robot_mode = int(feed['RobotMode'][0])`
   - `gripper_on = 1 if (self.gripper and self.gripper.is_gripping) else 0`  # 명령 기준, 센서 없음
3. **이미지**:
   - `ret, frame = self.camera.get_frame()` (RGB 640×480)
   - 성공 시: `frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)` 후 `cv2.imwrite(..., frame_bgr)`.
   - 실패 또는 카메라 없음: `np.zeros((480, 640, 3), dtype=np.uint8)` 블랙 이미지 저장.
4. **파일명**: `frame_{record_frame_count:06d}.jpg` (예: `frame_000000.jpg`).
5. **메모리 적재**:  
   `recorded_data.append({ 'frame_id', 'timestamp', 'image_path', 'joint_angles', 'tcp_pose', 'gripper_tooldo1', 'gripper_tooldo2', 'robot_mode' })`  
   그 다음 `record_frame_count += 1`.

**동기화 정리:**

- **이미지**: 해당 틱에서 `get_frame()` 호출한 **그 순간**의 카메라 프레임.
- **로봇**: 해당 틱에서 `feedBackData()` 호출한 **그 순간**의 로봇 상태 (최신 수신 패킷 1개).
- **그리퍼**: 해당 틱 시점의 **앱 메모리 상 `is_gripping`** (명령 이력 기준).
- **timestamp**: `time.time()` (Python 호출 시각).  
→ **실제 로봇/카메라 하드웨어 타임스탬프가 아닌, PC 기준 기록 시각**이며, 20Hz라고 가정하면 대략 50ms 간격이지만, `feedBackData()` 블로킹·카메라 지연 등으로 **간격이 일정하지 않을 수 있음**.

---

## 5. 저장 (_stop_20hz_recording_and_save)

- **호출**: 스텝이 성공으로 끝난 뒤 `on_pick_place_step_finished(True)` → `_stop_20hz_recording_and_save(True)`.
- **실패 시**: `step_success=False`면 저장 폴더 삭제 후 return (에피소드 미저장).
- **vla_dataset (기본)**: 마지막 프레임 TCP가 `INIT_*` 근처인지 검사. 벗어나면 폴더 삭제 후 return.
- **vla_dataset_random_pose**: 위 **초기 복귀 검증은 건너뜀** (`skip_init_check = True`).

저장 파일:

- **robot_data.csv**  
  - 헤더:  
    `frame_id,timestamp,image_path,j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz,gripper_tooldo1,gripper_tooldo2,robot_mode`  
  - 각 행: `recorded_data[i]`의 필드를 그 순서로 출력.  
  - **joint**: `QActual` 6개. **tcp**: `ToolVectorActual` 6개 (x,y,z,rx,ry,rz).
- **images/frame_000000.jpg, ...**  
  - 이미 위에서 틱마다 저장됨. 여기서는 추가 쓰기 없음.
- **dataset.npy**  
  - `np.save(record_save_dir + "/dataset.npy", self.recorded_data)`  
  - 즉 **메모리 상의 `recorded_data` 리스트**(dict 리스트) 그대로 저장.
- **metadata.txt**  
  - 폴더 번호, 날짜, 총 프레임 수, 20Hz, Step Success, VacuumCommandPickDuration_s, VacuumCommandPlaceDuration_s 등.

---

## 6. 디버깅 시 체크 포인트

| 확인 항목 | 위치 / 의미 |
|-----------|-------------|
| **로봇–이미지 시간 정렬** | 20Hz는 **타이머 50ms + 매 틱 (feedback → image → append)** 순서. feedback이 블로킹이라 틱이 50ms보다 길어지면 timestamp 간격 불균일. |
| **이미지–CSV 행 대응** | `frame_id` = 0부터 순차; `image_path` = `frame_{frame_id:06d}.jpg`. 동일 인덱스가 같은 틱에서 기록됨. |
| **그리퍼 값** | 실제 센서 없음. `gripper_tooldo1` = Worker가 `grip()`/`release()` 호출한 구간만 1/0. |
| **TCP 단위** | `ToolVectorActual`: 위치 mm, 자세는 도(°)인지 라디안인지 dobot_api/로봇 매뉴얼 확인 필요. |
| **초기 복귀 검증** | vla_dataset_random_pose는 마지막 프레임 INIT 검사 생략 → 중간 실패해도 step_success=True면 저장됨. |

---

## 7. 파일/코드 위치 요약

- 20Hz 간격: `pick_place_gui_new.py` → `RECORD_INTERVAL_MS = 50`
- 타이머 연결: `self.record_timer.timeout.connect(self._on_record_tick)`
- 기록 시작: `_start_20hz_recording()` (시그널 `recording_begin_at_initial`로 호출)
- 한 틱: `_on_record_tick()` — `robot.feed.feedBackData()`, `camera.get_frame()`, `recorded_data.append(...)`
- 저장: `_stop_20hz_recording_and_save(step_success)` — CSV/NPY/metadata 작성, (vla_dataset일 때) 마지막 프레임 INIT 검사
- Feedback 구조: `dobot_api.py` → `MyType` (QActual, ToolVectorActual, RobotMode 등), `DobotApiFeedBack.feedBackData()`
- 그리퍼 상태: `suction_gripper.py` → `is_gripping`; DO 제어는 `dobot_e6_controller.set_digital_output()` → Dashboard `ToolDOInstant`

이 문서만 보면 “어디서 무엇을 읽고, 어떤 주기로, 어떤 파일로 쓰는지”까지 디버깅에 필요한 수집 파이프라인을 복원할 수 있습니다.
