# Pick-and-Place GUI 작동 원리

## 📋 전체 구조

```
pick_place_gui.py (GUI)
    ↓
pick_place_logic.py (로직)
    ↓
dobot_e6_controller.py (로봇 제어)
    ↓
dobot_api.py (TCP/IP 통신)
```

---

## 🔄 전체 워크플로우

### 1. 초기화 단계

```python
# GUI 시작 시
1. init_ui() - UI 구성 요소 생성
2. load_config() - robot_config.yaml 로드
   → pick_locations, place_locations 읽기
3. load_camera_calibration() - 카메라 내부 캘리브레이션 로드
   → camera_matrix, dist_coeffs
4. load_transform_matrix() - 카메라-로봇 변환 행렬 로드
   → transform_matrix, translation_offset
5. init_camera() - 카메라 초기화 (선택사항)
```

### 2. 로봇 연결

```python
connect_robot()
    ↓
DobotE6Controller(ip, port).connect()
    ↓
TCP/IP 연결 (Dashboard: 29999, Motion: 30003)
    ↓
EnableRobot(), RequestControl()
    ↓
SuctionGripper 초기화
    ↓
PickAndPlace 객체 생성
```

---

## 🎯 Pick-and-Place 실행 흐름

### 방법 1: 드롭다운에서 선택 후 실행

```
사용자 액션:
1. "Pick Object" 드롭다운에서 선택 (예: object_1)
2. "Place Destination" 드롭다운에서 선택 (예: destination_1)
3. "▶️ Execute Pick-and-Place" 버튼 클릭
```

**코드 흐름:**

```python
execute_pick_and_place() [GUI]
    ↓
PickPlaceWorker 스레드 생성
    ↓
pnp.execute_pick_and_place(object_name, location_name) [pick_place_logic.py]
    ↓
    ├─ go_home() - 홈 위치로 이동
    │   └─ robot.move_j(home_x, home_y, home_z, ...)
    │
    ├─ pick_object(object_name)
    │   ├─ 1. Approach: move_j(x, y, z+150, ...)  # 접근 높이
    │   ├─ 2. Descend: move_l(x, y, z, ...)        # 선형 이동으로 정밀 하강
    │   ├─ 3. Grip: gripper.grip()                 # 석션 ON
    │   └─ 4. Lift: move_l(x, y, z+150, ...)       # 상승
    │
    ├─ place_object(location_name)
    │   ├─ 1. Approach: move_j(x, y, z+150, ...)  # 접근 높이
    │   ├─ 2. Descend: move_l(x, y, z, ...)        # 선형 이동으로 정밀 하강
    │   ├─ 3. Release: gripper.release()           # 석션 OFF
    │   └─ 4. Retract: move_l(x, y, z+150, ...)   # 후퇴
    │
    └─ go_home() - 다시 홈으로
```

---

### 방법 2: 카메라에서 위치 선택 후 실행

```
사용자 액션:
1. "Start Camera" 버튼 클릭 → 카메라 스트리밍 시작
2. "Select Pick Position" 또는 "Select Place Position" 버튼 클릭
3. 카메라 화면에서 원하는 위치 클릭
4. Z 높이 조정 다이얼로그에서 확인/수정
5. 이동 테스트 (선택사항)
6. 위치 저장
7. 드롭다운에서 선택 후 실행
```

**코드 흐름:**

```python
# 1. 카메라 클릭
on_camera_click(event)
    ↓
# 2. 픽셀 좌표 계산 (640x480 기준)
pix_x, pix_y = calculate_pixel_coordinates(event.x(), event.y())
    ↓
# 3. 픽셀 → 로봇 좌표 변환
pixel_to_robot_3d(pix_x, pix_y)
    ↓
    ├─ pixel_homogeneous = [pix_x, pix_y, 1]
    ├─ robot_2d = transform_matrix @ pixel_homogeneous
    ├─ robot_x, robot_y = normalize(robot_2d)
    └─ robot_z = translation_offset (또는 사용자 입력)
    ↓
# 4. 작업 공간 검증
validate_workspace(robot_x, robot_y, robot_z)
    ↓
# 5. Z 높이 조정 다이얼로그
QInputDialog.getDouble("Z Height Adjustment", ...)
    ↓
# 6. 이동 테스트 (선택사항)
test_position_movement(robot_pos)
    ↓
# 7. 위치 저장
save_pick_position() 또는 save_place_position()
    ↓
    └─ config 파일에 저장 (robot_config.yaml)
```

---

## 🔧 핵심 함수 상세 설명

### 1. `pixel_to_robot_3d(pixel_x, pixel_y, z_height=None)`

**목적:** 카메라 픽셀 좌표를 로봇 3D 좌표로 변환

**과정:**
```python
# 1. 동차 좌표로 변환
pixel_homogeneous = [[pixel_x], [pixel_y], [1.0]]

# 2. 변환 행렬 적용 (2D 호모그래피)
robot_2d = transform_matrix @ pixel_homogeneous
# transform_matrix는 3x3 행렬:
# [[a, b, tx],
#  [c, d, ty],
#  [e, f, 1]]

# 3. 정규화
robot_2d_normalized = robot_2d / robot_2d[2, 0]
robot_x = robot_2d_normalized[0, 0]
robot_y = robot_2d_normalized[1, 0]

# 4. Z 좌표 결정
if z_height is None:
    z_height = translation_offset  # 캘리브레이션 시 사용한 Z 높이
robot_z = z_height
```

**변환 행렬의 의미:**
- `transform_matrix`: 픽셀 좌표 → 그리퍼 끝단(TCP) 위치
- 캘리브레이션 시: 체스보드 코너 픽셀 좌표 ↔ 그리퍼 끝단이 코너에 닿을 때의 로봇 좌표
- 실제 사용 시: 객체 픽셀 좌표 → 그리퍼 끝단이 그 위치에 가야 하는 로봇 좌표

---

### 2. `pick_object(object_name)`

**목적:** 지정된 위치에서 객체 집기

**단계:**
```python
# 1. 접근 위치로 이동 (객체 위 150mm)
approach_z = pos['z'] + pick_approach_height  # 예: 50 + 150 = 200mm
robot.move_j(x, y, approach_z, rx, ry, rz, velocity=20.0)
    ↓
# 2. 선형 이동으로 정밀 하강
robot.move_l(x, y, z, rx, ry, rz, velocity=20.0)
    ↓
# 3. 석션 그리퍼 활성화
gripper.grip()  # Digital Output ON
    ↓
# 4. 상승
robot.move_l(x, y, approach_z, rx, ry, rz, velocity=30.0)
```

**이유:**
- `move_j`: 관절 공간 이동 (빠름, 곡선 경로)
- `move_l`: 직교 좌표계 선형 이동 (느림, 직선 경로, 정밀)

---

### 3. `place_object(location_name)`

**목적:** 지정된 위치에 객체 놓기

**단계:**
```python
# 1. 접근 위치로 이동
approach_z = pos['z'] + place_approach_height
robot.move_j(x, y, approach_z, rx, ry, rz, velocity=20.0)
    ↓
# 2. 선형 이동으로 정밀 하강
robot.move_l(x, y, z, rx, ry, rz, velocity=20.0)
    ↓
# 3. 석션 그리퍼 비활성화
gripper.release()  # Digital Output OFF
    ↓
# 4. 후퇴
robot.move_l(x, y, approach_z, rx, ry, rz, velocity=30.0)
```

---

### 4. `test_position_movement(target_pos)`

**목적:** 위치 이동 테스트 (IK 솔루션 검증)

**과정:**
```python
# 1. 사전 검증
- 낮은 Z (< 150mm) + 큰 반경 (> 400mm) → 경고 다이얼로그
- 사용자가 "No" 선택 시 취소

# 2. 여러 전략 시도
전략 1: 직접 이동 (use_waypoint=False)
전략 2: Waypoint 경유 (use_waypoint=True)
전략 3: 안전 위치 경유 후 이동

# 3. 이동 검증
- MovJ 명령 후 1초 대기
- 현재 위치 확인
- 이동 거리 < 10mm → IK 실패로 판단
```

---

## 📊 데이터 흐름

### 설정 파일 로드

```
robot_config.yaml
    ↓
pick_locations: {object_1: {x, y, z, rx, ry, rz}, ...}
place_locations: {destination_1: {x, y, z, rx, ry, rz}, ...}
home: {x, y, z, rx, ry, rz}
pick_approach_height: 150
place_approach_height: 150
```

### 카메라 캘리브레이션 로드

```
hikrobot_calibration_*.npz
    ↓
camera_matrix: 3x3 (내부 파라미터)
dist_coeffs: 5x1 (왜곡 계수)
```

### 변환 행렬 로드

```
camera_robot_transform.json
    ↓
transform_matrix: 3x3 (픽셀 → 로봇 2D 변환)
translation_offset: float (Z 높이 오프셋)
calibration_points: [[pixel_x, pixel_y, robot_x, robot_y, robot_z], ...]
```

---

## ⚙️ 로봇 제어 명령어

### `move_j(x, y, z, rx, ry, rz, ...)`
- **의미:** 관절 공간 이동 (Joint Space)
- **특징:** 빠름, 곡선 경로
- **사용:** 접근/후퇴 시

### `move_l(x, y, z, rx, ry, rz, ...)`
- **의미:** 직교 좌표계 선형 이동 (Linear)
- **특징:** 느림, 직선 경로, 정밀
- **사용:** 하강/상승 시 (객체 접촉 전후)

### `GetPose()`
- **의미:** 현재 로봇 위치 조회
- **반환:** `{x, y, z, rx, ry, rz}`

### `ToolDOInstant(index, status)`
- **의미:** Digital Output 제어 (석션 그리퍼)
- **사용:** `gripper.grip()` → `ToolDOInstant(1, 1)`
- **사용:** `gripper.release()` → `ToolDOInstant(1, 0)`

---

## 🎨 GUI 구성 요소

### 왼쪽 패널: 카메라 뷰
- 카메라 스트리밍 (640x480)
- 마우스 클릭으로 위치 선택
- 실시간 프레임 표시

### 오른쪽 패널: 제어
1. **로봇 연결**
   - IP 주소 입력
   - Connect/Disconnect 버튼

2. **Pick Object 선택**
   - 드롭다운 (config에서 로드)
   - "Add Custom Position" 버튼

3. **Place Destination 선택**
   - 드롭다운 (config에서 로드)
   - "Add Custom Position" 버튼

4. **액션 버튼**
   - 🏠 Go Home
   - 📍 Set Current as Home
   - 📦 Pick Only
   - 📍 Place Only
   - ▶️ Execute Pick-and-Place

5. **로그 창**
   - 실시간 상태 메시지
   - 오류 메시지

---

## 🔍 오류 처리

### IK 솔루션 실패
```python
# MovJ 응답 파싱
if "8047" in response:  # IK 실패
    return False
if "8048" in response:  # 충돌 감지
    return False

# 이동 검증
if distance_moved < 10mm:
    return False  # 실제로 이동하지 않음
```

### 충돌 감지
```python
# 사전 경고
if z < 150mm and radius > 400mm:
    QMessageBox.warning("위험한 위치")
    if user_cancels:
        return False
```

### TCP 모드 확인
```python
if "Control Mode Is Not Tcp" in response:
    return False  # TCP 모드로 전환 필요
```

---

## 📝 요약

1. **초기화**: 설정 파일, 캘리브레이션, 변환 행렬 로드
2. **연결**: 로봇 TCP/IP 연결
3. **위치 선택**: 
   - 드롭다운에서 선택 (config 파일)
   - 또는 카메라에서 클릭 → 픽셀 → 로봇 좌표 변환
4. **실행**: 
   - 홈 → 접근 → 하강 → 그립 → 상승 → 접근 → 하강 → 릴리즈 → 상승 → 홈
5. **제어**: 
   - `move_j`: 빠른 이동 (접근/후퇴)
   - `move_l`: 정밀 이동 (하강/상승)
   - `gripper.grip/release`: 석션 제어

---

## 💡 핵심 개념

- **변환 행렬**: 픽셀 좌표 → 그리퍼 끝단 위치
- **접근 높이**: 객체 위 150mm (충돌 방지)
- **선형 이동**: 정밀한 하강/상승에 사용
- **관절 이동**: 빠른 접근/후퇴에 사용
- **IK 솔루션**: 직교 좌표 → 관절 각도 변환 (로봇 내부 처리)
