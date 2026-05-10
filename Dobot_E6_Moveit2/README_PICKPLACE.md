# Dobot E6 Pick-and-Place GUI Application

PyQt5 GUI 애플리케이션으로 Dobot E6 로봇을 TCP/IP로 제어하여 석션 그리퍼로 pick-and-place 작업 수행

## 📋 기능

- **로봇 연결**: IP 주소로 Dobot E6에 TCP 연결
- **목표물 선택**: 사전 정의된 pick 위치 선택
- **목적지 선택**: 사전 정의된 place 위치 선택
- **석션 제어**: Digital Output으로 석션 그리퍼 ON/OFF
- **자동 시퀀스**: 완전 자동 pick-and-place 실행
- **수동 제어**: Pick only, Place only, Home 독립 실행
- **상태 모니터링**: 실시간 로봇 상태 및 로그 표시
- **비상 정지**: 긴급 상황 시 즉시 정지

## 🗂️ 파일 구조

```
Dobot_E6_Moveit2/
├── src/
│   ├── dobot_e6_controller.py   # TCP/IP 통신 모듈
│   ├── suction_gripper.py       # 석션 그리퍼 제어
│   ├── pick_place_logic.py      # Pick-and-place 로직
│   └── pick_place_gui.py        # PyQt5 GUI 메인
├── config/
│   └── robot_config.yaml        # 로봇 설정 및 위치
└── README_PICKPLACE.md          # 이 파일
```

## 🚀 실행 방법

### 1. 설정 파일 수정

`config/robot_config.yaml` 파일을 열어 로봇 IP와 위치를 설정:

```yaml
robot:
  ip: "192.168.1.6"  # 로봇 IP 주소로 변경
```

Pick 위치와 Place 위치를 실제 좌표로 수정:

```yaml
pick_locations:
  object_1:
    x: 300    # mm
    y: 200
    z: 50
    rx: 180   # degrees
    ry: 0
    rz: 0
```

### 2. GUI 실행

```bash
cd /home/sunbi/Dobot_E6_Moveit2
python3 src/pick_place_gui.py
```

### 3. 로봇 연결

1. GUI에서 IP 주소 확인/수정
2. "Connect" 버튼 클릭
3. 연결 성공 시 "🟢 Connected" 표시

### 4. Pick-and-Place 실행

1. **목표물 선택**: Pick Object 드롭다운에서 선택
2. **목적지 선택**: Place Destination 드롭다운에서 선택
3. **실행**: "▶️ Execute Pick-and-Place" 버튼 클릭

또는 개별 동작:
- "🏠 Home": 홈 위치로 이동
- "📦 Pick Only": 선택한 물체만 집기
- "📍 Place Only": 현재 물체를 선택한 위치에 놓기

## ⚠️ 안전 주의사항

> [!CAUTION]
> **첫 실행 전 필수 확인 사항**

1. **작업 공간 확인**: 로봇 동작 범위 내에 장애물 없는지 확인
2. **비상 정지 준비**: 물리적 비상 정지 버튼 위치 파악
3. **느린 속도로 테스트**: 처음에는 느린 속도로 경로 확인
4. **좌표 검증**: 각 위치를 개별적으로 테스트 후 자동 시퀀스 실행

> [!WARNING]
> **운영 중 주의사항**
> - 로봇 동작 중 작업 공간에 들어가지 마세요
> - 이상 동작 시 즉시 "🛑 EMERGENCY STOP" 버튼 클릭
> - 석션이 작동하지 않으면 물체가 떨어질 수 있습니다

## 🔧 설정 커스터마이징

### Pick/Place 위치 추가

`config/robot_config.yaml`에서:

```yaml
pick_locations:
  my_object:    # 새 이름
    x: 350
    y: 150
    z: 45
    rx: 180
    ry: 0
    rz: 0

place_locations:
  my_destination:  # 새 이름
    x: 350
    y: -150
    z: 45
    rx: 180
    ry: 0
    rz: 0
```

### 석션 설정 변경

```yaml
gripper:
  do_index: 1           # Digital Output 번호
  grip_wait_time: 0.5   # 석션 안정화 대기 시간 (초)
  release_wait_time: 0.3
```

### 속도 조정

```yaml
movement:
  default_velocity: 50.0      # 기본 속도 (%)
  approach_velocity: 30.0     # 접근 속도 (느리게)
  retreat_velocity: 50.0
```

## 📊 GUI 구성

### 연결 패널
- IP 주소 및 포트 설정
- Connect/Disconnect 버튼
- 연결 상태 표시

### 제어 패널
- **Pick Object**: 목표물 선택
- **Place Destination**: 목적지 선택
- **Actions**: 
  - Home: 홈 위치
  - Pick Only: 집기만
  - Place Only: 놓기만
  - Execute: 전체 시퀀스
  - Emergency Stop: 긴급 정지

### 상태 패널
- Current Position: 현재 위치
- Gripper Status: 그리퍼 상태 (Gripping/Released)
- Operation Status: 동작 상태 (Idle/Picking/Placing/etc)

### 로그 패널
- 실시간 로그 메시지
- 성공/실패 표시

## 🔍 문제 해결

### 연결 실패

```
✗ Connection failed
```

**원인**:
- 잘못된 IP 주소
- 네트워크 연결 문제
- 로봇 꺼짐 또는 대시보드 서버 비활성화

**해결**:
1. 로봇 IP 주소 확인 (`ping 192.168.1.6`)
2. 로봇 전원 확인
3. 방화벽 설정 확인

### Pick 실패

```
✗ Pick failed
```

**원인**:
- 물체 위치 부정확
- 석션 압력 부족
- 경로 충돌

**해결**:
1. 좌표 재확인 및 수동 조정
2. 석션 호스 연결 확인
3. 접근 높이 조정 (`pick_approach_height`)

### 석션 작동 안 함

**확인 사항**:
1. Digital Output 번호 확인 (`do_index`)
2. 석션 펌프 전원 확인
3. 호스 연결 확인
4. 로봇 티치 펜던트에서 수동 DO 테스트

## 📝 독립 실행 테스트

각 모듈을 개별적으로 테스트할 수 있습니다:

### 로봇 컨트롤러 테스트

```bash
python3 src/dobot_e6_controller.py
```

### 석션 그리퍼 테스트

```bash
python3 src/suction_gripper.py
```

### Pick-and-Place 로직 테스트

```bash
python3 src/pick_place_logic.py
```

## 🎯 Pick-and-Place 시퀀스

완전 자동 실행 시:

```
1. Home 위치 이동
2. Pick 위치 상단으로 이동 (접근)
3. 하강하여 물체 위치 도달
4. 석션 ON (물체 그립)
5. 상승 (접근 높이까지)
6. Place 위치 상단으로 이동
7. 하강하여 목적지 도달
8. 석션 OFF (물체 릴리스)
9. 상승
10. Home 복귀
```

## 📚 추가 정보

### Dobot E6 TCP 프로토콜

주요 명령어:
- `MovJ(x,y,z,rx,ry,rz)` - 관절 공간 이동
- `MovL(x,y,z,rx,ry,rz)` - 직선 이동
- `DO(index,value)` - Digital Output 제어
- `GetPose()` - 현재 위치 조회

### 좌표계

- **X, Y, Z**: 밀리미터(mm)
- **RX, RY, RZ**: 도(degrees)
- **원점**: 로봇 베이스 중심

---

**작성일**: 2026-01-29  
**버전**: 1.0  
**로봇**: Dobot E6  
**그리퍼**: 석션(Suction)
