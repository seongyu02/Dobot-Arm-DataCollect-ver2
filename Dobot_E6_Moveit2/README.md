# Dobot E6 MoveIt2 제어 가이드

Dobot E6 로봇팔을 MoveIt2로 제어하기 위한 Python 스크립트 및 사용 가이드입니다.

## 📋 목차

- [필요 사항](#필요-사항)
- [MoveIt2 설정 생성](#moveit2-설정-생성)
- [사용 방법](#사용-방법)
- [스크립트 기능](#스크립트-기능)
- [예제 코드](#예제-코드)
- [문제 해결](#문제-해결)

## 🔧 필요 사항

### 소프트웨어 요구사항

- **ROS 2 Humble** (또는 호환 버전)
- **MoveIt2** 패키지
- **Python 3**

### 설치 확인

```bash
# ROS 2 설치 확인
ros2 --version

# MoveIt2 패키지 확인
ros2 pkg list | grep moveit

# 필요시 MoveIt2 설치
sudo apt install ros-humble-moveit
```

## ⚙️ MoveIt2 설정 생성

현재 `Dobot_E6_Moveit2` 디렉토리에는 URDF 파일만 있습니다. 
제어 스크립트를 실행하기 전에 **MoveIt Setup Assistant**로 설정을 먼저 생성해야 합니다.

### 1. Setup Assistant 실행

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

### 2. URDF 파일 로드

- **"Create New MoveIt Configuration Package"** 선택
- **"Browse"** 클릭 후 URDF 파일 선택:
  ```
  /home/sunbi/Dobot_E6_Moveit2/urdf/me6_robot.urdf
  ```
- **"Load Files"** 클릭

> ⚠️ **주의**: URDF 파일 내의 mesh 경로가 다른 사용자 경로로 되어 있을 수 있습니다.
> 필요시 URDF 파일을 편집하여 mesh 파일 경로를 수정하세요.

### 3. Self-Collisions 설정

- **"Self-Collisions"** 탭 선택
- **"Generate Collision Matrix"** 클릭
- Sampling Density는 기본값 사용

### 4. Planning Groups 설정

가장 중요한 단계입니다!

- **"Planning Groups"** 탭 선택
- **"Add Group"** 클릭
- Group Name: `arm` (또는 `manipulator`)
- Kinematic Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
- **"Add Joints"** 클릭
- 다음 관절들을 선택하여 추가:
  - `joint1`
  - `joint2`
  - `joint3`
  - `joint4`
  - `joint5`
  - `joint6`
- **"Save"** 클릭

### 5. Robot Poses 설정 (선택사항)

- **"Robot Poses"** 탭 선택
- **"Add Pose"** 클릭
- Pose Name: `home`
- 모든 관절을 `0.0`으로 설정
- **"Save"** 클릭

### 6. End Effectors 설정 (선택사항)

- **"End Effectors"** 탭 선택
- **"Add End Effector"** 클릭
- End Effector Name: `gripper` 또는 `tool`
- Parent Link: `Link6`
- **"Save"** 클릭

### 7. ROS 2 Controllers 설정

- **"ROS 2 Controllers"** 탭 선택
- **"Auto Add FollowJointsTrajectory Controllers"** 클릭

### 8. 패키지 생성

- **"Configuration Files"** 탭 선택
- 저장 경로 선택:
  ```
  /home/sunbi/Dobot_E6_Moveit2/me6_moveit_config
  ```
- **"Generate Package"** 클릭

## 🚀 사용 방법

### 1. MoveIt2 데모 실행

먼저 MoveIt2가 제대로 설정되었는지 확인합니다:

```bash
cd /home/sunbi/Dobot_E6_Moveit2
ros2 launch me6_moveit_config demo.launch.py
```

RViz가 실행되고 로봇 모델이 보이면 성공입니다!

### 2. 제어 스크립트 실행

**새 터미널**을 열어서 제어 스크립트를 실행합니다:

```bash
cd /home/sunbi/Dobot_E6_Moveit2
chmod +x me6_control.py
python3 me6_control.py
```

### 3. 대화형 제어

스크립트 실행 후 나타나는 메뉴에서 명령을 선택합니다:

```
1 - Get current joint values        # 현재 관절 각도 조회
2 - Get current pose                # 현재 엔드 이펙터 위치 조회
3 - Move to home position           # 홈 위치로 이동
4 - Move to custom joint values     # 관절 각도로 이동
5 - Move to custom pose             # 좌표로 이동
6 - Run demonstration sequence      # 데모 시퀀스 실행
q - Quit                            # 종료
```

## 📖 스크립트 기능

### 주요 클래스: `DobotE6Controller`

#### 메서드

##### `get_current_joint_values()`
현재 6개 관절의 각도를 조회합니다 (라디안).

##### `get_current_pose()`
현재 엔드 이펙터의 위치와 방향을 조회합니다.

##### `move_to_joint_values(joint_values, execute=True)`
지정된 관절 각도로 로봇을 이동합니다.

**파라미터:**
- `joint_values`: 6개 관절 각도 리스트 (라디안)
- `execute`: True면 실행, False면 계획만 수립

**예제:**
```python
import math
controller.move_to_joint_values([0, math.pi/4, -math.pi/3, 0, math.pi/6, 0])
```

##### `move_to_pose(position, orientation=None, execute=True)`
지정된 위치로 엔드 이펙터를 이동합니다.

**파라미터:**
- `position`: [x, y, z] 좌표 (미터)
- `orientation`: [x, y, z, w] 쿼터니언 (None이면 현재 방향 유지)
- `execute`: True면 실행, False면 계획만 수립

**예제:**
```python
controller.move_to_pose([0.3, 0.2, 0.4])
```

##### `move_home()`
모든 관절을 0으로 설정하여 홈 위치로 이동합니다.

##### `demo_sequence()`
미리 정의된 동작 시퀀스를 실행합니다.

## 💻 예제 코드

### 예제 1: 기본 사용

```python
#!/usr/bin/env python3
import rclpy
from me6_control import DobotE6Controller

rclpy.init()
controller = DobotE6Controller()

# 현재 상태 조회
controller.get_current_joint_values()

# 홈 위치로 이동
controller.move_home()

# 종료
controller.destroy_node()
rclpy.shutdown()
```

### 예제 2: 관절 제어

```python
import math

# 특정 관절 각도로 이동
joint_angles = [
    math.pi / 4,    # joint1: 45도
    -math.pi / 6,   # joint2: -30도
    math.pi / 3,    # joint3: 60도
    0.0,            # joint4: 0도
    math.pi / 4,    # joint5: 45도
    0.0             # joint6: 0도
]

controller.move_to_joint_values(joint_angles)
```

### 예제 3: 포즈 제어

```python
# 특정 좌표로 이동 (미터 단위)
target_position = [0.4, 0.0, 0.5]  # x, y, z
controller.move_to_pose(target_position)

# 방향까지 지정하여 이동
target_position = [0.3, 0.2, 0.4]
target_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w (쿼터니언)
controller.move_to_pose(target_position, target_orientation)
```

### 예제 4: 경로 계획만 (실행 안 함)

```python
# 계획만 수립하고 실행하지 않음
joint_angles = [0, 0, 0, 0, 0, 0]
controller.move_to_joint_values(joint_angles, execute=False)
```

## 🔍 문제 해결

### 문제: "Failed to initialize MoveIt2"

**원인**: MoveIt2 설정이 없거나 잘못되었습니다.

**해결방법**:
1. MoveIt Setup Assistant로 설정을 먼저 생성했는지 확인
2. 생성한 패키지를 빌드했는지 확인:
   ```bash
   cd /home/sunbi/Dobot_E6_Moveit2
   colcon build
   source install/setup.bash
   ```

### 문제: "Planning failed!"

**원인**: 
- 목표 위치가 작업 공간을 벗어남
- 자기 충돌이 발생함
- IK 솔루션이 없음

**해결방법**:
1. 목표 관절 각도가 관절 제한 내에 있는지 확인
   - joint1: -6.27 ~ 6.27 rad
   - joint2: -2.356 ~ 2.356 rad
   - joint3: -2.6878 ~ 2.6878 rad
   - joint4: -2.7925 ~ 2.7925 rad
   - joint5: -3.0194 ~ 3.0194 rad
   - joint6: -6.27 ~ 6.27 rad
2. RViz에서 목표 위치가 도달 가능한지 시각적으로 확인

### 문제: Mesh 파일을 찾을 수 없음

**원인**: URDF 파일의 mesh 경로가 잘못되었습니다.

**해결방법**:
1. `urdf/me6_robot.urdf` 파일 편집
2. 모든 mesh 경로를 실제 경로로 수정:
   ```xml
   <!-- 변경 전 -->
   <mesh filename="file:///home/billy/26kp/ydg/isaac-sim/Mesh/me6/base_link.STL" />
   
   <!-- 변경 후 (실제 경로로) -->
   <mesh filename="file:///home/sunbi/Dobot_E6_Moveit2/urdf/me6_robot/base_link.STL" />
   ```

### 문제: Planning group "arm" not found

**원인**: 스크립트의 planning group 이름이 MoveIt 설정과 다릅니다.

**해결방법**:
`me6_control.py` 파일에서 planning group 이름을 수정:
```python
# 19번째 줄 근처
self.planning_group = "arm"  # MoveIt 설정에서 사용한 이름으로 변경
```

## 📝 참고사항

- **단위**: 모든 거리는 **미터(m)**, 각도는 **라디안(rad)**
- **좌표계**: base_link 기준
- **Planning Time**: 기본 5초 (필요시 수정 가능)
- **속도**: URDF에 정의된 velocity limit 적용 (10.0 rad/s)

## 📚 추가 자료

- [MoveIt2 공식 문서](https://moveit.picknik.ai/humble/index.html)
- [ROS 2 Humble 문서](https://docs.ros.org/en/humble/)
- [MoveIt Python API](https://github.com/moveit/moveit2/tree/humble/moveit_py)

---

**작성일**: 2026-01-29  
**로봇 모델**: Dobot E6 (ME6)  
**ROS 버전**: ROS 2 Humble
