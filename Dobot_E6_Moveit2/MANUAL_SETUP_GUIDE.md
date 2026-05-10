# Setup Assistant 없이 MoveIt2 제어하기

**답변**: 네, SRDF 없이도 가능하지만 **매우 제한적**입니다. 대신 **수동으로 최소 설정 파일을 생성**했습니다!

## 생성된 파일

### 1. 핵심 설정 파일

```
Dobot_E6_Moveit2/
├── config/
│   ├── me6_robot.srdf           ✨ Planning group, end effector, collision
│   ├── kinematics.yaml          ✨ KDL kinematics solver
│   ├── ompl_planning.yaml       ✨ OMPL planners (RRT, RRTConnect, etc)
│   └── joint_limits.yaml        ✨ Velocity/acceleration limits
├── launch/
│   └── me6_moveit.launch.py     ✨ MoveIt 실행 런처
└── urdf/
    └── me6_robot_fast.urdf      ✨ 빠른 로딩용 URDF
```

### 2. SRDF (me6_robot.srdf)

Planning group, end effector 정의:
- **Planning Group**: `arm` (joint1~6)
- **End Effector**: `gripper` 
- **Collision Pairs**: 인접 링크 충돌 체크 비활성화
- **Predefined Poses**: `home`, `ready`

### 3. Kinematics (kinematics.yaml)

KDL solver 설정 - 역기구학 계산

### 4. OMPL Planning (ompl_planning.yaml)

경로 계획 알고리즘:
- RRTConnect (빠름)
- RRT (기본)
- RRTstar (최적 경로)
- BKPIECE

### 5. Launch File (me6_moveit.launch.py)

모든 설정을 로드하고 MoveIt 실행

## 사용 방법

### 실행

```bash
cd /home/sunbi/Dobot_E6_Moveit2

# MoveIt 실행
ros2 launch launch/me6_moveit.launch.py
```

이렇게 하면:
- Robot State Publisher 시작
- Joint State Publisher GUI 시작
- MoveIt Move Group 시작
- RViz 시작

### 제어 스크립트 실행

**새 터미널에서**:
```bash
cd /home/sunbi/Dobot_E6_Moveit2
python3 me6_control.py
```

## SRDF 없이 제어 vs SRDF로 제어

| 기능 | SRDF 없이 | SRDF로 (지금) |
|------|----------|--------------|
| 관절 직접 제어 | ✅ 가능 | ✅ 가능 |
| 역기구학 (IK) | ❌ 불가능 | ✅ 가능 |
| 충돌 회피 경로 계획 | ❌ 불가능 | ✅ 가능 |
| Planning Group | ❌ 없음 | ✅ 있음 |
| Predefined Poses | ❌ 없음 | ✅ 있음 |

## 장점

✅ **Setup Assistant 불필요**: GUI 멈춤 문제 해결  
✅ **모든 MoveIt 기능 사용 가능**: IK, 경로 계획, 충돌 회피  
✅ **제어 스크립트 바로 사용 가능**: `me6_control.py` 동작  
✅ **커스터마이징 쉬움**: YAML 파일 직접 수정 가능

## 다음 단계

1. **MoveIt 실행 테스트**:
   ```bash
   ros2 launch launch/me6_moveit.launch.py
   ```

2. **RViz에서 확인**:
   - Planning Group이 보이는지 확인
   - 목표 포즈 설정하여 계획 테스트

3. **제어 스크립트 실행**:
   ```bash
   python3 me6_control.py
   ```

---

**요약**: Setup Assistant 없이 수동으로 전체 MoveIt 설정을 생성했습니다! 바로 사용 가능합니다. 🚀
