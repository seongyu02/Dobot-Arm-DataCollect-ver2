# Setup Assistant 멈춤 문제 해결 방법

Setup Assistant가 큰 STL mesh 파일을 로드할 때 멈추는 문제가 발생하고 있습니다. 두 가지 해결 방법이 있습니다.

## 해결 방법

### 옵션 1: 간단한 URDF 사용 (권장) ⭐

**파일**: `me6_robot_simple.urdf`

STL mesh 대신 **기본 도형(실린더)**을 사용하는 간단한 URDF입니다.

**장점**:
- ✅ Setup Assistant가 즉시 로드됨
- ✅ MoveIt2 계획에 충분한 정확도
- ✅ 빠른 실행 속도
- ✅ 메모리 사용량 적음

**사용 방법**:
```bash
cd /home/sunbi/Dobot_E6_Moveit2
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
URDF 로드 시: `/home/sunbi/Dobot_E6_Moveit2/urdf/me6_robot_simple.urdf` 선택

### 옵션 2: 원본 URDF + 명령줄 도구 사용

큰 mesh 파일이 있는 원본 URDF를 사용하되, Setup Assistant 대신 **명령줄로 직접 설정**:

```bash
# MoveIt 템플릿에서 시작
ros2 pkg create me6_moveit_config --build-type ament_cmake

# 수동으로 설정 파일 복사 및 수정
# (더 복잡하고 시간 소요)
```

## 권장 사항

**간단한 URDF (옵션 1)을 사용**하는 것을 강력히 권장합니다:

1. MoveIt 설정 생성 완료
2. 제어 스크립트 테스트
3. 필요시 나중에 상세한 mesh로 업그레이드

## 다음 단계

1. Setup Assistant 실행:
   ```bash
   ros2 launch moveit_setup_assistant setup_assistant.launch.py
   ```

2. `me6_robot_simple.urdf` 로드

3. Planning Group 설정:
   - Group name: `arm`
   - Joints: joint1 ~ joint6

4. 패키지 생성 완료

---

**참고**: 간단한 URDF는 **경로 계획**에만 사용됩니다. 실제 로봇 제어 시 정확한 기구학 파라미터는 그대로 유지됩니다.
