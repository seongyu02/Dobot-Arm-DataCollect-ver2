# 학습 데이터 수집 자동화 가이드

Dobot E6 로봇으로 비전 학습과 강화학습용 데이터를 자동으로 수집합니다.

## 📦 생성된 스크립트

### 1. Vision Learning Data Collector

**파일**: `src/vision_data_collector.py`

카메라 이미지와 라벨을 자동으로 수집합니다.

#### 기능
- 다각도 이미지 캡처 (물체 주변 5방향)
- 자동 라벨링 (물체 위치, 클래스, 로봇 포즈)
- JSON 메타데이터 저장
- 데이터셋 구조화

#### 수집 데이터
```
datasets/vision_data/
├── images/
│   ├── 20260129_120000_0001.jpg
│   ├── 20260129_120001_0002.jpg
│   └── ...
├── labels/
│   ├── 20260129_120000_0001.json
│   ├── 20260129_120001_0002.json
│   └── ...
└── dataset_metadata.json
```

#### 라벨 형식
```json
{
  "sample_id": "20260129_120000_0001",
  "object": {
    "name": "object_1",
    "class": "object",
    "position": {"x": 300, "y": 200, "z": 50, ...}
  },
  "robot": {
    "pose": {"x": 350, "y": 250, "z": 200, ...}
  },
  "image_file": "20260129_120000_0001.jpg"
}
```

#### 실행 방법
```bash
# 기본 실행 (10 samples/object)
python3 src/vision_data_collector.py

# 스크립트 내에서 설정 변경:
# - robot_ip: 로봇 IP 주소
# - camera_id: 카메라 장치 번호 (0=기본)
# - output_dir: 저장 경로
# - samples_per_object: 물체당 샘플 수
```

---

### 2. Reinforcement Learning Data Collector

**파일**: `src/rl_data_collector.py`

강화학습용 state-action-reward transition을 자동으로 수집합니다.

#### 기능
- Gym-like 환경 인터페이스
- State: 로봇 포즈(6) + 물체 위치(3) + 목표 위치(3) + 그리퍼 상태(1) = 13차원
- Action: 상대 이동(6) + 그리퍼 동작(1) = 7차원
- Reward: 거리 기반 + 성공 보너스
- 에피소드별 저장

#### 수집 데이터
```
datasets/rl_data/
├── rl_dataset.pkl          # Pickle (numpy arrays)
├── rl_dataset.json         # JSON (사람이 읽기 쉬움)
└── metadata.json           # 데이터셋 정보
```

#### Episode 구조
```json
{
  "episode_id": 0,
  "timestamp": "2026-01-29T16:00:00",
  "transitions": [
    {
      "step": 0,
      "state": [x, y, z, rx, ry, rz, obj_x, obj_y, obj_z, goal_x, goal_y, goal_z, has_obj],
      "action": [dx, dy, dz, drx, dry, drz, gripper],
      "reward": -0.5,
      "next_state": [...],
      "done": false,
      "info": {}
    },
    ...
  ],
  "total_reward": 8.5,
  "success": true,
  "steps": 25
}
```

#### 실행 방법
```bash
# 기본 실행 (100 episodes)
python3 src/rl_data_collector.py

# 스크립트 내에서 설정 변경:
# - robot_ip: 로봇 IP 주소
# - output_dir: 저장 경로
# - num_episodes: 에피소드 수
```

---

## 🚀 사용 예시

### Vision Data Collection

```python
# 수동으로 사용하기
from vision_data_collector import VisionDataCollector, CameraInterface
from dobot_e6_controller import DobotE6Controller
from suction_gripper import SuctionGripper

robot = DobotE6Controller(ip="192.168.1.6")
robot.connect()

gripper = SuctionGripper(robot, do_index=1)
camera = CameraInterface(camera_id=0)
camera.open()

collector = VisionDataCollector(robot, gripper, camera, "my_dataset")

# 특정 물체에 대해 수집
collector.collect_pick_sequence(
    object_name="object_1",
    object_class="cube",
    object_pos={'x': 300, 'y': 200, 'z': 50, 'rx': 180, 'ry': 0, 'rz': 0},
    num_views=8  # 8방향에서 촬영
)

collector.save_metadata()
```

### RL Data Collection

```python
# 수동으로 사용하기
from rl_data_collector import RLDataCollector, RobotEnvironment
from dobot_e6_controller import DobotE6Controller
from suction_gripper import SuctionGripper
import yaml

robot = DobotE6Controller(ip="192.168.1.6")
robot.connect()

gripper = SuctionGripper(robot, do_index=1)

with open("config/robot_config.yaml") as f:
    config = yaml.safe_load(f)

env = RobotEnvironment(robot, gripper, config)
collector = RLDataCollector(env, "my_rl_dataset")

# 에피소드 수집
for i in range(50):
    collector.collect_random_episode(max_steps=30)

collector.save_dataset()
```

---

## ⚙️ 커스터마이징

### Vision Collector 수정

**이미지 해상도 변경**:
```python
camera = CameraInterface(camera_id=0, width=1280, height=720)
```

**다른 각도에서 촬영**:
```python
collector.collect_pick_sequence(object_name, object_class, object_pos, num_views=10)
```

**라벨에 추가 정보 포함**:
`vision_data_collector.py`의 `capture_sample()` 메서드에서 `label` 딕셔너리 수정

### RL Collector 수정

**보상 함수 변경**:
`rl_data_collector.py`의 `RobotEnvironment.calculate_reward()` 수정

**State 차원 변경**:
`RobotEnvironment.get_state()`에서 더 많은 정보 추가

**Action space 변경**:
`RobotEnvironment.step()`에서 action 해석 방식 수정

---

## 📊 데이터 분석

### Vision Dataset

```python
import json
import cv2

# 메타데이터 로드
with open('datasets/vision_data/dataset_metadata.json') as f:
    metadata = json.load(f)

print(f"Total samples: {metadata['total_samples']}")
print(f"Classes: {metadata['classes']}")

# 샘플 확인
for sample in metadata['samples'][:5]:
    img_path = f"datasets/vision_data/images/{sample['image_file']}"
    img = cv2.imread(img_path)
    cv2.imshow("Sample", img)
    cv2.waitKey(500)
```

### RL Dataset

```python
import pickle
import numpy as np

# 데이터 로드
with open('datasets/rl_data/rl_dataset.pkl', 'rb') as f:
    episodes = pickle.load(f)

# 통계
total_transitions = sum(len(ep['transitions']) for ep in episodes)
success_rate = sum(ep['success'] for ep in episodes) / len(episodes)
avg_reward = np.mean([ep['total_reward'] for ep in episodes])

print(f"Episodes: {len(episodes)}")
print(f"Transitions: {total_transitions}")
print(f"Success rate: {success_rate:.2%}")
print(f"Average reward: {avg_reward:.2f}")

# 최고 에피소드 찾기
best_ep = max(episodes, key=lambda x: x['total_reward'])
print(f"\nBest episode: {best_ep['episode_id']}")
print(f"Reward: {best_ep['total_reward']:.2f}")
print(f"Steps: {best_ep['steps']}")
```

---

## ⚠️ 주의사항

> [!CAUTION]
> **데이터 수집 전 확인사항**

1. **작업 공간 안전**: 로봇 동작 범위 내 장애물 제거
2. **카메라 설정**: 카메라가 작업 공간을 잘 볼 수 있는 위치에 고정
3. **충분한 저장 공간**: 이미지는 용량이 큽니다 (100 samples ≈ 50MB)
4. **로봇 속도 조정**: 데이터 품질을 위해 느린 속도 권장

> [!WARNING]
> **실행 중 주의**
> - 자동 수집 중 로봇 근처에 가지 마세요
> - 이상 동작 시 즉시 비상 정지
> - 정기적으로 데이터 백업

---

## 🎯 활용 예시

### Vision Learning

수집된 데이터로:
- Object detection 모델 학습
- Pose estimation
- Grasp point prediction
- Semantic segmentation

### Reinforcement Learning

수집된 데이터로:
- Offline RL (Behavior cloning)
- Imitation learning
- Reward modeling
- State representation learning

---

**작성일**: 2026-01-29  
**버전**: 1.0  
**로봇**: Dobot E6
