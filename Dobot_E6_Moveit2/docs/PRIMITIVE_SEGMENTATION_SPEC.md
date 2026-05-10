# Primitive 구간 분할 정의서 (다른 서버 이관용)

에피소드별로 **approach → pick → move → place → return** (및 선택적 **init_hold**) 구간을 나누어 라벨링·학습·분석에 쓰기 위한 규칙이다.  
이 문서만으로 동일 로직을 다른 서버에서 재현할 수 있어야 한다.

---

## 1. 입력 데이터

- **필수:** 각 에피소드 폴더의 `robot_data.csv`
  - 컬럼 예: `frame_id`, `timestamp`, `image_path`, `x`, `y`, `z`, …, `gripper_tooldo1`, `gripper_tooldo2`
  - 단위: `x,y,z`는 로봇 베이스 기준 **mm** (기존 수집 파이프라인과 동일)
- **선택:** 에피소드 **의미 방향** 메타
  - 파일 예: `_episode_direction_classification.csv` (에피소드별 `class` 컬럼)

---

## 2. 저점(집기/놓기) 검출

- **저점 Z 임계값:** `z <= 112` (mm)
- 연속된 프레임 구간 중 `z <= 112`가 **2프레임 이상** 유지되는 것을 **저점 구간**으로 본다.
- 일반적인 성공 에피소드에서는 **첫 번째 저점 구간 ≈ pick**, **두 번째 저점 구간 ≈ place**로 본다.

저점 구간이 2개 미만이면 자동 분할이 불완전할 수 있으므로, (선택) 그리퍼 디지털 출력 변화나 수동 보정 규칙을 둔다.

---

## 3. 에피소드 의미 방향 (좌/중/우, L→R 등)

다음은 **에피소드 단위 라벨**을 만들 때 쓰는 기하 정의다.

### 3.1 중앙 마름모 (평면 XY)

꼭짓점(순서: 닫힌 다각형):

| 꼭짓점 | (x mm, y mm) |
|--------|----------------|
| top | (83.41, -339.22) |
| right | (112.31, -378.54) |
| bottom | (83.41, -417.86) |
| left | (54.51, -378.54) |

**점 (x,y)가 위 다각형 내부이면 구역 `M` (middle).**

### 3.2 좌/우 반평면 (마름모 밖일 때)

- **L (왼쪽 섹션):** `x >= 85` (mm)
- **R (오른쪽 섹션):** `x < 85` (mm)

### 3.3 pick / place 존에서의 구역

- 저점 구간마다 **median(x), median(y)** 로 구역을 부여한 뒤, pick·place 조합으로 `class`를 정한다 (기존 분류 스크립트와 동일).

---

## 4. 6개 시간 구간 (primitive) 정의

| segment | 설명 |
|---------|------|
| `init_hold` | 녹화 직후 INIT 근처. 프레임 **0**부터 (a) 최대 **25프레임**(20Hz) 또는 (b) **첫 유의미 움직임** 직전까지. 유의미 움직임: 시작 대비 sqrt(dx^2+dy^2) > 3 mm 또는 |dz| > 3 mm |
| `approach` | init_hold 직후 ~ **첫 저점 구간 시작 직전** |
| `pick` | **첫 번째** 저점 구간 (z<=112, 길이>=2) |
| `move` | 첫 저점 종료 ~ **둘째 저점 시작 직전** |
| `place` | **두 번째** 저점 구간 |
| `return` | place 종료 ~ **마지막 프레임** |

인덱스는 **frame_id와 동일한 행 순서(0-based)** 기준. 각 구간에 `image_path` 첫/끝 파일명을 기록한다.

---

## 5. move_right / move_left

- 베이스 **X 부호만**으로 좌우를 말하면 섹션 의미와 어긋날 수 있다.
- **권장:** `_episode_direction_classification.csv`의 **`class`** (left_to_right 등)를 **우선**.
- 보조: 첫·둘째 저점 구간의 median(x) 차이 `place_x - pick_x` 기록.

다른 서버에서 `move_right`/`move_left`로 통일할 때는 **팀 매핑 표**를 한 번 정해 `class` → 이름 변환.

---

## 6. 산출물 (권장)

- `episode_primitive_segments.csv` — 구간별 frame 범위·이미지 파일명
- `_episode_direction_classification.csv` — 에피소드 class (move 의미용, 권장)
- 본 정의서

---

## 7. 다른 서버 체크리스트

1. 동일 폴더 구조로 데이터 복사
2. 본 문서의 Z·마름모·x 분기·6구간 규칙을 동일 구현
3. (선택) 분류 CSV 복사 또는 §3으로 재계산
4. primitive CSV 생성
5. 샘플 몇 개만 이미지로 검증 후 Z/구간 파라미터만 필요 시 튜닝

---

## 8. 한계

- Z 노이즈로 저점이 쪼개지면 경계 오차
- 저점 1개만 잡히면 place/return 비어 있을 수 있음 → 그리퍼 보조 또는 수동 수정


---

## 9. 제어용 primitive 이름 (권장, 5단계)

에피소드 **전체**는 `high_level_task = pick_and_place` 로 두고, 프레임 구간은 아래 **재사용 가능한 5 primitive**로 나눈다.  
`move_left` / `move_right` 는 **별 태스크명이 아니라** `transport_to_target` 의 **파라미터**(`target_zone`, 또는 `episode_class`)로 둔다.

| primitive | 의미 |
|-----------|------|
| `reach_to_object` | INIT·대기 포함, 물체 쪽으로 이동·정렬 (구 `approach` 포함) |
| `grasp_object` | 첫 저점 구간: 하강·접촉·집기 |
| `transport_to_target` | 집은 뒤 목표까지 이동 |
| `release_object` | 둘째 저점 구간: 놓기 |
| `return_to_init` | 복귀 |

이전 문서의 `init_hold`+`approach` 는 여기서 **`reach_to_object` 하나**로 합친 것으로 보면 된다 (경계는 동일 Z 규칙으로 자동 분할).

**예시 5개 에피소드** (실제 `robot_data.csv` 로 프레임·이미지 파일명까지 표):  
`docs/PRIMITIVE_SEGMENTATION_EXAMPLES.md`

**다른 시스템 프롬프트·이관용 확정 요약 (한 파일):**  
`docs/PRIMITIVE_PROMPT_HANDOFF.md`