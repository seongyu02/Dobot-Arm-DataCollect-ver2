# Primitive 라벨 · 프롬프트 이관 (확정 요약)

이 문서는 **다른 서버/모델 프롬프트에 넣을 때** 기준으로 삼는 **단일 요약**이다.  
세부 수치·알고리즘은 `PRIMITIVE_SEGMENTATION_SPEC.md` 를 따른다.

---

## 1. 확정한 것 (한 줄)

- **에피소드 단위 목적:** `high_level_task = pick_and_place`
- **프레임 구간 primitive (5개, 이름 고정):**  
  `reach_to_object` → `grasp_object` → `transport_to_target` → `release_object` → `return_to_init`
- **좌/우/가운데 같은 방향은 별 primitive가 아님.**  
  → `transport_to_target` 에 붙이는 **파라미터**로 둔다:  
  `target_zone` ∈ {`left`,`right`,`middle`} 및/또는 `episode_class` (예: `left_to_right`, `right_to_left`, `to_middle`, `M_to_L`, `M_to_R`)

---

## 1.1 기준점 정의 (에피소드 전체 vs 구간별 프레임) — CSV로 이렇게 읽는다

**한 층:** 에피소드 **전체**는 항상 `high_level_task = pick_and_place` 하나로 고정한다.

**그 아래 층:** 같은 에피소드 안 시간축을 **5개 구간**으로만 나눈다. 각 구간 이름이 곧 **구간 태스크**이고, **어느 `frame_id`부터 어느 `frame_id`까지**인지는 `episode_primitive_segments_v2_pickplace_names.csv`에서 `episode` 열로 골라 읽는다.  
(`frame_id` = 해당 번호 폴더 `N/robot_data.csv`의 `frame_id` 열과 동일. 구간은 **양끝 포함**.)

### 예시: `episode == 2` (v2 CSV 5행과 1:1 대응)

| 순서 | 구간 태스크 (primitive) | `frame_id` 범위 (끝 포함) | `n_frames` | 이미지 범위 (`2/images/`) |
|:--:|-------------------------|---------------------------|:------------:|---------------------------|
| 1 | `reach_to_object` | **0 ~ 71** | 72 | `frame_000000.jpg` ~ `frame_000071.jpg` |
| 2 | `grasp_object` | **72 ~ 77** | 6 | `frame_000072.jpg` ~ `frame_000077.jpg` |
| 3 | `transport_to_target` | **78 ~ 109** | 32 | `frame_000078.jpg` ~ `frame_000109.jpg` |
| 4 | `release_object` | **110 ~ 132** | 23 | `frame_000110.jpg` ~ `frame_000132.jpg` |
| 5 | `return_to_init` | **133 ~ 184** | 52 | `frame_000133.jpg` ~ `frame_000184.jpg` |

- **에피소드 전체:** `pick_and_place` (에피소드 2 한 번 녹화 전체).
- **구간별:** 위 표의 1~5행 — “몇 프레임까지가 `reach_to_object`인지”는 **1행의 `start_frame`~`end_frame`** (0~71), “어디서부터 `grasp_object`인지”는 **2행** (72~77) … 처럼 **CSV 한 줄 = 구간 하나**로 고정된다.
- `transport_to_target` 행의 `move_semantic_from_class` (예: `left_to_right`)는 **같은 구간의 이동 의미**만 붙이는 필드이고, 별 primitive 이름이 아니다.

다른 에피소드 번호 `N`도 동일: v2 CSV에서 `episode == N`인 연속 5행이 곧 그 폴더 `N/`의 구간 정의다.

### 1.2 숫자가 정해지는 규칙 (전 에피소드 동일 공식)

표의 `start_frame` / `end_frame`은 **임의 라벨이 아니라** 아래를 **모든 에피소드에 동일하게** 적용한 결과다.

1. `N/robot_data.csv`에서 각 행의 `z`(mm)를 읽는다.
2. `z <= 112`가 **연속 2프레임 이상** 유지되는 구간을 **저점 구간**으로 모은다 (최대 연속 구간).
3. **첫 번째** 저점 구간 → `grasp_object`, **두 번째** → `release_object`.
4. 첫 저점 **이전** 전체 → `reach_to_object`; 두 저점 **사이** → `transport_to_target`; 둘째 저점 **이후** → `return_to_init`.
5. `transport_to_target`의 `move_semantic_from_class`는 `_episode_direction_classification.csv`의 해당 에피소드 `class`.

**전 에피소드 CSV를 같은 규칙으로 재생성하는 코드:** `vla_dataset_random_pose/generate_primitive_segments_v2.py`  
**수식·엣지 케이스 설명:** `vla_dataset_random_pose/PRIMITIVE_SEGMENTATION_README.md` (맨 위 「5 segments (v2)」)

---

## 2. 데이터·파일 (무엇을 복사하면 되나)

| 역할 | 경로 (프로젝트 기준) |
|------|----------------------|
| 구간 라벨 **v2 (프롬프트용 이름)** | `vla_dataset_random_pose/episode_primitive_segments_v2_pickplace_names.csv` |
| 구간 라벨 v1 (6행: init_hold, approach, …) | `vla_dataset_random_pose/episode_primitive_segments.csv` |
| 에피소드 **의미 방향** | `vla_dataset_random_pose/_episode_direction_classification.csv` |
| 규칙 전체 (Z, 마름모, L/R) | `Dobot_E6_Moveit2/docs/PRIMITIVE_SEGMENTATION_SPEC.md` |
| **예시 5 에피소드** (표+JSON) | `Dobot_E6_Moveit2/docs/PRIMITIVE_SEGMENTATION_EXAMPLES.md` |

**권장:** 프롬프트/학습 파이프라인에는 **v2 CSV** + **`_episode_direction_classification.csv`** 를 같이 쓴다.

### v2 CSV 컬럼

- `episode`, `primitive`, `start_frame`, `end_frame`, `n_frames`, `image_start`, `image_end`, `move_semantic_from_class` (transport 행에만 의미 방향 문자열)

`image_*` 는 해당 에피소드 폴더 `images/` 아래 파일명과 동일하다.

### 프레임 번호·구간 끊기 (다른 시스템에 넘길 때 필수)

1. **기준 인덱스는 `robot_data.csv` 의 `frame_id` 컬럼뿐이다.**  
   v2 CSV의 `start_frame` / `end_frame`은 **해당 에피소드 폴더 안 `robot_data.csv`의 `frame_id`와 같은 정수**다 (0부터 마지막까지 연속).

2. **구간은 양끝 포함(inclusive).**  
   한 primitive에 속하는 프레임은 **[start_frame, end_frame]** 전부다.  
   다음 primitive는 **반드시 `end_frame + 1`**부터 시작한다 (프레임 겹침 없음).

3. **개수:** `n_frames` = `end_frame - start_frame + 1` (CSV `n_frames`와 일치).

4. **이미지:** `robot_data.csv` 한 행의 `image_path`(예: `frame_000071.jpg`)는 **`frame_id == 71`인 행**과 대응한다.  
   v2의 `image_start` / `image_end`는 그 구간의 첫·마지막 파일명이다.

5. **배열/클립으로 자를 때** (`frame_id` 순으로 정렬된 텐서·리스트라고 가정):  
   해당 primitive는 인덱스 **`start_frame`부터 `end_frame`까지 포함**  
   (Python: `arr[start_frame : end_frame + 1]`).

6. **JSON·프롬프트에 넣는 숫자:** v2 CSV의 `start_frame`·`end_frame`을 **그대로** 쓴다. 임의의 +1/-1 보정을 하지 않는다.

**영문 한 줄 (프롬프트에 붙여도 됨):**  
`start_frame and end_frame are inclusive indices equal to robot_data.csv column frame_id; the next segment starts at end_frame+1.`

---

## 3. 자동 분할 규칙 (다른 곳에서 재현할 때)

1. `robot_data.csv` 의 각 행이 한 프레임; `image_path` 와 1:1.
2. **저점:** `z <= 112` (mm) 가 **연속 2프레임 이상** → 저점 구간.
3. **첫 저점 구간** ≈ 집기 → `grasp_object`  
   **둘째 저점 구간** ≈ 놓기 → `release_object`
4. **첫 저점 전** 전체(구 init_hold + approach 합침) → `reach_to_object`
5. **두 저점 사이** → `transport_to_target`
6. **둘째 저점 후** ~ 끝 → `return_to_init`
7. `target_zone`: `_episode_direction_classification.csv` 의 `place_zone` (L/R/M) → `left` / `right` / `middle` 로 매핑.

---

## 4. 프롬프트에 넣을 JSON 예시 (에피소드 단위)

아래를 **한 에피소드당** 메타로 쓰면 된다.

```json
{
  "high_level_task": "pick_and_place",
  "object": "red_block",
  "episode": 2,
  "episode_class": "left_to_right",
  "target_zone": "right",
  "segments": [
    {"primitive": "reach_to_object", "start_frame": 0, "end_frame": 71},
    {"primitive": "grasp_object", "start_frame": 72, "end_frame": 77},
    {"primitive": "transport_to_target", "start_frame": 78, "end_frame": 109},
    {"primitive": "release_object", "start_frame": 110, "end_frame": 132},
    {"primitive": "return_to_init", "start_frame": 133, "end_frame": 184}
  ]
}
```

**프레임별 이미지:** `episode` 폴더의 `images/` + 해당 행 `image_path`.

---

## 5. 짧은 시스템 프롬프트 문단 (복사용)

```
You are labeling a single-arm pick-and-place episode. The task is always high_level_task=pick_and_place.
Split time into exactly five sequential primitives (same names for every episode):
reach_to_object, grasp_object, transport_to_target, release_object, return_to_init.
Do NOT use move_left/move_right as separate task names; encode direction as parameters:
target_zone in {left, right, middle} and/or episode_class from metadata.
start_frame and end_frame are inclusive indices equal to robot_data.csv column frame_id; the next segment starts at end_frame+1.
```

---

## 6. v1 vs v2

- **v1** (`episode_primitive_segments.csv`): 내부용 6단계 (`init_hold`, `approach`, …).
- **v2** (`episode_primitive_segments_v2_pickplace_names.csv`): **프롬프트·대외용 5단계** (`init_hold`+`approach` → `reach_to_object` 로 병합).

경계 프레임은 v1에서 파생했으므로 **숫자는 일치**한다.

---

*이 문서와 SPEC/EXAMPLES 가 서로 모순되면 **SPEC** 을 우선한다.*
