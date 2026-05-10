# Phase 1 수집 규격 및 누적 합의 (최종 정리본)

본 문서는 대화·분석·합의를 바탕으로 한 **운영·구현 기준**이다. Gate 1 품질 산출물 경로는 `reports/gate1_vla_dataset_random_pose/` 를 참고한다.

---

## 1. 데이터셋·코드 맥락

- **수집 GUI**: `pick_place_gui_new.py` 기반; 랜덤 INIT 변형은 `pick_place_gui_random_pose.py` (`vla_dataset_random_pose`).
- **로봇 로그**: `feedBackData()` → `QActual`, `ToolVectorActual`, `RobotMode` → **명령이 아닌 피드백 궤적**.
- **그리퍼**: `gripper_tooldo1` = `is_gripping` 기반 **소프트웨어 플래그** (진공 센서 아님).
- **카메라**: Hik 스트림; row당 JPEG; `frame_XXXXXX.jpg` ↔ CSV `frame_id`.
- **A/B 섹션 (XY 다각형)**  
  - **A**: `A_SECTION_POINTS` = POS_1→…→POS_7 (닫힌 다각형).  
  - **B**: `B_SECTION_POINTS` = POS_6→7→8→9 (사각형).  
  - **B place 가시성**: `B_SECTION_PLACE_VISIBLE_POINTS` 등 (필요 시 추가 제한).

---

## 2. Gate 1 (수집 서버 품질) — 산출물 요약

**경로**: `reports/gate1_vla_dataset_random_pose/`

| 산출물 | 요지 |
|--------|------|
| `dt_report.json` | 연속 `timestamp` Δt 기준 **실측 Hz (~3.7Hz)**; **20Hz 가정 불가** |
| `episode_quality_report.csv` | 에피소드별 dt, 무결성, 첫 row z, phase 비율 등 |
| `warmup_drop_rule.txt` | 첫 프레임 워밍업 제거 규칙 |
| `phase_split_rule.txt` | pre/post 구간 정의·학습 시 subset·태그·마스크 |
| `raw_contract.md` | 피드백 궤적·단일 timestamp·gripper·hidden state 한계 |

**결론**: `dataset_meta.fps=20` 고정값 폐기 → 측정 dt 또는 timestamp 기반 horizon / resample.

---

## 3. Phase 1 수집 서버 시각 분석 (예: 181 에피소드)

**스크립트**: `scripts/phase1_collection_server_check.py`  
**출력**: `reports/phase1_collection_server/`

- 첫 프레임 모자이크, 블록 centroid scatter, TCP 궤적 샘플, `init_pose_variance.json`, `phase1_episode_flags.csv`.
- **RandomPose INIT** 데이터는 첫 row 분산이 크므로, Phase 1 “좁은 일관 init”과는 별도 설계가 필요하다.

---

## 4. 정책·프롬프트·메타

### 4.1 두 층 분리

- **수집 규율 문서** (사람·README·코드): 일관성·금지 사항.
- **에피소드 `prompt`**: 학습용 짧은 문구; **방향별로 분리 가능**.

### 4.2 Phase 1 목표

- **일관성 > 다양성**; 에피소드당 **pick–place 1회**; 실패·우회·재접근·재시도·비정상은 **저장 금지** 또는 학습 제외.

### 4.3 A/B 역할과 방향 (문구 고정)

- **Phase 1 기본 운영 모드:** `A_to_B`.
- **`A_to_B`에서는** A가 pick, B가 place (섹션 역할이 이 모드에서 이렇게 고정된다).
- **`B_to_A`를 도입하면** “전역으로 A는 항상 pick, B는 항상 place”라는 단일 문장으로 고정하지 않고, **`episode_direction`에 따라 source region / target region을 고정**한다.  
  - 예: `A_to_B` → pick 샘플 `A_center_small`, place 샘플 `B_center_small`.  
  - 예: `B_to_A` → pick 샘플 `B_center_small`, place 샘플 `A_center_small`.

### 4.4 혼합 금지 (학습)

- `A_to_B`와 `B_to_A`를 **같은 prompt로 무표시 혼합하지 않는다** (label ambiguity 방지).
- 방향은 **`episode_direction`**, **방향별 `prompt`**, 또는 **별도 데이터셋/run**으로 드러내야 한다.

### 4.5 Prompt 예시

- `A_to_B`: `pick up the red block from section A and place it in section B`
- `B_to_A`: `pick up the red block from section B and place it in section A`

### 4.6 수집 vs 학습

- 수집은 **좌표·region** 기반으로 해도 된다.
- 모델 입력에 목표 좌표를 넣지 않으면, 방향·과제 정보는 **prompt 또는 메타 조건**으로 반드시 구분한다.

### 4.7 메타 필드 (보정)

- `pick_xy`, `place_xy` (또는 동등 필드)는 **현 단계에서는 수집·분석용 메타데이터**로 저장한다.  
- **모델 입력으로 쓸지 여부는 파이프라인에서 별도 결정**한다 (기본은 이미지+state+prompt 위주일 수 있음).

권장 필드 예: `phase`, `episode_direction`, `pick_section`, `place_section`, `object_region`, `target_region`, `prompt`, `init_type`, `init_gate_passed`, `approach_type`, `episode_type`, `success`, `reject_reason`, `quality_flags`, **`sync_policy`**, (선택) `pick_xy`, `place_xy`.

---

## 5. A_center_small / B_center_small

**정의**: 바운딩 박스가 아니라, **원 섹션 다각형의 기하학적 centroid에 대해 α=0.6 선형 축소**한 내부 다각형  
`p' = c + 0.6 × (p − c)`.

**코드 상수명**: `A_CENTER_SMALL_POINTS`, `B_CENTER_SMALL_POINTS` (꼭짓점 순서는 각각 `A_SECTION_POINTS` / `B_SECTION_POINTS` 와 동일).

**샘플링**: `generate_random_point_in_section(A_CENTER_SMALL_POINTS)` / `(...B_CENTER_SMALL_POINTS)` — **다각형 내부 판정**만 유효. 축에 평행한 박스 균일 샘플은 사용하지 않는다.

**참고용 bounding box**는 사람이 범위를 감 잡기 위한 것이며, **정의 자체는 꼭짓점 리스트**이다.

**튜닝**: Phase 1 초기 α=0.6; 필요 시 0.5~0.7로 조정.

---

## 6. Image–State sync 규약 (코드 일치)

**근거**: `PickPlaceGUINew._on_record_tick` — 한 틱 내 순서:

1. `feedBackData()` → joints, TCP, `robot_mode`
2. `gripper.is_gripping`
3. `camera.get_frame()` → JPEG 저장
4. `timestamp = time.time()` (위 처리 **이후**)
5. row append

**`sync_policy` (권장 문자열):**  
`feedback_first_camera_second_row_timestamp_end`

**해석**: 이미지와 로봇 state는 **동일 물리 시각의 하드 동기가 아니다**. 고정된 **순차 규약**으로 묶인 **논리 샘플**이다. 시간 간격은 **`timestamp` 차이**로 두고 **고정 20Hz 가정은 금지**한다.

---

## 7. GUI에서 할 일

- **수집 모드**: `A_to_B` / `B_to_A` 분기 → **pick/place에 쓰는 source·target region만 방향에 맞게 반대**로, 나머지 규율(게이트·패턴·저장 금지)은 동일 계열로 유지.
- **`fixed_init_pose`** + **`visible_init_gate`** 통과 후에만 녹화.
- 기존 **에피소드마다 A↔B 자동 교대** 로직은 Phase 1 GUI에서 **사용하지 않거나 비활성화**.
- **`last_place` 등 이전 place 기억에 의존하는 hidden rule** 은 Phase 1 GUI에서 **비활성화** (imitation에서 관측 불가 상태 의존 제거).
- **랜덤 INIT** (`random_pose`류)는 Phase 1 목적과 충돌 시 **끄거나 별도 모드**로 분리.
- 상수 **`A_CENTER_SMALL_POINTS` / `B_CENTER_SMALL_POINTS`** 를 넣고 위 샘플링 API로만 샘플.

---

## 8. 한 줄 판정

> Gate 1·품질·center_small·sync·GUI 방향은 위와 같이 고정한다.  
> **Phase 1 기본 모드는 `A_to_B`** 이고, **`B_to_A`는 `episode_direction` 기준 source/target region으로만 정의**한다.  
> **`last_place` 류 hidden rule은 Phase 1에서 끈다.**  
> **`pick_xy` / `place_xy`는 메타(수집·분석)용이며 모델 입력 여부는 별도 결정**이다.

---

## 9. 스텝별 확인 (넘어갈 때마다 체크)

다음 순서로 구현·검증하면 문서와 코드가 어긋나지 않기 쉽다.

| Step | 내용 | 확인 |
|------|------|------|
| **S1** | Gate 1 산출물 경로·`dt_report`·`raw_contract` 읽고, **20Hz 가정 없이** 변환/학습 설계 | ☐ |
| **S2** | `A_CENTER_SMALL_POINTS` / `B_CENTER_SMALL_POINTS` 상수를 코드에 넣고, **다각형 내부 샘플만** 사용 | ☐ |
| **S3** | Phase 1 GUI: **`A_to_B` 기본**, (선택) `B_to_A` 모드 — **`episode_direction`·방향별 `prompt`·region** 메타 저장 | ☐ |
| **S4** | **A↔B 자동 교대·`last_place` 의존** 비활성화 | ☐ |
| **S5** | **`fixed_init_pose`** + **`visible_init_gate`** 통과 후에만 녹화 | ☐ |
| **S6** | 에피소드 메타에 **`sync_policy`** (`feedback_first_camera_second_row_timestamp_end`, §6과 동일 문자열) 기록 | ☐ |
| **S7** | `pick_xy` / `place_xy`는 **수집·분석용**으로만 넣을지 확정; **학습 입력 포함은 별도 결정** | ☐ |
| **S8** | 소규모 시험 수집 후 `phase1_collection_server_check`·Gate 스크립트로 **품질 재확인** | ☐ |

**학습 쪽 (수집과 분리):** `A_to_B`만 먼저 학습할지, `B_to_A`를 별도 run으로 둘지 **S3 이전 또는 직후**에 결정.

---

## 관련 경로

| 항목 | 경로 |
|------|------|
| Gate 1 산출물 | `reports/gate1_vla_dataset_random_pose/` |
| Phase 1 시각 분석 | `reports/phase1_collection_server/` |
| 파이프라인 개요 | `docs/DATA_COLLECTION_PIPELINE.md` |
| Phase 1 Define GUI (수집) | `Dobot_E6_Moveit2/src/pick_place_gui_define.py` (실행: `python pick_place_gui_define.py`) |
| Phase 1 Define 데이터 루트 | `TCP-IP-Python-V4/vla_dataset_define/ab/` (A→B), `.../ba/` (B→A) — 방향별 하위 폴더로 분리 (`pick_place_gui_define.py`) |

### `vla_dataset_define` 레이아웃 (에피소드당)

`pick_place_gui_new.PickPlaceGUINew`의 20Hz 저장 로직을 그대로 쓰므로, **에피소드 한 폴더 안의 파일 구조**는 `vla_dataset_random_pose`와 동일하다.

- 상위: `vla_dataset_define/ab/` ← `episode_direction == A_to_B` 일 때 저장
- 상위: `vla_dataset_define/ba/` ← `episode_direction == B_to_A` 일 때 저장
- 에피소드 폴더: 각 `ab` / `ba` 아래에서 독립적으로 `1/`, `2/`, `3/`, … (`_get_next_folder_number`는 해당 하위 루트 기준)
- 각 에피소드 디렉터리 예:
  - `robot_data.csv`
  - `dataset.npy`
  - `metadata.txt`
  - `episode_metadata.json` (Define GUI에서 Phase 1 메타; `dataset_root_relative` 등)
  - `images/frame_000000.jpg`, …

즉 **방향(ab/ba) → 에피소드 번호** 이중 구조이며, `random_pose`와 같이 **숫자 폴더당 한 에피소드** 규칙은 동일하다.
