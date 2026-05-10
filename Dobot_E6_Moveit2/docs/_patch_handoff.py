# one-off patch
from pathlib import Path
p = Path(__file__).with_name("PRIMITIVE_PROMPT_HANDOFF.md")
text = p.read_text(encoding="utf-8")
old = (
    "`image_*` 는 해당 에피소드 폴더 `images/` 아래 파일명과 동일하다.\n\n---\n\n## 3."
)
new = (
    "`image_*` 는 해당 에피소드 폴더 `images/` 아래 파일명과 동일하다.\n\n"
    "### 프레임 번호·구간 끊기 (다른 시스템에 넘길 때 필수)\n\n"
    "1. **기준 인덱스는 `robot_data.csv` 의 `frame_id` 컬럼뿐이다.**  \n"
    "   v2 CSV의 `start_frame` / `end_frame`은 **해당 에피소드 폴더 안 `robot_data.csv`의 `frame_id`와 같은 정수**다 (0부터 마지막까지 연속).\n\n"
    "2. **구간은 양끝 포함(inclusive).**  \n"
    "   한 primitive에 속하는 프레임은 **[start_frame, end_frame]** 전부다.  \n"
    "   다음 primitive는 **반드시 `end_frame + 1`**부터 시작한다 (프레임 겹침 없음).\n\n"
    "3. **개수:** `n_frames` = `end_frame - start_frame + 1` (CSV `n_frames`와 일치).\n\n"
    "4. **이미지:** `robot_data.csv` 한 행의 `image_path`(예: `frame_000071.jpg`)는 **`frame_id == 71`인 행**과 대응한다.  \n"
    "   v2의 `image_start` / `image_end`는 그 구간의 첫·마지막 파일명이다.\n\n"
    "5. **배열/클립으로 자를 때** (`frame_id` 순으로 정렬된 텐서·리스트라고 가정):  \n"
    "   해당 primitive는 인덱스 **`start_frame`부터 `end_frame`까지 포함**  \n"
    "   (Python: `arr[start_frame : end_frame + 1]`).\n\n"
    "6. **JSON·프롬프트에 넣는 숫자:** v2 CSV의 `start_frame`·`end_frame`을 **그대로** 쓴다. 임의의 +1/-1 보정을 하지 않는다.\n\n"
    "**영문 한 줄 (프롬프트에 붙여도 됨):**  \n"
    "`start_frame and end_frame are inclusive indices equal to robot_data.csv column frame_id; the next segment starts at end_frame+1.`\n\n"
    "---\n\n## 3."
)
if old not in text:
    raise SystemExit("old block not found")
text = text.replace(old, new)
old2 = (
    "Frame ranges must align with robot_data.csv rows and image filenames in each episode folder.\n"
    "```"
)
new2 = (
    "start_frame and end_frame are inclusive indices equal to robot_data.csv column frame_id; the next segment starts at end_frame+1.\n"
    "```"
)
if old2 not in text:
    raise SystemExit("old2 not found")
text = text.replace(old2, new2)
p.write_text(text, encoding="utf-8")
print("patched")
