from pathlib import Path

p = Path(__file__).resolve().parent / "pick_place_gui_define.py"
text = p.read_text(encoding="utf-8")
old = """            o = meta_extra.get("source_xy_origin")
            if d == "B_to_A" and o == "previous_episode_place":
                meta_extra["chained_from_previous_episode"] = True"""
new = """            o = meta_extra.get("source_xy_origin")
            if d == "B_to_A" and o == "previous_episode_place":
                meta_extra["chained_from_previous_episode"] = True
                meta_extra["previous_ab_place_xy_mm"] = [
                    float(sw.pick_x) if getattr(sw, "pick_x", None) is not None else None,
                    float(sw.pick_y) if getattr(sw, "pick_y", None) is not None else None,
                ]"""
if old not in text:
    raise SystemExit("meta block missing")
text = text.replace(old, new, 1)
old2 = "QTimer.singleShot(400, lambda: self._run_chained_b_to_a_step(px, py))"
new2 = "QTimer.singleShot(400, lambda p=px, q=py: self._run_chained_b_to_a_step(p, q))"
if old2 not in text:
    raise SystemExit("lambda missing")
text = text.replace(old2, new2, 1)
p.write_text(text, encoding="utf-8")
print("ok")
