"""경로 수정 기능 GUI 테스트 (화면이 없으면 건너뜀)."""

import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

tk = pytest.importorskip("tkinter")


class E:
    def __init__(self, x, y, state=0):
        self.x, self.y, self.state = x, y, state


@pytest.fixture()
def gui():
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("화면(display) 없음")
    root.geometry("1200x800+0+0")
    import app as appmod

    a = appmod.SweptPathApp(root)
    root.update()
    a.s, a.tx, a.ty = 10.0, 100.0, 500.0  # 1 m = 10 px
    yield a, appmod
    root.destroy()


def click(a, x, y):
    a._on_press(E(x, y))
    a._on_release(E(x, y))


def drag(a, x0, y0, x1, y1, state=0):
    a._on_press(E(x0, y0, state))
    a._on_drag(E((x0 + x1) / 2, (y0 + y1) / 2, state))
    a._on_drag(E(x1, y1, state))
    a._on_release(E(x1, y1, state))


def make_path(a, appmod):
    a.tool_var.set(appmod.TOOL_START)
    drag(a, *a.w2c(0, 0), *a.w2c(5, 0))  # 출발 (0,0) 동쪽
    a.tool_var.set(appmod.TOOL_WAYPOINT)
    click(a, *a.w2c(20, 0))
    click(a, *a.w2c(30, 10))
    a.tool_var.set(appmod.TOOL_DEST)
    drag(a, *a.w2c(30, 30), *a.w2c(30, 35))  # 목적지 북쪽


def test_move_point_by_drag_and_undo(gui):
    a, appmod = gui
    make_path(a, appmod)
    assert len(a.waypoints) == 2 and a.dest is not None
    a.tool_var.set(appmod.TOOL_WAYPOINT)  # 경유점 도구여도 점 위를 누르면 새 점 대신 이동
    drag(a, *a.w2c(20, 0), *a.w2c(22, 4))
    assert len(a.waypoints) == 2
    assert (a.waypoints[0].x, a.waypoints[0].y) == pytest.approx((22, 4), abs=0.05)
    a.undo()
    assert (a.waypoints[0].x, a.waypoints[0].y) == pytest.approx((20, 0), abs=0.05)


def test_move_start_and_rotate_heading(gui):
    a, appmod = gui
    make_path(a, appmod)
    drag(a, *a.w2c(0, 0), *a.w2c(-3, 1))  # 출발점 이동
    assert a.start[:2] == pytest.approx((-3, 1), abs=0.05)
    assert a.start[2] == pytest.approx(0.0, abs=1e-6)  # 방향 유지
    L = a._arrow_len()
    tip = a.w2c(-3 + L, 1)
    drag(a, *tip, *a.w2c(-3, 1 + L))  # 화살표 끝을 끌어 북쪽으로
    assert math.degrees(a.start[2]) == pytest.approx(90, abs=1)
    # 목적지 방향도 화살표 끝으로 수정, Shift 는 15° 단위
    dx, dy = a.dest.x, a.dest.y
    tip = a.w2c(dx + L * math.cos(a.dest.heading), dy + L * math.sin(a.dest.heading))
    drag(a, *tip, *a.w2c(dx + L, dy + L * 0.3), state=0x0001)
    deg = math.degrees(a.dest.heading)
    assert deg == pytest.approx(round(deg / 15) * 15, abs=1e-6)
    assert round(deg / 15) * 15 == 15  # (L, 0.3L) 방향 ≈ 16.7° → 15°


def test_click_without_move_does_not_create_undo_step(gui):
    a, appmod = gui
    make_path(a, appmod)
    n = len(a._undo)
    click(a, *a.w2c(20, 0))  # 점 위를 누르기만 함 → 선택만
    assert len(a._undo) == n and len(a.waypoints) == 2
    assert a._selected_point() == ("wp", 0)


def test_reset_and_undo_reset(gui):
    a, appmod = gui
    make_path(a, appmod)
    before = a._snapshot()
    a.clear_path()
    assert a.start is None and not a.waypoints and a.dest is None
    a.undo()
    assert a._snapshot() == before


def test_numeric_edit(gui):
    a, appmod = gui
    make_path(a, appmod)
    a.apply_point_values("wp", 1, 31.5, 12.25, None, appmod.REVERSE)
    w = a.waypoints[1]
    assert (w.x, w.y, w.gear) == (31.5, 12.25, appmod.REVERSE)
    a.apply_point_values("dest", None, 30, 40, None, appmod.FORWARD)
    assert a.dest.heading is None and (a.dest.x, a.dest.y) == (30, 40)
    a.apply_point_values("start", None, 1, 2, 45, None)
    assert a.start == pytest.approx((1, 2, math.radians(45)))
    # 대화상자도 열리고 닫히는지
    a._select_in_list("wp", 0)
    a.edit_selected_point()
    a.root.update()
    dlg = [w for w in a.root.winfo_children() if isinstance(w, appmod.PointDialog)][0]
    dlg.vx.set("21")
    dlg._apply()
    assert a.waypoints[0].x == 21


@pytest.fixture()
def gui_with_sample(gui):
    a, appmod = gui
    a.open_drawing(str(ROOT / "samples" / "sample_parking.dxf"), wait=True)
    a.root.update()
    return a, appmod


def test_pick_layer_by_clicking_line(gui_with_sample):
    a, appmod = gui_with_sample
    assert "주차구획" not in a.obstacle_layers and "벽" in a.obstacle_layers
    a.start_pick_mode()
    assert a.tool_var.get() == appmod.TOOL_PICK
    # 주차구획선 x=1.0 m, y 7.6~12 m
    cx, cy = a.w2c(1.0, 9.0)
    a._on_motion(E(cx + 3, cy))  # 3 px 옆에 마우스
    assert a._pick_layer == "주차구획"
    assert a.canvas.find_withtag("pickhl")
    a._on_press(E(cx + 3, cy))
    assert "주차구획" in a.obstacle_layers
    assert a.layer_names.index("주차구획") in a.layer_list.curselection()
    a._on_press(E(cx + 3, cy))  # 다시 누르면 해제
    assert "주차구획" not in a.obstacle_layers
    # 남쪽 벽(y=0) 클릭 → 벽 해제
    a._on_press(E(*a.w2c(20.0, 0.0)))
    assert "벽" not in a.obstacle_layers
    # 빈 곳 클릭은 아무것도 안 바뀜
    before = set(a.obstacle_layers)
    a._on_press(E(*a.w2c(20.0, 3.5)))
    assert a.obstacle_layers == before
    # 다른 도구로 바꾸면 강조 표시 사라짐
    a.tool_var.set(appmod.TOOL_WAYPOINT)
    assert not a.canvas.find_withtag("pickhl")


def test_layer_preset_save_and_load(gui_with_sample, tmp_path):
    a, appmod = gui_with_sample
    a.obstacle_layers = {"벽"}
    p = tmp_path / "preset.json"
    from swept_path.export import save_project

    save_project(p, {"type": "swept-path-layers", "obstacle_layers": ["벽", "기둥", "없는레이어"]})
    a.load_layer_preset(str(p))
    assert a.obstacle_layers == {"벽", "기둥"}
    assert "1개" in a.status.get()  # 없는 레이어 1개 건너뜀 안내


def test_layer_search_filter_keeps_hidden_selection(gui_with_sample):
    a, appmod = gui_with_sample
    assert a.obstacle_layers == {"벽", "기둥"}
    a.layer_filter.set("주차")
    assert a._layer_view == ["주차구획"]
    assert "검색 1개" in a.layer_count_var.get()
    a.layer_list.selection_set(0)
    a._on_layer_select()
    assert a.obstacle_layers == {"벽", "기둥", "주차구획"}  # 숨겨진 벽·기둥은 그대로
    a._set_all_layers(False)  # 검색 중 '전체 해제'는 검색된 레이어에만
    assert a.obstacle_layers == {"벽", "기둥"}
    a.layer_filter.set("없는이름")
    a._set_all_layers(True)  # 검색 결과 없음 → 아무것도 안 바뀜
    assert a.obstacle_layers == {"벽", "기둥"}
    a.layer_filter.set("")
    assert len(a._layer_view) == 3


def test_step_marks_and_empty_state(gui):
    a, appmod = gui
    assert a.empty_state.winfo_manager() == "place"  # 처음에는 안내 화면
    a.open_drawing(str(ROOT / "samples" / "sample_parking.dxf"), wait=True)
    a.root.update()
    assert a.empty_state.winfo_manager() == ""
    assert a.nb.tab(0, "text").startswith("✓")
    assert not a.nb.tab(2, "text").startswith("✓")  # 경로 아직 없음


def test_erase_line_layer_rect_and_undo(gui_with_sample):
    a, appmod = gui_with_sample
    d = a.drawing
    n_park = d.layer_counts["주차구획"]
    a.start_erase_mode()
    assert a.tool_var.get() == appmod.TOOL_ERASE
    cx, cy = a.w2c(1.0, 9.0)  # 주차구획선 x=1.0 m
    a._on_motion(E(cx + 3, cy))
    assert a.canvas.find_withtag("erasehl")
    click(a, cx + 3, cy)
    assert d.removed_count == 1
    assert len(d.layer_lines("주차구획")) == n_park - 1
    assert "지움 1" in a.layer_list.get(a._layer_view.index("주차구획"))
    assert str(a.restore_btn.cget("state")) == "normal"
    a.undo()
    assert d.removed_count == 0
    # Shift+클릭: 레이어 전체
    a._on_press(E(cx + 3, cy, 0x0001))
    a._on_release(E(cx + 3, cy, 0x0001))
    assert d.removed_count == n_park
    assert "모두 지움" in a.layer_list.get(a._layer_view.index("주차구획"))
    # 지운 선은 다시 고를 수 없음 → 빈 곳 클릭과 같음
    click(a, cx + 3, cy)
    assert d.removed_count == n_park
    a.undo()
    assert d.removed_count == 0
    # 끌기: 사각형 안에 완전히 들어간 선 (기둥 4개 + 주차구획 등). 화면 전체를 감싸면 모두
    x0, y0, x1, y1 = d.bounds_m()
    drag(a, *a.w2c(x0 - 1, y1 + 1), *a.w2c(x1 + 1, y0 - 1))
    assert d.removed_count == len(d.polylines)
    a.restore_all_lines()
    assert d.removed_count == 0 and str(a.restore_btn.cget("state")) == "disabled"
    # 경로 되돌리기와 섞여도 순서대로 되돌린다
    make_path(a, appmod)
    a.tool_var.set(appmod.TOOL_ERASE)
    click(a, cx + 3, cy)
    n_wp = len(a.waypoints)
    a.undo()  # 선 지우기 취소
    assert d.removed_count == 0 and len(a.waypoints) == n_wp
    a.undo()  # 목적지 취소
    assert a.dest is None


def test_erase_layer_from_list_and_obstacles(gui_with_sample):
    a, appmod = gui_with_sample
    a.erase_layer("기둥")
    assert a.drawing.removed_count == 4
    assert len(a._obstacles()[0]) == len(a.drawing.polylines_m({"벽"}))  # 기둥 선은 장애물에서 빠짐
    a.restore_layer("기둥")
    assert a.drawing.removed_count == 0


def test_project_saves_removed_lines(gui_with_sample, tmp_path, monkeypatch):
    a, appmod = gui_with_sample
    a.erase_layer("기둥")
    p = tmp_path / "work.json"
    monkeypatch.setattr(appmod.filedialog, "asksaveasfilename", lambda **k: str(p))
    monkeypatch.setattr(appmod.filedialog, "askopenfilename", lambda **k: str(p))
    a.save_project()
    a.restore_all_lines()
    a.load_project()
    assert a.drawing.removed_count == 4
    assert len(a.drawing.layer_lines("기둥")) == 0


def test_compare_all_vehicles(gui_with_sample, tmp_path):
    a, appmod = gui_with_sample
    # 샘플 도면 90° 코너: 소형·카운티 가능, 대형·세미트레일러 불가
    a.start = (18.0, 3.5, 0.0)
    a.waypoints = [appmod.Waypoint(30, 3.5), appmod.Waypoint(36.5, 10)]
    a.dest = appmod.Waypoint(36.5, 32, appmod.FORWARD, math.pi / 2)
    a._path_changed()
    a.compare_all_vehicles(wait=True)
    dlg = a.compare_dialog
    table = {row[0]: row for row in dlg.table()}
    assert table["소형자동차 (설계기준)"][1].startswith("가능")
    assert table["미니버스 (카운티 장축)"][1].startswith("가능")
    assert table["대형자동차 (설계기준)"][1] == "불가"
    assert table["세미트레일러 (설계기준)"][1] == "불가"
    assert str(a.compare_btn.cget("state")) == "normal" and not a._busy
    # 표에서 고른 차량 결과를 화면에 띄우기
    names = [v.name for v, _a, _e in dlg.rows]
    i = names.index("대형자동차 (설계기준)")
    dlg.tree.selection_set(str(i))
    dlg.show_selected()
    assert a.vehicle.name == "대형자동차 (설계기준)" and a.analysis is not None and not a.analysis.feasible
    # CSV 저장 (엑셀용 BOM)
    p = tmp_path / "cmp.csv"
    dlg.save_csv(str(p))
    text = p.read_bytes()
    assert text.startswith(b"\xef\xbb\xbf") and "세미트레일러" in text.decode("utf-8-sig")
    dlg.destroy()
