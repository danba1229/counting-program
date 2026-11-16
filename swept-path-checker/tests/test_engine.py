import math
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from swept_path.analysis import run_check  # noqa: E402
from swept_path.drawing import load_drawing  # noqa: E402
from swept_path.export import export_dxf, export_report, scenario_from_dict, scenario_to_dict  # noqa: E402
from swept_path.simulate import FORWARD, REVERSE, Scenario, Simulator, Waypoint, split_legs  # noqa: E402
from swept_path.vehicles import Vehicle, VehicleLibrary, builtin_vehicles, outer_front_radius  # noqa: E402

VEHICLES = {v.key: v for v in builtin_vehicles()}


@pytest.fixture(scope="module")
def sample(tmp_path_factory):
    sys.path.insert(0, str(ROOT / "samples"))
    from make_sample import make

    return make(tmp_path_factory.mktemp("dxf") / "sample.dxf")


@pytest.mark.parametrize("key", list(VEHICLES))
def test_max_steer_reproduces_min_turn_radius(key):
    v = VEHICLES[key]
    r = outer_front_radius(v.max_steer_rad, v.wheelbase, v.track)
    assert r == pytest.approx(v.min_turn_radius, abs=1e-6)


def test_full_lock_circle_matches_min_turn_radius():
    """최대 조향으로 원을 돌면 바깥 앞바퀴 궤적 반경 = 최소회전반경."""
    v = VEHICLES["large"]
    v_fixed = Vehicle.from_dict(v.to_dict())
    sim = Simulator(v_fixed)
    x, y, th, phi = 0.0, 0.0, 0.0, 0.0
    pts = []
    for _ in range(800):
        x, y, th, phi = sim._integrate(x, y, th, phi, v.max_steer_rad, FORWARD, 0.05)
        c, s = math.cos(th), math.sin(th)
        fx, fy = x + v.wheelbase * c, y + v.wheelbase * s
        pts.append((fx + s * v.track / 2, fy - c * v.track / 2))  # 오른쪽(바깥) 앞바퀴
    pts = np.array(pts)
    rear_r = v.wheelbase / math.tan(v.max_steer_rad)
    center = np.array([0.0, rear_r])
    radii = np.hypot(*(pts - center).T)
    assert radii.mean() == pytest.approx(v.min_turn_radius, rel=1e-3)


def test_split_legs_by_gear():
    wps = [Waypoint(1, 0), Waypoint(2, 0), Waypoint(1, 0, REVERSE), Waypoint(3, 0)]
    legs = split_legs(wps)
    assert [len(l) for l in legs] == [2, 1, 1]
    assert [l[0].gear for l in legs] == [FORWARD, REVERSE, FORWARD]


@pytest.mark.parametrize("key", list(VEHICLES))
def test_straight_forward_and_reverse(key):
    v = VEHICLES[key]
    sc = Scenario(0, 0, 0, [Waypoint(30, 0), Waypoint(5, 0, REVERSE, 0.0)])
    a = run_check(v, sc, [])
    assert a.feasible, a.reasons
    assert len(a.sim.legs) == 2
    assert max(l.max_deviation for l in a.sim.legs) < 0.05


@pytest.mark.parametrize("key", ["passenger", "minibus_county", "large", "semitrailer"])
def test_wide_right_turn_without_obstacles(key):
    v = VEHICLES[key]
    sc = Scenario(0, 0, 0, [Waypoint(20, 0), Waypoint(40, -20), Waypoint(40, -45, FORWARD, -math.pi / 2)])
    a = run_check(v, sc, [])
    assert a.feasible, a.reasons
    assert a.sim.legs[-1].heading_error_deg < 10


def test_reverse_turn_passenger():
    v = VEHICLES["passenger"]
    sc = Scenario(0, 0, 0, [Waypoint(10, 0), Waypoint(-10, 10, REVERSE),
                            Waypoint(-10, 25, REVERSE, -math.pi / 2)])
    a = run_check(v, sc, [])
    assert a.feasible, a.reasons


def test_semitrailer_reverse_arc():
    v = VEHICLES["semitrailer"]
    R = 25.0
    pts = [(-13.2 - R * math.sin(t), R - R * math.cos(t)) for t in np.linspace(0.05, math.pi / 2, 10)]
    wps = [Waypoint(x, y, REVERSE) for x, y in pts[:-1]] + [Waypoint(*pts[-1], REVERSE, -math.pi / 2)]
    a = run_check(v, Scenario(0, 0, 0, wps), [])
    assert a.feasible, a.reasons
    assert a.max_articulation_deg < v.max_articulation_deg


def test_impossible_tight_turn_is_rejected():
    """회전반경보다 훨씬 작은 급회전은 목표 도달 실패로 판정."""
    v = VEHICLES["large"]
    sc = Scenario(0, 0, 0, [Waypoint(10, 0), Waypoint(10, 4, FORWARD, math.pi / 2)])
    a = run_check(v, sc, [])
    assert not a.feasible


def test_wall_collision_detected():
    v = VEHICLES["passenger"]
    wall = [np.array([[10.0, -5.0], [10.0, 5.0]])]
    sc = Scenario(0, 0, 0, [Waypoint(20, 0)])
    a = run_check(v, sc, [wall[0]])
    assert not a.feasible
    assert a.collisions
    assert a.collisions[0].x == pytest.approx(10.0, abs=0.5)


def test_clearance_margin():
    v = VEHICLES["passenger"]
    # 차폭 1.7 → 중심선에서 0.85. 벽을 1.0 m 에 두면 여유 0.15 m
    wall = np.array([[-10.0, 1.0], [40.0, 1.0]])
    sc = Scenario(0, 0, 0, [Waypoint(20, 0)])
    assert not run_check(v, sc, [wall]).feasible  # 기본 여유폭 0.3
    from swept_path.analysis import CheckOptions

    assert run_check(v, sc, [wall], check_options=CheckOptions(clearance=0.1)).feasible


def test_sample_drawing_layers_and_units(sample):
    d = load_drawing(sample)
    assert d.unit_scale == 0.001
    assert set(d.layer_counts) >= {"벽", "기둥", "주차구획"}
    assert d.default_obstacle_layers() == {"벽", "기둥"}
    # 기둥 블록(0 레이어 객체)이 삽입 레이어로 들어왔는지
    assert d.layer_counts["기둥"] == 4
    x0, y0, x1, y1 = d.bounds_m()
    assert (x1 - x0) == pytest.approx(40.0, abs=0.01)


def test_sample_corner_results(sample):
    d = load_drawing(sample)
    obs = d.polylines_m(d.default_obstacle_layers())
    sc = Scenario(18, 3.5, 0, [Waypoint(30, 3.5), Waypoint(36.5, 10),
                               Waypoint(36.5, 32, FORWARD, math.pi / 2)])
    assert run_check(VEHICLES["passenger"], sc, obs).feasible
    assert run_check(VEHICLES["minibus_county"], sc, obs).feasible
    semi = run_check(VEHICLES["semitrailer"], sc, obs)
    assert not semi.feasible and semi.collisions


def test_export_roundtrip(sample, tmp_path):
    import ezdxf

    d = load_drawing(sample)
    sc = Scenario(18, 3.5, 0, [Waypoint(30, 3.5), Waypoint(36.5, 10), Waypoint(36.5, 32)])
    a = run_check(VEHICLES["large"], sc, d.polylines_m(d.default_obstacle_layers()))
    out = export_dxf(a, sc, tmp_path / "out.dxf", unit_scale=d.unit_scale, base_drawing=sample)
    doc = ezdxf.readfile(out)
    layers = {e.dxf.layer for e in doc.modelspace()}
    assert {"ST_ENVELOPE", "ST_WHEEL", "벽"} <= layers
    # 결과 좌표가 mm 단위로 들어갔는지
    env = next(e for e in doc.modelspace() if e.dxf.layer == "ST_ENVELOPE")
    xs = [p[0] for p in env.get_points("xy")]
    assert max(xs) > 1000
    rep = export_report(a, tmp_path / "r.txt", "sample.dxf")
    assert "판정" in rep.read_text(encoding="utf-8-sig")
    assert scenario_from_dict(scenario_to_dict(sc)) == sc


def test_vehicle_library_user_file(tmp_path):
    lib = VehicleLibrary(tmp_path / "v.json")
    v = Vehicle.from_dict({**VEHICLES["passenger"].to_dict(), "key": "user_1", "name": "내 차"})
    lib.add_or_replace(v)
    lib2 = VehicleLibrary(tmp_path / "v.json")
    assert lib2.by_name("내 차").wheelbase == v.wheelbase
    lib2.remove("user_1")
    assert "내 차" not in VehicleLibrary(tmp_path / "v.json").names()


def test_vehicle_validation():
    bad = Vehicle(key="x", name="x", wheelbase=5.0, min_turn_radius=4.0)
    assert bad.validate()


def test_block_cache_transforms(tmp_path):
    """블록 캐시 + 행렬 변환: 중첩 블록, 회전, 비균일 배율, 기준점, 0 레이어 상속."""
    import ezdxf

    doc = ezdxf.new()
    inner = doc.blocks.new("IN", base_point=(10, 0))
    inner.add_line((10, 0), (30, 0))
    inner.add_circle((20, 0), 1, dxfattribs={"layer": "원"})
    outer = doc.blocks.new("OUT", base_point=(100, 50))
    outer.add_blockref("IN", (100, 50), dxfattribs={"rotation": 45})
    msp = doc.modelspace()
    msp.add_blockref("OUT", (1000, 2000), dxfattribs={"rotation": 30, "xscale": 2, "yscale": -1, "layer": "기둥"})
    msp.add_blockref("OUT", (0, 0), dxfattribs={"rotation": 90, "layer": "기둥"})
    path = tmp_path / "blocks.dxf"
    doc.saveas(path)
    d = load_drawing(path)
    lines = [p for p, l in zip(d.polylines, d.layers) if l == "기둥"]
    assert {l for l in d.layers} == {"기둥", "원"}  # 0 레이어 → 삽입 레이어, 지정 레이어 유지

    def expected(local, scale, rot, ins):
        x, y = local[0] * scale[0], local[1] * scale[1]
        c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        return (x * c - y * s + ins[0], x * s + y * c + ins[1])

    # IN 블록 선 (0,0)-(20,0) → 45° 회전 → OUT 블록 좌표 (0,0)-(14.142, 14.142)
    end_local = (20 * math.cos(math.radians(45)), 20 * math.sin(math.radians(45)))
    got_ends = sorted(tuple(np.round(p[-1], 4)) for p in lines)
    want_ends = sorted([
        tuple(np.round(expected(end_local, (2, -1), 30, (1000, 2000)), 4)),
        tuple(np.round(expected(end_local, (1, 1), 90, (0, 0)), 4)),
    ])
    assert got_ends == want_ends
    # 원 중심 (IN 좌표 (10,0) 기준점 기준) 도 같은 변환을 따른다
    circles = [p for p, l in zip(d.polylines, d.layers) if l == "원"]
    centre_local = (10 * math.cos(math.radians(45)), 10 * math.sin(math.radians(45)))
    want_c = np.array(expected(centre_local, (1, 1), 90, (0, 0)))
    assert any(np.allclose(c.mean(axis=0), want_c, atol=0.05) for c in circles)


def test_renderer_draws_visible_lines_only(sample):
    from swept_path.render import DrawingRenderer, preview_transform

    d = load_drawing(sample)
    r = DrawingRenderer(d)
    s, tx, ty = 20.0, 10.0, 750.0  # 0~40 m 가 화면 안에 들어오는 변환
    img = r.render(900, 800, s, tx, ty, {"벽"})
    arr = np.asarray(img)
    # 남쪽 벽 (y=0 → 화면 y=750) 이 검게 그려져야 함
    assert (arr[748:753, 100:700] < 80).all(axis=2).any()
    # 화면 밖으로 이동하면 아무것도 안 그림
    empty = np.asarray(r.render(900, 800, s, tx + 5000, ty, {"벽"}))
    assert (empty == 255).all()
    # 미리보기: 오른쪽으로 30px 이동한 변환이면 이미지도 30px 이동
    moved = np.asarray(preview_transform(img, (s, tx, ty), (s, tx + 30, ty), (900, 800)))
    assert (moved[:, 130:600] == arr[:, 100:570]).all()


def test_drawing_cache_roundtrip_and_invalidation(sample, tmp_path):
    import os
    import shutil

    src = tmp_path / "c.dxf"
    shutil.copy(sample, src)
    cache = tmp_path / "cachedir"
    a = load_drawing(src, cache_dir=cache)
    assert len(list(cache.glob("*.npz"))) == 1
    b = load_drawing(src, cache_dir=cache)  # 캐시에서
    assert b.layers == a.layers and b.layer_names == a.layer_names
    assert all(np.array_equal(p, q) for p, q in zip(a.polylines, b.polylines))
    assert np.array_equal(a.bboxes, b.bboxes)
    assert b.default_obstacle_layers() == a.default_obstacle_layers()
    # 파일이 바뀌면 다시 변환 (새 캐시 생성)
    st = src.stat()
    os.utime(src, ns=(st.st_atime_ns, st.st_mtime_ns + 10_000_000_000))
    load_drawing(src, cache_dir=cache)
    assert len(list(cache.glob("*.npz"))) == 2


def _total_length(d):
    return sum(float(np.hypot(*np.diff(p, axis=0).T).sum()) for p in d.polylines)


def test_real_drawing_regressions(tmp_path):
    """실제 도면(이천송정)에서 발견된 문제 3가지 재현."""
    import ezdxf

    doc = ezdxf.new()
    doc.header["$INSUNITS"] = 4
    msp = doc.modelspace()
    # 1) 두께가 있는 폴리선: 중심선이 아니라 띠의 테두리로 읽어야 함
    msp.add_lwpolyline([(0, 0), (1000, 0)], dxfattribs={"const_width": 200, "layer": "두께"})
    # 2) 작은 단위로 그린 블록을 크게 확대해 삽입: 원이 각지게 나오면 안 됨
    blk = doc.blocks.new("작은원")
    blk.add_circle((0, 0), 0.5, dxfattribs={"layer": "원"})
    msp.add_blockref("작은원", (5000, 0), dxfattribs={"xscale": 1000, "yscale": 1000})
    # 3) 길이 0 구간(곡선 포함)이 섞인 해치: 깨진 조각만 건너뛰고 경계는 살려야 함
    blk2 = doc.blocks.new("해치")
    h = blk2.add_hatch(dxfattribs={"layer": "해치경계"})
    h.paths.add_polyline_path([(0.5, 0, -1), (0, 0, 1), (-0.5, 0, 1), (0.5, 0, -1)], is_closed=True)
    msp.add_blockref("해치", (9000, 0), dxfattribs={"xscale": -200, "yscale": 200})
    path = tmp_path / "regress.dxf"
    doc.saveas(path)

    d = load_drawing(path, use_cache=False)
    by_layer = {}
    for p, l in zip(d.polylines, d.layers):
        by_layer.setdefault(l, []).append(p)
    wide = np.concatenate(by_layer["두께"])
    assert wide[:, 1].max() == pytest.approx(100, abs=1) and wide[:, 1].min() == pytest.approx(-100, abs=1)
    circle = np.concatenate(by_layer["원"])
    radii = np.hypot(circle[:, 0] - 5000, circle[:, 1])
    assert radii.min() > 500 - 25  # 허용오차(20 mm) 안에서 매끈한 원
    assert len(circle) > 8  # 네모처럼 각지지 않을 것 (정밀도 기준은 위의 반지름 조건)
    hatch_len = sum(float(np.hypot(*np.diff(p, axis=0).T).sum()) for p in by_layer["해치경계"])
    # 곡선을 허용오차(20 mm) 안에서 직선으로 나누므로 길이는 몇 % 짧을 수 있다
    assert hatch_len == pytest.approx(math.pi * (0.25 + 0.25 + 0.5) * 200, rel=0.05)
    pts = np.concatenate(by_layer["해치경계"])
    # ezdxf 로 같은 블록(길이 0 구간 제외)을 변환한 기준값: x 8900~9100, y -100~50 (허용오차 20 mm)
    assert pts[:, 0].min() == pytest.approx(8900, abs=1) and pts[:, 0].max() == pytest.approx(9100, abs=1)
    assert pts[:, 1].min() == pytest.approx(-100, abs=21) and pts[:, 1].max() == pytest.approx(50, abs=21)


def _engine_test_dxf(path, version="R2010"):
    import ezdxf

    doc = ezdxf.new(version)
    doc.header["$INSUNITS"] = 4
    for name in ("벽", "기둥", "곡선"):
        doc.layers.add(name)
    inner = doc.blocks.new("IN", base_point=(100, 0))
    inner.add_line((100, 0), (600, 0))
    inner.add_circle((300, 200), 150, dxfattribs={"layer": "곡선"})
    inner.add_lwpolyline([(0, 0, 0.5), (400, 0, 0), (400, 300, -1.0), (0, 300, 0)], format="xyb", close=True)
    outer = doc.blocks.new("OUT", base_point=(50, 50))
    outer.add_blockref("IN", (500, 500), dxfattribs={"rotation": 30, "xscale": 2, "yscale": -1.5})
    outer.add_arc((0, 0), 800, 20, 250)
    outer.add_spline(fit_points=None).apply_construction_tool(
        ezdxf.math.BSpline([(0, 0), (500, 900), (1500, -400), (2500, 800), (3000, 0)]))
    msp = doc.modelspace()
    msp.add_blockref("OUT", (10000, 20000), dxfattribs={"rotation": 75, "xscale": 3, "yscale": 3, "layer": "기둥"})
    msp.add_blockref("IN", (0, 0), dxfattribs={"layer": "벽", "column_count": 3, "row_count": 2,
                                               "column_spacing": 2000, "row_spacing": 1500, "rotation": 10})
    msp.add_blockref("IN", (-5000, 0), dxfattribs={"layer": "벽", "extrusion": (0, 0, -1), "rotation": 20})
    msp.add_arc((3000, 3000), 700, 30, 120, dxfattribs={"layer": "곡선", "extrusion": (0, 0, -1)})
    msp.add_ellipse((6000, 0), major_axis=(1500, 500), ratio=0.4, dxfattribs={"layer": "곡선"})
    msp.add_spline(dxfattribs={"layer": "곡선"}).apply_construction_tool(
        ezdxf.math.BSpline([(0, 5000), (1000, 7000), (3000, 4000), (5000, 6500), (7000, 5000)], order=3))
    msp.add_polyline2d([(0, -3000), (2000, -3000, 0.7), (2000, -1000)], format="xyb", dxfattribs={"layer": "벽"})
    msp.add_lwpolyline([(8000, 8000), (9000, 8000), (9000, 9500)], dxfattribs={"const_width": 120, "layer": "벽"})
    h = msp.add_hatch(dxfattribs={"layer": "곡선"})
    h.paths.add_polyline_path([(12000, 0, 0.4), (13000, 0), (13000, 800), (12000, 800)], is_closed=True)
    msp.add_text("글자", dxfattribs={"layer": "벽"})
    psp = doc.layout("Layout1")
    psp.add_line((0, 0), (99999, 99999), dxfattribs={"layer": "종이공간"})  # 모델공간이 아니므로 읽지 않아야 함
    doc.saveas(path)
    return path


@pytest.mark.parametrize("version", ["R2010", "R2000"])
def test_fast_reader_matches_ezdxf(tmp_path, version):
    """빠른 판독기와 ezdxf 판독기의 결과가 허용오차 안에서 같은지 (레이어별 선 모양 비교)."""
    from shapely.geometry import MultiLineString

    path = _engine_test_dxf(tmp_path / f"engine_{version}.dxf", version)
    fast = load_drawing(path, use_cache=False, engine="fast")
    ref = load_drawing(path, use_cache=False, engine="ezdxf")
    assert fast.unit_scale == ref.unit_scale == 0.001
    assert set(fast.layer_counts) == set(ref.layer_counts) == {"벽", "기둥", "곡선"}
    assert "종이공간" not in fast.layer_counts
    for layer in ref.layer_counts:
        a = MultiLineString([p for p, l in zip(fast.polylines, fast.layers) if l == layer])
        b = MultiLineString([p for p, l in zip(ref.polylines, ref.layers) if l == layer])
        assert a.hausdorff_distance(b) < 45, layer  # 곡선 분할 허용오차 20 mm × 2 (양쪽) 이내
        assert a.length == pytest.approx(b.length, rel=0.01), layer


def test_fast_reader_falls_back_on_unsupported(tmp_path):
    """빠른 판독기가 못 읽는 파일(바이너리 DXF)은 자동으로 기존 방식으로 읽는다."""
    import ezdxf

    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4
    doc.modelspace().add_line((0, 0), (1000, 0), dxfattribs={"layer": "벽"})
    path = tmp_path / "binary.dxf"
    doc.saveas(path, fmt="bin")
    d = load_drawing(path, use_cache=False)
    assert d.layer_counts == {"벽": 1}
    with pytest.raises(Exception):
        load_drawing(path, use_cache=False, engine="fast")


def test_polylines_bbox_filter(sample):
    d = load_drawing(sample, use_cache=False)
    allp = d.polylines_m({"벽", "기둥"})
    near = d.polylines_m({"벽", "기둥"}, bbox_m=(5.0, 6.5, 7.0, 8.0))  # 첫 기둥 (6.0, 7.3) 주변
    assert 0 < len(near) < len(allp)
    assert any(abs(p[:, 0].mean() - 6.0) < 0.5 for p in near)


def test_pdf_and_png_report(sample, tmp_path):
    from PIL import Image

    from swept_path import report

    d = load_drawing(sample, use_cache=False)
    layers = d.default_obstacle_layers()
    sc = Scenario(18, 3.5, 0, [Waypoint(30, 3.5), Waypoint(36.5, 10), Waypoint(36.5, 32, FORWARD, math.pi / 2)])
    bad = run_check(VEHICLES["semitrailer"], sc, d.polylines_m(layers))
    pages = report.build_pages(bad, sc, d, layers, {"clearance": 0.3})
    assert len(pages) == 3  # 그림, 상세, 충돌 확대도
    good = run_check(VEHICLES["passenger"], sc, d.polylines_m(layers))
    assert len(report.build_pages(good, sc, d, layers)) == 2  # 충돌 없으면 확대도 없음
    pdf = report.export_pdf(bad, sc, tmp_path / "보고서.pdf", d, layers, {"clearance": 0.3})
    assert pdf.read_bytes()[:5] == b"%PDF-" and pdf.stat().st_size > 50_000
    png = report.export_png(good, sc, tmp_path / "그림.png")  # 도면 없이도 가능
    img = Image.open(png)
    assert img.size == report.PAGE
    arr = np.asarray(img.convert("RGB"))
    assert ((arr[:, :, 2] > 200) & (arr[:, :, 0] < 80)).sum() > 500  # 파란 궤적 외곽선이 그려짐


def test_removed_lines_are_excluded_everywhere(sample):
    from swept_path.render import DrawingRenderer

    d = load_drawing(sample)
    n = len(d.polylines)
    obs_layers = d.default_obstacle_layers()
    n_obs = len(d.polylines_m(obs_layers))
    pillars = d.layer_lines("기둥")
    assert len(pillars) == 4
    assert len(d.set_removed(pillars)) == 4
    assert len(d.set_removed(pillars)) == 0  # 이미 지운 선은 다시 바뀌지 않음
    assert d.removed_count == 4 and len(d.polylines) == n  # 원본 배열은 그대로
    assert len(d.polylines_m(obs_layers)) == n_obs - 4
    assert len(d.layer_lines("기둥")) == 0 and len(d.layer_lines("기둥", include_removed=True)) == 4
    assert d.active_layer_counts()["기둥"] == 0 and d.layer_counts["기둥"] == 4
    # 지운 선은 선 고르기·화면 그리기에서도 빠진다
    k = d.unit_scale
    p = d.verts[d.offsets[pillars[0]]]
    assert d.nearest_polyline(p[0], p[1], 0.05 / k) not in set(pillars.tolist())
    r = DrawingRenderer(d)
    assert r.visible_count(800, 600, 10.0, 100.0, 500.0) <= n - 4
    # 구간 저장 → 다시 적용
    ranges = d.removed_ranges()
    d.restore_all()
    assert d.removed_count == 0
    assert d.apply_removed_ranges(ranges) == 4
    assert set(np.nonzero(d.removed)[0].tolist()) == set(pillars.tolist())
    # 사각형 안에 완전히 들어간 선만
    x0, y0, x1, y1 = (v / k for v in d.bounds_m())
    assert len(d.lines_in_rect(x0 - 1, y0 - 1, x1 + 1, y1 + 1)) == n - 4
    assert len(d.lines_in_rect(x0, y0, x0 + 1e-6, y0 + 1e-6)) == 0


def test_removing_pillars_changes_verdict(sample):
    d = load_drawing(sample)
    sc = Scenario(18, 3.5, 0, [Waypoint(30, 3.5), Waypoint(36.5, 10),
                               Waypoint(36.5, 32, FORWARD, math.pi / 2)])
    before = run_check(VEHICLES["semitrailer"], sc, d.polylines_m(d.default_obstacle_layers()))
    d.set_removed(d.layer_lines("기둥"))
    after = run_check(VEHICLES["semitrailer"], sc, d.polylines_m(d.default_obstacle_layers()))
    # 기둥에 먼저 부딪히던 차량이, 기둥을 지우면 더 가서(다른 곳에서) 부딪히거나 통과한다
    assert after.feasible or after.collisions[0].dist_from > before.collisions[0].dist_from + 1.0
