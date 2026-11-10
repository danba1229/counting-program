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
