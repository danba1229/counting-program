"""검토 결과 내보내기 (DXF, 보고서, 작업 파일)."""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path

import numpy as np

from .analysis import Analysis
from .simulate import Scenario, Waypoint

RESULT_LAYERS = {
    "ST_ENVELOPE": 5,  # 차체 궤적 외곽 (파랑)
    "ST_CLEARANCE": 4,  # 여유폭 포함 외곽 (하늘)
    "ST_WHEEL": 3,  # 바퀴 궤적 (초록)
    "ST_CORNER": 30,  # 차체 모서리 궤적 (주황)
    "ST_VEHICLE": 8,  # 차량 윤곽 (회색)
    "ST_PATH": 6,  # 지정 경로 (자홍)
    "ST_COLLISION": 1,  # 충돌 위치 (빨강)
    "ST_TEXT": 7,
}


def _geoms(geom):
    if geom is None or geom.is_empty:
        return []
    return list(getattr(geom, "geoms", [geom]))


def export_dxf(analysis: Analysis, scenario: Scenario, out_path: str | Path,
               unit_scale: float = 1.0, base_drawing: str | Path | None = None) -> Path:
    """결과를 DXF로 저장. base_drawing 을 주면 원본 도면에 결과 레이어를 추가한 사본을 만든다.

    unit_scale: 도면 단위 → m 환산계수 (mm 도면이면 0.001)
    """
    import ezdxf

    from .drawing import read_document

    out_path = Path(out_path)
    if base_drawing:
        doc = read_document(Path(base_drawing))
    else:
        doc = ezdxf.new("R2010", setup=True)
        doc.header["$INSUNITS"] = {0.001: 4, 0.01: 5, 1.0: 6}.get(unit_scale, 0)
    msp = doc.modelspace()
    for name, color in RESULT_LAYERS.items():
        if name not in doc.layers:
            doc.layers.add(name, color=color)

    k = 1.0 / unit_scale

    def pts(arr):
        return [(float(p[0]) * k, float(p[1]) * k) for p in arr]

    for layer, geom in (("ST_ENVELOPE", analysis.envelope), ("ST_CLEARANCE", analysis.clearance_envelope)):
        for g in _geoms(geom):
            msp.add_lwpolyline(pts(np.asarray(g.exterior.coords)), close=True, dxfattribs={"layer": layer})
            for hole in g.interiors:
                msp.add_lwpolyline(pts(np.asarray(hole.coords)), close=True, dxfattribs={"layer": layer})

    for name, track in analysis.tracks.items():
        layer = "ST_WHEEL" if "wheel" in name else "ST_CORNER"
        sampled = track[:: max(1, len(track) // 2000)]
        if len(sampled) >= 2:
            msp.add_lwpolyline(pts(sampled), dxfattribs={"layer": layer})

    for snap in analysis.snapshots:
        for poly in snap.values():
            msp.add_lwpolyline(pts(poly), close=True, dxfattribs={"layer": "ST_VEHICLE"})

    path_pts = [(scenario.start_x, scenario.start_y)] + [(w.x, w.y) for w in scenario.waypoints]
    msp.add_lwpolyline(pts(np.array(path_pts)), dxfattribs={"layer": "ST_PATH"})

    for c in analysis.collisions:
        msp.add_circle((c.x * k, c.y * k), 0.6 * k, dxfattribs={"layer": "ST_COLLISION"})

    if "ST_KOR" not in doc.styles:
        doc.styles.add("ST_KOR", font="malgun.ttf")  # 한글 표시용 (맑은 고딕)
    text = ("회전 가능" if analysis.feasible else "회전 불가") + f" - {analysis.vehicle.name}"
    msp.add_text(text, height=0.8 * k, dxfattribs={"layer": "ST_TEXT", "style": "ST_KOR"}).set_placement(
        (scenario.start_x * k, (scenario.start_y + 3.0) * k)
    )
    doc.saveas(str(out_path))
    return out_path


def export_report(analysis: Analysis, out_path: str | Path, drawing_name: str = "") -> Path:
    out_path = Path(out_path)
    lines = [
        "차량 회전 궤적 검토 보고서",
        f"작성: {datetime.now():%Y-%m-%d %H:%M}",
    ]
    if drawing_name:
        lines.append(f"도면: {drawing_name}")
    v = analysis.vehicle
    lines += [
        "",
        "[차량 제원]",
        f"  전장 {v.overall_length:.3f} m / 폭 {v.width:.3f} m / 축간거리 {v.wheelbase:.3f} m",
        f"  앞내민 {v.front_overhang:.3f} m / 뒷내민 {v.rear_overhang:.3f} m / 윤거 {v.track:.3f} m",
        f"  최소회전반경 {v.min_turn_radius:.2f} m / 조향 끝~끝 {v.lock_to_lock_s:.1f} 초",
    ]
    if v.is_articulated:
        lines.append(
            f"  트레일러 축간거리 {v.trailer_wheelbase:.3f} m / 뒷내민 {v.trailer_rear_overhang:.3f} m / "
            f"최대 꺾임각 {v.max_articulation_deg:.0f}°"
        )
    if v.note:
        lines.append(f"  비고: {v.note}")
    lines += ["", "[검토 결과]"] + analysis.summary_lines()
    out_path.write_text("\n".join(lines), encoding="utf-8-sig")
    return out_path


# ----------------------------------------------------------------------
# 작업 파일 (도면 경로, 차량, 경로, 옵션 저장)
# ----------------------------------------------------------------------
def save_project(path: str | Path, data: dict) -> None:
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def load_project(path: str | Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def scenario_to_dict(sc: Scenario) -> dict:
    return {
        "start": [sc.start_x, sc.start_y, sc.start_heading],
        "waypoints": [asdict(w) for w in sc.waypoints],
    }


def scenario_from_dict(d: dict) -> Scenario:
    sx, sy, sh = d["start"]
    return Scenario(sx, sy, sh, [Waypoint(**w) for w in d["waypoints"]])
