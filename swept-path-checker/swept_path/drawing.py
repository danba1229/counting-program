"""DXF/DWG 도면 읽기.

DXF는 ezdxf로 직접 읽는다. DWG는 형식이 공개되어 있지 않아 무료 ODA File Converter가
설치되어 있으면 이를 이용해 자동 변환하여 읽는다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re

import numpy as np

# $INSUNITS 코드 → m 환산계수
INSUNITS_TO_M = {
    1: 0.0254,  # inch
    2: 0.3048,  # feet
    4: 0.001,  # mm
    5: 0.01,  # cm
    6: 1.0,  # m
}
UNIT_CHOICES = {"mm": 0.001, "cm": 0.01, "m": 1.0, "inch": 0.0254, "feet": 0.3048}

SKIP_TYPES = {
    "TEXT", "MTEXT", "ATTRIB", "ATTDEF", "DIMENSION", "ARC_DIMENSION", "LEADER", "MLEADER",
    "MULTILEADER", "POINT", "IMAGE", "VIEWPORT", "WIPEOUT", "TOLERANCE", "XLINE", "RAY",
    "OLE2FRAME", "UNDERLAY", "PDFUNDERLAY", "DWFUNDERLAY", "DGNUNDERLAY",
}

# 기본으로 장애물에서 제외할 레이어 이름 패턴
NON_OBSTACLE_PATTERN = re.compile(
    r"(DIM|TEXT|TXT|ANNO|HATCH|PAT|CEN|DEFPOINTS|GRID|AXIS|TITLE|FRAME|BORDER|SYMB|MARK|"
    r"치수|문자|글자|해치|중심|그리드|도곽|기호|주기|주차구획)",
    re.IGNORECASE,
)


@dataclass
class Drawing:
    path: Path
    polylines: list[np.ndarray] = field(default_factory=list)  # 도면 단위 좌표 (N,2)
    layers: list[str] = field(default_factory=list)  # polylines와 같은 길이
    layer_counts: dict[str, int] = field(default_factory=dict)
    insunits: int = 0
    unit_scale: float = 0.001  # 도면 단위 → m
    unit_guessed: bool = False
    skipped: dict[str, int] = field(default_factory=dict)

    def bounds_m(self) -> tuple[float, float, float, float] | None:
        if not self.polylines:
            return None
        allpts = np.concatenate(self.polylines)
        lo = allpts.min(axis=0) * self.unit_scale
        hi = allpts.max(axis=0) * self.unit_scale
        return float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])

    def default_obstacle_layers(self) -> set[str]:
        return {name for name in self.layer_counts if not NON_OBSTACLE_PATTERN.search(name)}

    def polylines_m(self, layers: set[str] | None = None) -> list[np.ndarray]:
        out = []
        for pts, layer in zip(self.polylines, self.layers):
            if layers is None or layer in layers:
                out.append(pts * self.unit_scale)
        return out


class DrawingError(Exception):
    pass


def read_document(path: Path):
    import ezdxf
    from ezdxf import recover

    suffix = path.suffix.lower()
    if suffix == ".dwg":
        from ezdxf.addons import odafc

        if not odafc.is_installed():
            raise DrawingError(
                "DWG 파일을 읽으려면 무료 'ODA File Converter'가 필요합니다.\n"
                "https://www.opendesign.com/guestfiles/oda_file_converter 에서 설치하거나,\n"
                "CAD에서 DXF로 저장(다른 이름으로 저장 → DXF)한 파일을 열어주세요."
            )
        try:
            return odafc.readfile(str(path))
        except Exception as exc:  # pragma: no cover - 외부 프로그램 의존
            raise DrawingError(f"DWG 변환 실패: {exc}") from exc
    try:
        return ezdxf.readfile(str(path))
    except ezdxf.DXFStructureError:
        doc, _auditor = recover.readfile(str(path))
        return doc
    except OSError as exc:
        raise DrawingError(f"파일을 열 수 없습니다: {exc}") from exc


def load_drawing(path: str | Path, flatten_tol_m: float = 0.02) -> Drawing:
    path = Path(path)
    doc = read_document(path)
    insunits = int(doc.header.get("$INSUNITS", 0) or 0)
    drawing = Drawing(path=path, insunits=insunits)
    scale = INSUNITS_TO_M.get(insunits)
    msp = doc.modelspace()

    # 단위를 먼저 정해야 곡선 분할 허용오차를 도면 단위로 바꿀 수 있다.
    if scale is None:
        scale = _guess_scale(msp)
        drawing.unit_guessed = True
    drawing.unit_scale = scale
    tol = flatten_tol_m / scale

    for entity in msp:
        _collect(entity, None, drawing, tol, depth=0)
    drawing.layer_counts = {}
    for layer in drawing.layers:
        drawing.layer_counts[layer] = drawing.layer_counts.get(layer, 0) + 1
    return drawing


def _guess_scale(msp) -> float:
    xs: list[float] = []
    for e in msp.query("LINE LWPOLYLINE"):
        try:
            if e.dxftype() == "LINE":
                xs += [abs(e.dxf.start.x - e.dxf.end.x), abs(e.dxf.start.y - e.dxf.end.y)]
            else:
                pts = np.array([p[:2] for p in e.get_points("xy")])
                if len(pts):
                    xs += list(pts.max(axis=0) - pts.min(axis=0))
        except Exception:
            continue
        if len(xs) > 2000:
            break
    if not xs:
        return 0.001
    typical = float(np.median([x for x in xs if x > 0] or [1.0]))
    # 건축/토목 도면에서 선 길이 중앙값이 수백 이상이면 mm 로 본다.
    return 0.001 if typical > 50 else 1.0


def _collect(entity, parent_layer: str | None, drawing: Drawing, tol: float, depth: int) -> None:
    from ezdxf import disassemble

    etype = entity.dxftype()
    layer = entity.dxf.get("layer", "0")
    if layer == "0" and parent_layer:
        layer = parent_layer
    if etype in SKIP_TYPES:
        drawing.skipped[etype] = drawing.skipped.get(etype, 0) + 1
        return
    if etype == "INSERT":
        if depth > 12:
            return
        try:
            children = list(entity.virtual_entities())
        except Exception:
            drawing.skipped["INSERT(오류)"] = drawing.skipped.get("INSERT(오류)", 0) + 1
            return
        for child in children:
            _collect(child, layer, drawing, tol, depth + 1)
        return
    try:
        prim = disassemble.make_primitive(entity, max_flattening_distance=tol)
        if prim.is_empty:
            return
        if prim.path is not None:
            for sub in prim.path.sub_paths():
                pts = np.array([(p.x, p.y) for p in sub.flattening(tol)], dtype=float)
                if len(pts) >= 2:
                    drawing.polylines.append(pts)
                    drawing.layers.append(layer)
            return
        if prim.mesh is not None:
            mesh = prim.mesh
            verts = [(v.x, v.y) for v in mesh.vertices]
            for face in mesh.faces:
                pts = np.array([verts[i] for i in face] + [verts[face[0]]], dtype=float)
                drawing.polylines.append(pts)
                drawing.layers.append(layer)
            return
        pts = np.array([(v.x, v.y) for v in prim.vertices()], dtype=float)
        if len(pts) >= 2:
            drawing.polylines.append(pts)
            drawing.layers.append(layer)
    except Exception:
        drawing.skipped[etype] = drawing.skipped.get(etype, 0) + 1
