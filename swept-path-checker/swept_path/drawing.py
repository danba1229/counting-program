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

    # 화면 그리기용 색인 (build_index 에서 생성)
    verts: np.ndarray | None = None  # 모든 점을 이어붙인 (M,2)
    offsets: np.ndarray | None = None  # 폴리라인 i 의 점 = verts[offsets[i]:offsets[i+1]]
    bboxes: np.ndarray | None = None  # (N,4) xmin, ymin, xmax, ymax (도면 단위)
    layer_index: np.ndarray | None = None  # (N,) layer_names 위치
    layer_names: list[str] = field(default_factory=list)

    def build_index(self) -> None:
        self.layer_names = sorted(self.layer_counts)
        pos = {name: i for i, name in enumerate(self.layer_names)}
        n = len(self.polylines)
        lengths = np.array([len(p) for p in self.polylines], dtype=np.int64)
        self.offsets = np.concatenate([[0], np.cumsum(lengths)]) if n else np.zeros(1, dtype=np.int64)
        self.verts = np.concatenate(self.polylines) if n else np.zeros((0, 2))
        if n:
            idx = np.repeat(np.arange(n), lengths)
            lo_x = np.full(n, np.inf)
            lo_y = np.full(n, np.inf)
            hi_x = np.full(n, -np.inf)
            hi_y = np.full(n, -np.inf)
            np.minimum.at(lo_x, idx, self.verts[:, 0])
            np.minimum.at(lo_y, idx, self.verts[:, 1])
            np.maximum.at(hi_x, idx, self.verts[:, 0])
            np.maximum.at(hi_y, idx, self.verts[:, 1])
            self.bboxes = np.stack([lo_x, lo_y, hi_x, hi_y], axis=1)
        else:
            self.bboxes = np.zeros((0, 4))
        self.layer_index = np.array([pos[name] for name in self.layers], dtype=np.int32)

    def bounds_m(self) -> tuple[float, float, float, float] | None:
        if not self.polylines:
            return None
        if self.bboxes is None:
            self.build_index()
        lo = self.bboxes[:, :2].min(axis=0) * self.unit_scale
        hi = self.bboxes[:, 2:].max(axis=0) * self.unit_scale
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


def load_drawing(path: str | Path, flatten_tol_m: float = 0.02, progress=None) -> Drawing:
    """도면을 읽어 선(폴리라인) 목록으로 만든다.

    progress: 진행 상황 문자열을 받는 콜백 (백그라운드 스레드에서 호출될 수 있음)
    """
    path = Path(path)
    if progress:
        progress("파일 해석 중...")
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

    collector = _Collector(doc, drawing, flatten_tol_m / scale)
    entities = list(msp)
    n = len(entities)
    for i, entity in enumerate(entities):
        collector.add_top(entity)
        if progress and i % 5000 == 0 and i:
            progress(f"객체 변환 중... {i:,} / {n:,}")
    drawing.layer_counts = {}
    for layer in drawing.layers:
        drawing.layer_counts[layer] = drawing.layer_counts.get(layer, 0) + 1
    drawing.build_index()
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


def _transform(pts: np.ndarray, m: np.ndarray) -> np.ndarray:
    """ezdxf Matrix44(행 벡터 규약) 2D 변환."""
    return pts @ m[:2, :2] + m[3, :2]


def _matrix(insert) -> np.ndarray:
    mx = insert.matrix44()
    return np.array([list(mx.get_row(i)) for i in range(4)], dtype=float)


class _Collector:
    """엔티티 → 폴리라인 변환. 블록은 한 번만 풀어 캐시하고 삽입마다 행렬만 적용한다."""

    MAX_DEPTH = 12

    def __init__(self, doc, drawing: Drawing, tol: float):
        self.doc = doc
        self.d = drawing
        self.tol = tol
        self.block_cache: dict[str, list[tuple[np.ndarray, str]]] = {}

    def skip(self, key: str) -> None:
        self.d.skipped[key] = self.d.skipped.get(key, 0) + 1

    def add_top(self, entity) -> None:
        for pts, layer in self.entity_geometry(entity, depth=0):
            self.d.polylines.append(pts)
            self.d.layers.append(layer)

    def entity_geometry(self, entity, depth: int) -> list[tuple[np.ndarray, str]]:
        """(폴리라인, 레이어) 목록. 레이어가 '0'이면 상위 삽입 레이어를 따른다."""
        etype = entity.dxftype()
        layer = entity.dxf.get("layer", "0")
        if etype in SKIP_TYPES:
            self.skip(etype)
            return []
        if etype == "INSERT":
            return self.insert_geometry(entity, layer, depth)
        return [(pts, layer) for pts in self.primitive(entity)]

    def insert_geometry(self, insert, layer: str, depth: int) -> list[tuple[np.ndarray, str]]:
        if depth > self.MAX_DEPTH:
            return []
        try:
            inserts = list(insert.multi_insert()) if insert.mcount > 1 else [insert]
            name = insert.dxf.name
            geom = self.block_geometry(name, depth + 1)
            out = []
            for ins in inserts:
                m = _matrix(ins)
                for pts, lay in geom:
                    out.append((_transform(pts, m), layer if lay == "0" else lay))
            return out
        except Exception:
            self.skip("INSERT(오류)")
            return []

    def block_geometry(self, name: str, depth: int) -> list[tuple[np.ndarray, str]]:
        cached = self.block_cache.get(name)
        if cached is not None:
            return cached
        self.block_cache[name] = []  # 자기 참조 블록 방지
        block = self.doc.blocks.get(name)
        out: list[tuple[np.ndarray, str]] = []
        if block is not None:
            for entity in block:
                out.extend(self.entity_geometry(entity, depth))
        self.block_cache[name] = out
        return out

    def primitive(self, entity) -> list[np.ndarray]:
        from ezdxf import disassemble

        etype = entity.dxftype()
        try:
            # 빠른 경로: 직선, 호가 없는 폴리선
            if etype == "LINE":
                s, e = entity.dxf.start, entity.dxf.end
                if entity.dxf.extrusion.isclose((0, 0, 1)):
                    return [np.array([[s.x, s.y], [e.x, e.y]], dtype=float)]
            elif etype == "LWPOLYLINE" and not entity.has_arc and entity.dxf.extrusion.isclose((0, 0, 1)):
                pts = np.array(entity.get_points("xy"), dtype=float)
                if len(pts) >= 2:
                    if entity.closed:
                        pts = np.vstack([pts, pts[:1]])
                    return [pts]
                return []
            prim = disassemble.make_primitive(entity, max_flattening_distance=self.tol)
            if prim.is_empty:
                return []
            if prim.path is not None:
                out = []
                for sub in prim.path.sub_paths():
                    pts = np.array([(p.x, p.y) for p in sub.flattening(self.tol)], dtype=float)
                    if len(pts) >= 2:
                        out.append(pts)
                return out
            if prim.mesh is not None:
                mesh = prim.mesh
                verts = [(v.x, v.y) for v in mesh.vertices]
                return [np.array([verts[i] for i in face] + [verts[face[0]]], dtype=float)
                        for face in mesh.faces]
            pts = np.array([(v.x, v.y) for v in prim.vertices()], dtype=float)
            return [pts] if len(pts) >= 2 else []
        except Exception:
            self.skip(etype)
            return []
