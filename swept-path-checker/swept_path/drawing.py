"""DXF/DWG 도면 읽기.

DXF는 ezdxf로 직접 읽는다. DWG는 형식이 공개되어 있지 않아 무료 ODA File Converter가
설치되어 있으면 이를 이용해 자동 변환하여 읽는다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import os
from pathlib import Path
import re
import sys

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

# 변환 방식이 바뀌면 올려서 예전 캐시를 무효화한다.
CACHE_VERSION = 1
CACHE_KEEP = 20  # 캐시 파일 최대 개수 (오래된 것부터 삭제)

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
        # 개별 폴리라인은 verts 의 뷰로 바꿔 메모리 중복을 없앤다.
        self.polylines = np.split(self.verts, self.offsets[1:-1]) if n else []

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


def load_drawing(path: str | Path, flatten_tol_m: float = 0.02, progress=None,
                 use_cache: bool = True, cache_dir: Path | None = None) -> Drawing:
    """도면을 읽어 선(폴리라인) 목록으로 만든다.

    progress: 진행 상황 문자열을 받는 콜백 (백그라운드 스레드에서 호출될 수 있음)
    use_cache: 한 번 변환한 도면을 캐시 파일로 저장해 다음에는 바로 연다.
               (파일 크기·수정 시각이 바뀌면 다시 변환)
    """
    path = Path(path)
    cache_file = None
    if use_cache:
        cache_file = _cache_path(path, flatten_tol_m, cache_dir)
        if cache_file is not None and cache_file.exists():
            if progress:
                progress("저장된 변환 결과(캐시) 불러오는 중...")
            cached = _read_cache(cache_file, path)
            if cached is not None:
                return cached
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
    if cache_file is not None:
        if progress:
            progress("변환 결과 저장 중 (다음에는 빠르게 열림)...")
        _write_cache(cache_file, drawing)
    return drawing


# ----------------------------------------------------------------------
# 변환 결과 캐시
# ----------------------------------------------------------------------
def default_cache_dir() -> Path:
    if sys.platform == "win32":
        base = Path(os.environ.get("LOCALAPPDATA") or Path.home() / "AppData" / "Local")
    else:
        base = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return base / "SweptPathChecker" / "drawings"


def _cache_path(path: Path, tol: float, cache_dir: Path | None) -> Path | None:
    try:
        st = path.stat()
    except OSError:
        return None
    key = f"{path.resolve()}|{st.st_size}|{st.st_mtime_ns}|{tol}|{CACHE_VERSION}"
    digest = hashlib.sha1(key.encode("utf-8")).hexdigest()[:20]
    return (cache_dir or default_cache_dir()) / f"{digest}.npz"


def _write_cache(cache_file: Path, d: Drawing) -> None:
    try:
        cache_file.parent.mkdir(parents=True, exist_ok=True)
        meta = {
            "path": str(d.path), "insunits": d.insunits, "unit_scale": d.unit_scale,
            "unit_guessed": d.unit_guessed, "skipped": d.skipped, "layer_names": d.layer_names,
            "layer_counts": d.layer_counts, "version": CACHE_VERSION,
        }
        tmp = cache_file.with_suffix(".tmp.npz")
        np.savez(tmp, verts=d.verts, offsets=d.offsets, layer_index=d.layer_index,
                 meta=np.frombuffer(json.dumps(meta, ensure_ascii=False).encode("utf-8"), dtype=np.uint8))
        os.replace(tmp, cache_file)
        files = sorted(cache_file.parent.glob("*.npz"), key=lambda f: f.stat().st_mtime, reverse=True)
        for old in files[CACHE_KEEP:]:
            old.unlink(missing_ok=True)
    except OSError:
        pass  # 캐시는 있으면 좋은 것일 뿐, 실패해도 도면 열기에는 지장 없음


def cached_drawing(path: str | Path, flatten_tol_m: float = 0.02,
                   cache_dir: Path | None = None) -> Drawing | None:
    """캐시가 있으면 바로 읽어 반환, 없으면 None."""
    path = Path(path)
    cache_file = _cache_path(path, flatten_tol_m, cache_dir)
    if cache_file is None or not cache_file.exists():
        return None
    return _read_cache(cache_file, path)


def subprocess_load_worker(path: str, queue, cache_dir: str | None = None) -> None:
    """별도 프로세스에서 도면을 변환해 캐시 파일(npz)로 저장하고 그 경로를 돌려준다.

    해석 중 GIL 을 오래 잡는 작업이 있어도 화면(메인 프로세스)이 멈추지 않는다.
    """
    import tempfile

    try:
        cdir = Path(cache_dir) if cache_dir else None
        d = load_drawing(path, progress=lambda m: queue.put(("progress", m)), cache_dir=cdir)
        cache_file = _cache_path(Path(path), 0.02, cdir)
        if cache_file is None or not cache_file.exists():
            # 캐시 폴더에 쓸 수 없으면 임시 파일로 전달
            cache_file = Path(tempfile.mkdtemp()) / "drawing.npz"
            _write_cache(cache_file, d)
            queue.put(("ok", (str(cache_file), True)))
        else:
            queue.put(("ok", (str(cache_file), False)))
    except DrawingError as exc:
        queue.put(("error", ("DrawingError", str(exc))))
    except Exception as exc:  # pragma: no cover
        queue.put(("error", (type(exc).__name__, str(exc))))


def read_npz(cache_file: str | Path, path: str | Path) -> Drawing | None:
    return _read_cache(Path(cache_file), Path(path))


def _read_cache(cache_file: Path, path: Path) -> Drawing | None:
    try:
        with np.load(cache_file) as z:
            meta = json.loads(bytes(z["meta"]).decode("utf-8"))
            if meta.get("version") != CACHE_VERSION:
                return None
            d = Drawing(path=path, insunits=meta["insunits"], unit_scale=meta["unit_scale"],
                        unit_guessed=meta["unit_guessed"], skipped=meta["skipped"])
            d.verts = z["verts"]
            d.offsets = z["offsets"]
            d.layer_index = z["layer_index"]
        d.layer_names = meta["layer_names"]
        d.layer_counts = meta["layer_counts"]
        d.layers = [d.layer_names[i] for i in d.layer_index.tolist()]
        n = len(d.layer_index)
        lengths = np.diff(d.offsets)
        idx = np.repeat(np.arange(n), lengths)
        bb = np.empty((n, 4))
        for col, (fn, axis) in enumerate(((np.minimum, 0), (np.minimum, 1), (np.maximum, 0), (np.maximum, 1))):
            bb[:, col] = np.inf if fn is np.minimum else -np.inf
            fn.at(bb[:, col], idx, d.verts[:, axis])
        d.bboxes = bb
        d.polylines = np.split(d.verts, d.offsets[1:-1]) if n else []
        try:
            os.utime(cache_file)  # 최근 사용 표시 (오래된 캐시부터 지움)
        except OSError:
            pass
        return d
    except Exception:
        return None


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
