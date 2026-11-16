"""DXF/DWG 도면 읽기.

DXF는 ezdxf로 직접 읽는다. DWG는 형식이 공개되어 있지 않아 무료 ODA File Converter가
설치되어 있으면 이를 이용해 자동 변환하여 읽는다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import math
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

_U_ESCAPE = re.compile(r"\\U\+([0-9A-Fa-f]{4})")

# 변환 방식이 바뀌면 올려서 예전 캐시를 무효화한다.
CACHE_VERSION = 5
CACHE_KEEP = 20  # 캐시 파일 최대 개수 (오래된 것부터 삭제)

# 기본으로 장애물에서 제외할 레이어 이름 패턴
NON_OBSTACLE_PATTERN = re.compile(
    r"(DIM|TEXT|TXT|ANNO|HATCH|PAT|CEN|DEFPOINTS|GRID|AXIS|TITLE|FRAME|BORDER|SYMB|MARK|"
    r"치수|문자|글자|해치|중심|그리드|도곽|기호|주기|주차구획)",
    re.IGNORECASE,
)


class _PolylineView:
    """verts/offsets 배열을 폴리라인 목록처럼 보여준다 (수백만 개여도 메모리를 따로 쓰지 않음)."""

    def __init__(self, verts, offsets):
        self._v, self._o = verts, offsets

    def __len__(self):
        return len(self._o) - 1

    def __getitem__(self, i):
        if isinstance(i, slice):
            return [self[j] for j in range(*i.indices(len(self)))]
        if i < 0:
            i += len(self)
        if not 0 <= i < len(self):
            raise IndexError(i)
        return self._v[self._o[i]:self._o[i + 1]]

    def __iter__(self):
        v, o = self._v, self._o.tolist()
        for a, b in zip(o[:-1], o[1:]):
            yield v[a:b]


class _LayerView:
    def __init__(self, index, names):
        self._i, self._n = index, names

    def __len__(self):
        return len(self._i)

    def __getitem__(self, i):
        if isinstance(i, slice):
            return [self._n[j] for j in self._i[i].tolist()]
        return self._n[int(self._i[i])]

    def __iter__(self):
        n = self._n
        return (n[j] for j in self._i.tolist())

    def __eq__(self, other):
        return list(self) == list(other)


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
    # 프로그램 안에서 지운 선 (원본 파일은 그대로). 화면·충돌 검사·선 고르기에서 빠진다.
    # 백그라운드 그리기와 겹쳐도 안전하도록 바꿀 때마다 새 배열로 교체한다.
    removed: np.ndarray | None = None  # (N,) bool

    def build_index(self) -> None:
        """polylines / layers 목록 → 배열 색인."""
        names = sorted(set(self.layers))
        pos = {name: i for i, name in enumerate(names)}
        n = len(self.polylines)
        lengths = np.array([len(p) for p in self.polylines], dtype=np.int64)
        offsets = np.concatenate([[0], np.cumsum(lengths)]) if n else np.zeros(1, dtype=np.int64)
        verts = np.concatenate(list(self.polylines)) if n else np.zeros((0, 2))
        self.set_arrays(verts, offsets, np.array([pos[x] for x in self.layers], dtype=np.int32), names)

    def set_arrays(self, verts: np.ndarray, offsets: np.ndarray, layer_index: np.ndarray,
                   layer_names: list[str]) -> None:
        """배열로 직접 채운다. polylines / layers 는 배열을 들여다보는 가벼운 목록이 된다."""
        self.verts = np.ascontiguousarray(verts, dtype=float)
        self.offsets = np.asarray(offsets, dtype=np.int64)
        self.layer_index = np.asarray(layer_index, dtype=np.int32)
        self.layer_names = list(layer_names)
        n = len(self.layer_index)
        if n:
            starts = self.offsets[:-1]
            self.bboxes = np.stack([
                np.minimum.reduceat(self.verts[:, 0], starts), np.minimum.reduceat(self.verts[:, 1], starts),
                np.maximum.reduceat(self.verts[:, 0], starts), np.maximum.reduceat(self.verts[:, 1], starts),
            ], axis=1)
        else:
            self.bboxes = np.zeros((0, 4))
        counts = np.bincount(self.layer_index, minlength=len(self.layer_names)) if n else []
        self.layer_counts = {name: int(c) for name, c in zip(self.layer_names, counts) if c}
        self.polylines = _PolylineView(self.verts, self.offsets)
        self.layers = _LayerView(self.layer_index, self.layer_names)
        self.removed = np.zeros(n, dtype=bool)

    # ---------------- 프로그램 안에서 선 지우기 ----------------
    def set_removed(self, indices, removed: bool = True) -> np.ndarray:
        """선을 지우거나(removed=True) 되살린다. 실제로 상태가 바뀐 선 번호를 돌려준다."""
        if self.bboxes is None:
            self.build_index()
        idx = np.unique(np.asarray(indices, dtype=np.int64).ravel())
        idx = idx[(idx >= 0) & (idx < len(self.removed))]
        idx = idx[self.removed[idx] != removed]
        if len(idx):
            new = self.removed.copy()
            new[idx] = removed
            self.removed = new
        return idx

    def restore_all(self) -> np.ndarray:
        """지운 선을 모두 되살린다. 되살린 선 번호를 돌려준다."""
        return self.set_removed(np.nonzero(self.removed)[0], False)

    @property
    def removed_count(self) -> int:
        return 0 if self.removed is None else int(np.count_nonzero(self.removed))

    def layer_lines(self, name: str, include_removed: bool = False) -> np.ndarray:
        """레이어의 선 번호."""
        if name not in self.layer_names:
            return np.zeros(0, dtype=np.int64)
        mask = self.layer_index == self.layer_names.index(name)
        if not include_removed:
            mask &= ~self.removed
        return np.nonzero(mask)[0]

    def lines_in_rect(self, x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
        """사각형 [도면 단위] 안에 완전히 들어가는 (지우지 않은) 선 번호."""
        if self.bboxes is None or not len(self.bboxes):
            return np.zeros(0, dtype=np.int64)
        x0, x1 = min(x0, x1), max(x0, x1)
        y0, y1 = min(y0, y1), max(y0, y1)
        bb = self.bboxes
        mask = (bb[:, 0] >= x0) & (bb[:, 2] <= x1) & (bb[:, 1] >= y0) & (bb[:, 3] <= y1) & ~self.removed
        return np.nonzero(mask)[0]

    def active_layer_counts(self) -> dict[str, int]:
        """지우지 않은 선 기준 레이어별 선 개수 (모두 지운 레이어는 0)."""
        if self.layer_index is None or not len(self.layer_index):
            return {name: 0 for name in self.layer_counts}
        counts = np.bincount(self.layer_index[~self.removed], minlength=len(self.layer_names))
        return {name: int(counts[i]) for i, name in enumerate(self.layer_names) if name in self.layer_counts}

    def removed_ranges(self) -> list[list[int]]:
        """지운 선 번호를 [시작, 끝(포함)] 구간 목록으로 (작업 파일 저장용)."""
        idx = np.nonzero(self.removed)[0] if self.removed is not None else np.zeros(0, dtype=np.int64)
        if not len(idx):
            return []
        breaks = np.nonzero(np.diff(idx) > 1)[0]
        starts = np.concatenate([[idx[0]], idx[breaks + 1]])
        ends = np.concatenate([idx[breaks], [idx[-1]]])
        return [[int(a), int(b)] for a, b in zip(starts.tolist(), ends.tolist())]

    def apply_removed_ranges(self, ranges) -> int:
        """removed_ranges() 로 저장한 구간을 적용한다. 지운 선 개수를 돌려준다."""
        if self.bboxes is None:
            self.build_index()
        new = np.zeros(len(self.layer_index), dtype=bool)
        for a, b in ranges:
            a, b = max(int(a), 0), min(int(b), len(new) - 1)
            if a <= b:
                new[a:b + 1] = True
        self.removed = new
        return int(np.count_nonzero(new))

    def bounds_m(self) -> tuple[float, float, float, float] | None:
        if not self.polylines:
            return None
        if self.bboxes is None:
            self.build_index()
        bb = self.bboxes[~self.removed] if self.removed is not None and self.removed.any() else self.bboxes
        if not len(bb):
            return None
        lo = bb[:, :2].min(axis=0) * self.unit_scale
        hi = bb[:, 2:].max(axis=0) * self.unit_scale
        return float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])

    def nearest_polyline(self, x: float, y: float, tol: float) -> int | None:
        """(x, y) [도면 단위] 에서 tol 이내로 가장 가까운 폴리라인 번호. 없으면 None."""
        if self.bboxes is None or not len(self.bboxes):
            return None
        bb = self.bboxes
        cand = np.nonzero((bb[:, 0] - tol <= x) & (bb[:, 2] + tol >= x) &
                          (bb[:, 1] - tol <= y) & (bb[:, 3] + tol >= y) & ~self.removed)[0]
        if not len(cand):
            return None
        best, best_d = None, tol
        p = np.array([x, y])
        for i in cand[:5000]:
            pts = self.verts[self.offsets[i]:self.offsets[i + 1]]
            a, b = pts[:-1], pts[1:]
            ab = b - a
            L2 = np.einsum("ij,ij->i", ab, ab)
            t = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.where(L2 > 0, L2, 1), 0, 1)
            d = np.hypot(*(a + ab * t[:, None] - p).T).min() if len(a) else np.hypot(*(pts[0] - p))
            if d <= best_d:
                best, best_d = int(i), float(d)
        return best

    def default_obstacle_layers(self) -> set[str]:
        return {name for name in self.layer_counts if not NON_OBSTACLE_PATTERN.search(name)}

    def polylines_m(self, layers: set[str] | None = None,
                    bbox_m: tuple[float, float, float, float] | None = None) -> list[np.ndarray]:
        """m 단위 폴리라인. layers: 이 레이어만 / bbox_m: 이 범위(xmin, ymin, xmax, ymax)에 걸친 것만."""
        if self.bboxes is None:
            self.build_index()
        mask = ~self.removed
        if layers is not None:
            flags = np.array([name in layers for name in self.layer_names], dtype=bool)
            mask &= flags[self.layer_index] if len(flags) else False
        if bbox_m is not None:
            k = self.unit_scale
            x0, y0, x1, y1 = (v / k for v in bbox_m)
            bb = self.bboxes
            mask &= (bb[:, 2] >= x0) & (bb[:, 0] <= x1) & (bb[:, 3] >= y0) & (bb[:, 1] <= y1)
        o, v, k = self.offsets, self.verts, self.unit_scale
        return [v[o[i]:o[i + 1]] * k for i in np.nonzero(mask)[0].tolist()]


class DrawingError(Exception):
    pass


def read_document(path: Path):
    import ezdxf
    from ezdxf import recover

    suffix = path.suffix.lower()
    if suffix == ".dwg":
        from ezdxf.addons import odafc
        from . import gstarcad

        if gstarcad.is_installed():
            import shutil

            try:
                dxf = gstarcad.convert_to_dxf(path)
            except gstarcad.GstarCadError as exc:
                raise DrawingError(str(exc)) from exc
            try:
                return read_document(dxf)
            finally:
                shutil.rmtree(dxf.parent, ignore_errors=True)
        if not odafc.is_installed():
            raise DrawingError(
                "DWG 파일을 읽으려면 GstarCAD 또는 무료 'ODA File Converter'가 필요합니다.\n"
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
                 use_cache: bool = True, cache_dir: Path | None = None, engine: str = "auto") -> Drawing:
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
    drawing = None
    if path.suffix.lower() == ".dwg":
        drawing = _load_dwg_via_gstarcad(path, flatten_tol_m, progress, engine)
    if drawing is None and engine in ("auto", "fast") and path.suffix.lower() == ".dxf":
        from .fastdxf import FastDxfUnsupported

        try:
            drawing = _load_fast(path, flatten_tol_m, progress)
        except FastDxfUnsupported:
            if engine == "fast":
                raise
        except OSError as exc:
            raise DrawingError(f"파일을 열 수 없습니다: {exc}") from exc
        except Exception:
            if engine == "fast":
                raise
            drawing = None  # 예상 못 한 구조 → 기존 방식으로 다시 읽는다
    if drawing is None:
        drawing = _load_ezdxf(path, flatten_tol_m, progress)
    if cache_file is not None:
        if progress:
            progress("변환 결과 저장 중 (다음에는 빠르게 열림)...")
        _write_cache(cache_file, drawing)
    return drawing


def _load_dwg_via_gstarcad(path: Path, flatten_tol_m: float, progress, engine: str) -> Drawing | None:
    """GstarCAD 가 있으면 DWG 를 임시 DXF 로 바꿔 읽는다. 없으면 None (ODA 경로로 넘어감)."""
    from . import gstarcad

    if not gstarcad.is_installed():
        return None
    import shutil

    if progress:
        progress("GstarCAD 로 DWG → DXF 변환 중... (처음 한 번, 10~60초)")
    try:
        dxf = gstarcad.convert_to_dxf(path)
    except gstarcad.GstarCadError as exc:
        raise DrawingError(
            f"{exc}\nGstarCAD 에서 이 도면을 DXF 로 저장(다른 이름으로 저장 → DXF)한 뒤 열어 보세요."
        ) from exc
    try:
        drawing = None
        if engine in ("auto", "fast"):
            from .fastdxf import FastDxfUnsupported

            try:
                drawing = _load_fast(dxf, flatten_tol_m, progress)
            except FastDxfUnsupported:
                if engine == "fast":
                    raise
            except Exception:
                if engine == "fast":
                    raise
                drawing = None
        if drawing is None:
            drawing = _load_ezdxf(dxf, flatten_tol_m, progress)
        drawing.path = path  # 화면·작업 파일에는 원래 DWG 경로를 남긴다
        return drawing
    finally:
        shutil.rmtree(dxf.parent, ignore_errors=True)


def _load_fast(path: Path, flatten_tol_m: float, progress) -> Drawing:
    """빠른 판독기 (fastdxf): 흔한 객체는 직접 계산, 나머지만 ezdxf 로 변환."""
    from . import fastdxf

    if progress:
        progress("파일 읽는 중...")
    parsed = fastdxf.parse_file(path, SKIP_TYPES, progress)
    drawing = Drawing(path=path, insunits=parsed.insunits, skipped=parsed.skipped)
    scale = INSUNITS_TO_M.get(parsed.insunits)
    if scale is None:
        scale = fastdxf.guess_scale(parsed)
        drawing.unit_guessed = True
    drawing.unit_scale = scale
    helper = _Collector(None, drawing, flatten_tol_m / scale)

    def fallback(text, vertex_texts, tol):
        from ezdxf.entities import factory
        from ezdxf.lldxf.extendedtags import ExtendedTags

        try:
            entity = factory.load(ExtendedTags.from_text(text))
            if vertex_texts:
                for vt in vertex_texts:
                    entity.vertices.append(factory.load(ExtendedTags.from_text(vt)))
        except Exception:
            helper.skip("읽기 오류")
            return []
        if entity.dxftype() == "POLYLINE" and not vertex_texts:
            return []
        return helper.primitive(entity, tol)

    conv = fastdxf.Converter(parsed, flatten_tol_m / scale, fallback)
    verts, counts, layers = conv.records(parsed.msp, conv.tol, 0, progress)
    # 쓰이는 레이어만, 이름순으로 다시 번호를 매긴다
    used = np.unique(layers)
    names = [parsed.layer_names[i] for i in used.tolist()]
    order = np.argsort(np.array(names, dtype=object), kind="stable") if names else np.zeros(0, dtype=int)
    remap = np.zeros(len(parsed.layer_names), dtype=np.int32)
    remap[used[order]] = np.arange(len(order), dtype=np.int32)
    offsets = np.concatenate([[0], np.cumsum(counts)]).astype(np.int64)
    drawing.set_arrays(verts, offsets, remap[layers] if len(layers) else layers, [names[i] for i in order.tolist()])
    return drawing


def _load_ezdxf(path: Path, flatten_tol_m: float, progress) -> Drawing:
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
    drawing.build_index()
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
            verts, offsets, layer_index = z["verts"], z["offsets"], z["layer_index"]
        d.set_arrays(verts, offsets, layer_index, meta["layer_names"])
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
        self.block_cache: dict[tuple[str, float], list[tuple[np.ndarray, str]]] = {}

    def skip(self, key: str) -> None:
        self.d.skipped[key] = self.d.skipped.get(key, 0) + 1

    def add_top(self, entity) -> None:
        for pts, layer in self.entity_geometry(entity, depth=0, tol=self.tol):
            self.d.polylines.append(pts)
            self.d.layers.append(layer)

    def entity_geometry(self, entity, depth: int, tol: float) -> list[tuple[np.ndarray, str]]:
        """(폴리라인, 레이어) 목록. 레이어가 '0'이면 상위 삽입 레이어를 따른다.

        tol: 이 좌표계에서의 곡선 분할 허용오차 (블록 안이면 삽입 배율만큼 작아진다)
        """
        etype = entity.dxftype()
        layer = entity.dxf.get("layer", "0")
        if "\\U+" in layer:  # 옛 형식 DXF 의 유니코드 표기 (\\U+BCBD → 벽)
            layer = _U_ESCAPE.sub(lambda m: chr(int(m.group(1), 16)), layer)
        if etype in SKIP_TYPES:
            self.skip(etype)
            return []
        if etype == "INSERT":
            return self.insert_geometry(entity, layer, depth, tol)
        return [(pts, layer) for pts in self.primitive(entity, tol)]

    def insert_geometry(self, insert, layer: str, depth: int, tol: float) -> list[tuple[np.ndarray, str]]:
        if depth > self.MAX_DEPTH:
            return []
        try:
            inserts = list(insert.multi_insert()) if insert.mcount > 1 else [insert]
            name = insert.dxf.name
            # 블록을 크게 확대해 넣으면 블록 좌표에서는 더 촘촘히 나눠야 결과가 매끈하다.
            scale = max(abs(insert.dxf.get("xscale", 1.0)), abs(insert.dxf.get("yscale", 1.0)), 1e-9)
            child_tol = 2.0 ** math.floor(math.log2(tol / scale))
            geom = self.block_geometry(name, depth + 1, child_tol)
            out = []
            for ins in inserts:
                m = _matrix(ins)
                for pts, lay in geom:
                    out.append((_transform(pts, m), layer if lay == "0" else lay))
            return out
        except Exception:
            self.skip("INSERT(오류)")
            return []

    def block_geometry(self, name: str, depth: int, tol: float) -> list[tuple[np.ndarray, str]]:
        key = (name, tol)
        cached = self.block_cache.get(key)
        if cached is not None:
            return cached
        self.block_cache[key] = []  # 자기 참조 블록 방지
        block = self.doc.blocks.get(name)
        out: list[tuple[np.ndarray, str]] = []
        if block is not None:
            for entity in block:
                out.extend(self.entity_geometry(entity, depth, tol))
        self.block_cache[key] = out
        return out

    def primitive(self, entity, tol: float) -> list[np.ndarray]:
        from ezdxf import disassemble

        etype = entity.dxftype()
        try:
            # 빠른 경로: 직선, 호가 없는 폴리선
            if etype == "LINE":
                s, e = entity.dxf.start, entity.dxf.end
                if entity.dxf.extrusion.isclose((0, 0, 1)):
                    return [np.array([[s.x, s.y], [e.x, e.y]], dtype=float)]
            elif (etype == "LWPOLYLINE" and not entity.has_arc and not entity.has_width
                  and entity.dxf.extrusion.isclose((0, 0, 1))):
                # 두께(폭)가 있는 폴리선은 띠 모양 테두리로 읽어야 하므로 아래 일반 경로로 보낸다.
                pts = np.array(entity.get_points("xy"), dtype=float)
                if len(pts) >= 2:
                    if entity.closed:
                        pts = np.vstack([pts, pts[:1]])
                    return [pts]
                return []
            prim = disassemble.make_primitive(entity, max_flattening_distance=tol)
            if prim.is_empty:
                return []
            if prim.path is not None:
                out = []
                for sub in prim.path.sub_paths():
                    pts = np.array([(p.x, p.y) for p in sub.flattening(tol)], dtype=float)
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
            if etype in ("HATCH", "MPOLYGON"):
                pts_list = _hatch_boundaries(entity, tol)
                if pts_list:
                    return pts_list
            self.skip(etype)
            return []


def _hatch_boundaries(hatch, tol: float) -> list[np.ndarray]:
    """해치 경계를 직접 읽는다. 반지름 0인 호처럼 깨진 조각은 건너뛰고 나머지는 살린다."""
    import math

    from ezdxf.math import Vec3

    out: list[np.ndarray] = []
    try:
        ocs = hatch.ocs()
        elev = hatch.dxf.get("elevation", Vec3()).z
    except Exception:
        return out

    def to_wcs(pts):
        return [(v.x, v.y) for v in (ocs.to_wcs(Vec3(x, y, elev)) for x, y in pts)]

    def arc_pts(cx, cy, r, a0, a1, ccw=True):
        """해치 원호 조각 → 점 목록. 시계방향(ccw=False) 조각은 DXF 규약대로 각도 부호를 뒤집는다."""
        if r <= 0:
            return []
        if ccw:
            start, span = a0, (a1 - a0) % 360 or 360
        else:
            start, span = -a0, -((a1 - a0) % 360 or 360)
        step = 2.0 * math.degrees(math.acos(max(1 - tol / r, -1.0))) if r > tol else 90.0
        n = max(int(math.ceil(abs(span) / max(step, 1.0))), 2)
        return [(cx + r * math.cos(math.radians(start + span * i / n)),
                 cy + r * math.sin(math.radians(start + span * i / n))) for i in range(n + 1)]

    for path in hatch.paths:
        pts: list[tuple[float, float]] = []
        try:
            if path.type.name.startswith("Polyline") or hasattr(path, "vertices"):
                from ezdxf.math import bulge_to_arc

                verts = [(v[0], v[1], v[2] if len(v) > 2 else 0.0) for v in path.vertices]
                if getattr(path, "is_closed", False) and verts:
                    verts.append(verts[0])
                for (x0, y0, bulge), (x1, y1, _b) in zip(verts, verts[1:]):
                    if not pts:
                        pts.append((x0, y0))
                    if bulge and math.hypot(x1 - x0, y1 - y0) > 1e-9:
                        try:
                            c, a0, a1, r = bulge_to_arc((x0, y0), (x1, y1), bulge)
                            arc = arc_pts(c[0], c[1], r, math.degrees(a0), math.degrees(a1), True)
                            if arc and math.hypot(arc[0][0] - x0, arc[0][1] - y0) > r * 1e-3:
                                arc = arc[::-1]  # 시계방향 bulge: 끝점에서 시작
                            pts.extend(arc[1:] or [(x1, y1)])
                            continue
                        except Exception:
                            pass  # 깨진 곡선 구간은 직선으로
                    pts.append((x1, y1))
            else:
                for edge in path.edges:
                    name = type(edge).__name__
                    try:
                        if name == "LineEdge":
                            seg = [(edge.start[0], edge.start[1]), (edge.end[0], edge.end[1])]
                        elif name == "ArcEdge":
                            seg = arc_pts(edge.center[0], edge.center[1], edge.radius, edge.start_angle,
                                          edge.end_angle, edge.ccw)
                        elif name == "EllipseEdge":
                            from ezdxf.math import ConstructionEllipse

                            e = ConstructionEllipse(
                                center=Vec3(edge.center[0], edge.center[1]),
                                major_axis=Vec3(edge.major_axis[0], edge.major_axis[1]),
                                ratio=edge.ratio, start_param=math.radians(edge.start_angle),
                                end_param=math.radians(edge.end_angle), ccw=edge.ccw)
                            seg = [(v.x, v.y) for v in e.flattening(tol)]
                        elif name == "SplineEdge":
                            ctrl = edge.fit_points or edge.control_points
                            seg = [(v[0], v[1]) for v in ctrl]
                        else:
                            seg = []
                    except Exception:
                        seg = []  # 깨진 조각만 건너뜀
                    pts.extend(seg)
        except Exception:
            continue
        if len(pts) >= 2:
            out.append(np.array(to_wcs(pts), dtype=float))
    return out
