"""빠른 DXF 판독기.

ezdxf 로 문서 전체를 메모리에 올리면 큰 도면(수십~수백 MB)에서 시간이 오래 걸리고 메모리도 많이 쓴다.
여기서는 파일을 한 줄씩 흘려 읽으면서 필요한 것(블록 정의, 모델공간 객체)만 간단한 기록으로 남기고,
흔한 객체(LINE, LWPOLYLINE, ARC, CIRCLE, INSERT)는 직접 계산한다.
그 밖의 객체(SPLINE, ELLIPSE, HATCH, 옛 POLYLINE, 두께 있는 폴리선 등)만 ezdxf 로 넘겨 변환한다.

읽을 수 없는 형식(바이너리 DXF 등)이나 예상 못 한 구조를 만나면 FastDxfUnsupported 를 던지고,
호출하는 쪽(drawing.load_drawing)이 기존 ezdxf 방식으로 다시 읽는다.
"""

from __future__ import annotations

import math
from pathlib import Path
import re

import numpy as np

ZERO_LAYER = "0"

# 선으로 그릴 것이 없는 객체 (세지만 변환하지 않음)
NO_GEOMETRY = {
    b"3DSOLID", b"REGION", b"BODY", b"SURFACE", b"EXTRUDEDSURFACE", b"LOFTEDSURFACE", b"REVOLVEDSURFACE",
    b"SWEPTSURFACE", b"PLANESURFACE", b"NURBSSURFACE", b"ACAD_PROXY_ENTITY", b"ACAD_TABLE", b"OLEFRAME",
    b"LIGHT", b"SUN", b"CAMERA", b"GEOPOSITIONMARKER", b"SHAPE", b"ARCALIGNEDTEXT", b"RTEXT",
}


_READ_SECTIONS = (b"", b"BLOCKS", b"ENTITIES")


class FastDxfUnsupported(Exception):
    pass


_U_ESCAPE = re.compile(r"\\U\+([0-9A-Fa-f]{4})")


def _decode(raw: bytes, enc: str) -> str:
    s = raw.strip().decode(enc, errors="replace")
    if "\\U+" in s:
        s = _U_ESCAPE.sub(lambda m: chr(int(m.group(1), 16)), s)
    return s


def _encoding(acadver: str, codepage: str) -> str:
    if acadver >= "AC1021":  # R2007 이후는 UTF-8
        return "utf-8"
    cp = codepage.upper()
    if cp.startswith("ANSI_"):
        return "cp" + cp[5:]
    if cp.startswith("DOS"):
        return "cp" + cp[3:]
    return "cp949"


# ----------------------------------------------------------------------
# 1단계: 파일 → 기록
# ----------------------------------------------------------------------
class Parsed:
    def __init__(self):
        self.blocks: dict[str, tuple[float, float, list]] = {}  # 이름(소문자) → (기준 x, y, 기록 목록)
        self.msp: list = []
        self.layer_ids: dict[bytes, int] = {}
        self.layer_names: list[str] = []
        self.insunits = 0
        self.encoding = "cp949"
        self.skipped: dict[str, int] = {}

    def layer_id(self, raw: bytes) -> int:
        i = self.layer_ids.get(raw)
        if i is None:
            name = _decode(raw, self.encoding)
            # 같은 이름이 다른 바이트 표기로 올 수 있으므로 이름 기준으로 합친다
            try:
                i = self.layer_names.index(name)
            except ValueError:
                i = len(self.layer_names)
                self.layer_names.append(name)
            self.layer_ids[raw] = i
        return i


def parse_file(path: str | Path, skip_types: set[str], progress=None) -> Parsed:
    path = Path(path)
    size = max(path.stat().st_size, 1)
    out = Parsed()
    zero_raw = b"0"
    out.layer_ids[zero_raw] = 0
    out.layer_names.append(ZERO_LAYER)
    skip_b = {t.encode("ascii") for t in skip_types}
    skipped = out.skipped
    layer_id = out.layer_id

    section = b""
    target: list | None = None  # 지금 객체를 넣을 목록 (블록 또는 모델공간)
    cur_poly = None  # 옛 POLYLINE 기록 (뒤따르는 VERTEX 를 모은다)
    block_name = None
    block_recs: list = []
    block_base = (0.0, 0.0)
    done = False
    count = 0

    def text_of(etype: bytes, tags) -> str:
        enc = out.encoding
        parts = ["  0", etype.decode("ascii", "replace")]
        for code, v in tags:
            parts.append(str(code))
            parts.append(v.rstrip(b"\r\n").decode(enc, errors="replace"))
        return "\n".join(parts) + "\n"

    def finish(etype: bytes, tags) -> None:
        nonlocal section, target, cur_poly, block_name, block_recs, block_base, done
        if etype == b"SECTION":
            section = b""
            for code, v in tags:
                if code == 2:
                    section = v.strip()
                    break
            if section == b"HEADER":
                var = b""
                vals: dict[bytes, bytes] = {}
                for code, v in tags:
                    if code == 9:
                        var = v.strip()
                    elif var and var not in vals:
                        vals[var] = v.strip()
                acadver = vals.get(b"$ACADVER", b"AC1015").decode("ascii", "replace")
                codepage = vals.get(b"$DWGCODEPAGE", b"ANSI_949").decode("ascii", "replace")
                out.encoding = _encoding(acadver, codepage)
                try:
                    out.insunits = int(vals.get(b"$INSUNITS", b"0"))
                except ValueError:
                    out.insunits = 0
            elif section == b"ENTITIES":
                target = out.msp
            return
        if etype == b"ENDSEC":
            if section == b"ENTITIES":
                done = True
            section = b""
            target = None
            return
        if section == b"BLOCKS":
            if etype == b"BLOCK":
                name, bx, by = b"", 0.0, 0.0
                for code, v in tags:
                    if code == 2:
                        name = v
                    elif code == 10:
                        bx = float(v)
                    elif code == 20:
                        by = float(v)
                block_name = _decode(name, out.encoding).lower()
                block_base = (bx, by)
                block_recs = []
                target = block_recs
                cur_poly = None
                return
            if etype == b"ENDBLK":
                if block_name is not None:
                    out.blocks[block_name] = (block_base[0], block_base[1], block_recs)
                block_name = None
                target = None
                return
        if target is None:
            return

        # ---------------- 객체 ----------------
        if etype == b"LINE":
            lay = zero_raw
            x1 = y1 = x2 = y2 = 0.0
            for code, v in tags:
                if code == 10:
                    x1 = float(v)
                elif code == 20:
                    y1 = float(v)
                elif code == 11:
                    x2 = float(v)
                elif code == 21:
                    y2 = float(v)
                elif code == 8:
                    lay = v
                elif code == 67 and int(v) == 1:
                    return  # 종이공간
            target.append(("L", layer_id(lay), x1, y1, x2, y2))
            return
        if etype == b"LWPOLYLINE":
            lay = zero_raw
            xs: list[float] = []
            bulges = None
            closed = False
            wide = False
            elev = 0.0
            ext = None
            for code, v in tags:
                if code == 10:
                    xs.append(float(v))
                elif code == 20:
                    xs.append(float(v))
                elif code == 42:
                    b = float(v)
                    if b:
                        if bulges is None:
                            bulges = {}
                        bulges[len(xs) // 2 - 1] = b
                elif code == 8:
                    lay = v
                elif code == 70:
                    closed = bool(int(v) & 1)
                elif code == 43 or code == 40 or code == 41:
                    if float(v):
                        wide = True
                elif code == 38:
                    elev = float(v)
                elif code == 210 or code == 220 or code == 230:
                    if ext is None:
                        ext = [0.0, 0.0, 1.0]
                    ext[(code - 210) // 10] = float(v)
                elif code == 67 and int(v) == 1:
                    return
            if wide:  # 두께 있는 폴리선은 띠 모양 테두리로 읽어야 하므로 ezdxf 로
                target.append(("F", layer_id(lay), text_of(etype, tags), None))
            elif len(xs) >= 4 and len(xs) % 2 == 0:
                if ext is not None and ext[0] == 0.0 and ext[1] == 0.0 and ext[2] == 1.0:
                    ext = None
                target.append(("P", layer_id(lay), xs, closed, bulges, ext, elev))
            return
        if etype == b"ARC" or etype == b"CIRCLE":
            lay = zero_raw
            cx = cy = cz = r = 0.0
            a0, a1 = 0.0, 360.0
            ext = None
            for code, v in tags:
                if code == 10:
                    cx = float(v)
                elif code == 20:
                    cy = float(v)
                elif code == 30:
                    cz = float(v)
                elif code == 40:
                    r = float(v)
                elif code == 50:
                    a0 = float(v)
                elif code == 51:
                    a1 = float(v)
                elif code == 8:
                    lay = v
                elif code == 210 or code == 220 or code == 230:
                    if ext is None:
                        ext = [0.0, 0.0, 1.0]
                    ext[(code - 210) // 10] = float(v)
                elif code == 67 and int(v) == 1:
                    return
            if r > 0:
                if ext is not None and ext[0] == 0.0 and ext[1] == 0.0 and ext[2] == 1.0:
                    ext = None
                full = etype == b"CIRCLE"
                target.append(("A", layer_id(lay), cx, cy, r, 0.0 if full else a0, 360.0 if full else a1,
                               full, ext, cz))
            return
        if etype == b"INSERT":
            lay = zero_raw
            name = b""
            ix = iy = iz = rot = 0.0
            sx = sy = 1.0
            ncol = nrow = 1
            dcol = drow = 0.0
            ext = None
            for code, v in tags:
                if code == 2:
                    name = v
                elif code == 10:
                    ix = float(v)
                elif code == 20:
                    iy = float(v)
                elif code == 30:
                    iz = float(v)
                elif code == 41:
                    sx = float(v)
                elif code == 42:
                    sy = float(v)
                elif code == 50:
                    rot = float(v)
                elif code == 70:
                    ncol = max(int(v), 1)
                elif code == 71:
                    nrow = max(int(v), 1)
                elif code == 44:
                    dcol = float(v)
                elif code == 45:
                    drow = float(v)
                elif code == 8:
                    lay = v
                elif code == 210 or code == 220 or code == 230:
                    if ext is None:
                        ext = [0.0, 0.0, 1.0]
                    ext[(code - 210) // 10] = float(v)
                elif code == 67 and int(v) == 1:
                    return
            if ext is not None and ext[0] == 0.0 and ext[1] == 0.0 and ext[2] == 1.0:
                ext = None
            target.append(("I", layer_id(lay), _decode(name, out.encoding).lower(), ix, iy, iz, sx, sy, rot,
                           ncol, nrow, dcol, drow, ext))
            return
        if etype == b"SPLINE":
            lay = zero_raw
            deg = 3
            knots: list[float] = []
            ctrl: list[float] = []
            weights: list[float] = []
            for code, v in tags:
                if code == 10 or code == 20:
                    ctrl.append(float(v))
                elif code == 40:
                    knots.append(float(v))
                elif code == 41:
                    weights.append(float(v))
                elif code == 71:
                    deg = int(v)
                elif code == 8:
                    lay = v
                elif code == 67 and int(v) == 1:
                    return
            n = len(ctrl) // 2
            if n > deg >= 1 and len(ctrl) % 2 == 0 and len(knots) == n + deg + 1 \
                    and (not weights or len(weights) == n):
                target.append(("S", layer_id(lay), deg, knots, ctrl, weights or None))
            else:  # 맞춤점만 있는 스플라인 등은 ezdxf 로
                target.append(("F", layer_id(lay), text_of(etype, tags), None))
            return
        if etype == b"VERTEX":
            if cur_poly is not None:
                cur_poly.append(text_of(etype, tags))
            return
        if etype == b"SEQEND":
            cur_poly = None
            return
        if etype in skip_b or etype in NO_GEOMETRY:
            key = etype.decode("ascii", "replace")
            skipped[key] = skipped.get(key, 0) + 1
            return
        # 그 밖의 객체: ezdxf 로 변환하도록 원문을 남긴다
        lay = zero_raw
        for code, v in tags:
            if code == 8:
                lay = v
            elif code == 67 and int(v) == 1:
                return
        verts = [] if etype == b"POLYLINE" else None
        target.append(("F", layer_id(lay), text_of(etype, tags), verts))
        if verts is not None:
            cur_poly = verts

    with open(path, "rb") as f:
        head = f.read(32)
        if head.startswith(b"AutoCAD Binary DXF"):
            raise FastDxfUnsupported("바이너리 DXF")
        f.seek(0)
        etype = None
        tags: list | None = None
        try:
            for c, v in zip(f, f):
                code = int(c)
                if code:
                    if tags is not None:
                        tags.append((code, v))
                    continue
                if etype is not None:
                    finish(etype, tags)
                    if done:
                        break
                etype = v.strip()
                # 표·클래스 구역의 내용은 읽지 않는다 (구역 시작/끝만 확인)
                if section in _READ_SECTIONS or etype == b"ENDSEC" or etype == b"SECTION":
                    tags = []
                else:
                    tags = None
                count += 1
                if progress and count % 200000 == 0:
                    progress(f"파일 읽는 중... {100 * f.tell() // size}%")
        except ValueError as exc:
            raise FastDxfUnsupported(f"DXF 구조를 읽을 수 없음: {exc}") from exc
    if not done:
        raise FastDxfUnsupported("ENTITIES 구역을 찾지 못함")
    return out


# ----------------------------------------------------------------------
# 2단계: 기록 → 선
# ----------------------------------------------------------------------
def _ocs_matrix(ext) -> tuple[np.ndarray, np.ndarray]:
    """OCS(객체 좌표계) → 평면 좌표 변환 (행 벡터용 2x2, z 방향의 xy 성분)."""
    from ezdxf.math import OCS, Vec3

    ocs = OCS(Vec3(ext))
    return (np.array([[ocs.ux.x, ocs.ux.y], [ocs.uy.x, ocs.uy.y]]), np.array([ocs.uz.x, ocs.uz.y]))


def _arc_points(cx, cy, r, start_rad, sweep_rad, tol) -> np.ndarray:
    step = min(2.0 * math.acos(max(1.0 - tol / r, -1.0)), math.pi / 4) if r > tol else math.pi / 4
    n = max(int(math.ceil(abs(sweep_rad) / max(step, 1e-3))), 2)
    a = start_rad + sweep_rad * np.linspace(0.0, 1.0, n + 1)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], axis=1)


def _bulge_polyline(pts: np.ndarray, bulges: dict, tol: float) -> np.ndarray:
    out = [pts[:1]]
    for i in range(len(pts) - 1):
        b = bulges.get(i)
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        if b and (dx or dy):
            k = (1.0 - b * b) / (4.0 * b)
            cx, cy = (x0 + x1) / 2 - dy * k, (y0 + y1) / 2 + dx * k
            r = math.hypot(x0 - cx, y0 - cy)
            arc = _arc_points(cx, cy, r, math.atan2(y0 - cy, x0 - cx), 4.0 * math.atan(b), tol)
            arc[-1] = (x1, y1)
            out.append(arc[1:])
        else:
            out.append(pts[i + 1:i + 2])
    return np.concatenate(out)


def _bspline_eval(deg: int, k: np.ndarray, cw: np.ndarray, t: np.ndarray) -> np.ndarray:
    """B-스플라인 값 (The NURBS Book A2.2 를 매개변수 여러 개에 대해 한꺼번에)."""
    n = len(cw)
    span = np.clip(np.searchsorted(k, t, side="right") - 1, deg, n - 1)
    m = len(t)
    basis = np.zeros((m, deg + 1))
    basis[:, 0] = 1.0
    left = np.zeros((m, deg + 1))
    right = np.zeros((m, deg + 1))
    for j in range(1, deg + 1):
        left[:, j] = t - k[span + 1 - j]
        right[:, j] = k[span + j] - t
        saved = np.zeros(m)
        for r in range(j):
            den = right[:, r + 1] + left[:, j - r]
            temp = np.divide(basis[:, r], den, out=np.zeros(m), where=den != 0)
            basis[:, r] = saved + right[:, r + 1] * temp
            saved = left[:, j - r] * temp
        basis[:, j] = saved
    idx = span[:, None] - deg + np.arange(deg + 1)[None, :]
    return np.einsum("mj,mjd->md", basis, cw[idx])


def _spline_points(deg: int, knots, ctrl, weights, tol: float) -> np.ndarray:
    """스플라인 → 폴리라인. 구간마다 휘는 정도를 재서, 선분과 곡선의 차이가 tol 이하가 되도록 나눈다."""
    c = np.array(ctrl, dtype=float).reshape(-1, 2)
    n = len(c)
    if deg == 1:
        return c
    k = np.array(knots, dtype=float)
    if weights is not None:
        w = np.array(weights, dtype=float)
        cw = np.concatenate([c * w[:, None], w[:, None]], axis=1)
    else:
        cw = c

    def ev(t):
        p = _bspline_eval(deg, k, cw, t)
        return p[:, :2] / p[:, 2:3] if weights is not None else p

    u = np.unique(k[deg:n + 1])
    if len(u) < 2:
        return c[:1].repeat(2, axis=0)
    a, b = u[:-1], u[1:]
    n1 = 4
    frac = np.linspace(0.0, 1.0, 2 * n1 + 1)
    pts = ev((a[:, None] + (b - a)[:, None] * frac[None, :]).ravel()).reshape(len(a), 2 * n1 + 1, 2)
    mid = (pts[:, 0:-2:2] + pts[:, 2::2]) / 2  # n1 등분했을 때 각 선분의 중점
    dev = np.hypot(*(pts[:, 1::2] - mid).transpose(2, 0, 1)).max(axis=1)  # 곡선과의 차이(근사)
    need = np.clip(np.ceil(n1 * np.sqrt(dev / tol)), 1, 256).astype(np.int64)
    rep = np.repeat(np.arange(len(a)), need)
    first = np.repeat(np.cumsum(need) - need, need)
    t = a[rep] + (b - a)[rep] * (np.arange(need.sum()) - first) / need[rep]
    return ev(np.append(t, u[-1]))


class Converter:
    def __init__(self, parsed: Parsed, tol: float, fallback, max_depth: int = 12):
        """fallback(text, vertex_texts, tol) → 폴리라인 배열 목록 (ezdxf 로 변환)."""
        self.p = parsed
        self.tol = tol
        self.fallback = fallback
        self.max_depth = max_depth
        self.cache: dict[tuple[str, float], tuple] = {}
        self.empty = (np.zeros((0, 2)), np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int32))

    def records(self, recs: list, tol: float, depth: int, progress=None):
        """기록 목록 → (점 배열, 폴리라인별 점 수, 폴리라인별 레이어 번호)"""
        v_chunks: list[np.ndarray] = []
        n_chunks: list[np.ndarray] = []
        l_chunks: list[np.ndarray] = []

        lines = [r for r in recs if r[0] == "L"]
        if lines:
            v_chunks.append(np.array([r[2:6] for r in lines], dtype=float).reshape(-1, 2))
            n_chunks.append(np.full(len(lines), 2, dtype=np.int64))
            l_chunks.append(np.array([r[1] for r in lines], dtype=np.int32))
        del lines

        s_verts: list[np.ndarray] = []  # 낱개 폴리라인
        s_layers: list[int] = []
        total = len(recs)
        for k, r in enumerate(recs):
            kind = r[0]
            if kind == "L":
                continue
            if progress and k % 20000 == 0 and k:
                progress(f"객체 변환 중... {100 * k // total}%")
            if kind == "P":
                _, lay, xs, closed, bulges, ext, elev = r
                pts = np.array(xs, dtype=float).reshape(-1, 2)
                if closed:
                    pts = np.concatenate([pts, pts[:1]])  # 마지막 점의 bulge 는 닫는 구간에 적용됨
                if bulges:
                    pts = _bulge_polyline(pts, bulges, tol)
                if ext is not None:
                    a, uz = _ocs_matrix(ext)
                    pts = pts @ a + elev * uz
                s_verts.append(pts)
                s_layers.append(lay)
            elif kind == "A":
                _, lay, cx, cy, rad, a0, a1, full, ext, cz = r
                span = 360.0 if full else ((a1 - a0) % 360.0 or 360.0)
                pts = _arc_points(cx, cy, rad, math.radians(a0), math.radians(span), tol)
                if ext is not None:
                    a, uz = _ocs_matrix(ext)
                    pts = pts @ a + cz * uz
                s_verts.append(pts)
                s_layers.append(lay)
            elif kind == "S":
                _, lay, deg, knots, ctrl, weights = r
                s_verts.append(_spline_points(deg, knots, ctrl, weights, tol))
                s_layers.append(lay)
            elif kind == "I":
                got = self.insert(r, tol, depth)
                if got is not None:
                    v_chunks.append(got[0])
                    n_chunks.append(got[1])
                    l_chunks.append(got[2])
            elif kind == "F":
                _, lay, text, vtexts = r
                for pts in self.fallback(text, vtexts, tol):
                    s_verts.append(pts)
                    s_layers.append(lay)
        if s_verts:
            v_chunks.append(np.concatenate(s_verts))
            n_chunks.append(np.array([len(p) for p in s_verts], dtype=np.int64))
            l_chunks.append(np.array(s_layers, dtype=np.int32))
        if not v_chunks:
            return self.empty
        return np.concatenate(v_chunks), np.concatenate(n_chunks), np.concatenate(l_chunks)

    def block(self, name: str, tol: float, depth: int):
        key = (name, tol)
        got = self.cache.get(key)
        if got is not None:
            return got
        self.cache[key] = self.empty  # 자기 참조 방지
        blk = self.p.blocks.get(name)
        if blk is None or depth > self.max_depth:
            return self.empty
        bx, by, recs = blk
        verts, counts, layers = self.records(recs, tol, depth)
        if bx or by:
            verts = verts - (bx, by)
        got = (verts, counts, layers)
        self.cache[key] = got
        return got

    def insert(self, r, tol: float, depth: int):
        _, lay, name, ix, iy, iz, sx, sy, rot, ncol, nrow, dcol, drow, ext = r
        scale = max(abs(sx), abs(sy), 1e-9)
        # 크게 확대해 넣는 블록은 블록 좌표에서 더 촘촘히 나눠야 결과가 매끈하다
        child_tol = 2.0 ** math.floor(math.log2(tol / scale))
        verts, counts, layers = self.block(name, child_tol, depth + 1)
        if not len(counts):
            return None
        c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        m = np.array([[sx * c, sx * s], [-sy * s, sy * c]])  # 배율 → 회전 (행 벡터)
        rot_m = np.array([[c, s], [-s, c]])
        a = uz = None
        if ext is not None:
            a, uz = _ocs_matrix(ext)
        outs = []
        for row in range(nrow):
            for col in range(ncol):
                off = np.array([col * dcol, row * drow]) @ rot_m
                t = np.array([ix, iy]) + off
                pts = verts @ m + t
                if a is not None:
                    pts = pts @ a + iz * uz
                outs.append(pts)
        lay_out = np.where(layers == 0, lay, layers).astype(np.int32)  # 블록 안 0 레이어 → 삽입 레이어
        if len(outs) == 1:
            return outs[0], counts, lay_out
        return np.concatenate(outs), np.tile(counts, len(outs)), np.tile(lay_out, len(outs))


def guess_scale(parsed: Parsed) -> float:
    """단위 정보가 없을 때: 모델공간 선 길이로 mm / m 추정 (기존 방식과 같은 기준)."""
    xs: list[float] = []
    for r in parsed.msp:
        if r[0] == "L":
            xs += [abs(r[2] - r[4]), abs(r[3] - r[5])]
        elif r[0] == "P":
            pts = r[2]
            xs += [max(pts[0::2]) - min(pts[0::2]), max(pts[1::2]) - min(pts[1::2])]
        if len(xs) > 2000:
            break
    if not xs:
        return 0.001
    typical = float(np.median([x for x in xs if x > 0] or [1.0]))
    return 0.001 if typical > 50 else 1.0
