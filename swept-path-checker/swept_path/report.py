"""궤적 그림이 들어간 검토 보고서 (PDF / PNG).

1쪽: 판정 + 전체 궤적 그림,  2쪽: 차량 제원·검토 조건·구간별 결과,  3쪽~: 충돌 위치 확대도
"""

from __future__ import annotations

from datetime import datetime
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .analysis import Analysis
from .render import DrawingRenderer
from .simulate import FORWARD, Scenario

PAGE = (2339, 1654)  # A4 가로, 200 dpi
DPI = 200
MARGIN = 70

C_TEXT = (31, 41, 55)
C_MUTED = (107, 114, 128)
C_BORDER = (209, 213, 219)
C_OK = (21, 128, 61)
C_NG = (220, 38, 38)
C_WARN = (180, 83, 9)
C_ENV = (31, 95, 255)
C_CLEAR = (56, 170, 235)
C_WHEEL = (0, 150, 60)
C_SNAP = (120, 120, 120)
C_FWD = (208, 0, 208)
C_REV = (255, 122, 0)

_FONT_CANDIDATES = [
    ("malgun.ttf", "malgunbd.ttf"),
    ("C:/Windows/Fonts/malgun.ttf", "C:/Windows/Fonts/malgunbd.ttf"),
    ("/System/Library/Fonts/AppleSDGothicNeo.ttc", "/System/Library/Fonts/AppleSDGothicNeo.ttc"),
    ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"),
    ("/usr/share/fonts/truetype/nanum/NanumGothic.ttf", "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf"),
    ("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"),
    ("/usr/share/fonts/opentype/unifont/unifont.otf", "/usr/share/fonts/opentype/unifont/unifont.otf"),
]
_font_files: tuple[str, str] | None = None
_font_cache: dict = {}


def font(size: int, bold: bool = False):
    """한글이 나오는 글꼴 (Windows: 맑은 고딕)."""
    global _font_files
    key = (size, bold)
    if key in _font_cache:
        return _font_cache[key]
    f = None
    if _font_files is None:
        for regular, heavy in _FONT_CANDIDATES:
            try:
                ImageFont.truetype(regular, 12)
            except OSError:
                continue
            try:
                ImageFont.truetype(heavy, 12)
            except OSError:
                heavy = regular
            _font_files = (regular, heavy)
            break
        else:
            _font_files = ("", "")
    if _font_files[0]:
        f = ImageFont.truetype(_font_files[1 if bold else 0], size)
    else:  # 한글 글꼴이 없는 환경 (글자가 깨질 수 있음)
        f = ImageFont.load_default(size)
    _font_cache[key] = f
    return f


def _geoms(geom):
    if geom is None or geom.is_empty:
        return []
    return list(getattr(geom, "geoms", [geom]))


def _nice_length(meters: float) -> float:
    base = 10 ** math.floor(math.log10(max(meters, 1e-6)))
    for m in (1, 2, 5, 10):
        if base * m >= meters * 0.6:
            return base * m
    return base * 10


def render_figure(analysis: Analysis, scenario: Scenario, size: tuple[int, int], drawing=None,
                  obstacle_layers: set[str] | None = None, bounds: tuple[float, float, float, float] | None = None,
                  line_scale: float = 1.0, labels: bool = True) -> Image.Image:
    """도면 + 궤적 그림. bounds (m): 그릴 범위, 없으면 궤적 전체에 여유를 둔다."""
    w, h = size
    if bounds is None:
        b = analysis.clearance_envelope.bounds if not analysis.clearance_envelope.is_empty else None
        xs = [scenario.start_x] + [p.x for p in scenario.waypoints]
        ys = [scenario.start_y] + [p.y for p in scenario.waypoints]
        x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
        if b:
            x0, y0, x1, y1 = min(x0, b[0]), min(y0, b[1]), max(x1, b[2]), max(y1, b[3])
        pad = max(0.12 * max(x1 - x0, y1 - y0), 3.0)
        bounds = (x0 - pad, y0 - pad, x1 + pad, y1 + pad)
    x0, y0, x1, y1 = bounds
    s = min(w / max(x1 - x0, 1e-6), h / max(y1 - y0, 1e-6))
    tx = w / 2 - (x0 + x1) / 2 * s
    ty = h / 2 + (y0 + y1) / 2 * s

    def T(p):
        return (p[0] * s + tx, -p[1] * s + ty)

    def pts(arr):
        a = np.asarray(arr, dtype=float)
        return list(zip((a[:, 0] * s + tx).tolist(), (-a[:, 1] * s + ty).tolist()))

    if drawing is not None:
        img = DrawingRenderer(drawing).render(w, h, s, tx, ty, obstacle_layers or set())
    else:
        img = Image.new("RGB", (w, h), (255, 255, 255))
    lw = max(int(round(2 * line_scale)), 1)

    # 차체 궤적 채움 (반투명)
    overlay = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    for g in _geoms(analysis.envelope):
        od.polygon(pts(g.exterior.coords), fill=(31, 95, 255, 46))
        for hole in g.interiors:
            od.polygon(pts(hole.coords), fill=(0, 0, 0, 0))
    img = Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB")
    d = ImageDraw.Draw(img)

    for g in _geoms(analysis.clearance_envelope):
        d.line(pts(g.exterior.coords), fill=C_CLEAR, width=max(lw // 2, 1))
    for snap in analysis.snapshots:
        for poly in snap.values():
            p = pts(poly)
            d.line(p + p[:1], fill=C_SNAP, width=1)
    for name, track in analysis.tracks.items():
        if "wheel" in name and len(track) >= 2:
            step = max(1, len(track) // 4000)
            d.line(pts(track[::step]), fill=C_WHEEL, width=max(lw // 2, 1))
    for g in _geoms(analysis.envelope):
        d.line(pts(g.exterior.coords), fill=C_ENV, width=lw + 1)

    # 지정 경로와 점
    f_lbl = font(int(22 * line_scale), True)
    prev = (scenario.start_x, scenario.start_y)
    for i, wp in enumerate(scenario.waypoints):
        color = C_FWD if wp.gear == FORWARD else C_REV
        d.line([T(prev), T((wp.x, wp.y))], fill=color, width=lw)
        prev = (wp.x, wp.y)
    r = 6 * line_scale
    sx, sy = T((scenario.start_x, scenario.start_y))
    d.ellipse([sx - r, sy - r, sx + r, sy + r], fill=(0, 112, 255))
    if labels:
        d.text((sx + 10, sy - 30 * line_scale), "출발", fill=(0, 112, 255), font=f_lbl)
    for i, wp in enumerate(scenario.waypoints):
        color = C_FWD if wp.gear == FORWARD else C_REV
        px, py = T((wp.x, wp.y))
        d.ellipse([px - r, py - r, px + r, py + r], fill=color)
        if labels:
            last = i == len(scenario.waypoints) - 1
            d.text((px + 10, py - 30 * line_scale), "도착" if last else str(i + 1), fill=color, font=f_lbl)

    for i, c in enumerate(analysis.collisions, 1):
        cx, cy = T((c.x, c.y))
        rr = 16 * line_scale
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=C_NG, width=lw + 2)
        if labels:
            d.text((cx + rr + 4, cy - rr), f"충돌 {i}", fill=C_NG, font=f_lbl)

    # 축척 막대, 북쪽(위) 표시
    length = _nice_length(0.18 * w / s)
    bx, by = 30, h - 40
    d.line([(bx, by), (bx + length * s, by)], fill=C_TEXT, width=4)
    for x in (bx, bx + length * s):
        d.line([(x, by - 8), (x, by + 8)], fill=C_TEXT, width=3)
    d.text((bx, by - 36), f"{length:g} m", fill=C_TEXT, font=font(22))
    return img


def _legend(d: ImageDraw.ImageDraw, x: int, y: int) -> None:
    items = [("차체 궤적", C_ENV, 5), ("여유폭 외곽", C_CLEAR, 2), ("바퀴 궤적", C_WHEEL, 2),
             ("경로(전진)", C_FWD, 3), ("경로(후진)", C_REV, 3), ("충돌", C_NG, 0)]
    f = font(22)
    for label, color, width in items:
        if width:
            d.line([(x, y + 14), (x + 46, y + 14)], fill=color, width=width)
        else:
            d.ellipse([x + 12, y + 3, x + 34, y + 25], outline=color, width=4)
        d.text((x + 56, y), label, fill=C_TEXT, font=f)
        x += 76 + int(d.textlength(label, font=f))


def _header(d: ImageDraw.ImageDraw, title: str, analysis: Analysis, page: int, pages: int) -> int:
    w = PAGE[0]
    d.text((MARGIN, 50), title, fill=C_TEXT, font=font(44, True))
    ok = analysis.feasible
    badge = "회전 가능" if ok else "회전 불가"
    f = font(40, True)
    tw = int(d.textlength(badge, font=f))
    bx = w - MARGIN - tw - 60
    d.rounded_rectangle([bx, 42, w - MARGIN, 112], radius=12, fill=C_OK if ok else C_NG)
    d.text((bx + 30, 52), badge, fill=(255, 255, 255), font=f)
    d.line([(MARGIN, 130), (w - MARGIN, 130)], fill=C_BORDER, width=3)
    d.text((w - MARGIN - 90, PAGE[1] - 52), f"{page} / {pages}", fill=C_MUTED, font=font(22))
    return 150


def _wrap(d, text: str, f, width: int) -> list[str]:
    out, line = [], ""
    for ch in text:
        if ch == "\n" or d.textlength(line + ch, font=f) > width:
            out.append(line)
            line = "" if ch == "\n" else ch
        else:
            line += ch
    out.append(line)
    return out


def _detail_page(analysis: Analysis, drawing_name: str, conditions: dict | None, page: int, pages: int) -> Image.Image:
    img = Image.new("RGB", PAGE, (255, 255, 255))
    d = ImageDraw.Draw(img)
    y0 = _header(d, "차량 회전 궤적 검토 - 상세", analysis, page, pages)
    v, sim = analysis.vehicle, analysis.sim
    f, fb, fh = font(26), font(26, True), font(32, True)
    col_w = (PAGE[0] - 2 * MARGIN - 60) // 2

    def table(x, y, title, rows):
        d.text((x, y), title, fill=C_TEXT, font=fh)
        y += 54
        for k, val in rows:
            d.text((x, y), k, fill=C_MUTED, font=f)
            for line in _wrap(d, str(val), fb, col_w - 330):
                d.text((x + 320, y), line, fill=C_TEXT, font=fb)
                y += 38
            y += 4
        return y + 26

    x1, x2 = MARGIN, MARGIN + col_w + 60
    rows = [("차량", v.name), ("전장 × 차폭", f"{v.overall_length:.2f} m × {v.width:.2f} m"),
            ("축간거리", f"{v.wheelbase:.2f} m"),
            ("앞 / 뒤 내민길이", f"{v.front_overhang:.2f} m / {v.rear_overhang:.2f} m"),
            ("윤거", f"{v.track:.2f} m"),
            ("최소회전반경", f"{v.min_turn_radius:.2f} m (바깥 앞바퀴)"),
            ("최대 조향각", f"{math.degrees(v.max_steer_rad):.1f}°"),
            ("핸들 끝~끝", f"{v.lock_to_lock_s:.1f} 초")]
    if v.is_articulated:
        rows += [("킹핀~트레일러축", f"{v.trailer_wheelbase:.2f} m"),
                 ("트레일러 뒤 내민길이", f"{v.trailer_rear_overhang:.2f} m"),
                 ("최대 꺾임각", f"{v.max_articulation_deg:.0f}°")]
    if v.note:
        rows.append(("비고", v.note))
    y = table(x1, y0 + 20, "차량 제원", rows)
    cond = conditions or {}
    rows = [("도면", drawing_name or "(없음)")]
    labels = {"clearance": ("장애물 여유폭", "m"), "speed": ("전진 속도", "km/h"), "rev_speed": ("후진 속도", "km/h"),
              "pos_tol": ("목표점 허용오차", "m"), "head_tol": ("도착 방향 허용오차", "°")}
    for key, (label, unit) in labels.items():
        if key in cond:
            rows.append((label, f"{cond[key]:g} {unit}"))
    if "obstacle_layers" in cond:
        rows.append(("장애물 레이어", f"{cond['obstacle_layers']}개"))
    table(x1, y, "검토 조건", rows)

    rows = [("판정", "회전 가능" if analysis.feasible else "회전 불가"),
            ("총 주행거리", f"{sim.total_distance:.1f} m"),
            ("구간 / 방향전환", f"{len(sim.legs)}구간 / {max(len(sim.legs) - 1, 0)}회"),
            ("사용 최대 조향각", f"{analysis.max_steer_deg:.1f}°"),
            ("충돌·여유폭 부족", f"{len(analysis.collisions)}곳")]
    if v.is_articulated:
        rows.append(("최대 꺾임각", f"{analysis.max_articulation_deg:.1f}°"))
    y = table(x2, y0 + 20, "검토 결과", rows)
    d.text((x2, y), "판정 사유 · 구간별 결과", fill=C_TEXT, font=fh)
    y += 54
    lines: list[tuple[str, tuple]] = []
    for r in analysis.reasons:
        lines.append((f"[불가] {r}", C_NG))
    for wmsg in analysis.warnings:
        lines.append((f"[주의] {wmsg}", C_WARN))
    if not analysis.reasons and not analysis.warnings:
        lines.append(("문제없이 목적지까지 도달했습니다.", C_MUTED))
    for leg in sim.legs:
        gear = "전진" if leg.gear == FORWARD else "후진"
        extra = f", 방향오차 {leg.heading_error_deg:.1f}°" if leg.heading_error_deg is not None else ""
        lines.append((f"{leg.index + 1}구간 {gear} {leg.length:.1f} m — {'도달' if leg.reached else '실패'} "
                      f"(도착오차 {leg.end_error:.2f} m, 최대 경로이탈 {leg.max_deviation:.2f} m{extra})",
                      C_TEXT if leg.reached else C_NG))
    for i, c in enumerate(analysis.collisions, 1):
        part = "트레일러" if c.part == "trailer" else "차체"
        lines.append((f"충돌 {i}: {part}  x={c.x:,.2f}, y={c.y:,.2f}  (주행 {c.dist_from:.1f}~{c.dist_to:.1f} m)", C_TEXT))
    limit = PAGE[1] - 150
    for text, color in lines:
        for line in _wrap(d, text, f, col_w):
            if y > limit:
                d.text((x2, y), "… (이하 생략, 보고서(txt) 참고)", fill=C_MUTED, font=f)
                break
            d.text((x2, y), line, fill=color, font=f)
            y += 38
        else:
            y += 6
            continue
        break
    note = ("이 보고서는 운동학 모델에 의한 설계 검토 보조 자료입니다. 차체는 평면 사각형으로 가정하며, "
            "최종 판단은 관련 기준과 실제 차량 제원으로 확인하십시오.")
    yy = PAGE[1] - 110
    for line in _wrap(d, note, font(22), PAGE[0] - 2 * MARGIN - 120):
        d.text((MARGIN, yy), line, fill=C_MUTED, font=font(22))
        yy += 30
    return img


def build_pages(analysis: Analysis, scenario: Scenario, drawing=None, obstacle_layers: set[str] | None = None,
                conditions: dict | None = None) -> list[Image.Image]:
    drawing_name = drawing.path.name if drawing is not None and drawing.path else ""
    ncol = min(len(analysis.collisions), 4)
    pages = 2 + (1 if ncol else 0)

    # 1쪽: 전체 그림
    p1 = Image.new("RGB", PAGE, (255, 255, 255))
    d = ImageDraw.Draw(p1)
    top = _header(d, "차량 회전 궤적 검토", analysis, 1, pages)
    fig_box = (MARGIN, top + 60, PAGE[0] - MARGIN, PAGE[1] - 130)
    fw, fh_ = fig_box[2] - fig_box[0], fig_box[3] - fig_box[1]
    fig = render_figure(analysis, scenario, (fw, fh_), drawing, obstacle_layers)
    p1.paste(fig, fig_box[:2])
    d.rectangle(fig_box, outline=C_BORDER, width=2)
    sim = analysis.sim
    info = (f"{analysis.vehicle.name}  ·  주행 {sim.total_distance:.1f} m  ·  방향전환 {max(len(sim.legs) - 1, 0)}회"
            f"  ·  충돌 {len(analysis.collisions)}곳" + (f"  ·  도면: {drawing_name}" if drawing_name else ""))
    d.text((MARGIN, top + 14), info, fill=C_TEXT, font=font(26))
    _legend(d, MARGIN, PAGE[1] - 112)
    d.text((MARGIN, PAGE[1] - 60), f"작성 {datetime.now():%Y-%m-%d %H:%M}  ·  차량 회전 궤적 검토 프로그램",
           fill=C_MUTED, font=font(22))
    out = [p1, _detail_page(analysis, drawing_name, conditions, 2, pages)]

    # 3쪽: 충돌 위치 확대도 (최대 4곳)
    if ncol:
        p3 = Image.new("RGB", PAGE, (255, 255, 255))
        d = ImageDraw.Draw(p3)
        top = _header(d, "충돌 위치 확대도", analysis, 3, pages)
        cols = 1 if ncol == 1 else 2
        rows_n = 1 if ncol <= 2 else 2
        gap = 30
        cw = (PAGE[0] - 2 * MARGIN - gap * (cols - 1)) // cols
        ch = (PAGE[1] - top - 150 - gap * (rows_n - 1)) // rows_n
        half = max(analysis.vehicle.overall_length * 0.9, 8.0)
        for i, c in enumerate(analysis.collisions[:4]):
            x = MARGIN + (i % cols) * (cw + gap)
            y = top + 50 + (i // cols) * (ch + gap)
            hh = half * ch / cw if cw >= ch else half
            hw = half if cw >= ch else half * cw / ch
            fig = render_figure(analysis, scenario, (cw, ch), drawing, obstacle_layers,
                                bounds=(c.x - hw, c.y - hh, c.x + hw, c.y + hh), line_scale=1.2)
            p3.paste(fig, (x, y))
            d.rectangle([x, y, x + cw, y + ch], outline=C_BORDER, width=2)
            part = "트레일러" if c.part == "trailer" else "차체"
            d.text((x + 4, y - 40), f"충돌 {i + 1}: {part}  x={c.x:,.2f}, y={c.y:,.2f} m", fill=C_NG, font=font(26, True))
        if len(analysis.collisions) > 4:
            d.text((MARGIN, PAGE[1] - 100), f"※ 충돌 {len(analysis.collisions)}곳 중 앞의 4곳만 표시",
                   fill=C_MUTED, font=font(22))
        out.append(p3)
    return out


def export_pdf(analysis: Analysis, scenario: Scenario, out_path: str | Path, drawing=None,
               obstacle_layers: set[str] | None = None, conditions: dict | None = None) -> Path:
    out_path = Path(out_path)
    pages = build_pages(analysis, scenario, drawing, obstacle_layers, conditions)
    pages[0].save(out_path, "PDF", resolution=DPI, save_all=True, append_images=pages[1:],
                  title="차량 회전 궤적 검토", author="SweptPathChecker")
    return out_path


def export_png(analysis: Analysis, scenario: Scenario, out_path: str | Path, drawing=None,
               obstacle_layers: set[str] | None = None, conditions: dict | None = None) -> Path:
    """1쪽(판정 + 전체 궤적 그림)을 이미지로 저장."""
    out_path = Path(out_path)
    build_pages(analysis, scenario, drawing, obstacle_layers, conditions)[0].save(out_path, dpi=(DPI, DPI))
    return out_path


if __name__ == "__main__":  # pragma: no cover
    print(font(20), _font_files, sys.platform)
