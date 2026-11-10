"""도면을 한 장의 이미지로 그리는 렌더러.

선 수만 개를 Tk 캔버스 객체로 만들면 확대·이동 때마다 모든 객체를 다시 계산해 느려진다.
여기서는 화면에 보이는 선만 골라 PIL 이미지 한 장에 그리고, 캔버스에는 그 이미지 하나만 올린다.
"""

from __future__ import annotations

import numpy as np
from PIL import Image, ImageDraw

from .drawing import Drawing

COLOR_BG = (255, 255, 255)
COLOR_OBSTACLE = (32, 32, 32)
COLOR_OTHER = (200, 200, 200)
CHUNK = 20000  # 한 번에 처리할 폴리라인 수


class DrawingRenderer:
    def __init__(self, drawing: Drawing):
        self.d = drawing
        if drawing.bboxes is None:
            drawing.build_index()

    def _visible(self, width, height, s, tx, ty) -> np.ndarray:
        d = self.d
        k = s * d.unit_scale
        vx0, vx1 = (-2 - tx) / k, (width + 2 - tx) / k
        vy0, vy1 = (ty - height - 2) / k, (ty + 2) / k
        bb = d.bboxes
        return (bb[:, 2] >= vx0) & (bb[:, 0] <= vx1) & (bb[:, 3] >= vy0) & (bb[:, 1] <= vy1)

    def visible_count(self, width: int, height: int, s: float, tx: float, ty: float) -> int:
        if self.d.bboxes is None or len(self.d.bboxes) == 0:
            return 0
        return int(np.count_nonzero(self._visible(width, height, s, tx, ty)))

    def render(self, width: int, height: int, s: float, tx: float, ty: float,
               obstacle_layers: set[str]) -> Image.Image:
        """화면 변환 canvas = (x*s + tx, -y*s + ty) [x, y 는 m] 로 width x height 이미지 생성."""
        width, height = max(int(width), 1), max(int(height), 1)
        img = Image.new("RGB", (width, height), COLOR_BG)
        d = self.d
        if d.verts is None or len(d.verts) == 0:
            return img
        k = s * d.unit_scale  # 도면 단위 → 화면 px
        bb = d.bboxes
        idx = np.nonzero(self._visible(width, height, s, tx, ty))[0]
        if len(idx) == 0:
            return img

        obst_flags = np.array([name in obstacle_layers for name in d.layer_names], dtype=bool)
        is_obst = obst_flags[d.layer_index[idx]] if len(obst_flags) else np.zeros(len(idx), bool)
        size_px = ((bb[idx, 2] - bb[idx, 0]) + (bb[idx, 3] - bb[idx, 1])) * k
        tiny = size_px < 1.0

        draw = ImageDraw.Draw(img)
        offsets = d.offsets
        # 보이는 폴리라인의 점만 변환
        starts = offsets[idx]
        ends = offsets[idx + 1]

        def draw_group(sel_all: np.ndarray, color) -> None:
            # 백그라운드에서 그릴 때 화면이 끊기지 않도록 조금씩 나눠 처리한다.
            ids = np.nonzero(sel_all)[0]
            for c0 in range(0, len(ids), CHUNK):
                sel = np.zeros(len(sel_all), bool)
                sel[ids[c0:c0 + CHUNK]] = True
                draw_chunk(sel, color)

        def draw_chunk(sel: np.ndarray, color) -> None:
            if not sel.any():
                return
            g_starts, g_ends = starts[sel], ends[sel]
            g_tiny = tiny[sel]
            # 1px 보다 작은 객체는 점 하나로
            if g_tiny.any():
                p = d.verts[g_starts[g_tiny]]
                xs = p[:, 0] * k + tx
                ys = -p[:, 1] * k + ty
                draw.point(list(zip(xs.tolist(), ys.tolist())), fill=color)
            big = ~g_tiny
            if not big.any():
                return
            b_starts, b_ends = g_starts[big], g_ends[big]
            lengths = b_ends - b_starts
            # 필요한 점들을 한 번에 모아 변환
            gather = np.repeat(b_starts - np.concatenate([[0], np.cumsum(lengths)[:-1]]), lengths) + \
                np.arange(lengths.sum())
            pts = d.verts[gather]
            flat = np.empty(len(pts) * 2)
            flat[0::2] = pts[:, 0] * k + tx
            flat[1::2] = -pts[:, 1] * k + ty
            np.clip(flat, -1e6, 1e6, out=flat)
            flat_list = flat.tolist()
            pos = 0
            for n in lengths.tolist():
                draw.line(flat_list[pos:pos + 2 * n], fill=color, width=1)
                pos += 2 * n

        draw_group(~is_obst, COLOR_OTHER)
        draw_group(is_obst, COLOR_OBSTACLE)
        return img


def preview_transform(base: Image.Image, base_view: tuple[float, float, float],
                      view: tuple[float, float, float], size: tuple[int, int]) -> Image.Image:
    """이전에 그린 이미지를 새 화면 변환에 맞게 늘리거나 옮긴 미리보기 (다시 그리기 전 임시 표시)."""
    s0, tx0, ty0 = base_view
    s, tx, ty = view
    r = s0 / s
    # 출력 픽셀 (X, Y) → 원본 픽셀 (r*X + tx0 - tx*r, r*Y + ty0 - ty*r)
    data = (r, 0.0, tx0 - tx * r, 0.0, r, ty0 - ty * r)
    return base.transform(size, Image.Transform.AFFINE, data, resample=Image.Resampling.NEAREST,
                          fillcolor=COLOR_BG)
