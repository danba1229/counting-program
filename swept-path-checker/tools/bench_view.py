"""화면 반응 속도 측정: 예전 방식(선마다 캔버스 객체) vs 현재 방식(도면 이미지).

    python tools/bench_view.py            # 선 약 4만 개짜리 시험 도면 자동 생성
    python tools/bench_view.py 도면.dxf    # 실제 도면으로 측정
"""

from __future__ import annotations

import math
from pathlib import Path
import random
import sys
import tempfile
import time
import tkinter as tk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import app as appmod  # noqa: E402


def make_big_dxf(path: Path) -> Path:
    import ezdxf

    random.seed(1)
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4
    for n in ("벽", "기둥", "주차구획", "가구", "설비"):
        doc.layers.add(n)
    col = doc.blocks.new("COL")
    col.add_lwpolyline([(-400, -400), (400, -400), (400, 400), (-400, 400)], close=True)
    col.add_circle((0, 0), 300)
    car = doc.blocks.new("CAR")
    car.add_lwpolyline([(0, 0), (1800, 0), (1800, 4700), (0, 4700)], close=True)
    for i in range(12):
        car.add_arc((900, 1000 + i * 200), 700, 200, 340)
    msp = doc.modelspace()
    w, h = 150000, 100000
    for _ in range(25000):
        x, y = random.uniform(0, w), random.uniform(0, h)
        a, length = random.uniform(0, 6.28), random.uniform(500, 8000)
        msp.add_line((x, y), (x + length * math.cos(a), y + length * math.sin(a)),
                     dxfattribs={"layer": random.choice(["벽", "주차구획", "설비"])})
    for _ in range(3000):
        msp.add_arc((random.uniform(0, w), random.uniform(0, h)), random.uniform(500, 6000), 0,
                    random.uniform(30, 300), dxfattribs={"layer": "벽"})
    for _ in range(1500):
        msp.add_blockref("COL", (random.uniform(0, w), random.uniform(0, h)), dxfattribs={"layer": "기둥"})
    for _ in range(600):
        msp.add_blockref("CAR", (random.uniform(0, w), random.uniform(0, h)), dxfattribs={"layer": "가구"})
    doc.saveas(path)
    return path


class E:
    def __init__(self, x, y):
        self.x, self.y = x, y


def interact(root, pan, zoom) -> tuple[float, float]:
    t = time.perf_counter()
    for i in range(30):
        pan(i)
        root.update()
    t_pan = (time.perf_counter() - t) / 30
    t = time.perf_counter()
    for _ in range(10):
        zoom()
        root.update()
    t_zoom = (time.perf_counter() - t) / 10
    return t_pan * 1000, t_zoom * 1000


def main() -> None:
    if len(sys.argv) > 1:
        dxf = Path(sys.argv[1])
    else:
        dxf = make_big_dxf(Path(tempfile.mkdtemp()) / "bench.dxf")
    root = tk.Tk()
    root.geometry("1400x860+0+0")
    app = appmod.SweptPathApp(root)
    root.update()
    t = time.perf_counter()
    app.open_drawing(str(dxf), wait=True)
    root.update()
    t_open = time.perf_counter() - t
    d = app.drawing
    print(f"도면: 선 {len(d.polylines):,}개, 점 {len(d.verts):,}개, 열기 {t_open:.2f}초")

    # ---- 현재 방식 ----
    app._pan_start(E(500, 400))
    new = interact(root, lambda i: app._pan_move(E(500 + i * 5 + 5, 400 + i * 3 + 3)),
                   lambda: app._zoom(900, 450, 1.1))
    time.sleep(0.2)
    root.update()

    # ---- 예전 방식: 모든 선을 캔버스 객체로 ----
    c = app.canvas
    c.delete("dwgimg")
    app.renderer = None
    t = time.perf_counter()
    sc = d.unit_scale
    for pts in d.polylines:
        c.create_line(app._flat(pts * sc), fill="#202020", width=1, tags=("world", "old"))
    root.update()
    t_create = time.perf_counter() - t
    old = interact(root, lambda i: c.move("world", 5, 3), lambda: c.scale("world", 900, 450, 1.1, 1.1))
    print(f"예전 방식: 처음 그리기 {t_create:.2f}초, 끌기 1회 {old[0]:.1f} ms, 휠 1회 {old[1]:.1f} ms")
    print(f"현재 방식: 끌기 1회 {new[0]:.1f} ms, 휠 1회 {new[1]:.1f} ms")
    print(f"개선: 끌기 {old[0] / max(new[0], 0.01):.0f}배, 휠 {old[1] / max(new[1], 0.01):.0f}배 빠름")
    root.destroy()


if __name__ == "__main__":
    main()
