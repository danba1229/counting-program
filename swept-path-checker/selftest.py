"""실행 파일 자체 점검 (--selftest).

    SweptPathChecker.exe --selftest

내장 샘플 도면으로 계산 엔진, DXF/보고서 저장, GUI 동작을 차례로 점검하고
exe 옆에 selftest_result.txt 를 남긴다. 모두 통과하면 종료코드 0.
"""

from __future__ import annotations

import math
from pathlib import Path
import platform
import sys
import tempfile
import time
import traceback


def run_selftest(sample: Path, out_dir: Path, app_class) -> int:
    lines: list[str] = []
    failures = 0

    def log(msg: str) -> None:
        lines.append(msg)
        if sys.stdout is not None:  # 창 모드 exe 에서는 stdout 이 없다
            try:
                print(msg, flush=True)
            except Exception:
                pass

    def check(name: str, func) -> None:
        nonlocal failures
        t0 = time.time()
        try:
            detail = func()
            log(f"[통과] {name} ({time.time() - t0:.2f}초)" + (f" - {detail}" if detail else ""))
        except Exception as exc:
            failures += 1
            log(f"[실패] {name}: {type(exc).__name__}: {exc}")
            log(traceback.format_exc())

    log("차량 회전 궤적 검토 - 자체 점검")
    log(f"실행 환경: {platform.platform()} / Python {platform.python_version()} / "
        f"{'실행파일' if getattr(sys, 'frozen', False) else '스크립트'}")
    log(f"샘플 도면: {sample}")

    from swept_path.analysis import run_check
    from swept_path.drawing import load_drawing
    from swept_path.export import export_dxf, export_report
    from swept_path.simulate import FORWARD, REVERSE, Scenario, Waypoint
    from swept_path.vehicles import builtin_vehicles

    vehicles = {v.key: v for v in builtin_vehicles()}
    state: dict = {}
    corner = Scenario(18, 3.5, 0, [Waypoint(30, 3.5), Waypoint(36.5, 10),
                                   Waypoint(36.5, 32, FORWARD, math.pi / 2)])

    def t_import():
        import ezdxf
        import numpy
        import shapely

        return f"ezdxf {ezdxf.__version__}, shapely {shapely.__version__}, numpy {numpy.__version__}"

    def t_load():
        d = load_drawing(sample)
        assert d.unit_scale == 0.001, d.unit_scale
        assert d.default_obstacle_layers() == {"벽", "기둥"}, d.default_obstacle_layers()
        state["drawing"] = d
        state["obs"] = d.polylines_m(d.default_obstacle_layers())
        return f"선 {len(d.polylines)}개, 레이어 {sorted(d.layer_counts)}"

    expected = {"passenger": True, "minibus_county": True, "large": False, "semitrailer": False}

    def t_corner():
        out = []
        for key, ok in expected.items():
            a = run_check(vehicles[key], corner, state["obs"])
            assert a.feasible == ok, f"{key}: 기대 {ok}, 결과 {a.feasible} {a.reasons}"
            out.append(f"{vehicles[key].name}={'가능' if a.feasible else '불가'}")
            state.setdefault("analysis", a)
        return ", ".join(out)

    def t_multi_turn():
        sc = Scenario(20, 3.5, 0, [Waypoint(30, 3.5), Waypoint(22, 3.5, REVERSE), Waypoint(28, 3.5)])
        a = run_check(vehicles["passenger"], sc, state["obs"])
        assert a.feasible and len(a.sim.legs) == 3, a.reasons
        return "전진→후진→전진 3구간 도달"

    def t_semi_reverse():
        R = 25.0
        pts = [(-13.2 - R * math.sin(t), R - R * math.cos(t))
               for t in [0.05 + i * (math.pi / 2 - 0.05) / 9 for i in range(10)]]
        wps = [Waypoint(x, y, REVERSE) for x, y in pts[:-1]] + [Waypoint(*pts[-1], REVERSE, -math.pi / 2)]
        a = run_check(vehicles["semitrailer"], Scenario(0, 0, 0, wps), [])
        assert a.feasible, a.reasons
        return f"최대 꺾임각 {a.max_articulation_deg:.1f}°"

    def t_export():
        import ezdxf

        tmp = Path(tempfile.mkdtemp())
        a = state["analysis"]
        dxf = export_dxf(a, corner, tmp / "결과.dxf", unit_scale=0.001, base_drawing=sample)
        doc = ezdxf.readfile(dxf)
        layers = {e.dxf.layer for e in doc.modelspace()}
        assert {"ST_ENVELOPE", "ST_WHEEL", "벽"} <= layers, layers
        rep = export_report(a, tmp / "보고서.txt", sample.name)
        assert "판정" in rep.read_text(encoding="utf-8-sig")
        return f"DXF {dxf.stat().st_size // 1024} KB, 보고서 저장"

    def t_gui():
        import tkinter as tk

        root = tk.Tk()
        try:
            gui = app_class(root)
            root.update()
            gui.open_drawing(str(sample))  # 백그라운드 읽기
            deadline = time.time() + 30
            while gui.drawing is None and time.time() < deadline:
                root.update()
                time.sleep(0.02)
            assert gui.drawing is not None, "도면을 30초 안에 읽지 못함"
            root.update()
            assert gui.canvas.find_withtag("dwgimg"), "도면 이미지가 그려지지 않음"
            gui.vehicle_var.set(vehicles["large"].name)
            gui._on_vehicle_change()
            gui.start = (18.0, 3.5, 0.0)
            gui.waypoints = [Waypoint(30, 3.5), Waypoint(36.5, 10)]
            gui.dest = Waypoint(36.5, 32, FORWARD, math.pi / 2)
            gui._path_changed()
            gui.run_check()
            deadline = time.time() + 30
            while gui.analysis is None and time.time() < deadline:
                root.update()
                time.sleep(0.02)
            assert gui.analysis is not None, "검토 결과가 30초 안에 나오지 않음"
            verdict = gui.verdict_label.cget("text").strip()
            assert verdict == "회전 불가", verdict
            root.update()
            items = len(gui.canvas.find_withtag("result"))
            assert items > 10, f"결과 궤적 요소 {items}개"
            assert gui.canvas.find_withtag("dwgimg"), "도면 이미지 없음"
            # 확대·이동 후에도 도면 이미지가 다시 그려지는지
            gui._zoom(300, 300, 1.5)
            gui._pan_start(type("E", (), {"x": 100, "y": 100})())
            gui._pan_move(type("E", (), {"x": 160, "y": 130})())
            root.update()
            time.sleep(0.2)
            root.update()
            assert gui._dwg_base is not None and gui._dwg_base[1] == (gui.s, gui.tx, gui.ty), "이동 후 다시 그리기 안 됨"
            return f"화면 판정 '{verdict}', 결과 궤적 요소 {items}개, 확대·이동 후 다시 그리기 정상"
        finally:
            root.destroy()

    check("라이브러리 불러오기", t_import)
    check("샘플 도면 읽기 (mm, 블록 기둥)", t_load)
    if "obs" in state:
        check("90° 코너 차량별 판정", t_corner)
        check("여러 번 꺾기 (전진/후진)", t_multi_turn)
    check("세미트레일러 후진", t_semi_reverse)
    if "analysis" in state:
        check("DXF·보고서 저장", t_export)
    check("GUI 실행 및 검토", t_gui)

    def t_subprocess_load():
        """큰 파일용 별도 프로세스 해석 경로 (임계값을 0으로 낮춰 샘플로 시험)."""
        import shutil
        import tkinter as tk

        mod = sys.modules[app_class.__module__]
        tmp = Path(tempfile.mkdtemp())
        copy = tmp / "sub.dxf"
        shutil.copy(sample, copy)  # 캐시가 없는 새 파일
        old = mod.SUBPROCESS_LOAD_MB
        mod.SUBPROCESS_LOAD_MB = 0.0
        root = tk.Tk()
        try:
            gui = app_class(root)
            t0 = time.time()
            gui.open_drawing(str(copy))
            while gui.drawing is None and gui._loading and time.time() - t0 < 60:
                root.update()
                time.sleep(0.02)
            assert gui.drawing is not None, "별도 프로세스 해석 실패: " + gui.status.get()
            assert len(gui.drawing.polylines) == len(state["drawing"].polylines)
            # 두 번째는 캐시에서
            from swept_path.drawing import cached_drawing
            assert cached_drawing(copy) is not None, "캐시가 저장되지 않음"
            return f"{time.time() - t0:.1f}초, 캐시 저장 확인"
        finally:
            mod.SUBPROCESS_LOAD_MB = old
            root.destroy()

    if "drawing" in state:
        check("큰 파일 별도 프로세스 해석 + 캐시", t_subprocess_load)

    log("")
    log("결과: " + ("모두 통과" if failures == 0 else f"{failures}개 실패"))
    try:
        (out_dir / "selftest_result.txt").write_text("\n".join(lines), encoding="utf-8-sig")
    except OSError:
        pass
    return 0 if failures == 0 else 1
