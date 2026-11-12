"""차량 회전 궤적 검토 프로그램 (Tkinter 데스크톱 앱).

사용 순서
 1. [도면] 탭에서 DXF(또는 DWG) 도면을 열고 장애물로 볼 레이어를 고른다.
 2. [차량] 탭에서 차량을 고른다.
 3. [경로] 탭에서 출발점(드래그로 방향) → 경유점 → 목적지를 찍는다. 후진 구간은 '후진'을 고른 뒤 찍는다.
 4. [회전 검토 실행]
"""

from __future__ import annotations

import copy
import math
import multiprocessing
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import traceback

import numpy as np
from PIL import ImageTk

sys.path.insert(0, str(Path(__file__).resolve().parent))

from swept_path.analysis import Analysis, CheckOptions, analyze, build_obstacles  # noqa: E402
from swept_path.drawing import (  # noqa: E402
    UNIT_CHOICES, Drawing, DrawingError, cached_drawing, load_drawing, read_npz, subprocess_load_worker,
)
from swept_path.export import (  # noqa: E402
    export_dxf, export_report, load_project, save_project, scenario_from_dict, scenario_to_dict,
)
from swept_path.geometry import body_polygons  # noqa: E402
from swept_path.render import DrawingRenderer, preview_transform  # noqa: E402
from swept_path.ui_theme import (  # noqa: E402
    BG, BORDER, DANGER, DANGER_BG, MUTED, PRIMARY, PRIMARY_DARK, SUCCESS, SUCCESS_BG, TEXT, WARN, WARN_BG,
    apply_theme, card, draw_vehicle_diagram, enable_dpi_awareness,
)
from swept_path.simulate import FORWARD, REVERSE, Scenario, SimOptions, Simulator, Waypoint  # noqa: E402
from swept_path.vehicles import SEMITRAILER, SINGLE, Vehicle, VehicleLibrary  # noqa: E402

if getattr(sys, "frozen", False):
    # 실행 파일(exe): 사용자 파일은 exe 옆, 내장 자료는 압축 해제 폴더
    APP_DIR = Path(sys.executable).resolve().parent
    RESOURCE_DIR = Path(getattr(sys, "_MEIPASS", APP_DIR))
else:
    APP_DIR = Path(__file__).resolve().parent
    RESOURCE_DIR = APP_DIR
USER_VEHICLE_FILE = APP_DIR / "user_vehicles.json"
SAMPLE_DRAWING = RESOURCE_DIR / "samples" / "sample_parking.dxf"

COLOR_OBSTACLE = "#202020"
COLOR_OTHER = "#c8c8c8"
COLOR_PATH_FWD = "#d000d0"
COLOR_PATH_REV = "#ff7a00"
COLOR_ENVELOPE = "#1f5fff"
COLOR_CLEAR = "#55c8ff"
COLOR_WHEEL = "#00a040"
COLOR_CORNER = "#ff9a00"
COLOR_SNAP = "#888888"
COLOR_COLLISION = "#ff0000"

SUBPROCESS_LOAD_MB = 8.0  # 이보다 큰 파일은 별도 프로세스에서 해석
ASYNC_RENDER_LINES = 30000  # 화면에 보이는 선이 이보다 많으면 백그라운드에서 그림

TOOL_START, TOOL_WAYPOINT, TOOL_DEST, TOOL_PAN = "start", "waypoint", "dest", "pan"
TOOL_PICK = "pick"  # 도면의 선을 클릭해 장애물 레이어 켜기/끄기
PICK_PX = 8  # 선 고르기 거리 (화면 px)
PICK_HIGHLIGHT_MAX = 4000  # 강조 표시할 같은 레이어 선 최대 개수

VEHICLE_FIELDS = [
    ("name", "차량 이름", str),
    ("width", "차폭 (m)", float),
    ("height", "차고 (m)", float),
    ("wheelbase", "축간거리 / 트랙터 축간거리 (m)", float),
    ("front_overhang", "앞 내민길이 (m)", float),
    ("rear_overhang", "뒤 내민길이 / 트랙터 뒷부분 (m)", float),
    ("track", "윤거 (좌우 바퀴 간격, m)", float),
    ("min_turn_radius", "최소회전반경 (바깥 앞바퀴, m)", float),
    ("max_steer_deg", "최대 조향각 (°, 비우면 회전반경으로 계산)", float),
    ("lock_to_lock_s", "핸들 끝~끝 조작시간 (초)", float),
    ("hitch_offset", "[세미] 킹핀 위치 (뒷축 뒤 +, m)", float),
    ("trailer_wheelbase", "[세미] 킹핀~트레일러축 (m)", float),
    ("trailer_front_overhang", "[세미] 킹핀~트레일러 앞면 (m)", float),
    ("trailer_rear_overhang", "[세미] 트레일러 뒤 내민길이 (m)", float),
    ("trailer_width", "[세미] 트레일러 폭 (m)", float),
    ("trailer_track", "[세미] 트레일러 윤거 (m)", float),
    ("max_articulation_deg", "[세미] 최대 꺾임각 (°)", float),
]


class SweptPathApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("차량 회전 궤적 검토")
        # 화면 크기에 맞춰 창 크기 결정 (작업표시줄 여유 포함)
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        w, h = min(1400, sw - 40), min(860, sh - 100)
        root.geometry(f"{w}x{h}+{max((sw - w) // 2, 0)}+{max((sh - h) // 2 - 30, 0)}")
        root.minsize(min(1000, w), min(600, h))

        self.library = VehicleLibrary(USER_VEHICLE_FILE)
        self.drawing: Drawing | None = None
        self.layer_names: list[str] = []
        self.obstacle_layers: set[str] = set()
        self._obstacle_cache: tuple | None = None

        # 경로 상태 (m 단위)
        self.start: tuple[float, float, float] | None = None
        self.waypoints: list[Waypoint] = []
        self.dest: Waypoint | None = None
        self.analysis: Analysis | None = None
        self.analysis_scenario: Scenario | None = None

        # 화면 변환: canvas = (x*s + tx, -y*s + ty)
        self.s = 10.0
        self.tx = 100.0
        self.ty = 500.0
        self._drag = None
        self._edit = None  # 점 이동/방향 수정 중: (종류, 번호, 모드, 잡은 위치 오프셋)
        self._undo: list = []  # 경로 되돌리기 기록
        self._busy = False
        self._loading = False
        # 도면 이미지 렌더링 상태
        self.renderer: DrawingRenderer | None = None
        self._dwg_photo = None
        self._dwg_base = None  # (PIL 이미지, (s, tx, ty))
        self._render_job = None
        self._render_thread: threading.Thread | None = None
        self._render_pending = False
        self._render_q: queue.Queue = queue.Queue()

        self._build_ui()
        self._refresh_vehicle_list()
        self._bind_canvas()
        self._update_hud()
        self.set_status("도면을 열거나, 도면 없이 바로 출발점/경유점을 찍어 시험할 수 있습니다.")

    # ==================================================================
    # UI 구성
    # ==================================================================
    def _build_ui(self) -> None:
        self.fonts = apply_theme(self.root)
        F = self.fonts
        menubar = tk.Menu(self.root)
        fmenu = tk.Menu(menubar, tearoff=0)
        fmenu.add_command(label="도면 열기...", accelerator="Ctrl+O", command=self.open_drawing)
        fmenu.add_command(label="샘플 도면 열기", command=self.open_sample)
        fmenu.add_separator()
        fmenu.add_command(label="작업 저장...", accelerator="Ctrl+S", command=self.save_project)
        fmenu.add_command(label="작업 불러오기...", command=self.load_project)
        fmenu.add_separator()
        fmenu.add_command(label="종료", command=self.root.destroy)
        menubar.add_cascade(label="파일", menu=fmenu)
        hmenu = tk.Menu(menubar, tearoff=0)
        hmenu.add_command(label="단축키 / 사용법", command=self.show_help)
        menubar.add_cascade(label="도움말", menu=hmenu)
        self.root.config(menu=menubar)

        # 하단 요소를 먼저 배치해야 창이 작을 때도 잘리지 않는다.
        self.status = tk.StringVar()
        bar = ttk.Frame(self.root, style="TFrame")
        bar.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Label(bar, textvariable=self.status, style="Status.TLabel", anchor="w").pack(
            side=tk.LEFT, fill=tk.X, expand=True)
        self.coord_var = tk.StringVar()
        ttk.Label(bar, textvariable=self.coord_var, style="Status.TLabel", width=36, anchor="e").pack(side=tk.RIGHT)

        main = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True)
        self._paned = main
        self.root.after(60, lambda: main.sashpos(0, 420))  # 왼쪽 설정 패널 폭

        left = ttk.Frame(main, width=410, padding=(8, 8, 4, 8))
        main.add(left, weight=0)
        right = ttk.Frame(main, padding=(4, 8, 8, 8))
        main.add(right, weight=1)

        run_frame = ttk.Frame(left)
        run_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(8, 0))
        self.run_btn = ttk.Button(run_frame, text="▶  회전 검토 실행   (Ctrl+Enter)", style="Accent.TButton",
                                  command=self.run_check)
        self.run_btn.pack(fill=tk.X)

        self.nb = ttk.Notebook(left)
        self.nb.pack(fill=tk.BOTH, expand=True)
        self._build_drawing_tab()
        self._build_vehicle_tab()
        self._build_path_tab()
        self._build_result_tab()

        self._build_toolbar(right)
        canvas_card = ttk.Frame(right, style="Card.TFrame", padding=1)
        canvas_card.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(canvas_card, bg="white", highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.verdict_label = tk.Label(self.canvas, text="", font=F["header"], bg="white", padx=4, pady=2)
        self.verdict_label.place(x=12, y=10)
        self._build_legend()
        self._build_empty_state()

    # ---------------- 도구 막대 ----------------
    TOOL_HINTS = {
        TOOL_START: "출발점: 차량 앞축 중심을 누른 채, 차량이 바라볼 방향으로 끌었다 놓으세요. (Shift: 15° 단위)",
        TOOL_WAYPOINT: "경유점: 차량 기준점이 지나갈 위치를 차례로 클릭하세요. 찍은 점을 끌면 위치가 바뀝니다.",
        TOOL_DEST: "목적지: 클릭하거나, 누른 채 끌어서 도착 방향까지 지정하세요.",
        TOOL_PAN: "화면 이동: 끌어서 이동, 휠로 확대·축소. (다른 도구에서도 휠 버튼·오른쪽 버튼으로 이동)",
        TOOL_PICK: "레이어 고르기: 선 위에 마우스를 올리면 같은 레이어가 주황색으로 표시됩니다. 클릭 = 장애물 켜기/끄기",
    }

    def _build_toolbar(self, parent) -> None:
        tb = ttk.Frame(parent, style="Card.TFrame", padding=(6, 5))
        tb.pack(fill=tk.X, pady=(0, 6))
        self.tool_var = tk.StringVar(value=TOOL_START)
        self.tool_var.trace_add("write", lambda *_: self._on_tool_change())
        for text, val in (("●  출발점", TOOL_START), ("◆  경유점", TOOL_WAYPOINT), ("⚑  목적지", TOOL_DEST),
                          ("✋  이동", TOOL_PAN), ("▤  레이어 고르기", TOOL_PICK)):
            ttk.Radiobutton(tb, text=text, value=val, variable=self.tool_var, style="Tool.TRadiobutton",
                            takefocus=False).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Separator(tb, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Label(tb, text="진행", style="CardMuted.TLabel").pack(side=tk.LEFT, padx=(0, 4))
        self.gear_var = tk.IntVar(value=FORWARD)
        for text, val in (("▲ 전진", FORWARD), ("▼ 후진", REVERSE)):
            ttk.Radiobutton(tb, text=text, value=val, variable=self.gear_var, style="Tool.TRadiobutton",
                            takefocus=False).pack(side=tk.LEFT, padx=(0, 2))
        row2 = ttk.Frame(parent)
        row2.pack(fill=tk.X, pady=(0, 6))
        for text, cmd in (("⤢ 화면 맞춤 (F)", self.fit_view), ("경로 초기화", self.clear_path),
                          ("↶ 되돌리기", self.undo)):
            ttk.Button(row2, text=text, command=cmd, style="Small.TButton", takefocus=False).pack(
                side=tk.RIGHT, padx=(4, 0))
        self.hint_var = tk.StringVar(value=self.TOOL_HINTS[TOOL_START])
        self.hint_label = ttk.Label(row2, textvariable=self.hint_var, style="Hint.TLabel", anchor="w")
        self.hint_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.hint_label.bind("<Configure>", lambda e: self.hint_label.configure(wraplength=max(e.width - 20, 200)))

    def _build_legend(self) -> None:
        lg = tk.Frame(self.canvas, bg="#ffffff", highlightthickness=1, highlightbackground=BORDER, padx=8, pady=6)
        lg.place(relx=1.0, rely=1.0, x=-10, y=-10, anchor="se")
        self.legend = lg
        self.legend_btn = tk.Label(self.canvas, text="범례 ▴", bg="#ffffff", fg=MUTED, font=self.fonts["small"],
                                   highlightthickness=1, highlightbackground=BORDER, padx=6, pady=2, cursor="hand2")
        self.legend_btn.bind("<Button-1>", lambda e: self._toggle_legend(True))
        tk.Label(lg, text="범례  (클릭하면 접기)", bg="#ffffff", fg=MUTED, font=self.fonts["small"]).grid(
            row=99, column=0, columnspan=4, sticky="w", pady=(4, 0))
        for w in [lg, *lg.winfo_children()]:
            w.bind("<Button-1>", lambda e: self._toggle_legend(False))
            w.configure(cursor="hand2")
        items = [("장애물 선", COLOR_OBSTACLE, "line"), ("장애물 아님", COLOR_OTHER, "line"),
                 ("경로 (전진)", COLOR_PATH_FWD, "line"), ("경로 (후진)", COLOR_PATH_REV, "dash"),
                 ("차체 궤적", COLOR_ENVELOPE, "box"), ("여유폭", COLOR_CLEAR, "dash"),
                 ("바퀴 궤적", COLOR_WHEEL, "line"), ("충돌", COLOR_COLLISION, "circle")]
        for i, (label, color, kind) in enumerate(items):
            c = tk.Canvas(lg, width=26, height=12, bg="#ffffff", highlightthickness=0)
            if kind == "box":
                c.create_rectangle(3, 2, 23, 10, outline=color, fill="#cfe0ff", width=2)
            elif kind == "circle":
                c.create_oval(8, 1, 18, 11, outline=color, width=2)
            else:
                c.create_line(2, 6, 24, 6, fill=color, width=2, dash=(4, 2) if kind == "dash" else ())
            c.grid(row=i // 2, column=(i % 2) * 2, padx=(0, 3), pady=1)
            tk.Label(lg, text=label, bg="#ffffff", fg=TEXT, font=self.fonts["small"]).grid(
                row=i // 2, column=(i % 2) * 2 + 1, sticky="w", padx=(0, 10))

    def _toggle_legend(self, show: bool) -> None:
        if show:
            self.legend_btn.place_forget()
            self.legend.place(relx=1.0, rely=1.0, x=-10, y=-10, anchor="se")
        else:
            self.legend.place_forget()
            self.legend_btn.place(relx=1.0, rely=1.0, x=-10, y=-10, anchor="se")

    def _build_empty_state(self) -> None:
        box = tk.Frame(self.canvas, bg="#ffffff")
        tk.Label(box, text="도면을 열어 시작하세요", bg="#ffffff", fg=TEXT, font=self.fonts["header"]).pack()
        tk.Label(box, text="DXF(또는 ODA 변환기가 있으면 DWG) 도면을 열고, 차량을 고른 뒤\n"
                           "출발점 → 경유점 → 목적지를 찍고 [회전 검토 실행]을 누르면 됩니다.",
                 bg="#ffffff", fg=MUTED, font=self.fonts["base"], justify="center").pack(pady=(6, 12))
        btns = tk.Frame(box, bg="#ffffff")
        btns.pack()
        ttk.Button(btns, text="도면 열기 (Ctrl+O)", style="Accent.TButton", command=self.open_drawing).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(btns, text="샘플 도면으로 시작", command=self.open_sample).pack(side=tk.LEFT, padx=4)
        tk.Label(box, text="도면 없이 바로 점을 찍어 시험해 볼 수도 있습니다.", bg="#ffffff", fg=MUTED,
                 font=self.fonts["small"]).pack(pady=(12, 0))
        self.empty_state = box
        self._update_empty_state()

    def _update_empty_state(self) -> None:
        if not hasattr(self, "empty_state"):
            return
        if self.drawing is None and self.start is None and not self._loading:
            self.empty_state.place(relx=0.5, rely=0.45, anchor="center")
        else:
            self.empty_state.place_forget()

    # ---------------- 도면 탭 ----------------
    def _build_drawing_tab(self) -> None:
        tab = ttk.Frame(self.nb, padding=8)
        self.nb.add(tab, text="① 도면")
        c1 = card(tab, "도면 파일")
        c1.pack(fill=tk.X)
        ttk.Button(c1, text="도면 열기 (DXF / DWG)", style="Accent.TButton", command=self.open_drawing).pack(fill=tk.X)
        self.file_var = tk.StringVar(value="열린 도면 없음")
        ttk.Label(c1, textvariable=self.file_var, style="CardMuted.TLabel", wraplength=360, justify="left").pack(
            fill=tk.X, pady=(6, 4))
        row = ttk.Frame(c1, style="Inner.TFrame")
        row.pack(fill=tk.X)
        ttk.Label(row, text="도면 단위", style="Card.TLabel").pack(side=tk.LEFT)
        self.unit_var = tk.StringVar(value="mm")
        unit_box = ttk.Combobox(row, textvariable=self.unit_var, values=list(UNIT_CHOICES), width=7,
                                state="readonly")
        unit_box.pack(side=tk.LEFT, padx=6)
        unit_box.bind("<<ComboboxSelected>>", lambda e: self._on_unit_change())
        self.unit_info = tk.StringVar()
        ttk.Label(row, textvariable=self.unit_info, style="CardMuted.TLabel").pack(side=tk.LEFT)

        c2 = card(tab, "장애물 레이어")
        c2.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        ttk.Label(c2, text="선택된(파란) 레이어의 선을 벽·기둥 같은 장애물로 봅니다.", style="CardMuted.TLabel").pack(
            anchor="w")
        srow = ttk.Frame(c2, style="Inner.TFrame")
        srow.pack(fill=tk.X, pady=(6, 4))
        ttk.Label(srow, text="🔍", style="Card.TLabel").pack(side=tk.LEFT)
        self.layer_filter = tk.StringVar()
        ent = ttk.Entry(srow, textvariable=self.layer_filter)
        ent.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 6))
        self.layer_filter.trace_add("write", lambda *_: self._fill_layer_list())
        self.layer_count_var = tk.StringVar(value="")
        ttk.Label(srow, textvariable=self.layer_count_var, style="CardMuted.TLabel").pack(side=tk.RIGHT)
        # 아래쪽 버튼을 먼저 배치 (창이 작아도 잘리지 않게), 목록은 남는 공간을 채운다.
        pf = ttk.Frame(c2, style="Inner.TFrame")
        pf.pack(side=tk.BOTTOM, fill=tk.X, pady=(4, 0))
        ttk.Button(pf, text="레이어 설정 저장", style="Small.TButton", command=self.save_layer_preset).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        ttk.Button(pf, text="레이어 설정 불러오기", style="Small.TButton", command=self.load_layer_preset).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))
        ttk.Button(c2, text="▤  도면에서 클릭해 장애물 레이어 켜기/끄기", command=self.start_pick_mode).pack(
            side=tk.BOTTOM, fill=tk.X, pady=(6, 0))
        btns = ttk.Frame(c2, style="Inner.TFrame")
        btns.pack(side=tk.BOTTOM, fill=tk.X, pady=(6, 0))
        for text, cmd in (("전체 선택", lambda: self._set_all_layers(True)),
                          ("전체 해제", lambda: self._set_all_layers(False)), ("기본값", self._default_layers)):
            ttk.Button(btns, text=text, style="Small.TButton", command=cmd).pack(side=tk.LEFT, padx=(0, 4))
        lf = ttk.Frame(c2, style="Inner.TFrame")
        lf.pack(fill=tk.BOTH, expand=True)
        self.layer_list = tk.Listbox(lf, selectmode=tk.MULTIPLE, exportselection=False, height=4,
                                     activestyle="none")
        sb = ttk.Scrollbar(lf, orient=tk.VERTICAL, command=self.layer_list.yview)
        self.layer_list.config(yscrollcommand=sb.set)
        self.layer_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.layer_list.bind("<<ListboxSelect>>", lambda e: self._on_layer_select())

    # ---------------- 차량 탭 ----------------
    def _build_vehicle_tab(self) -> None:
        tab = ttk.Frame(self.nb, padding=8)
        self.nb.add(tab, text="② 차량")
        c1 = card(tab, "차량 유형")
        c1.pack(fill=tk.X)
        self.vehicle_var = tk.StringVar()
        self.vehicle_box = ttk.Combobox(c1, textvariable=self.vehicle_var, state="readonly")
        self.vehicle_box.pack(fill=tk.X)
        self.vehicle_box.bind("<<ComboboxSelected>>", lambda e: self._on_vehicle_change())
        self.vehicle_canvas = tk.Canvas(c1, height=170, bg="#ffffff", highlightthickness=0)
        self.vehicle_canvas.pack(fill=tk.X, pady=(8, 0))
        self.vehicle_canvas.bind("<Configure>", lambda e: self._draw_vehicle())

        c2 = card(tab, "제원")
        c2.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.spec_frame = ttk.Frame(c2, style="Card.TFrame", padding=0)
        self.spec_frame.configure(borderwidth=0)
        self.spec_frame.pack(fill=tk.X)
        self.vehicle_note = tk.StringVar()
        ttk.Label(c2, textvariable=self.vehicle_note, style="CardMuted.TLabel", wraplength=360,
                  justify="left").pack(fill=tk.X, pady=(8, 0))
        btns = ttk.Frame(c2, style="Inner.TFrame")
        btns.pack(fill=tk.X, side=tk.BOTTOM, pady=(8, 0))
        ttk.Button(btns, text="제원 편집 / 새 차량", command=self.edit_vehicle).pack(side=tk.LEFT)
        ttk.Button(btns, text="사용자 차량 삭제", command=self.delete_vehicle).pack(side=tk.LEFT, padx=4)

    # ---------------- 경로 탭 ----------------
    def _build_path_tab(self) -> None:
        tab = ttk.Frame(self.nb, padding=8)
        self.nb.add(tab, text="③ 경로")
        opt = card(tab, "검토 조건")
        opt.pack(side=tk.BOTTOM, fill=tk.X, pady=(8, 0))  # 아래 고정: 작은 화면에서도 잘리지 않게
        grid = ttk.Frame(opt, style="Inner.TFrame")
        grid.pack(fill=tk.X)
        self.opt_vars: dict[str, tk.Variable] = {}
        rows = [
            ("clearance", "장애물 여유폭", 0.3, "m"),
            ("speed", "전진 속도", 8.0, "km/h"),
            ("rev_speed", "후진 속도", 4.0, "km/h"),
            ("pos_tol", "목표점 허용오차", 0.5, "m"),
            ("head_tol", "도착 방향 허용오차", 10.0, "°"),
        ]
        for i, (key, label, default, unit) in enumerate(rows):
            ttk.Label(grid, text=label, style="Card.TLabel").grid(row=i, column=0, sticky="w", pady=2)
            var = tk.DoubleVar(value=default)
            ttk.Entry(grid, textvariable=var, width=8, justify="right").grid(row=i, column=1, sticky="w", padx=6)
            ttk.Label(grid, text=unit, style="CardMuted.TLabel").grid(row=i, column=2, sticky="w")
            self.opt_vars[key] = var
        self.stationary_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(opt, text="정지 상태 핸들 조작 허용 (출발·방향전환 시)", style="Card.TCheckbutton",
                        variable=self.stationary_var).pack(anchor="w", pady=(6, 0))

        c1 = card(tab, "찍은 점")
        c1.pack(fill=tk.BOTH, expand=True)
        ttk.Label(c1, style="CardMuted.TLabel", wraplength=360, justify="left",
                  text="도면 위 도구 막대에서 출발점 → 경유점 → 목적지를 찍으세요. 점을 끌면 이동, "
                       "화살표 끝을 끌면 방향 수정, 더블클릭하면 숫자로 수정합니다.").pack(fill=tk.X, pady=(0, 6))
        pb = ttk.Frame(c1, style="Inner.TFrame")
        pb.pack(side=tk.BOTTOM, fill=tk.X, pady=(6, 0))
        for text, cmd in (("수정", self.edit_selected_point), ("삭제", self.delete_selected_point),
                          ("전진/후진 전환", self.toggle_selected_gear)):
            ttk.Button(pb, text=text, style="Small.TButton", command=cmd).pack(side=tk.LEFT, padx=(0, 4))

        tf = ttk.Frame(c1, style="Inner.TFrame")
        tf.pack(fill=tk.BOTH, expand=True)
        cols = ("kind", "gear", "x", "y", "heading")
        self.point_list = ttk.Treeview(tf, columns=cols, show="headings", height=3, selectmode="browse")
        for col, text, w, anchor in (("kind", "구분", 70, "w"), ("gear", "진행", 52, "center"),
                                     ("x", "x (m)", 82, "e"), ("y", "y (m)", 82, "e"),
                                     ("heading", "방향", 58, "e")):
            self.point_list.heading(col, text=text)
            self.point_list.column(col, width=w, anchor=anchor, stretch=col in ("x", "y"))
        self.point_list.tag_configure("rev", foreground=COLOR_PATH_REV)
        self.point_list.tag_configure("start", foreground=PRIMARY_DARK)
        psb = ttk.Scrollbar(tf, orient=tk.VERTICAL, command=self.point_list.yview)
        self.point_list.configure(yscrollcommand=psb.set)
        self.point_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        psb.pack(side=tk.RIGHT, fill=tk.Y)
        self.point_list.bind("<Double-Button-1>", lambda e: self.edit_selected_point())
        self.point_list.bind("<<TreeviewSelect>>", lambda e: self.draw_path())

    # ---------------- 결과 탭 ----------------
    def _build_result_tab(self) -> None:
        tab = ttk.Frame(self.nb, padding=8)
        self.nb.add(tab, text="④ 결과")
        # 아래쪽(저장 버튼, 표시 설정)을 먼저 배치해 창이 작아도 잘리지 않게 한다.
        eb = ttk.Frame(tab)
        eb.pack(side=tk.BOTTOM, fill=tk.X, pady=(8, 0))
        ttk.Button(eb, text="원본+결과 DXF", command=lambda: self.export_dxf(True)).pack(
            side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(eb, text="결과만 DXF", command=lambda: self.export_dxf(False)).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        ttk.Button(eb, text="보고서 (txt)", command=self.export_report).pack(side=tk.LEFT, fill=tk.X, expand=True)

        c3 = card(tab, "화면 표시", pad=8)
        c3.pack(side=tk.BOTTOM, fill=tk.X, pady=(8, 0))
        show = ttk.Frame(c3, style="Inner.TFrame")
        show.pack(fill=tk.X)
        self.show_vars = {
            "envelope": tk.BooleanVar(value=True),
            "clearance": tk.BooleanVar(value=True),
            "wheels": tk.BooleanVar(value=True),
            "corners": tk.BooleanVar(value=False),
            "snapshots": tk.BooleanVar(value=True),
        }
        labels = {"envelope": "차체 궤적", "clearance": "여유폭 외곽", "wheels": "바퀴 궤적",
                  "corners": "모서리 궤적", "snapshots": "차량 윤곽"}
        for i, (k, var) in enumerate(self.show_vars.items()):
            ttk.Checkbutton(show, text=labels[k], variable=var, style="Card.TCheckbutton",
                            command=self.draw_results).grid(row=i // 3, column=i % 3, sticky="w", padx=(0, 10))
        srow = ttk.Frame(c3, style="Inner.TFrame")
        srow.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(srow, text="차량 위치", style="Card.TLabel").pack(side=tk.LEFT)
        self.scrub_label = tk.StringVar(value="")
        ttk.Label(srow, textvariable=self.scrub_label, style="CardMuted.TLabel").pack(side=tk.RIGHT)
        self.scrub_var = tk.DoubleVar(value=0.0)
        self.scrub = ttk.Scale(c3, from_=0, to=1, variable=self.scrub_var, command=lambda e: self.draw_scrub())
        self.scrub.pack(fill=tk.X, pady=(2, 0))

        self.verdict_card = tk.Label(tab, text="아직 검토 전입니다", font=self.fonts["header"], bg="#e5e7eb",
                                     fg=MUTED, pady=8)
        self.verdict_card.pack(fill=tk.X)
        c1 = card(tab, "요약", pad=8)
        c1.pack(fill=tk.X, pady=(8, 0))
        self.metric_frame = ttk.Frame(c1, style="Inner.TFrame")
        self.metric_frame.pack(fill=tk.X)
        c2 = card(tab, "판정 사유 · 구간별 결과", pad=8)
        c2.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        tf = ttk.Frame(c2, style="Inner.TFrame")
        tf.pack(fill=tk.BOTH, expand=True)
        self.result_text = tk.Text(tf, height=4, wrap="word", relief="flat", bg="#ffffff", fg=TEXT,
                                   font=self.fonts["base"], padx=2, pady=2, highlightthickness=0)
        rsb = ttk.Scrollbar(tf, orient=tk.VERTICAL, command=self.result_text.yview)
        self.result_text.configure(yscrollcommand=rsb.set)
        self.result_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        rsb.pack(side=tk.RIGHT, fill=tk.Y)
        self.result_text.tag_configure("bad", foreground=DANGER, font=self.fonts["bold"])
        self.result_text.tag_configure("warn", foreground=WARN)
        self.result_text.tag_configure("head", foreground=TEXT, font=self.fonts["bold"], spacing1=6)
        self.result_text.tag_configure("muted", foreground=MUTED)
        self.result_text.config(state="disabled")

    # ---------------- 단계 표시 / 도움말 ----------------
    def _update_steps(self) -> None:
        if not hasattr(self, "nb"):
            return
        done = [self.drawing is not None, True, self.scenario() is not None, self.analysis is not None]
        names = ["도면", "차량", "경로", "결과"]
        marks = ["①", "②", "③", "④"]
        for i, (ok, name) in enumerate(zip(done, names)):
            self.nb.tab(i, text=f"{'✓' if ok else marks[i]} {name}")

    def show_help(self) -> None:
        messagebox.showinfo("단축키 / 사용법", "\n".join([
            "사용 순서: ① 도면 열기 → ② 차량 선택 → ③ 출발점·경유점·목적지 찍기 → 회전 검토 실행",
            "",
            "Ctrl+O  도면 열기          Ctrl+S  작업 저장",
            "Ctrl+Enter  회전 검토 실행   Ctrl+Z  되돌리기",
            "F  화면 맞춤               Delete  선택 점 삭제",
            "Esc  끌기 취소",
            "",
            "마우스 휠: 확대·축소 / 휠 버튼·오른쪽 버튼 끌기: 화면 이동",
            "찍은 점 끌기: 위치 이동 / 화살표 끝 끌기: 방향 수정 (Shift: 15° 단위)",
        ]))

    # ==================================================================
    # 좌표 변환 / 캔버스
    # ==================================================================
    def w2c(self, x, y):
        return x * self.s + self.tx, -y * self.s + self.ty

    def c2w(self, cx, cy):
        return (cx - self.tx) / self.s, -(cy - self.ty) / self.s

    def _flat(self, pts) -> list[float]:
        arr = np.asarray(pts, dtype=float)
        out = np.empty((len(arr), 2))
        out[:, 0] = arr[:, 0] * self.s + self.tx
        out[:, 1] = -arr[:, 1] * self.s + self.ty
        return out.ravel().tolist()

    def _bind_canvas(self) -> None:
        c = self.canvas
        c.bind("<ButtonPress-1>", self._on_press)
        c.bind("<B1-Motion>", self._on_drag)
        c.bind("<ButtonRelease-1>", self._on_release)
        for btn in ("2", "3"):
            c.bind(f"<ButtonPress-{btn}>", self._pan_start)
            c.bind(f"<B{btn}-Motion>", self._pan_move)
        c.bind("<MouseWheel>", lambda e: self._zoom(e.x, e.y, 1.2 if e.delta > 0 else 1 / 1.2))
        c.bind("<Button-4>", lambda e: self._zoom(e.x, e.y, 1.2))
        c.bind("<Button-5>", lambda e: self._zoom(e.x, e.y, 1 / 1.2))
        c.bind("<Motion>", self._on_motion)
        c.bind("<Configure>", lambda e: (self._update_hud(), self.schedule_render(preview=False)))
        self.root.bind("<Delete>", lambda e: self.delete_selected_point())
        self.root.bind("<Escape>", lambda e: self._cancel_drag())
        self.root.bind("<Control-z>", lambda e: self.undo())
        self.root.bind("<Control-o>", lambda e: self.open_drawing())
        self.root.bind("<Control-s>", lambda e: self.save_project())
        self.root.bind("<Control-Return>", lambda e: self.run_check())

        def fit_key(e):
            if not isinstance(e.widget, (tk.Entry, ttk.Entry, tk.Text, ttk.Combobox)):
                self.fit_view()

        self.root.bind("<KeyPress-f>", fit_key)
        self.root.bind("<Control-Z>", lambda e: self.undo())

    def _zoom(self, cx, cy, f) -> None:
        self.canvas.scale("world", cx, cy, f, f)
        self.s *= f
        self.tx = cx + (self.tx - cx) * f
        self.ty = cy + (self.ty - cy) * f
        self._update_hud()
        self.schedule_render(preview=True)

    def _pan_start(self, e) -> None:
        self._pan = (e.x, e.y)

    def _pan_move(self, e) -> None:
        dx, dy = e.x - self._pan[0], e.y - self._pan[1]
        self._pan = (e.x, e.y)
        self.canvas.move("world", dx, dy)
        self.canvas.move("dwgimg", dx, dy)  # 이미지는 옮기기만 하고, 멈추면 다시 그린다
        self.tx += dx
        self.ty += dy
        self.schedule_render(preview=False)

    def _on_motion(self, e) -> None:
        x, y = self.c2w(e.x, e.y)
        self.coord_var.set(f"x = {x:,.2f} m   y = {y:,.2f} m")
        if self.tool_var.get() == TOOL_PICK:
            self._pick_hover(e.x, e.y)

    # ---------------- 도면에서 레이어 고르기 ----------------
    def _on_tool_change(self) -> None:
        if hasattr(self, "hint_var"):
            self.hint_var.set(self.TOOL_HINTS.get(self.tool_var.get(), ""))
        if self.tool_var.get() != TOOL_PICK and hasattr(self, "canvas"):
            self.canvas.delete("pickhl")
            self.canvas.config(cursor="crosshair")
            self._pick_layer = None

    def start_pick_mode(self) -> None:
        if not self.drawing:
            messagebox.showinfo("안내", "먼저 도면을 여세요.")
            return
        self.tool_var.set(TOOL_PICK)
        self.set_status("도면의 선 위에 마우스를 올리면 레이어가 표시됩니다. 클릭하면 장애물 ↔ 장애물 아님 전환. "
                        "끝나면 경로 탭에서 다른 도구를 고르세요.")

    def _pick_at(self, cx: float, cy: float):
        """화면 좌표 근처 선의 레이어 이름 (없으면 None)."""
        d = self.drawing
        if d is None:
            return None
        x, y = self.c2w(cx, cy)
        k = d.unit_scale
        i = d.nearest_polyline(x / k, y / k, PICK_PX / (self.s * k))
        return None if i is None else d.layers[i]

    def _pick_hover(self, cx: float, cy: float) -> None:
        layer = self._pick_at(cx, cy)
        if layer == getattr(self, "_pick_layer", None):
            return
        self._pick_layer = layer
        self.canvas.delete("pickhl")
        if layer is None:
            self.canvas.config(cursor="crosshair")
            return
        self.canvas.config(cursor="hand2")
        d = self.drawing
        li = d.layer_names.index(layer)
        vis = self.renderer._visible(max(self.canvas.winfo_width(), 1), max(self.canvas.winfo_height(), 1),
                                     self.s, self.tx, self.ty) if self.renderer else None
        mask = d.layer_index == li
        if vis is not None:
            mask &= vis
        ids = np.nonzero(mask)[0][:PICK_HIGHLIGHT_MAX]
        k = d.unit_scale
        for i in ids:
            pts = d.verts[d.offsets[i]:d.offsets[i + 1]] * k
            self.canvas.create_line(self._flat(pts), fill="#ff8c00", width=3, tags=("world", "pickhl"))
        self.canvas.tag_raise("path")
        self.canvas.tag_raise("result")
        state = "장애물" if layer in self.obstacle_layers else "장애물 아님"
        self.set_status(f"레이어: {layer}  (선 {d.layer_counts[layer]:,}개)  현재 {state}  → 클릭하면 전환")

    def _pick_click(self, cx: float, cy: float) -> None:
        layer = self._pick_at(cx, cy)
        if layer is None:
            self.set_status("근처에 선이 없습니다. 선 위를 클릭하세요.")
            return
        on = layer not in self.obstacle_layers
        if on:
            self.obstacle_layers.add(layer)
        else:
            self.obstacle_layers.discard(layer)
        self._obstacle_cache = None
        view = getattr(self, "_layer_view", self.layer_names)
        if layer in view:
            i = view.index(layer)
            if on:
                self.layer_list.selection_set(i)
            else:
                self.layer_list.selection_clear(i)
            self.layer_list.see(i)
        self._update_layer_count()
        self.clear_results()
        self.render_drawing()
        self._pick_layer = None
        self._pick_hover(cx, cy)
        self.set_status(f"'{layer}' 레이어를 {'장애물로 지정' if on else '장애물에서 제외'}했습니다. "
                        f"(장애물 레이어 {len(self.obstacle_layers)}개)")

    # ---------------- 레이어 설정 저장/불러오기 ----------------
    def save_layer_preset(self) -> None:
        if not self.drawing:
            messagebox.showinfo("안내", "먼저 도면을 여세요.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="장애물레이어.json",
                                            filetypes=[("레이어 설정", "*.json")])
        if not path:
            return
        save_project(path, {"type": "swept-path-layers", "obstacle_layers": sorted(self.obstacle_layers),
                            "drawing": self.drawing.path.name})
        self.set_status(f"장애물 레이어 {len(self.obstacle_layers)}개를 저장했습니다: {path}")

    def load_layer_preset(self, path: str | None = None) -> None:
        if not self.drawing:
            messagebox.showinfo("안내", "먼저 도면을 여세요.")
            return
        if path is None:
            path = filedialog.askopenfilename(filetypes=[("레이어 설정", "*.json")])
        if not path:
            return
        try:
            data = load_project(path)
            wanted = set(data["obstacle_layers"])
        except Exception as exc:
            messagebox.showerror("불러오기 실패", f"레이어 설정 파일이 아닙니다.\n{exc}")
            return
        present = set(self.layer_names)
        self.obstacle_layers = wanted & present
        missing = len(wanted - present)
        self._obstacle_cache = None
        self._fill_layer_list()
        self.clear_results()
        self.render_drawing()
        msg = f"장애물 레이어 {len(self.obstacle_layers)}개를 적용했습니다."
        if missing:
            msg += f" (설정에 있지만 이 도면에 없는 레이어 {missing}개는 건너뜀)"
        self.set_status(msg)

    # ---------------- 점 수정 ----------------
    HIT_PX = 9

    def _arrow_len(self) -> float:
        """방향 화살표 길이(m). 축소해도 화면에서 30 px 이상 되게 한다."""
        return max(3.0, 30.0 / self.s)

    def _point_xy(self, kind: str, idx) -> tuple[float, float]:
        if kind == "start":
            return self.start[0], self.start[1]
        w = self.waypoints[idx] if kind == "wp" else self.dest
        return w.x, w.y

    def _point_heading(self, kind: str, idx):
        if kind == "start":
            return self.start[2]
        if kind == "dest":
            return self.dest.heading
        return None

    def _hit_point(self, cx: float, cy: float):
        """화면 좌표 근처의 점 → (종류, 번호, 'move'|'rotate'). 없으면 None."""
        cands = []
        if self.start:
            cands.append(("start", None))
        cands += [("wp", i) for i in range(len(self.waypoints))]
        if self.dest:
            cands.append(("dest", None))
        best = None
        for kind, idx in cands:
            x, y = self._point_xy(kind, idx)
            px, py = self.w2c(x, y)
            d = math.hypot(cx - px, cy - py)
            if d <= self.HIT_PX and (best is None or d < best[0]):
                best = (d, kind, idx, "move")
            h = self._point_heading(kind, idx)
            if h is not None:
                L = self._arrow_len()
                hx, hy = self.w2c(x + L * math.cos(h), y + L * math.sin(h))
                d = math.hypot(cx - hx, cy - hy)
                if d <= self.HIT_PX and (best is None or d < best[0]):
                    best = (d, kind, idx, "rotate")
        return None if best is None else best[1:]

    def _apply_edit_drag(self, e) -> None:
        if math.hypot(e.x - self._edit_press[0], e.y - self._edit_press[1]) < 3:
            return  # 누르기만 했으면 그대로 둔다 (선택만)
        kind, idx, mode, (ox, oy) = self._edit
        if mode == "move":
            x, y = self.c2w(e.x - ox, e.y - oy)
            if kind == "start":
                self.start = (x, y, self.start[2])
            else:
                w = self.waypoints[idx] if kind == "wp" else self.dest
                w.x, w.y = x, y
            self.coord_var.set(f"이동: x = {x:,.2f} m   y = {y:,.2f} m")
        else:
            px, py = self._point_xy(kind, idx)
            wx, wy = self.c2w(e.x, e.y)
            ang = self._snap_heading(math.atan2(wy - py, wx - px), e)
            if kind == "start":
                self.start = (self.start[0], self.start[1], ang)
            else:
                self.dest.heading = ang
            self.coord_var.set(f"방향 {math.degrees(ang):.1f}°  (Shift: 15° 단위)")
        self.clear_results()
        self.draw_path()

    def _list_index(self, kind: str, idx) -> int:
        base = 1 if self.start else 0
        if kind == "start":
            return 0
        if kind == "wp":
            return base + idx
        return base + len(self.waypoints)

    def _select_in_list(self, kind: str, idx) -> None:
        i = str(self._list_index(kind, idx))
        if self.point_list.exists(i):
            self.point_list.selection_set(i)
            self.point_list.see(i)

    def _snapshot(self):
        return copy.deepcopy((self.start, self.waypoints, self.dest))

    def _push_undo(self) -> None:
        self._undo.append(self._snapshot())
        del self._undo[:-100]  # 최근 100단계까지

    def undo(self) -> None:
        if not self._undo:
            self.set_status("되돌릴 작업이 없습니다.")
            return
        self.start, self.waypoints, self.dest = self._undo.pop()
        self._path_changed()
        self.set_status(f"되돌렸습니다. (남은 되돌리기 {len(self._undo)}단계)")

    def edit_selected_point(self) -> None:
        sel = self._selected_point()
        if sel is None:
            messagebox.showinfo("안내", "목록에서 수정할 점을 선택하세요.")
            return
        kind, idx = sel
        PointDialog(self.root, self, kind, idx)

    def apply_point_values(self, kind: str, idx, x: float, y: float, heading_deg, gear: int | None) -> None:
        """숫자 입력으로 점 수정 (heading_deg 가 None 이면 방향 없음)."""
        self._push_undo()
        h = None if heading_deg is None else math.radians(heading_deg)
        if kind == "start":
            self.start = (x, y, h if h is not None else self.start[2])
        else:
            w = self.waypoints[idx] if kind == "wp" else self.dest
            w.x, w.y = x, y
            if gear is not None:
                w.gear = gear
            if kind == "dest":
                w.heading = h
        self._path_changed()
        self._select_in_list(kind, idx)

    def _cancel_drag(self) -> None:
        self._drag = None
        self.canvas.delete("rubber")

    def _snap_heading(self, ang: float, event) -> float:
        if event.state & 0x0001:  # Shift: 15° 단위
            step = math.radians(15)
            return round(ang / step) * step
        return ang

    def _on_press(self, e) -> None:
        tool = self.tool_var.get()
        if tool == TOOL_PAN:
            self._pan_start(e)
            return
        if tool == TOOL_PICK:
            self._pick_click(e.x, e.y)
            return
        hit = self._hit_point(e.x, e.y)
        if hit is not None:
            # 이미 찍은 점(또는 방향 화살표 끝)을 누르면 새로 찍지 않고 수정한다.
            kind, idx, mode = hit
            px, py = self.w2c(*self._point_xy(kind, idx))
            self._edit = (kind, idx, mode, (e.x - px, e.y - py))
            self._edit_before = self._snapshot()  # 실제로 바뀌었을 때만 되돌리기에 기록
            self._edit_press = (e.x, e.y)
            self._select_in_list(kind, idx)
            return
        x, y = self.c2w(e.x, e.y)
        if tool == TOOL_WAYPOINT:
            if self.start is None:
                messagebox.showinfo("안내", "먼저 출발점을 지정하세요.")
                return
            self._push_undo()
            self.waypoints.append(Waypoint(x, y, self.gear_var.get()))
            self._path_changed()
            return
        if tool == TOOL_DEST and self.start is None:
            messagebox.showinfo("안내", "먼저 출발점을 지정하세요.")
            return
        self._drag = (tool, x, y, e.x, e.y)

    def _on_drag(self, e) -> None:
        if self.tool_var.get() == TOOL_PAN:
            self._pan_move(e)
            return
        if self._edit:
            self._apply_edit_drag(e)
            return
        if not self._drag:
            return
        _tool, x, y, px, py = self._drag
        self.canvas.delete("rubber")
        self.canvas.create_line(px, py, e.x, e.y, arrow=tk.LAST, width=2, fill="#0070ff",
                                tags=("rubber",))
        if math.hypot(e.x - px, e.y - py) > 8:
            wx, wy = self.c2w(e.x, e.y)
            ang = self._snap_heading(math.atan2(wy - y, wx - x), e)
            self.coord_var.set(f"방향 {math.degrees(ang):.1f}°  (Shift: 15° 단위)")

    def _on_release(self, e) -> None:
        if self._edit:
            kind, idx = self._edit[0], self._edit[1]
            self._apply_edit_drag(e)
            self._edit = None
            if self._snapshot() != self._edit_before:
                self._undo.append(self._edit_before)
                del self._undo[:-100]
            self._path_changed()
            self._select_in_list(kind, idx)
            self.draw_path()
            return
        if not self._drag:
            return
        tool, x, y, px, py = self._drag
        self._drag = None
        self.canvas.delete("rubber")
        heading = None
        if math.hypot(e.x - px, e.y - py) > 8:
            wx, wy = self.c2w(e.x, e.y)
            heading = self._snap_heading(math.atan2(wy - y, wx - x), e)
        self._push_undo()
        if tool == TOOL_START:
            if heading is None:
                heading = self.start[2] if self.start else 0.0
            self.start = (x, y, heading)
            if self.tool_var.get() == TOOL_START:
                self.tool_var.set(TOOL_WAYPOINT)
        elif tool == TOOL_DEST:
            self.dest = Waypoint(x, y, self.gear_var.get(), heading)
        self._path_changed()

    # ==================================================================
    # 도면
    # ==================================================================
    def open_drawing(self, path: str | None = None, wait: bool = False) -> None:
        """도면 열기. 기본은 백그라운드에서 읽어 창이 멈추지 않게 한다 (wait=True 면 바로 읽음)."""
        if self._loading:
            return
        if path is None:
            path = filedialog.askopenfilename(
                title="도면 열기",
                filetypes=[("CAD 도면", "*.dxf *.dwg"), ("DXF", "*.dxf"), ("DWG", "*.dwg"), ("모든 파일", "*.*")],
            )
        if not path:
            return
        t0 = time.time()
        if wait:
            try:
                drawing = load_drawing(path)
            except Exception as exc:
                self._on_drawing_error(exc)
                return
            self._apply_drawing(drawing, path, time.time() - t0)
            return

        self._loading = True
        self._update_empty_state()
        self.root.config(cursor="watch")
        size_mb = Path(path).stat().st_size / 1e6 if Path(path).exists() else 0
        use_process = size_mb >= SUBPROCESS_LOAD_MB and cached_drawing(path) is None
        if use_process:
            # 큰 파일은 별도 프로세스에서 해석 (화면이 멈추지 않고, 해석용 메모리도 그쪽에서만 사용)
            ctx = multiprocessing.get_context("spawn")
            q = ctx.Queue()
            proc = ctx.Process(target=subprocess_load_worker, args=(str(path), q), daemon=True)
            proc.start()
        else:
            q = queue.Queue()
            proc = None

            def work():
                try:
                    d = load_drawing(path, progress=lambda msg: q.put(("progress", msg)))
                    q.put(("ok", d))
                except Exception as exc:
                    traceback.print_exc()
                    q.put(("error", exc))

        last = {"msg": "도면 읽는 중..."}

        def poll():
            try:
                while True:
                    kind, payload = q.get_nowait()
                    if kind == "progress":
                        last["msg"] = payload
                        continue
                    if kind == "ok" and proc is not None:
                        npz, is_temp = payload
                        payload = read_npz(npz, path)
                        if is_temp:
                            Path(npz).unlink(missing_ok=True)
                        if payload is None:
                            kind, payload = "error", RuntimeError("변환 결과를 읽지 못했습니다.")
                    elif kind == "error" and proc is not None:
                        name, msg = payload
                        payload = DrawingError(msg) if name == "DrawingError" else RuntimeError(f"{name}: {msg}")
                    self._loading = False
                    self.root.config(cursor="")
                    if kind == "ok":
                        self._apply_drawing(payload, path, time.time() - t0)
                    else:
                        self._on_drawing_error(payload)
                    return
            except queue.Empty:
                pass
            if proc is not None and not proc.is_alive() and q.empty():
                self._loading = False
                self.root.config(cursor="")
                self._on_drawing_error(RuntimeError(f"도면 변환 프로세스가 비정상 종료되었습니다 (코드 {proc.exitcode})."))
                return
            self.set_status(f"{Path(path).name}: {last['msg']} ({time.time() - t0:.0f}초)")
            self.root.after(100, poll)

        if proc is None:
            threading.Thread(target=work, daemon=True).start()
        self.set_status(f"{Path(path).name}: 도면 읽는 중...")
        self.root.after(100, poll)

    def _on_drawing_error(self, exc: Exception) -> None:
        self._update_empty_state()
        msg = str(exc) if isinstance(exc, DrawingError) else f"{type(exc).__name__}: {exc}"
        messagebox.showerror("도면 열기 실패", msg)
        self.set_status("도면 열기 실패")

    def _apply_drawing(self, drawing: Drawing, path: str, elapsed: float) -> None:
        self.drawing = drawing
        self.renderer = DrawingRenderer(drawing)
        self._dwg_base = None
        self._obstacle_cache = None
        unit_name = next((k for k, v in UNIT_CHOICES.items() if abs(v - drawing.unit_scale) < 1e-12), "mm")
        self.unit_var.set(unit_name)
        self.unit_info.set("(추정값 - 확인 필요)" if drawing.unit_guessed else "(도면 설정값)")
        self.file_var.set(f"{Path(path).name}\n선 {len(drawing.polylines):,}개, 레이어 {len(drawing.layer_counts)}개"
                          f"  ({elapsed:.1f}초)")
        self.layer_names = list(drawing.layer_names)
        self.obstacle_layers = drawing.default_obstacle_layers()
        if hasattr(self, "layer_filter"):
            self.layer_filter.set("")
        self._fill_layer_list()
        self._update_empty_state()
        self._update_steps()
        self.clear_results()
        self.fit_view()
        skipped = ", ".join(f"{k} {v}" for k, v in sorted(drawing.skipped.items()))
        self.set_status("도면을 열었습니다." + (f"  제외된 객체: {skipped}" if skipped else ""))

    def open_sample(self) -> None:
        if SAMPLE_DRAWING.exists():
            self.open_drawing(str(SAMPLE_DRAWING))
        else:
            messagebox.showinfo("안내", f"샘플 도면이 없습니다:\n{SAMPLE_DRAWING}")

    def _fill_layer_list(self) -> None:
        """레이어 목록 (검색어가 있으면 이름에 검색어가 들어간 레이어만)."""
        self.layer_list.delete(0, tk.END)
        self._layer_view: list[str] = []
        if not self.drawing:
            self._update_layer_count()
            return
        q = self.layer_filter.get().strip().lower() if hasattr(self, "layer_filter") else ""
        self._layer_view = [n for n in self.layer_names if q in n.lower()] if q else list(self.layer_names)
        for i, name in enumerate(self._layer_view):
            self.layer_list.insert(tk.END, f"{name}   ({self.drawing.layer_counts[name]:,})")
            if name in self.obstacle_layers:
                self.layer_list.selection_set(i)
        self._update_layer_count()

    def _update_layer_count(self) -> None:
        if not hasattr(self, "layer_count_var"):
            return
        if not self.drawing:
            self.layer_count_var.set("")
            return
        txt = f"장애물 {len(self.obstacle_layers)} / {len(self.layer_names)}"
        view = getattr(self, "_layer_view", self.layer_names)
        if len(view) != len(self.layer_names):
            txt += f"  · 검색 {len(view)}개"
        self.layer_count_var.set(txt)

    def _on_layer_select(self) -> None:
        view = getattr(self, "_layer_view", self.layer_names)
        picked = {view[i] for i in self.layer_list.curselection()}
        # 검색으로 안 보이는 레이어의 선택 상태는 그대로 둔다.
        self.obstacle_layers = (self.obstacle_layers - set(view)) | picked
        self._obstacle_cache = None
        self.clear_results()  # 장애물이 바뀌었으므로 이전 판정은 지움
        self._update_layer_count()
        self._recolor_layers()

    def _set_all_layers(self, on: bool) -> None:
        """전체 선택/해제. 검색 중이면 검색된 레이어에만 적용."""
        view = set(getattr(self, "_layer_view", self.layer_names))  # 검색 결과가 없으면 아무것도 안 바뀜
        self.obstacle_layers = (self.obstacle_layers | view) if on else (self.obstacle_layers - view)
        self.clear_results()
        self._fill_layer_list()
        self._obstacle_cache = None
        self._recolor_layers()

    def _default_layers(self) -> None:
        if self.drawing:
            self.obstacle_layers = self.drawing.default_obstacle_layers()
            self._fill_layer_list()
            self._obstacle_cache = None
            self._recolor_layers()

    def _recolor_layers(self) -> None:
        self.render_drawing()

    def _on_unit_change(self) -> None:
        if not self.drawing:
            return
        new = UNIT_CHOICES[self.unit_var.get()]
        old = self.drawing.unit_scale
        if abs(new - old) < 1e-12:
            return
        ratio = new / old
        self.drawing.unit_scale = new
        self._obstacle_cache = None
        if self.start:
            self.start = (self.start[0] * ratio, self.start[1] * ratio, self.start[2])
        for w in self.waypoints + ([self.dest] if self.dest else []):
            w.x *= ratio
            w.y *= ratio
        self.unit_info.set("(사용자 지정)")
        self.clear_results()
        self.fit_view()

    # ---------------- 도면 이미지 ----------------
    def render_drawing(self) -> None:
        """현재 화면 변환으로 도면 이미지를 다시 그린다."""
        if self._render_job is not None:
            self.root.after_cancel(self._render_job)
            self._render_job = None
        c = self.canvas
        if not self.renderer:
            c.delete("dwgimg")
            self._dwg_photo = None
            return
        w, h = max(c.winfo_width(), 1), max(c.winfo_height(), 1)
        view = (self.s, self.tx, self.ty)
        if self.renderer.visible_count(w, h, *view) < ASYNC_RENDER_LINES:
            img = self.renderer.render(w, h, *view, self.obstacle_layers)
            self._dwg_base = (img, view)
            self._show_image(img)
            return
        # 큰 도면: 백그라운드에서 그리고, 그동안 화면 조작은 막지 않는다.
        if self._render_thread is not None and self._render_thread.is_alive():
            self._render_pending = True
            return
        self._render_pending = False
        renderer, layers, size = self.renderer, set(self.obstacle_layers), (w, h)

        def work():
            img = renderer.render(size[0], size[1], *view, layers)
            self._render_q.put((renderer, view, size, img))

        self._render_thread = threading.Thread(target=work, daemon=True)
        self._render_thread.start()
        self.root.after(15, self._poll_render)

    def _poll_render(self) -> None:
        try:
            renderer, view, size, img = self._render_q.get_nowait()
        except queue.Empty:
            self.root.after(15, self._poll_render)
            return
        if renderer is not self.renderer:
            return  # 그사이 다른 도면을 열었음
        c = self.canvas
        cur_size = (max(c.winfo_width(), 1), max(c.winfo_height(), 1))
        self._dwg_base = (img, view)
        current = (self.s, self.tx, self.ty)
        if view == current and size == cur_size and not self._render_pending:
            self._show_image(img)
            return
        # 그리는 동안 화면이 바뀌었으면 받은 이미지를 맞춰 보여주고 다시 그린다.
        self._show_image(preview_transform(img, view, current, cur_size))
        self._render_pending = False
        self.render_drawing()

    def _show_image(self, img) -> None:
        c = self.canvas
        photo = self._dwg_photo
        items = c.find_withtag("dwgimg")
        if photo is not None and items and (photo.width(), photo.height()) == img.size:
            photo.paste(img)  # 같은 크기면 기존 이미지에 덮어쓰기 (새로 만드는 것보다 빠름)
            c.coords(items[0], 0, 0)
            return
        self._dwg_photo = ImageTk.PhotoImage(img)
        c.delete("dwgimg")
        c.create_image(0, 0, image=self._dwg_photo, anchor="nw", tags=("dwgimg",))
        c.tag_lower("dwgimg")

    def schedule_render(self, preview: bool) -> None:
        """확대·이동 중에는 기존 이미지를 늘려 바로 보여주고, 멈추면 선명하게 다시 그린다."""
        if not self.renderer:
            return
        if preview and self._dwg_base is not None:
            base, view = self._dwg_base
            c = self.canvas
            size = (max(c.winfo_width(), 1), max(c.winfo_height(), 1))
            self._show_image(preview_transform(base, view, (self.s, self.tx, self.ty), size))
        if self._render_job is not None:
            self.root.after_cancel(self._render_job)
        self._render_job = self.root.after(120, self.render_drawing)

    def fit_view(self) -> None:
        bounds = None
        if self.drawing:
            bounds = self.drawing.bounds_m()
        pts = self._path_points()
        if pts:
            arr = np.array(pts)
            pb = (arr[:, 0].min() - 10, arr[:, 1].min() - 10, arr[:, 0].max() + 10, arr[:, 1].max() + 10)
            if bounds is None:
                bounds = pb
        if bounds is None:
            bounds = (-20.0, -20.0, 60.0, 40.0)
        x0, y0, x1, y1 = bounds
        w = max(self.canvas.winfo_width(), 400)
        h = max(self.canvas.winfo_height(), 300)
        self.s = 0.92 * min(w / max(x1 - x0, 1e-6), h / max(y1 - y0, 1e-6))
        self.tx = w / 2 - (x0 + x1) / 2 * self.s
        self.ty = h / 2 + (y0 + y1) / 2 * self.s
        self.redraw_all()

    def redraw_all(self) -> None:
        self.canvas.delete("world")
        self.render_drawing()
        self.draw_path()
        self.draw_results()
        self._update_hud()

    def _update_hud(self) -> None:
        """축척 막대 (화면 고정)."""
        c = self.canvas
        c.delete("hud")
        h = c.winfo_height() or 600
        target_px = 120
        meters = target_px / self.s
        nice = 10 ** math.floor(math.log10(meters))
        for m in (1, 2, 5, 10):
            if nice * m * self.s >= target_px * 0.6:
                nice *= m
                break
        px = nice * self.s
        y = h - 20
        c.create_line(15, y, 15 + px, y, width=3, tags=("hud",))
        c.create_line(15, y - 5, 15, y + 5, width=2, tags=("hud",))
        c.create_line(15 + px, y - 5, 15 + px, y + 5, width=2, tags=("hud",))
        label = f"{nice:g} m" if nice >= 1 else f"{nice * 1000:g} mm"
        c.create_text(15 + px / 2, y - 10, text=label, tags=("hud",))

    # ==================================================================
    # 차량
    # ==================================================================
    def _refresh_vehicle_list(self, select_key: str | None = None) -> None:
        names = self.library.names()
        self.vehicle_box["values"] = names
        if select_key and select_key in self.library.vehicles:
            self.vehicle_var.set(self.library.vehicles[select_key].name)
        elif self.vehicle_var.get() not in names:
            self.vehicle_var.set(names[0])
        self._on_vehicle_change()

    @property
    def vehicle(self) -> Vehicle:
        return self.library.by_name(self.vehicle_var.get())

    def _on_vehicle_change(self) -> None:
        v = self.vehicle
        try:
            steer = f"{math.degrees(v.max_steer_rad):.1f}°"
        except ValueError as exc:
            steer = f"오류: {exc}"
        rows = [
            ("전장", f"{v.overall_length:.2f} m", "차폭", f"{v.width:.2f} m"),
            ("축간거리", f"{v.wheelbase:.2f} m", "차고", f"{v.height:.2f} m"),
            ("앞 내민길이", f"{v.front_overhang:.2f} m", "뒤 내민길이", f"{v.rear_overhang:.2f} m"),
            ("윤거", f"{v.track:.2f} m", "최대 조향각", steer),
            ("최소회전반경", f"{v.min_turn_radius:.2f} m", "핸들 끝~끝", f"{v.lock_to_lock_s:.1f} 초"),
        ]
        if v.is_articulated:
            rows += [
                ("킹핀~트레일러축", f"{v.trailer_wheelbase:.2f} m", "킹핀 위치", f"{v.hitch_offset:+.2f} m"),
                ("트레일러 앞내민", f"{v.trailer_front_overhang:.2f} m", "트레일러 뒷내민",
                 f"{v.trailer_rear_overhang:.2f} m"),
                ("최대 꺾임각", f"{v.max_articulation_deg:.0f}°", "", ""),
            ]
        for w in self.spec_frame.winfo_children():
            w.destroy()
        for r, (k1, v1, k2, v2) in enumerate(rows):
            ttk.Label(self.spec_frame, text=k1, style="CardMuted.TLabel").grid(row=r, column=0, sticky="w", pady=2)
            ttk.Label(self.spec_frame, text=v1, style="Value.TLabel").grid(row=r, column=1, sticky="w", padx=(6, 16))
            ttk.Label(self.spec_frame, text=k2, style="CardMuted.TLabel").grid(row=r, column=2, sticky="w")
            ttk.Label(self.spec_frame, text=v2, style="Value.TLabel").grid(row=r, column=3, sticky="w", padx=(6, 0))
        self.vehicle_note.set(("최소회전반경은 바깥쪽 앞바퀴 기준입니다.\n") + (f"비고: {v.note}" if v.note else ""))
        self._draw_vehicle()
        self.clear_results()
        self.draw_path()

    def _draw_vehicle(self) -> None:
        try:
            draw_vehicle_diagram(self.vehicle_canvas, self.vehicle, self.fonts)
        except Exception:
            pass

    def edit_vehicle(self) -> None:
        VehicleDialog(self.root, self.vehicle, self._on_vehicle_saved)

    def _on_vehicle_saved(self, vehicle: Vehicle) -> None:
        others = {v.name for v in self.library.vehicles.values() if v.key != vehicle.key}
        base, n = vehicle.name, 2
        while vehicle.name in others:
            vehicle.name = f"{base} {n}"
            n += 1
        self.library.add_or_replace(vehicle)
        self._refresh_vehicle_list(vehicle.key)

    def delete_vehicle(self) -> None:
        v = self.vehicle
        if v.builtin:
            messagebox.showinfo("안내", "기본 차량은 삭제할 수 없습니다.")
            return
        if messagebox.askyesno("삭제", f"'{v.name}' 차량을 삭제할까요?"):
            self.library.remove(v.key)
            self._refresh_vehicle_list()

    # ==================================================================
    # 경로
    # ==================================================================
    def _all_points(self) -> list[Waypoint]:
        return self.waypoints + ([self.dest] if self.dest else [])

    def _path_points(self) -> list[tuple[float, float]]:
        pts = []
        if self.start:
            pts.append(self.start[:2])
        pts += [(w.x, w.y) for w in self._all_points()]
        return pts

    def scenario(self) -> Scenario | None:
        if not self.start or not self._all_points():
            return None
        return Scenario(self.start[0], self.start[1], self.start[2],
                        [Waypoint(w.x, w.y, w.gear, w.heading) for w in self._all_points()])

    def _path_changed(self) -> None:
        self.clear_results()
        self._refresh_point_list()
        self.draw_path()
        self._update_steps()
        self._update_empty_state()

    def _refresh_point_list(self) -> None:
        tv = self.point_list
        tv.delete(*tv.get_children())
        rows = []
        if self.start:
            x, y, h = self.start
            rows.append(("출발", "", x, y, f"{math.degrees(h):.0f}°", "start"))
        for i, w in enumerate(self.waypoints, 1):
            rows.append((f"경유 {i}", "전진" if w.gear == FORWARD else "후진", w.x, w.y, "",
                         "rev" if w.gear == REVERSE else ""))
        if self.dest:
            w = self.dest
            rows.append(("목적지", "전진" if w.gear == FORWARD else "후진", w.x, w.y,
                         "" if w.heading is None else f"{math.degrees(w.heading):.0f}°",
                         "rev" if w.gear == REVERSE else ""))
        for i, (kind, gear, x, y, head, tag) in enumerate(rows):
            tv.insert("", tk.END, iid=str(i), values=(kind, gear, f"{x:,.2f}", f"{y:,.2f}", head),
                      tags=(tag,) if tag else ())

    def _point_sel_index(self) -> int | None:
        sel = self.point_list.selection()
        return int(sel[0]) if sel else None

    def _selected_point(self):
        idx = self._point_sel_index()
        if idx is None:
            return None
        if self.start:
            if idx == 0:
                return ("start", None)
            idx -= 1
        if idx < len(self.waypoints):
            return ("wp", idx)
        return ("dest", None)

    def delete_selected_point(self) -> None:
        if not self.start and not self._all_points():
            return
        self._push_undo()
        sel = self._selected_point()
        if sel is None:
            if self.dest:
                self.dest = None
            elif self.waypoints:
                self.waypoints.pop()
            self._path_changed()
            return
        kind, idx = sel
        if kind == "start":
            self.start = None
        elif kind == "wp":
            self.waypoints.pop(idx)
        else:
            self.dest = None
        self._path_changed()

    def toggle_selected_gear(self) -> None:
        sel = self._selected_point()
        if not sel or sel[0] == "start":
            return
        self._push_undo()
        w = self.waypoints[sel[1]] if sel[0] == "wp" else self.dest
        w.gear = -w.gear
        self._path_changed()

    def clear_path(self) -> None:
        if self.start or self._all_points():
            self._push_undo()
            self.set_status("경로를 초기화했습니다. (Ctrl+Z 로 되돌리기)")
        self.start = None
        self.waypoints = []
        self.dest = None
        self.tool_var.set(TOOL_START)
        self._path_changed()

    def draw_path(self) -> None:
        c = self.canvas
        c.delete("path")
        if not self.start:
            return
        tags = ("world", "path", "overlay")
        sx, sy, sh = self.start
        # 출발 위치 차량 윤곽 (출발점 = 앞축 중심)
        try:
            v = self.vehicle
            rx, ry = sx - v.wheelbase * math.cos(sh), sy - v.wheelbase * math.sin(sh)
            for poly in body_polygons(v, rx, ry, sh, sh).values():
                c.create_polygon(self._flat(poly), outline="#0070ff", fill="", width=2, tags=tags)
        except Exception:
            pass
        cx, cy = self.w2c(sx, sy)
        L = self._arrow_len()
        hx, hy = self.w2c(sx + L * math.cos(sh), sy + L * math.sin(sh))
        c.create_line(cx, cy, hx, hy, arrow=tk.LAST, width=2, fill="#0070ff", tags=tags)
        c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#0070ff", outline="", tags=tags)
        c.create_text(cx + 8, cy - 12, text="출발", fill="#0070ff", anchor="w", tags=tags)
        prev = (sx, sy)
        pts = self._all_points()
        for i, w in enumerate(pts):
            color = COLOR_PATH_FWD if w.gear == FORWARD else COLOR_PATH_REV
            dash = () if w.gear == FORWARD else (6, 4)
            c.create_line(self._flat([prev, (w.x, w.y)]), fill=color, width=2, dash=dash, tags=tags)
            px, py = self.w2c(w.x, w.y)
            is_dest = w is self.dest
            cusp = i + 1 < len(pts) and pts[i + 1].gear != w.gear
            r = 6 if is_dest else 4
            c.create_oval(px - r, py - r, px + r, py + r, fill=color, outline="black" if cusp else "",
                          width=2, tags=tags)
            label = "목적지" if is_dest else (f"{i + 1} 전환" if cusp else str(i + 1))
            c.create_text(px + 8, py - 10, text=label, fill=color, anchor="w", tags=tags)
            if w.heading is not None:
                ex, ey = self.w2c(w.x + L * math.cos(w.heading), w.y + L * math.sin(w.heading))
                c.create_line(px, py, ex, ey, arrow=tk.LAST, width=2, fill=color, tags=tags)
            prev = (w.x, w.y)
        sel = self._selected_point()
        if sel is not None:
            try:
                px, py = self.w2c(*self._point_xy(*sel))
                c.create_oval(px - 11, py - 11, px + 11, py + 11, outline="#ffb000", width=3, tags=tags)
            except (IndexError, AttributeError, TypeError):
                pass
        c.tag_raise("overlay")

    # ==================================================================
    # 실행
    # ==================================================================
    def _options(self) -> tuple[SimOptions, CheckOptions]:
        try:
            vals = {k: float(v.get()) for k, v in self.opt_vars.items()}
        except (tk.TclError, ValueError):
            raise ValueError("검토 조건에 숫자를 입력하세요.")
        sim = SimOptions(
            speed_kmh=vals["speed"], reverse_speed_kmh=vals["rev_speed"],
            position_tol=vals["pos_tol"], heading_tol_deg=vals["head_tol"],
            stationary_steer=bool(self.stationary_var.get()),
        )
        chk = CheckOptions(clearance=max(vals["clearance"], 0.0))
        return sim, chk

    def _obstacles(self):
        if not self.drawing:
            return [], None
        key = (frozenset(self.obstacle_layers), self.drawing.unit_scale)
        if self._obstacle_cache and self._obstacle_cache[0] == key:
            return self._obstacle_cache[1]
        lines, tree = build_obstacles(self.drawing.polylines_m(self.obstacle_layers))
        self._obstacle_cache = (key, (lines, tree))
        return lines, tree

    def run_check(self) -> None:
        if self._busy:
            return
        sc = self.scenario()
        if sc is None:
            messagebox.showinfo("안내", "출발점과 목적지(또는 경유점)를 지정하세요.")
            return
        try:
            sim_opt, chk_opt = self._options()
            vehicle = self.vehicle
            errs = vehicle.validate()
            if errs:
                raise ValueError("\n".join(errs))
        except ValueError as exc:
            messagebox.showerror("입력 오류", str(exc))
            return
        self._busy = True
        self.run_btn.config(state="disabled", text="검토 중...")
        self.set_status("검토 중...")

        q: queue.Queue = queue.Queue()

        def work():
            try:
                lines, tree = self._obstacles()
                sim = Simulator(vehicle, sim_opt).run(sc)
                q.put(("ok", analyze(vehicle, sim, lines, tree, chk_opt)))
            except Exception as exc:
                traceback.print_exc()
                q.put(("error", f"{type(exc).__name__}: {exc}"))

        def poll():
            try:
                kind, payload = q.get_nowait()
            except queue.Empty:
                self.root.after(50, poll)
                return
            if kind == "ok":
                self._on_result(payload, sc)
            else:
                self._on_error(payload)

        threading.Thread(target=work, daemon=True).start()
        self.root.after(50, poll)

    def _on_error(self, msg: str) -> None:
        self._busy = False
        self.run_btn.config(state="normal", text="▶  회전 검토 실행   (Ctrl+Enter)")
        self.set_status("검토 실패")
        messagebox.showerror("오류", msg)

    def _on_result(self, result: Analysis, sc: Scenario) -> None:
        self._busy = False
        self.run_btn.config(state="normal", text="▶  회전 검토 실행   (Ctrl+Enter)")
        self.analysis = result
        self.analysis_scenario = sc
        self._show_result_summary(result)
        self.scrub.config(to=max(result.sim.total_distance, 0.01))
        self.scrub_var.set(0.0)
        self.draw_results()
        self.nb.select(3)
        verdict = "회전 가능" if result.feasible else "회전 불가"
        self.verdict_label.config(text=f"  {'✔' if result.feasible else '✖'} {verdict}  ", fg="white",
                                  bg=SUCCESS if result.feasible else DANGER)
        self._update_steps()
        self.set_status(f"검토 완료: {verdict}")

    def _show_result_summary(self, a: Analysis) -> None:
        ok = a.feasible
        warn = bool(a.warnings)
        if ok:
            text, fg, bg = ("✔ 회전 가능" + ("  (주의 있음)" if warn else ""), SUCCESS, SUCCESS_BG)
        else:
            text, fg, bg = ("✖ 회전 불가", DANGER, DANGER_BG)
        self.verdict_card.config(text=text, fg=fg, bg=bg)
        v, sim = a.vehicle, a.sim
        metrics = [
            ("차량", v.name),
            ("총 주행거리", f"{sim.total_distance:.1f} m"),
            ("구간 / 방향전환", f"{len(sim.legs)}구간 / {max(len(sim.legs) - 1, 0)}회"),
            ("사용 최대 조향각", f"{a.max_steer_deg:.1f}° (한계 {math.degrees(v.max_steer_rad):.1f}°)"),
            ("충돌·여유폭 부족", f"{len(a.collisions)}곳"),
        ]
        if v.is_articulated:
            metrics.append(("최대 꺾임각", f"{a.max_articulation_deg:.1f}° (한계 {v.max_articulation_deg:.0f}°)"))
        for w in self.metric_frame.winfo_children():
            w.destroy()
        for r, (k, val) in enumerate(metrics):
            ttk.Label(self.metric_frame, text=k, style="CardMuted.TLabel").grid(row=r, column=0, sticky="w", pady=1)
            lbl = ttk.Label(self.metric_frame, text=val, style="Value.TLabel")
            lbl.grid(row=r, column=1, sticky="w", padx=(10, 0))
            if k.startswith("충돌") and a.collisions:
                lbl.configure(foreground=DANGER)
        t = self.result_text
        t.config(state="normal")
        t.delete("1.0", tk.END)
        if not a.reasons and not a.warnings:
            t.insert(tk.END, "문제없이 목적지까지 도달했습니다.\n", "muted")
        for r in a.reasons:
            t.insert(tk.END, f"✖ {r}\n", "bad")
        for w in a.warnings:
            t.insert(tk.END, f"⚠ {w}\n", "warn")
        t.insert(tk.END, "구간별 결과\n", "head")
        for leg in sim.legs:
            gear = "전진" if leg.gear == FORWARD else "후진"
            extra = f", 방향오차 {leg.heading_error_deg:.1f}°" if leg.heading_error_deg is not None else ""
            line = (f"{leg.index + 1}. {gear} {leg.length:.1f} m — {'도달' if leg.reached else '실패'} "
                    f"(도착오차 {leg.end_error:.2f} m, 최대 경로이탈 {leg.max_deviation:.2f} m{extra})")
            t.insert(tk.END, line + (f" · {leg.message}" if leg.message else "") + "\n",
                     () if leg.reached else ("bad",))
        if a.collisions:
            t.insert(tk.END, "충돌 위치\n", "head")
            for i, c in enumerate(a.collisions, 1):
                part = "트레일러" if c.part == "trailer" else "차체"
                t.insert(tk.END, f"{i}. {part}  x={c.x:,.2f}, y={c.y:,.2f}  (주행 {c.dist_from:.1f}~{c.dist_to:.1f} m)\n")
        t.config(state="disabled")

    def clear_results(self) -> None:
        had = self.analysis is not None
        self.analysis = None
        self.analysis_scenario = None
        if hasattr(self, "canvas"):
            self.canvas.delete("result")
            self.canvas.delete("scrub")
            self.verdict_label.config(text="", bg="white")
        if had and hasattr(self, "verdict_card"):
            self.verdict_card.config(text="조건이 바뀌어 다시 검토가 필요합니다", fg=MUTED, bg="#e5e7eb")
            for w in self.metric_frame.winfo_children():
                w.destroy()
            self.result_text.config(state="normal")
            self.result_text.delete("1.0", tk.END)
            self.result_text.config(state="disabled")
            self.scrub_label.set("")
        self._update_steps()

    def draw_results(self) -> None:
        c = self.canvas
        c.delete("result")
        a = self.analysis
        if a is None:
            return
        tags = ("world", "result", "overlay")
        show = {k: v.get() for k, v in self.show_vars.items()}

        def polys(geom):
            if geom is None or geom.is_empty:
                return []
            return list(getattr(geom, "geoms", [geom]))

        if show["clearance"]:
            for g in polys(a.clearance_envelope):
                c.create_polygon(self._flat(g.exterior.coords), outline=COLOR_CLEAR, fill="", dash=(4, 3),
                                 tags=tags)
        if show["envelope"]:
            for g in polys(a.envelope):
                c.create_polygon(self._flat(g.exterior.coords), outline=COLOR_ENVELOPE, fill="#cfe0ff",
                                 stipple="gray25", width=2, tags=tags)
        if show["snapshots"]:
            for snap in a.snapshots:
                for poly in snap.values():
                    c.create_polygon(self._flat(poly), outline=COLOR_SNAP, fill="", tags=tags)
        for name, track in a.tracks.items():
            is_wheel = "wheel" in name
            if (is_wheel and show["wheels"]) or (not is_wheel and show["corners"]):
                step = max(1, len(track) // 3000)
                c.create_line(self._flat(track[::step]), fill=COLOR_WHEEL if is_wheel else COLOR_CORNER,
                              tags=tags)
        for i, col in enumerate(a.collisions, 1):
            px, py = self.w2c(col.x, col.y)
            c.create_oval(px - 10, py - 10, px + 10, py + 10, outline=COLOR_COLLISION, width=3, tags=tags)
            c.create_text(px + 12, py + 12, text=f"충돌{i}", fill=COLOR_COLLISION, anchor="w",
                          font=("", 10, "bold"), tags=tags)
        c.tag_raise("path")
        c.tag_raise("result")
        self.draw_scrub()

    def draw_scrub(self) -> None:
        c = self.canvas
        c.delete("scrub")
        a = self.analysis
        if a is None or len(a.sim.dist) == 0:
            return
        d = float(self.scrub_var.get())
        i = int(np.clip(np.searchsorted(a.sim.dist, d), 0, len(a.sim.dist) - 1))
        sim = a.sim
        for poly in body_polygons(a.vehicle, sim.x[i], sim.y[i], sim.theta[i], sim.phi[i]).values():
            c.create_polygon(self._flat(poly), outline="black", fill="#ffe08a", width=2,
                             tags=("world", "scrub", "overlay"))
        gear = "전진" if sim.gear[i] == FORWARD else "후진"
        msg = f"주행 {sim.dist[i]:.1f} / {sim.total_distance:.1f} m · {gear} · 조향 {math.degrees(sim.steer[i]):+.1f}°"
        self.scrub_label.set(msg)
        self.set_status(msg)

    # ==================================================================
    # 저장
    # ==================================================================
    def export_dxf(self, with_base: bool) -> None:
        if not self.analysis:
            messagebox.showinfo("안내", "먼저 회전 검토를 실행하세요.")
            return
        base = None
        scale = 1.0
        if self.drawing:
            scale = self.drawing.unit_scale
            if with_base:
                base = self.drawing.path
        elif with_base:
            messagebox.showinfo("안내", "열린 도면이 없어 결과만 저장합니다.")
        default = (Path(base).stem + "_회전검토.dxf") if base else "회전검토결과.dxf"
        path = filedialog.asksaveasfilename(defaultextension=".dxf", initialfile=default,
                                            filetypes=[("DXF", "*.dxf")])
        if not path:
            return
        try:
            export_dxf(self.analysis, self.analysis_scenario, path, unit_scale=scale, base_drawing=base)
        except Exception as exc:
            traceback.print_exc()
            messagebox.showerror("저장 실패", f"{type(exc).__name__}: {exc}")
            return
        self.set_status(f"저장했습니다: {path}")

    def export_report(self) -> None:
        if not self.analysis:
            messagebox.showinfo("안내", "먼저 회전 검토를 실행하세요.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".txt", initialfile="회전검토보고서.txt",
                                            filetypes=[("텍스트", "*.txt")])
        if not path:
            return
        name = self.drawing.path.name if self.drawing else ""
        export_report(self.analysis, path, name)
        self.set_status(f"저장했습니다: {path}")

    def save_project(self) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="회전검토작업.json",
                                            filetypes=[("작업 파일", "*.json")])
        if not path:
            return
        sc = self.scenario()
        data = {
            "drawing": str(self.drawing.path) if self.drawing else None,
            "unit": self.unit_var.get(),
            "obstacle_layers": sorted(self.obstacle_layers),
            "vehicle": self.vehicle.to_dict(),
            "start": list(self.start) if self.start else None,
            "scenario": scenario_to_dict(sc) if sc else None,
            "has_dest": self.dest is not None,
            "options": {k: float(v.get()) for k, v in self.opt_vars.items()},
            "stationary_steer": bool(self.stationary_var.get()),
        }
        save_project(path, data)
        self.set_status(f"작업을 저장했습니다: {path}")

    def load_project(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("작업 파일", "*.json")])
        if not path:
            return
        try:
            data = load_project(path)
            if data.get("drawing") and Path(data["drawing"]).exists():
                self.open_drawing(data["drawing"], wait=True)
                if data.get("unit") in UNIT_CHOICES and self.drawing:
                    self.unit_var.set(data["unit"])
                    self.drawing.unit_scale = UNIT_CHOICES[data["unit"]]
                    self._dwg_base = None
                self.obstacle_layers = set(data.get("obstacle_layers", [])) & set(self.layer_names)
                self._fill_layer_list()
            elif data.get("drawing"):
                messagebox.showwarning("안내", f"도면 파일을 찾을 수 없습니다:\n{data['drawing']}")
            vdata = data.get("vehicle")
            if vdata:
                key = vdata.get("key")
                if key not in self.library.vehicles:
                    self.library.add_or_replace(Vehicle.from_dict(vdata))
                self._refresh_vehicle_list(key)
            for k, v in data.get("options", {}).items():
                if k in self.opt_vars:
                    self.opt_vars[k].set(v)
            self.stationary_var.set(data.get("stationary_steer", True))
            self.start = tuple(data["start"]) if data.get("start") else None
            self.waypoints, self.dest = [], None
            if data.get("scenario"):
                sc = scenario_from_dict(data["scenario"])
                wps = sc.waypoints
                if data.get("has_dest") and wps:
                    self.dest = wps[-1]
                    wps = wps[:-1]
                self.waypoints = wps
            self._obstacle_cache = None
            self._path_changed()
            self.fit_view()
        except Exception as exc:
            traceback.print_exc()
            messagebox.showerror("불러오기 실패", f"{type(exc).__name__}: {exc}")

    def set_status(self, text: str) -> None:
        self.status.set(text)


class PointDialog(tk.Toplevel):
    """점 좌표·방향·진행방향 숫자 입력 수정."""

    def __init__(self, master, app: "SweptPathApp", kind: str, idx):
        super().__init__(master)
        self.configure(bg=BG)
        self.app, self.kind, self.idx = app, kind, idx
        names = {"start": "출발점", "dest": "목적지"}
        self.title(f"{names.get(kind, f'{idx + 1}번 경유점')} 수정")
        self.resizable(False, False)
        self.transient(master)
        x, y = app._point_xy(kind, idx)
        heading = app._point_heading(kind, idx)
        frm = ttk.Frame(self, padding=10)
        frm.pack()
        self.vx = tk.StringVar(value=f"{x:.3f}")
        self.vy = tk.StringVar(value=f"{y:.3f}")
        self.vh = tk.StringVar(value="" if heading is None else f"{math.degrees(heading):.1f}")
        ttk.Label(frm, text="x (m)").grid(row=0, column=0, sticky="w")
        ttk.Entry(frm, textvariable=self.vx, width=14).grid(row=0, column=1, pady=2)
        ttk.Label(frm, text="y (m)").grid(row=1, column=0, sticky="w")
        ttk.Entry(frm, textvariable=self.vy, width=14).grid(row=1, column=1, pady=2)
        row = 2
        if kind in ("start", "dest"):
            label = "차량 방향 (°, 동쪽=0, 반시계+)" + ("" if kind == "start" else "\n비우면 방향 지정 안 함")
            ttk.Label(frm, text=label, justify="left").grid(row=row, column=0, sticky="w")
            ttk.Entry(frm, textvariable=self.vh, width=14).grid(row=row, column=1, pady=2)
            row += 1
        self.vg = None
        if kind != "start":
            w = app.waypoints[idx] if kind == "wp" else app.dest
            self.vg = tk.IntVar(value=w.gear)
            gf = ttk.Frame(frm)
            gf.grid(row=row, column=0, columnspan=2, sticky="w", pady=2)
            ttk.Label(gf, text="이 점까지").pack(side=tk.LEFT)
            ttk.Radiobutton(gf, text="전진", value=FORWARD, variable=self.vg).pack(side=tk.LEFT, padx=4)
            ttk.Radiobutton(gf, text="후진", value=REVERSE, variable=self.vg).pack(side=tk.LEFT)
            row += 1
        btns = ttk.Frame(frm)
        btns.grid(row=row, column=0, columnspan=2, sticky="e", pady=(8, 0))
        ttk.Button(btns, text="적용", command=self._apply).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="취소", command=self.destroy).pack(side=tk.LEFT)
        self.bind("<Return>", lambda e: self._apply())
        self.bind("<Escape>", lambda e: self.destroy())
        self.grab_set()

    def _apply(self) -> None:
        try:
            x, y = float(self.vx.get()), float(self.vy.get())
            raw = self.vh.get().strip()
            heading = None if raw == "" else float(raw)
        except ValueError:
            messagebox.showerror("입력 오류", "숫자를 입력하세요.", parent=self)
            return
        if self.kind == "start" and heading is None:
            messagebox.showerror("입력 오류", "출발점은 차량 방향이 필요합니다.", parent=self)
            return
        self.app.apply_point_values(self.kind, self.idx, x, y, heading,
                                    self.vg.get() if self.vg is not None else None)
        self.destroy()


class VehicleDialog(tk.Toplevel):
    def __init__(self, master, vehicle: Vehicle, on_save):
        super().__init__(master)
        self.configure(bg=BG)
        self.title("차량 제원 편집")
        self.resizable(False, False)
        self.transient(master)
        self.vehicle = vehicle
        self.on_save = on_save
        frm = ttk.Frame(self, padding=10)
        frm.pack(fill=tk.BOTH, expand=True)

        ttk.Label(frm, text="차량 형식").grid(row=0, column=0, sticky="w")
        self.kind_var = tk.StringVar(value="세미트레일러" if vehicle.is_articulated else "단일차량")
        ttk.Combobox(frm, textvariable=self.kind_var, values=["단일차량", "세미트레일러"], state="readonly",
                     width=14).grid(row=0, column=1, sticky="w", pady=2)
        self.vars: dict[str, tk.StringVar] = {}
        for i, (name, label, _typ) in enumerate(VEHICLE_FIELDS, start=1):
            ttk.Label(frm, text=label).grid(row=i, column=0, sticky="w")
            val = getattr(vehicle, name)
            if name == "name" and vehicle.builtin:
                val = f"{vehicle.name} (수정)"
            var = tk.StringVar(value="" if val is None else str(val))
            ttk.Entry(frm, textvariable=var, width=30 if name == "name" else 12).grid(
                row=i, column=1, sticky="w", pady=1)
            self.vars[name] = var
        ttk.Label(frm, foreground="#666", wraplength=420, justify="left",
                  text="기본 차량을 수정하면 새 '사용자 차량'으로 저장됩니다. "
                       "[세미] 항목은 세미트레일러에서만 사용합니다.").grid(
            row=len(VEHICLE_FIELDS) + 1, column=0, columnspan=2, sticky="w", pady=6)
        btns = ttk.Frame(frm)
        btns.grid(row=len(VEHICLE_FIELDS) + 2, column=0, columnspan=2, sticky="e")
        ttk.Button(btns, text="저장", command=self._save).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="취소", command=self.destroy).pack(side=tk.LEFT)
        self.grab_set()

    def _save(self) -> None:
        data = self.vehicle.to_dict()
        try:
            for name, label, typ in VEHICLE_FIELDS:
                raw = self.vars[name].get().strip()
                if typ is float:
                    if name == "max_steer_deg" and raw == "":
                        data[name] = None
                    else:
                        data[name] = float(raw)
                else:
                    data[name] = raw
        except ValueError:
            messagebox.showerror("입력 오류", f"'{label}' 값이 숫자가 아닙니다.", parent=self)
            return
        data["kind"] = SEMITRAILER if self.kind_var.get() == "세미트레일러" else SINGLE
        if data["kind"] == SEMITRAILER:
            for k, default in (("trailer_wheelbase", 9.0), ("trailer_width", data["width"]),
                               ("trailer_track", data["track"])):
                if not data.get(k):
                    data[k] = default
        if not data["name"]:
            messagebox.showerror("입력 오류", "차량 이름을 입력하세요.", parent=self)
            return
        if self.vehicle.builtin:
            data["key"] = f"user_{int(time.time())}"
        vehicle = Vehicle.from_dict(data)
        errors = vehicle.validate()
        if errors:
            messagebox.showerror("제원 오류", "\n".join(errors), parent=self)
            return
        self.on_save(vehicle)
        self.destroy()


def main() -> None:
    multiprocessing.freeze_support()  # 실행 파일(exe)에서 별도 프로세스 사용
    if "--selftest" in sys.argv:
        from selftest import run_selftest

        sys.exit(run_selftest(SAMPLE_DRAWING, APP_DIR, SweptPathApp))
    enable_dpi_awareness()  # 고해상도 화면에서 글자 선명하게 (Tk 생성 전에)
    root = tk.Tk()
    app = SweptPathApp(root)
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if args:
        root.after(100, lambda: app.open_drawing(args[0]))
    root.mainloop()


if __name__ == "__main__":
    main()
