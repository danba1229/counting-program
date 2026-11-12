"""화면 디자인: 색상·글꼴·위젯 스타일, 차량 평면도 그림."""

from __future__ import annotations

import sys
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

# ---------------- 색상 ----------------
BG = "#f3f4f6"  # 창 배경
CARD = "#ffffff"  # 카드(구역) 배경
BORDER = "#d9dce1"
TEXT = "#1f2937"
MUTED = "#6b7280"
PRIMARY = "#2563eb"
PRIMARY_DARK = "#1d4ed8"
PRIMARY_LIGHT = "#dbeafe"
SUCCESS = "#15803d"
SUCCESS_BG = "#dcfce7"
DANGER = "#dc2626"
DANGER_BG = "#fee2e2"
WARN = "#b45309"
WARN_BG = "#fef3c7"
TOOLBAR = "#ffffff"


def enable_dpi_awareness() -> None:
    """Windows 고해상도 화면에서 글자가 흐리게 늘어나지 않도록 한다. (Tk 생성 전에 호출)"""
    if sys.platform != "win32":
        return
    try:
        import ctypes

        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass


def _family(root) -> str:
    families = set(tkfont.families(root))
    for name in ("Malgun Gothic", "맑은 고딕", "Apple SD Gothic Neo", "Noto Sans CJK KR", "NanumGothic",
                 "Segoe UI"):
        if name in families:
            return name
    return tkfont.nametofont("TkDefaultFont").actual("family")


def apply_theme(root: tk.Tk) -> dict[str, tuple]:
    """ttk 스타일 적용. 자주 쓰는 글꼴 묶음을 돌려준다."""
    fam = _family(root)
    fonts = {
        "base": (fam, 10),
        "small": (fam, 9),
        "bold": (fam, 10, "bold"),
        "title": (fam, 11, "bold"),
        "header": (fam, 13, "bold"),
        "big": (fam, 18, "bold"),
        "mono": ("Consolas" if sys.platform == "win32" else "DejaVu Sans Mono", 9),
    }
    for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
        try:
            tkfont.nametofont(name).configure(family=fam, size=10)
        except tk.TclError:
            pass
    root.configure(bg=BG)
    root.option_add("*Font", fonts["base"])
    root.option_add("*Listbox.font", fonts["base"])
    root.option_add("*Listbox.selectBackground", PRIMARY_LIGHT)
    root.option_add("*Listbox.selectForeground", TEXT)
    root.option_add("*Listbox.relief", "flat")
    root.option_add("*Listbox.highlightThickness", 1)
    root.option_add("*Listbox.highlightColor", BORDER)
    root.option_add("*Listbox.highlightBackground", BORDER)

    st = ttk.Style(root)
    try:
        st.theme_use("clam")
    except tk.TclError:
        pass
    st.configure(".", background=BG, foreground=TEXT, font=fonts["base"], bordercolor=BORDER,
                 focuscolor=PRIMARY_LIGHT)
    st.configure("TFrame", background=BG)
    st.configure("Card.TFrame", background=CARD, relief="solid", borderwidth=1)
    st.configure("Toolbar.TFrame", background=TOOLBAR)
    st.configure("Inner.TFrame", background=CARD, borderwidth=0)
    st.configure("TLabel", background=BG, foreground=TEXT)
    st.configure("Card.TLabel", background=CARD)
    st.configure("Muted.TLabel", background=BG, foreground=MUTED, font=fonts["small"])
    st.configure("CardMuted.TLabel", background=CARD, foreground=MUTED, font=fonts["small"])
    st.configure("CardTitle.TLabel", background=CARD, foreground=TEXT, font=fonts["title"])
    st.configure("Value.TLabel", background=CARD, foreground=TEXT, font=fonts["bold"])
    st.configure("Toolbar.TLabel", background=TOOLBAR, foreground=MUTED, font=fonts["small"])
    st.configure("Hint.TLabel", background=PRIMARY_LIGHT, foreground=PRIMARY_DARK, font=fonts["small"],
                 padding=(10, 4))
    st.configure("Status.TLabel", background="#e5e7eb", foreground=TEXT, font=fonts["small"], padding=(8, 3))

    st.configure("TButton", padding=(10, 5), background="#f9fafb", bordercolor="#c5cad3", relief="solid",
                 borderwidth=1, lightcolor="#f9fafb", darkcolor="#f9fafb")
    st.map("TButton", background=[("pressed", "#e5e7eb"), ("active", "#eef2ff"), ("disabled", "#f3f4f6")],
           bordercolor=[("active", PRIMARY)], foreground=[("disabled", "#9ca3af")])
    st.configure("Accent.TButton", background=PRIMARY, foreground="#ffffff", bordercolor=PRIMARY_DARK,
                 lightcolor=PRIMARY, darkcolor=PRIMARY, font=fonts["title"], padding=(12, 9))
    st.map("Accent.TButton", background=[("pressed", PRIMARY_DARK), ("active", PRIMARY_DARK),
                                         ("disabled", "#93c5fd")], bordercolor=[("active", PRIMARY_DARK)],
           foreground=[("disabled", "#eff6ff")])
    st.configure("Small.TButton", padding=(8, 3), font=fonts["small"])

    # 도구 막대 토글 버튼 (선택된 도구는 파란색)
    st.configure("Tool.TRadiobutton", background="#ffffff", foreground=TEXT, padding=(10, 5),
                 indicatorthickness=0, indicatorsize=0, indicatormargin=0, relief="flat",
                 bordercolor=BORDER, borderwidth=1, font=fonts["base"])
    st.layout("Tool.TRadiobutton", [("Radiobutton.border", {"sticky": "nswe", "children": [
        ("Radiobutton.padding", {"sticky": "nswe", "children": [
            ("Radiobutton.label", {"sticky": "nswe"})]})]})])
    st.map("Tool.TRadiobutton",
           background=[("selected", PRIMARY), ("active", "#eef2ff")],
           foreground=[("selected", "#ffffff")])

    st.configure("TCheckbutton", background=BG)
    st.configure("Card.TCheckbutton", background=CARD)
    st.configure("Card.TRadiobutton", background=CARD)
    st.configure("TRadiobutton", background=BG)
    st.configure("TLabelframe", background=CARD, bordercolor=BORDER, relief="solid", borderwidth=1)
    st.configure("TLabelframe.Label", background=CARD, foreground=TEXT, font=fonts["title"])
    st.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(2, 4, 2, 0))
    st.configure("TNotebook.Tab", padding=(12, 6), background="#e5e7eb", foreground=MUTED, font=fonts["bold"])
    st.map("TNotebook.Tab", background=[("selected", CARD)], foreground=[("selected", PRIMARY_DARK)])
    st.configure("TEntry", padding=4, fieldbackground="#ffffff")
    st.configure("TCombobox", padding=4)
    st.configure("Treeview", rowheight=26, fieldbackground="#ffffff", background="#ffffff",
                 bordercolor=BORDER)
    st.configure("Treeview.Heading", font=fonts["bold"], background="#f3f4f6", relief="flat")
    st.map("Treeview", background=[("selected", PRIMARY_LIGHT)], foreground=[("selected", TEXT)])
    st.configure("Horizontal.TScale", background=CARD)
    st.configure("TPanedwindow", background=BG)
    return fonts


def card(parent, title: str | None = None, pad: int = 10) -> ttk.Frame:
    """흰색 카드 구역. title 이 있으면 제목을 붙이고 내용 프레임을 돌려준다."""
    outer = ttk.Frame(parent, style="Card.TFrame", padding=pad)
    if title:
        ttk.Label(outer, text=title, style="CardTitle.TLabel").pack(anchor="w", pady=(0, 6))
    return outer


# ---------------- 차량 평면도 ----------------
def draw_vehicle_diagram(canvas: tk.Canvas, v, fonts) -> None:
    """차량 평면도와 주요 치수 (전장, 축간거리, 내민길이, 폭)."""
    canvas.delete("all")
    w = max(canvas.winfo_width(), 340)
    h = max(canvas.winfo_height(), 150)
    total = v.overall_length
    width = v.trailer_width if v.is_articulated else v.width
    scale = min((w - 70) / max(total, 0.1), (h - 70) / max(width, 0.1))
    x0 = (w - 30 - total * scale) / 2  # 오른쪽 30 px 은 진행 방향 화살표 자리
    yc = h / 2 - 8
    small = fonts["small"]

    def X(d):  # 앞범퍼로부터 거리 → 화면 x (앞이 오른쪽)
        return x0 + (total - d) * scale

    def wheel(d, half_track):
        for sgn in (-1, 1):
            cy = yc + sgn * half_track * scale
            canvas.create_rectangle(X(d) - 0.45 * scale, cy - 0.14 * scale, X(d) + 0.45 * scale,
                                    cy + 0.14 * scale, fill="#374151", outline="")

    def body(d0, d1, half_w, color):
        canvas.create_rectangle(X(d1), yc - half_w * scale, X(d0), yc + half_w * scale,
                                fill=color, outline="#1e3a8a", width=2)

    front_axle = v.front_overhang
    rear_axle = v.front_overhang + v.wheelbase
    if v.is_articulated:
        hitch = rear_axle + v.hitch_offset
        trailer_axle = hitch + v.trailer_wheelbase
        body(hitch - v.trailer_front_overhang, trailer_axle + v.trailer_rear_overhang,
             v.trailer_width / 2, "#e0e7ff")
        body(0, rear_axle + v.rear_overhang, v.width / 2, PRIMARY_LIGHT)
        wheel(trailer_axle, v.trailer_track / 2)
        canvas.create_oval(X(hitch) - 4, yc - 4, X(hitch) + 4, yc + 4, fill="#f59e0b", outline="")
    else:
        body(0, total, v.width / 2, PRIMARY_LIGHT)
    wheel(front_axle, v.track / 2)
    wheel(rear_axle, v.track / 2)
    # 진행 방향 화살표
    canvas.create_line(X(0) + 6, yc, X(0) + 26, yc, arrow=tk.LAST, width=2, fill=PRIMARY)
    canvas.create_text(X(0) + 16, yc - 12, text="앞", fill=PRIMARY, font=small)

    # 치수선
    def dim(d0, d1, y, label):
        xa, xb = X(d1), X(d0)
        canvas.create_line(xa, y, xb, y, arrow=tk.BOTH, fill=MUTED)
        canvas.create_text((xa + xb) / 2, y + 9, text=label, fill=TEXT, font=small)

    ybot = yc + width / 2 * scale + 14
    dim(0, total, ybot + 18, f"전장 {total:.2f} m")
    if v.is_articulated:
        dim(front_axle, rear_axle, ybot, f"축간 {v.wheelbase:.1f}")
        dim(hitch, trailer_axle, ybot, f"킹핀~트레일러축 {v.trailer_wheelbase:.1f}")
    else:
        dim(0, front_axle, ybot, f"{v.front_overhang:.2f}")
        dim(front_axle, rear_axle, ybot, f"축간거리 {v.wheelbase:.2f}")
        dim(rear_axle, total, ybot, f"{v.rear_overhang:.2f}")
    ytop = yc - width / 2 * scale
    canvas.create_text(X(total) - 4, ytop - 10, text=f"폭 {v.width:.2f} m", fill=TEXT, font=small, anchor="w")
