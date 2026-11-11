"""차량 회전 궤적 검토 - 시작하기 (더블클릭으로 실행)

exe 다운로드가 브라우저에서 막힐 때 쓰는 실행 방법입니다.
python.org 에서 Python(3.10 이상)을 설치한 뒤 이 파일을 더블클릭하면,
처음 한 번은 필요한 부품을 이 폴더의 .venv 에 설치하고 프로그램을 엽니다.
그다음부터는 바로 열립니다.

명령 프롬프트 점검용:  python 시작하기.pyw --check   (설치 후 자체 점검, 종료코드 0 = 정상)
"""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import threading

HERE = Path(__file__).resolve().parent
VENV = HERE / ".venv"
WIN = sys.platform == "win32"
VENV_PY = VENV / ("Scripts/python.exe" if WIN else "bin/python")
VENV_PYW = VENV / ("Scripts/pythonw.exe" if WIN else "bin/python")
STAMP = VENV / "설치완료.txt"
REQ = HERE / "requirements.txt"
NO_WINDOW = 0x08000000 if WIN else 0  # 검은 콘솔 창 숨김


def installed() -> bool:
    return VENV_PY.exists() and STAMP.exists() and STAMP.read_text(encoding="utf-8") == REQ.read_text(encoding="utf-8")


def install(log) -> None:
    if sys.version_info < (3, 10):
        raise RuntimeError(f"Python 3.10 이상이 필요합니다 (현재 {sys.version.split()[0]}).")
    if not VENV_PY.exists():
        log("가상환경 만드는 중...")
        run([sys.executable, "-m", "venv", str(VENV)], log)
    log("필요한 부품 설치 중... (처음 한 번, 1~3분)")
    run([str(VENV_PY), "-m", "pip", "install", "--disable-pip-version-check", "-r", str(REQ)], log)
    STAMP.write_text(REQ.read_text(encoding="utf-8"), encoding="utf-8")
    log("설치 완료")


def run(cmd: list[str], log) -> None:
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            encoding="utf-8", errors="replace", creationflags=NO_WINDOW)
    assert proc.stdout is not None
    for line in proc.stdout:
        log(line.rstrip())
    if proc.wait() != 0:
        raise RuntimeError(
            "설치 중 오류가 났습니다. 인터넷 연결을 확인하세요.\n"
            "회사망에서 pip 접속이 막혀 있으면 전산 담당자에게 pypi.org 접속 허용을 요청하세요."
        )


def launch() -> None:
    subprocess.Popen([str(VENV_PYW), str(HERE / "app.py")], cwd=str(HERE), creationflags=NO_WINDOW)


def check_mode() -> int:
    """CI/명령줄 점검: 설치 후 프로그램 자체 점검을 실행."""
    if not installed():
        install(print)
    return subprocess.call([str(VENV_PY), str(HERE / "app.py"), "--selftest"], cwd=str(HERE))


def gui_mode() -> None:
    if installed():
        launch()
        return
    import tkinter as tk
    from tkinter import messagebox

    root = tk.Tk()
    root.title("차량 회전 궤적 검토 - 처음 설치")
    root.geometry("640x360")
    tk.Label(root, text="처음 한 번 필요한 부품을 설치합니다. 창을 닫지 마세요.", font=("", 11, "bold")).pack(pady=8)
    text = tk.Text(root, height=16, wrap="word")
    text.pack(fill="both", expand=True, padx=8, pady=8)

    def log(msg: str) -> None:
        root.after(0, lambda: (text.insert("end", msg + "\n"), text.see("end")))

    def work():
        try:
            install(log)
            root.after(0, lambda: (launch(), root.destroy()))
        except Exception as exc:
            msg = str(exc)
            root.after(0, lambda: (messagebox.showerror("설치 실패", msg, parent=root), root.destroy()))

    threading.Thread(target=work, daemon=True).start()
    root.mainloop()


if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(check_mode())
    gui_mode()
