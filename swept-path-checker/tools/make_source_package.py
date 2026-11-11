"""실행 파일(.exe/.bat) 없이 소스만 담은 배포 폴더를 만든다.

    python tools/make_source_package.py <출력 폴더>

git 에 등록된 파일만 복사하므로 build/dist/.venv 같은 작업 파일은 들어가지 않는다.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
EXCLUDE_SUFFIX = {".bat", ".cmd", ".exe"}


def main() -> int:
    out = Path(sys.argv[1]).resolve()
    if out.exists():
        shutil.rmtree(out)
    raw = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT)
    files = [f for f in raw.decode("utf-8").split("\0") if f]
    copied = 0
    for rel in files:
        src = ROOT / rel
        if src.suffix.lower() in EXCLUDE_SUFFIX or not src.is_file():
            continue
        dst = out / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        copied += 1
    launcher = out / "시작하기.pyw"
    if not launcher.exists():
        print("시작하기.pyw 가 없습니다", file=sys.stderr)
        return 1
    print(f"{copied}개 파일 → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
