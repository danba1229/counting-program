"""GstarCAD 로 DWG → DXF 변환 (Windows 전용, 추가 설치 불필요).

PC 에 GstarCAD 가 설치되어 있으면 COM 자동화(GStarCAD.Application)로
DWG 를 읽기 전용으로 열어 DXF 로 저장한다. 별도 Python 패키지(pywin32) 없이
Windows 기본 PowerShell 로 COM 을 호출한다.

- GstarCAD 가 이미 열려 있으면 그 창에 변환용 도면을 읽기 전용으로 잠깐 열었다가 닫는다.
  사용자가 열어 둔 도면은 건드리지 않고, 프로그램도 종료하지 않는다.
- GstarCAD 가 꺼져 있으면 화면에 보이지 않게 띄워 변환한 뒤,
  이 변환이 띄운 프로세스만 종료한다.
"""

from __future__ import annotations

import base64
import os
from pathlib import Path
import subprocess
import sys
import tempfile

PROG_IDS = ("GStarCAD.Application", "Gcad.Application")
DXF_SAVE_TYPE = 61  # AcSaveAsType: ac2013_dxf
TIMEOUT_S = 300

# 경로는 환경변수로 넘긴다 (한글·공백 경로를 안전하게 전달).
_SCRIPT = r"""
$ErrorActionPreference = 'Stop'
function Fail($msg, $code) { [Console]::Error.WriteLine($msg); exit $code }
$src = $env:SPC_DWG_SRC
$dst = $env:SPC_DXF_DST
$saveType = [int]$env:SPC_SAVE_TYPE
$ids = $env:SPC_PROG_IDS -split ','
$app = $null
$ownPids = @()
# 1) 이미 열려 있는 GstarCAD 에 붙는다 (새로 띄우면 라이선스상 실패하는 경우가 있음)
foreach ($id in $ids) {
  try { $app = [System.Runtime.InteropServices.Marshal]::GetActiveObject($id); break } catch { }
}
# 2) 없으면 새로 띄우고, 이 스크립트가 띄운 프로세스만 끝에 종료한다
if ($null -eq $app) {
  $before = @(Get-Process gcad -ErrorAction SilentlyContinue | ForEach-Object { $_.Id })
  $err = ''
  foreach ($id in $ids) {
    try { $app = New-Object -ComObject $id; break } catch { $err = $_.Exception.Message }
  }
  $ownPids = @(Get-Process gcad -ErrorAction SilentlyContinue | Where-Object { $before -notcontains $_.Id } | ForEach-Object { $_.Id })
  if ($null -eq $app) {
    foreach ($p in $ownPids) { Stop-Process -Id $p -Force -ErrorAction SilentlyContinue }
    Fail "GstarCAD 를 실행할 수 없습니다: $err" 3
  }
  try { $app.Visible = $false } catch { }
}
$doc = $null
$failMsg = $null
try {
  $doc = $app.Documents.Open($src, $true)
  $doc.SaveAs($dst, $saveType)
} catch {
  $failMsg = $_.Exception.Message
} finally {
  if ($null -ne $doc) { try { $doc.Close($false) } catch { } }
  if ($ownPids.Count -gt 0) {
    try { $app.Quit() } catch { }
    Start-Sleep -Milliseconds 500
    foreach ($p in $ownPids) { Stop-Process -Id $p -Force -ErrorAction SilentlyContinue }
  }
  try { [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($app) } catch { }
}
if ($failMsg) { Fail $failMsg 2 }
if (-not (Test-Path -LiteralPath $dst)) { Fail 'DXF 파일이 만들어지지 않았습니다.' 4 }
exit 0
"""


class GstarCadError(Exception):
    pass


def is_installed() -> bool:
    """GstarCAD COM 자동화가 등록되어 있는지 확인."""
    if sys.platform != "win32":
        return False
    import winreg

    for pid in PROG_IDS:
        try:
            winreg.CloseKey(winreg.OpenKey(winreg.HKEY_CLASSES_ROOT, pid + r"\CLSID"))
            return True
        except OSError:
            continue
    return False


def convert_to_dxf(dwg: str | Path, dxf: str | Path | None = None, timeout: float = TIMEOUT_S) -> Path:
    """DWG 를 DXF 로 변환해 그 경로를 돌려준다. dxf 를 생략하면 임시 폴더에 만든다."""
    dwg = Path(dwg).resolve()
    if dxf is None:
        dxf = Path(tempfile.mkdtemp(prefix="spc_gstar_")) / (dwg.stem + ".dxf")
    dxf = Path(dxf).resolve()
    dxf.unlink(missing_ok=True)
    env = dict(os.environ, SPC_DWG_SRC=str(dwg), SPC_DXF_DST=str(dxf),
               SPC_SAVE_TYPE=str(DXF_SAVE_TYPE), SPC_PROG_IDS=",".join(PROG_IDS))
    encoded = base64.b64encode(_SCRIPT.encode("utf-16-le")).decode("ascii")
    try:
        proc = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass",
             "-EncodedCommand", encoded],
            env=env, capture_output=True, timeout=timeout,
            creationflags=0x08000000,  # 검은 콘솔 창 숨김
        )
    except subprocess.TimeoutExpired as exc:
        raise GstarCadError(f"GstarCAD 변환이 {int(timeout)}초 안에 끝나지 않았습니다.") from exc
    except OSError as exc:
        raise GstarCadError(f"PowerShell 을 실행할 수 없습니다: {exc}") from exc
    if proc.returncode != 0 or not dxf.exists():
        msg = _decode(proc.stderr).strip().splitlines()
        detail = next((m for m in msg if m.strip()), f"종료 코드 {proc.returncode}")
        raise GstarCadError(f"GstarCAD 로 DWG 변환 실패: {detail}")
    return dxf


def _decode(data: bytes) -> str:
    for enc in ("utf-8", "cp949"):
        try:
            return data.decode(enc)
        except UnicodeDecodeError:
            continue
    return data.decode("utf-8", errors="replace")
