# -*- mode: python ; coding: utf-8 -*-
# PyInstaller 빌드 설정:  pyinstaller --noconfirm SweptPathChecker.spec
# 결과: dist/SweptPathChecker/SweptPathChecker.exe (폴더째 배포)

from PyInstaller.utils.hooks import collect_data_files

datas = [("samples/sample_parking.dxf", "samples")]
# ezdxf 리소스만 포함 (C 소스 등 빌드용 파일 제외)
datas += collect_data_files("ezdxf", excludes=["**/*.c", "**/*.pyx", "**/*.pxd", "**/*.h"])
hiddenimports = ["ezdxf.addons.odafc", "selftest"]

a = Analysis(
    ["app.py"],
    pathex=["."],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    runtime_hooks=[],
    excludes=["matplotlib", "PyQt5", "PyQt6", "PySide6", "IPython", "pytest", "scipy", "pandas"],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="SweptPathChecker",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,  # 창 모드 (검은 콘솔 창 없음)
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    name="SweptPathChecker",
)
