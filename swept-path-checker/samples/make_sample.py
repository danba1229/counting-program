"""시험용 샘플 도면(mm 단위) 생성: 지하주차장 통로 + 90° 코너 + 기둥 + 막다른 회차 공간.

    python samples/make_sample.py
"""

from pathlib import Path

import ezdxf


def make(path: Path) -> Path:
    doc = ezdxf.new("R2010", setup=True)
    doc.header["$INSUNITS"] = 4  # mm
    for name, color in (("벽", 7), ("기둥", 2), ("치수", 3), ("주차구획", 8), ("문자", 4)):
        doc.layers.add(name, color=color)

    # 기둥 블록 (블록 내부는 0 레이어 → 삽입 레이어를 따른다)
    col = doc.blocks.new("COL600")
    col.add_lwpolyline([(-300, -300), (300, -300), (300, 300), (-300, 300)], close=True)

    msp = doc.modelspace()
    wall = {"layer": "벽"}
    # 동서 방향 통로 (폭 7.0 m): y = 0 ~ 7000, 통로 북쪽은 기둥열 + 주차구획 + 뒷벽(y=12000)
    # 북쪽으로 꺾이는 통로 (폭 7.0 m): x = 33000 ~ 40000, y = 12000 ~ 35000
    msp.add_lwpolyline([(0, 0), (40000, 0), (40000, 35000)], dxfattribs=wall)
    msp.add_lwpolyline([(0, 12000), (33000, 12000), (33000, 35000)], dxfattribs=wall)
    msp.add_line((0, 0), (0, 12000), dxfattribs=wall)  # 서쪽 끝 벽
    # 코너 바깥 모서리 곡면 벽
    msp.add_arc((38000, 2000), 2000, 270, 360, dxfattribs=wall)

    # 기둥 600x600 (통로 쪽 면이 y = 7000)
    for x in range(6000, 31000, 8100):
        msp.add_blockref("COL600", (x, 7300), dxfattribs={"layer": "기둥"})

    # 주차구획선 (장애물 아님)
    for x in range(1000, 31000, 2500):
        msp.add_line((x, 7600), (x, 12000), dxfattribs={"layer": "주차구획"})
    # 치수/문자 (장애물 아님)
    dim = msp.add_linear_dim(base=(0, -2000), p1=(0, 0), p2=(40000, 0), dimstyle="EZDXF",
                             dxfattribs={"layer": "치수"})
    dim.render()
    msp.add_text("SAMPLE B2 RAMP", height=500, dxfattribs={"layer": "문자"}).set_placement((2000, 3000))
    doc.saveas(path)
    return path


if __name__ == "__main__":
    out = make(Path(__file__).with_name("sample_parking.dxf"))
    print(out)
