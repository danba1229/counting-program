"""차량 제원과 기본 차량 목록.

모든 길이 단위는 m, 각도 단위는 도(deg)이다.

최소회전반경은 「도로의 구조·시설 기준에 관한 규칙」과 같이
"바깥쪽 앞바퀴 중심이 그리는 원의 반경"으로 정의한다.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields
import json
import math
from pathlib import Path

SINGLE = "single"
SEMITRAILER = "semitrailer"


@dataclass
class Vehicle:
    key: str
    name: str
    kind: str = SINGLE  # "single" 또는 "semitrailer"
    width: float = 1.7
    height: float = 2.0
    wheelbase: float = 2.7  # 단일차량 축간거리 / 세미트레일러는 트랙터 축간거리
    front_overhang: float = 0.8  # 앞축 ~ 앞범퍼
    rear_overhang: float = 1.2  # 뒷축 ~ 뒷범퍼 (세미트레일러는 트랙터 뒷부분)
    track: float = 1.5  # 좌우 바퀴 중심 간격(윤거)
    min_turn_radius: float = 6.0  # 바깥쪽 앞바퀴 기준 최소회전반경
    max_steer_deg: float | None = None  # 지정하면 최소회전반경 대신 사용
    lock_to_lock_s: float = 4.0  # 왼쪽 끝 ~ 오른쪽 끝까지 조향에 걸리는 시간
    # --- 세미트레일러 전용 ---
    hitch_offset: float = 0.0  # 트랙터 뒷축 기준 킹핀 위치 (+: 뒷축보다 뒤)
    trailer_wheelbase: float = 0.0  # 킹핀 ~ 트레일러 축
    trailer_front_overhang: float = 0.0  # 킹핀 ~ 트레일러 앞면
    trailer_rear_overhang: float = 0.0  # 트레일러 축 ~ 트레일러 뒷면
    trailer_width: float = 0.0
    trailer_track: float = 0.0
    max_articulation_deg: float = 80.0  # 트랙터-트레일러 최대 꺾임각
    note: str = ""
    builtin: bool = False

    # ------------------------------------------------------------------
    @property
    def is_articulated(self) -> bool:
        return self.kind == SEMITRAILER

    @property
    def overall_length(self) -> float:
        if not self.is_articulated:
            return self.front_overhang + self.wheelbase + self.rear_overhang
        tractor_front = self.front_overhang + self.wheelbase + self.hitch_offset
        return tractor_front + self.trailer_wheelbase + self.trailer_rear_overhang

    @property
    def max_steer_rad(self) -> float:
        """자전거 모델(앞축 중심) 기준 최대 조향각."""
        if self.max_steer_deg:
            return math.radians(self.max_steer_deg)
        return steer_from_turn_radius(self.min_turn_radius, self.wheelbase, self.track)

    def validate(self) -> list[str]:
        errors: list[str] = []
        positive = ["width", "wheelbase", "track", "min_turn_radius", "lock_to_lock_s"]
        for name in positive:
            if getattr(self, name) <= 0:
                errors.append(f"{name} 값은 0보다 커야 합니다.")
        for name in ["front_overhang", "rear_overhang"]:
            if getattr(self, name) < 0:
                errors.append(f"{name} 값은 음수일 수 없습니다.")
        if self.track > self.width:
            errors.append("윤거(track)가 차폭보다 클 수 없습니다.")
        if not self.max_steer_deg and self.min_turn_radius <= math.hypot(
            self.wheelbase, self.track / 2
        ):
            errors.append("최소회전반경이 축간거리에 비해 너무 작습니다.")
        if self.max_steer_deg is not None and not (0 < self.max_steer_deg < 70):
            errors.append("최대 조향각은 0~70도 사이여야 합니다.")
        if self.is_articulated:
            if self.trailer_wheelbase <= 0:
                errors.append("트레일러 축간거리는 0보다 커야 합니다.")
            if self.trailer_width <= 0 or self.trailer_track <= 0:
                errors.append("트레일러 폭/윤거는 0보다 커야 합니다.")
        return errors

    def to_dict(self) -> dict:
        data = asdict(self)
        data.pop("builtin", None)
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "Vehicle":
        names = {f.name for f in fields(cls)}
        clean = {k: v for k, v in data.items() if k in names and k != "builtin"}
        return cls(**clean)


def steer_from_turn_radius(radius: float, wheelbase: float, track: float) -> float:
    """바깥 앞바퀴 회전반경 → 자전거 모델 조향각(rad).

    뒷축 중심의 회전반경을 Rr 이라 하면 바깥 앞바퀴는
    sqrt((Rr + T/2)^2 + L^2) = R 을 만족한다.
    """
    inner = radius * radius - wheelbase * wheelbase
    if inner <= 0:
        raise ValueError("최소회전반경이 축간거리보다 작습니다.")
    rear_center = math.sqrt(inner) - track / 2
    if rear_center <= 0:
        raise ValueError("최소회전반경이 너무 작습니다.")
    return math.atan(wheelbase / rear_center)


def outer_front_radius(steer: float, wheelbase: float, track: float) -> float:
    """조향각(rad) → 바깥 앞바퀴 회전반경."""
    rear_center = wheelbase / math.tan(abs(steer))
    return math.hypot(rear_center + track / 2, wheelbase)


# ----------------------------------------------------------------------
# 기본 차량
# ----------------------------------------------------------------------
DESIGN_STANDARD_NOTE = "도로의 구조·시설 기준에 관한 규칙 설계기준자동차 제원"


def builtin_vehicles() -> list[Vehicle]:
    return [
        Vehicle(
            key="passenger",
            name="소형자동차 (설계기준)",
            width=1.7,
            height=2.0,
            wheelbase=2.7,
            front_overhang=0.8,
            rear_overhang=1.2,
            track=1.5,
            min_turn_radius=6.0,
            lock_to_lock_s=3.0,
            note=DESIGN_STANDARD_NOTE + " (윤거는 가정값)",
            builtin=True,
        ),
        Vehicle(
            key="minibus_county",
            name="미니버스 (카운티 장축)",
            width=2.035,
            height=2.755,
            wheelbase=4.085,
            front_overhang=1.2,
            rear_overhang=2.425,
            track=1.7,
            min_turn_radius=7.6,
            lock_to_lock_s=4.0,
            note=(
                "현대 카운티 장축 공개 제원(전장 7,710 / 전폭 2,035 / 축거 4,085mm) 기준. "
                "앞·뒤 내민길이, 윤거, 최소회전반경은 추정값이므로 제작사 제원표로 확인 필요"
            ),
            builtin=True,
        ),
        Vehicle(
            key="large",
            name="대형자동차 (설계기준)",
            width=2.5,
            height=4.0,
            wheelbase=6.5,
            front_overhang=2.5,
            rear_overhang=4.0,
            track=2.1,
            min_turn_radius=12.0,
            lock_to_lock_s=5.0,
            note=DESIGN_STANDARD_NOTE + " (윤거는 가정값)",
            builtin=True,
        ),
        Vehicle(
            key="semitrailer",
            name="세미트레일러 (설계기준)",
            kind=SEMITRAILER,
            width=2.5,
            height=4.0,
            wheelbase=4.2,
            front_overhang=1.3,
            rear_overhang=1.0,
            track=2.1,
            min_turn_radius=12.0,
            lock_to_lock_s=5.0,
            hitch_offset=0.0,
            trailer_wheelbase=9.0,
            trailer_front_overhang=1.5,
            trailer_rear_overhang=2.2,
            trailer_width=2.5,
            trailer_track=2.1,
            max_articulation_deg=80.0,
            note=(
                DESIGN_STANDARD_NOTE
                + " (축간거리 앞 4.2 / 뒤 9.0). 트랙터 뒷부분 길이, 트레일러 앞 내민길이, 윤거는 가정값"
            ),
            builtin=True,
        ),
    ]


class VehicleLibrary:
    """기본 차량 + 사용자 차량(JSON 파일) 관리."""

    def __init__(self, user_file: Path | None = None):
        self.user_file = user_file
        self.vehicles: dict[str, Vehicle] = {v.key: v for v in builtin_vehicles()}
        if user_file and user_file.exists():
            self.load_user_file()

    def load_user_file(self) -> None:
        assert self.user_file is not None
        data = json.loads(self.user_file.read_text(encoding="utf-8"))
        for item in data.get("vehicles", []):
            vehicle = Vehicle.from_dict(item)
            self.vehicles[vehicle.key] = vehicle

    def save_user_file(self) -> None:
        if self.user_file is None:
            return
        items = [v.to_dict() for v in self.vehicles.values() if not v.builtin]
        self.user_file.write_text(
            json.dumps({"vehicles": items}, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    def names(self) -> list[str]:
        return [v.name for v in self.vehicles.values()]

    def by_name(self, name: str) -> Vehicle:
        for v in self.vehicles.values():
            if v.name == name:
                return v
        raise KeyError(name)

    def add_or_replace(self, vehicle: Vehicle) -> None:
        vehicle.builtin = False
        self.vehicles[vehicle.key] = vehicle
        self.save_user_file()

    def remove(self, key: str) -> None:
        vehicle = self.vehicles.get(key)
        if vehicle and not vehicle.builtin:
            del self.vehicles[key]
            self.save_user_file()
