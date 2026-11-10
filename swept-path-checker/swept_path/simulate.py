"""경유점 경로를 따라가는 차량 주행 시뮬레이션 (운동학 모델).

- 단일차량: 자전거 모델. 상태 = 뒷축 중심 (x, y), 차량 방향 theta
- 세미트레일러: 트랙터 자전거 모델 + 킹핀으로 연결된 트레일러 방향 phi

조향 제어
- 전진: 앞축 중심이 경로를 따라가도록 Stanley 제어
- 후진(단일차량): 뒷축 중심이 경로를 따라가도록 Pure Pursuit
- 후진(세미트레일러): 트레일러 축이 경로를 따라가도록 목표 꺾임각을 구하고,
  트랙터 조향으로 꺾임각을 맞추는 2단 제어
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .geometry import guide_point
from .vehicles import Vehicle

FORWARD = 1
REVERSE = -1


@dataclass
class Waypoint:
    x: float
    y: float
    gear: int = FORWARD  # 이 점까지 가는 동안의 진행 방향
    heading: float | None = None  # 도착 시 차량 방향(rad). 구간 끝점에서만 사용


@dataclass
class Scenario:
    start_x: float
    start_y: float
    start_heading: float  # rad, 차량이 바라보는 방향
    waypoints: list[Waypoint]


@dataclass
class SimOptions:
    step: float = 0.05  # m
    speed_kmh: float = 8.0  # 전진 주행 속도 (조향 속도 제한 계산용)
    reverse_speed_kmh: float = 4.0  # 후진 주행 속도
    stationary_steer: bool = True  # 정지 상태(출발/방향전환 시) 핸들 조작 허용
    position_tol: float = 0.5  # 목표점 도달 허용오차 (m)
    heading_tol_deg: float = 10.0  # 목적지 방향 허용오차
    lookahead: float | None = None  # 전진 시 앞을 내다보는 거리(m). None 이면 차량 축간거리로 자동
    max_travel_factor: float = 3.0


@dataclass
class LegReport:
    index: int
    gear: int
    target: tuple[float, float]
    reached: bool
    end_error: float
    heading_error_deg: float | None
    length: float
    max_deviation: float
    message: str = ""


@dataclass
class SimResult:
    x: np.ndarray
    y: np.ndarray
    theta: np.ndarray
    phi: np.ndarray
    steer: np.ndarray
    gear: np.ndarray
    leg: np.ndarray
    dist: np.ndarray
    deviation: np.ndarray
    legs: list[LegReport] = field(default_factory=list)
    jackknife_at: float | None = None
    completed: bool = True
    failure: str = ""

    @property
    def total_distance(self) -> float:
        return float(self.dist[-1]) if len(self.dist) else 0.0


# ----------------------------------------------------------------------
# 경로(폴리라인) 유틸
# ----------------------------------------------------------------------
class PolyPath:
    def __init__(self, pts: np.ndarray):
        pts = np.asarray(pts, dtype=float)
        keep = [0]
        for i in range(1, len(pts)):
            if np.hypot(*(pts[i] - pts[keep[-1]])) > 1e-6:
                keep.append(i)
        self.pts = pts[keep]
        seg = np.diff(self.pts, axis=0)
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.cum[-1])
        self.seg_dir = seg / self.seg_len[:, None] if len(seg) else np.zeros((0, 2))

    def point_at(self, s: float) -> np.ndarray:
        s = min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self.cum, s, side="right") - 1)
        i = min(max(i, 0), len(self.seg_len) - 1)
        return self.pts[i] + self.seg_dir[i] * (s - self.cum[i])

    def point_ext(self, s: float) -> np.ndarray:
        """경로 끝 너머는 마지막 진행방향으로 연장한 점."""
        if s <= self.length:
            return self.point_at(s)
        return self.pts[-1] + self.seg_dir[-1] * (s - self.length)

    def tangent_at(self, s: float) -> np.ndarray:
        i = int(np.searchsorted(self.cum, min(max(s, 0.0), self.length), side="right") - 1)
        i = min(max(i, 0), len(self.seg_len) - 1)
        return self.seg_dir[i]

    def project(self, p: np.ndarray, s_min: float, s_max: float) -> tuple[float, float]:
        """s 구간 [s_min, s_max] 안에서 p의 최근접 투영. (s, 부호있는 횡방향 오차) 반환.

        오차 부호: 점이 경로 진행방향 왼쪽이면 +.
        """
        lo = max(int(np.searchsorted(self.cum, s_min, side="right") - 1), 0)
        hi = min(int(np.searchsorted(self.cum, s_max, side="right")), len(self.seg_len))
        best = (s_min, 0.0, math.inf)
        for i in range(lo, max(hi, lo + 1)):
            a = self.pts[i]
            d = self.seg_dir[i]
            t = float(np.clip(np.dot(p - a, d), 0.0, self.seg_len[i]))
            s = self.cum[i] + t
            if s < s_min - 1e-9 or s > s_max + 1e-9:
                s = min(max(s, s_min), s_max)
                t = s - self.cum[i]
            q = a + d * t
            off = p - q
            dist = math.hypot(off[0], off[1])
            if dist < best[2]:
                cross = d[0] * off[1] - d[1] * off[0]
                best = (s, math.copysign(dist, cross) if dist > 1e-12 else 0.0, dist)
        return best[0], best[1]


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ----------------------------------------------------------------------
# 시뮬레이터
# ----------------------------------------------------------------------
def split_legs(waypoints: list[Waypoint]) -> list[list[Waypoint]]:
    legs: list[list[Waypoint]] = []
    for wp in waypoints:
        if legs and legs[-1][-1].gear == wp.gear:
            legs[-1].append(wp)
        else:
            legs.append([wp])
    return legs


def _leg_polyline(start: np.ndarray, leg: list[Waypoint], approach: float) -> np.ndarray:
    pts = [start] + [np.array([w.x, w.y]) for w in leg]
    end = leg[-1]
    if end.heading is not None:
        # 도착 방향을 맞추기 위해 끝점 직전에 직선 진입 구간을 둔다.
        h = np.array([math.cos(end.heading), math.sin(end.heading)])
        move_dir = h if end.gear == FORWARD else -h
        pre = np.array([end.x, end.y]) - move_dir * approach
        if np.hypot(*(pre - pts[-2])) > approach * 0.2 and np.dot(pre - pts[-2], move_dir) > 0:
            pts.insert(len(pts) - 1, pre)
    return np.array(pts)


class Simulator:
    def __init__(self, vehicle: Vehicle, options: SimOptions | None = None):
        self.v = vehicle
        self.o = options or SimOptions()
        self.max_steer = vehicle.max_steer_rad
        steer_rate = 2 * self.max_steer / max(vehicle.lock_to_lock_s, 0.1)  # rad/s
        self.steer_rate_per_m = {
            FORWARD: steer_rate / (max(self.o.speed_kmh, 0.5) / 3.6),
            REVERSE: steer_rate / (max(self.o.reverse_speed_kmh, 0.5) / 3.6),
        }
        self.max_art = math.radians(vehicle.max_articulation_deg)
        # 세미트레일러 후진 제어 파라미터
        self.trailer_look_factor = 1.0
        self.gamma_gain = 4.0
        if vehicle.is_articulated:
            # 최대 조향으로 유지할 수 있는 꺾임각 (이보다 커지면 되돌릴 수 없음)
            sin_sus = min(1.0, vehicle.trailer_wheelbase * math.tan(self.max_steer) / vehicle.wheelbase)
            self.gamma_sustain = math.asin(sin_sus)
            self.gamma_limit = min(math.radians(50), 0.7 * self.gamma_sustain,
                                   self.max_art - math.radians(10))
        self.lookahead = self.o.lookahead or float(np.clip(0.8 * vehicle.wheelbase, 2.5, 6.0))

    # --------------------------------------------------------------
    def run(self, scenario: Scenario) -> SimResult:
        v = self.v
        th = scenario.start_heading
        # 출발점 = 앞축 중심
        x = scenario.start_x - v.wheelbase * math.cos(th)
        y = scenario.start_y - v.wheelbase * math.sin(th)
        phi = th
        steer = 0.0
        rec = {k: [] for k in ("x", "y", "theta", "phi", "steer", "gear", "leg", "dist", "dev")}
        dist = 0.0

        def record(gear: int, leg: int, dev: float) -> None:
            rec["x"].append(x)
            rec["y"].append(y)
            rec["theta"].append(th)
            rec["phi"].append(phi)
            rec["steer"].append(steer)
            rec["gear"].append(gear)
            rec["leg"].append(leg)
            rec["dist"].append(dist)
            rec["dev"].append(dev)

        legs = split_legs(scenario.waypoints)
        reports: list[LegReport] = []
        jackknife_at = None
        failure = ""
        record(legs[0][0].gear if legs else FORWARD, 0, 0.0)

        for li, leg in enumerate(legs):
            gear = leg[0].gear
            g0 = guide_point(v, x, y, th, phi, gear)
            guide_wb = v.trailer_wheelbase if (v.is_articulated and gear == REVERSE) else v.wheelbase
            approach = max(1.5 * guide_wb, 3.0)
            path = PolyPath(_leg_polyline(g0, leg, approach))
            target = np.array([leg[-1].x, leg[-1].y])
            if path.length < 1e-6:
                reports.append(LegReport(li, gear, tuple(target), True, 0.0, None, 0.0, 0.0))
                continue
            s_proj = 0.0
            leg_dist = 0.0
            max_dev = 0.0
            limit = path.length * self.o.max_travel_factor + 20.0
            first = True
            reached = False
            msg = ""
            while True:
                g = guide_point(v, x, y, th, phi, gear)
                s_proj, dev = path.project(g, s_proj, s_proj + 3.0)
                max_dev = max(max_dev, abs(dev))
                end_gap = path.length - s_proj
                to_end = target - g
                end_t = path.tangent_at(path.length)
                if end_gap < 1e-3 or (end_gap < 2.0 and np.dot(to_end, end_t) <= 0):
                    reached = True
                    break
                if leg_dist > limit:
                    msg = "지정 경로를 따라가지 못함(주행거리 초과)"
                    break
                cmd = self._steer_command(path, s_proj, dev, gear, x, y, th, phi)
                if first and self.o.stationary_steer:
                    steer = cmd
                else:
                    lim = self.steer_rate_per_m[gear] * self.o.step
                    d = np.clip(cmd - steer, -lim, lim)
                    steer = float(np.clip(steer + d, -self.max_steer, self.max_steer))
                first = False
                x, y, th, phi = self._integrate(x, y, th, phi, steer, gear, self.o.step)
                dist += self.o.step
                leg_dist += self.o.step
                record(gear, li, dev)
                if v.is_articulated and abs(wrap(th - phi)) > self.max_art:
                    jackknife_at = dist
                    msg = "트랙터-트레일러 꺾임각 한계 초과(잭나이프)"
                    break

            g = guide_point(v, x, y, th, phi, gear)
            err = float(np.hypot(*(g - target)))
            head_err = None
            if leg[-1].heading is not None:
                body = phi if (v.is_articulated and gear == REVERSE) else th
                head_err = abs(math.degrees(wrap(body - leg[-1].heading)))
            ok = reached and err <= self.o.position_tol
            if reached and not ok:
                msg = f"목표점에서 {err:.2f} m 벗어남"
            if ok and head_err is not None and head_err > self.o.heading_tol_deg:
                ok = False
                msg = f"도착 방향 오차 {head_err:.1f}°"
            reports.append(
                LegReport(li, gear, (float(target[0]), float(target[1])), ok, err, head_err,
                          leg_dist, max_dev, msg)
            )
            if not ok:
                failure = msg
                break

        arr = {k: np.asarray(val, dtype=float) for k, val in rec.items()}
        return SimResult(
            x=arr["x"], y=arr["y"], theta=arr["theta"], phi=arr["phi"], steer=arr["steer"],
            gear=arr["gear"].astype(int), leg=arr["leg"].astype(int), dist=arr["dist"],
            deviation=arr["dev"], legs=reports, jackknife_at=jackknife_at,
            completed=not failure and len(reports) == len(legs), failure=failure,
        )

    # --------------------------------------------------------------
    def _integrate(self, x, y, th, phi, steer, gear, ds):
        v = self.v
        sigma = float(gear)
        n = 2
        h = ds / n
        for _ in range(n):
            tan_d = math.tan(steer)
            dth = sigma * tan_d / v.wheelbase
            th_mid = th + 0.5 * h * dth
            x += sigma * h * math.cos(th_mid)
            y += sigma * h * math.sin(th_mid)
            if v.is_articulated:
                gam = th_mid - phi
                dphi = sigma * (math.sin(gam) - v.hitch_offset * tan_d * math.cos(gam) / v.wheelbase)
                phi += h * dphi / v.trailer_wheelbase
            th += h * dth
        return x, y, th, phi

    def _steer_command(self, path: PolyPath, s_proj, dev, gear, x, y, th, phi) -> float:
        v = self.v
        if gear == FORWARD:
            # 앞축 기준 Pure Pursuit: 앞축은 방향 (theta+delta)로 반경 L/sin(delta)의 원을 그린다.
            # 목표점이 그 원 위에 오도록 하면 sin(d) = k sin(a-d), k = 2L/Ld
            #   → d = atan2(k sin a, 1 + k cos a)
            fx = x + v.wheelbase * math.cos(th)
            fy = y + v.wheelbase * math.sin(th)
            look = self.lookahead
            tgt = path.point_ext(s_proj + look)
            alpha = wrap(math.atan2(tgt[1] - fy, tgt[0] - fx) - th)
            ld = max(math.hypot(tgt[0] - fx, tgt[1] - fy), 0.5)
            k = 2 * v.wheelbase / ld
            cmd = math.atan2(k * math.sin(alpha), 1 + k * math.cos(alpha))
            return float(np.clip(cmd, -self.max_steer, self.max_steer))

        if not v.is_articulated:
            look = float(np.clip(1.2 * v.wheelbase, 3.0, 8.0))
            tgt = path.point_ext(s_proj + look)
            motion = th + math.pi
            alpha = wrap(math.atan2(tgt[1] - y, tgt[0] - x) - motion)
            ld = max(math.hypot(tgt[0] - x, tgt[1] - y), 0.5)
            kappa = 2 * math.sin(alpha) / ld
            return float(np.clip(-math.atan(v.wheelbase * kappa), -self.max_steer, self.max_steer))

        # 세미트레일러 후진
        L1, L2, c = v.wheelbase, v.trailer_wheelbase, v.hitch_offset
        hx, hy = x - c * math.cos(th), y - c * math.sin(th)
        ax, ay = hx - L2 * math.cos(phi), hy - L2 * math.sin(phi)
        look = max(4.0, self.trailer_look_factor * L2)
        tgt = path.point_ext(s_proj + look)
        motion = phi + math.pi
        alpha = wrap(math.atan2(tgt[1] - ay, tgt[0] - ax) - motion)
        ld = max(math.hypot(tgt[0] - ax, tgt[1] - ay), 0.5)
        kappa = 2 * math.sin(alpha) / ld
        gam_lim = self.gamma_limit
        # 후진하는 트레일러는 꺾임각 gamma 를 조향각으로 하는 자전거 모델과 같다.
        gam_d = float(np.clip(-math.atan(L2 * kappa), -gam_lim, gam_lim))
        gam = wrap(th - phi)
        k_gam = self.gamma_gain / L2
        rate = k_gam * (gam_d - gam)
        tan_d = L1 * (rate * -1.0 + math.sin(gam) / L2) / (1 + c * math.cos(gam) / L2)
        return float(np.clip(math.atan(tan_d), -self.max_steer, self.max_steer))
