"""시뮬레이션 결과로 충돌 검사·궤적(스웹트 패스) 계산·회전 가능 여부 판정."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np
import shapely
from shapely import STRtree
from shapely.geometry import LineString, Polygon
import shapely.ops
from shapely.ops import unary_union

from .geometry import body_polygons, tractor_points, trailer_points
from .simulate import FORWARD, Scenario, SimOptions, SimResult, Simulator
from .vehicles import Vehicle


@dataclass
class CheckOptions:
    clearance: float = 0.3  # 차체와 장애물 사이 최소 여유폭 (m)
    deviation_warn: float = 1.0  # 경로 이탈 경고 기준 (m)
    sample_step: float = 0.1  # 충돌 검사 간격 (m)
    envelope_step: float = 0.25  # 궤적 외곽선 계산 간격 (m)
    snapshot_step: float = 5.0  # 화면에 그릴 차량 윤곽 간격 (m)


@dataclass
class Collision:
    part: str  # tractor / trailer
    x: float
    y: float
    dist_from: float
    dist_to: float
    leg: int
    penetration_hint: str = ""
    contact: bool = False  # True: 차체가 장애물에 닿음, False: 여유폭 안으로만 들어옴
    min_gap: float = 0.0  # 이 구간에서 차체와 장애물 사이 최소 거리 (m, 닿으면 0)

    @property
    def label(self) -> str:
        return "충돌" if self.contact else "여유폭 부족"


@dataclass
class Analysis:
    vehicle: Vehicle
    sim: SimResult
    feasible: bool
    reasons: list[str]
    warnings: list[str]
    collisions: list[Collision]
    envelope: object  # shapely geometry (차체 궤적)
    clearance_envelope: object  # 여유폭 포함 궤적
    tracks: dict[str, np.ndarray]  # 바퀴/모서리 궤적
    snapshots: list[dict[str, np.ndarray]] = field(default_factory=list)
    max_steer_deg: float = 0.0
    max_articulation_deg: float = 0.0
    # 전체 주행 중 차체와 장애물 사이 최소 거리 (m). 가까운 장애물이 없으면 None (= gap_probe 이상)
    min_gap: float | None = None
    min_gap_at: tuple[float, float, float] | None = None  # (x, y, 주행거리)
    gap_probe: float = 1.0

    @property
    def contact_count(self) -> int:
        return sum(1 for c in self.collisions if c.contact)

    @property
    def clearance_count(self) -> int:
        return sum(1 for c in self.collisions if not c.contact)

    def min_gap_text(self) -> str:
        if self.min_gap is None:
            return f"{self.gap_probe:.1f} m 이상"
        return "0 (닿음)" if self.min_gap <= 1e-6 else f"{self.min_gap:.2f} m"

    def summary_lines(self) -> list[str]:
        v = self.vehicle
        sim = self.sim
        lines = [
            f"판정: {'회전 가능' if self.feasible else '회전 불가'}",
            f"차량: {v.name}  (전장 {v.overall_length:.2f} m, 폭 {v.width:.2f} m, "
            f"최대조향 {math.degrees(v.max_steer_rad):.1f}°)",
            f"총 주행거리: {sim.total_distance:.1f} m, 구간 수: {len(sim.legs)} "
            f"(방향전환 {max(len(sim.legs) - 1, 0)}회)",
            f"사용 최대 조향각: {self.max_steer_deg:.1f}°",
            f"차체-장애물 최소 이격: {self.min_gap_text()}",
        ]
        if v.is_articulated:
            lines.append(f"최대 꺾임각: {self.max_articulation_deg:.1f}°")
        for r in self.reasons:
            lines.append(f"[불가] {r}")
        for w in self.warnings:
            lines.append(f"[주의] {w}")
        lines.append("")
        lines.append("구간별 결과")
        for leg in sim.legs:
            gear = "전진" if leg.gear == FORWARD else "후진"
            state = "도달" if leg.reached else "실패"
            extra = f", 방향오차 {leg.heading_error_deg:.1f}°" if leg.heading_error_deg is not None else ""
            lines.append(
                f"  {leg.index + 1}. {gear} {leg.length:.1f} m → {state} "
                f"(도착오차 {leg.end_error:.2f} m, 최대 경로이탈 {leg.max_deviation:.2f} m{extra})"
                + (f" - {leg.message}" if leg.message else "")
            )
        if self.collisions:
            lines.append("")
            lines.append("충돌·여유폭 부족 위치")
            for i, c in enumerate(self.collisions, 1):
                part = "트레일러" if c.part == "trailer" else "차체"
                lines.append(
                    f"  {i}. [{c.label}] {part}  x={c.x:.2f}, y={c.y:.2f}  "
                    f"(주행 {c.dist_from:.1f}~{c.dist_to:.1f} m 구간, 최소 이격 {c.min_gap:.2f} m)"
                )
        return lines


def build_obstacles(polylines_m: list[np.ndarray]) -> tuple[list[LineString], STRtree | None]:
    lines = []
    for pts in polylines_m:
        if len(pts) >= 2:
            ls = LineString(pts)
            if ls.length > 1e-6:
                lines.append(ls)
    return lines, (STRtree(lines) if lines else None)


def run_check(vehicle: Vehicle, scenario: Scenario, obstacles: list[np.ndarray],
              sim_options: SimOptions | None = None,
              check_options: CheckOptions | None = None) -> Analysis:
    co = check_options or CheckOptions()
    errors = vehicle.validate()
    if errors:
        raise ValueError("\n".join(errors))
    if not scenario.waypoints:
        raise ValueError("경유점 또는 목적지를 하나 이상 지정해야 합니다.")
    sim = Simulator(vehicle, sim_options).run(scenario)
    lines, tree = build_obstacles(obstacles)
    return analyze(vehicle, sim, lines, tree, co)


def _sample_indices(dist: np.ndarray, step: float) -> np.ndarray:
    if len(dist) == 0:
        return np.zeros(0, dtype=int)
    idx = [0]
    last = dist[0]
    for i in range(1, len(dist)):
        if dist[i] - last >= step - 1e-9:
            idx.append(i)
            last = dist[i]
    if idx[-1] != len(dist) - 1:
        idx.append(len(dist) - 1)
    return np.array(idx)


def analyze(vehicle: Vehicle, sim: SimResult, lines: list[LineString], tree: STRtree | None,
            co: CheckOptions) -> Analysis:
    reasons: list[str] = []
    warnings: list[str] = []

    # ---------------- 충돌 검사 ----------------
    # 차체(여유폭 없이)와 장애물 선 사이 실제 거리로 판정한다.
    #   거리 0        → 충돌 (차체가 닿음)
    #   거리 < 여유폭 → 여유폭 부족
    collisions: list[Collision] = []
    open_events: dict[str, Collision] = {}
    probe = max(co.clearance + 0.7, 1.0)  # 이 거리 안의 장애물만 거리를 잰다 (최소 이격 표시용)
    min_gap: float | None = None
    min_gap_at = None
    if tree is not None:
        line_arr = np.array(lines, dtype=object)
        for i in _sample_indices(sim.dist, co.sample_step):
            st = (sim.x[i], sim.y[i], sim.theta[i], sim.phi[i])
            body = body_polygons(vehicle, *st)
            near = body_polygons(vehicle, *st, margin=probe)
            hit_parts = set()
            for part, pts in body.items():
                cand = tree.query(Polygon(near[part]), predicate="intersects")
                if not len(cand):
                    continue
                poly = Polygon(pts)
                gaps = shapely.distance(poly, line_arr[cand])
                j = int(np.argmin(gaps))
                gap = float(gaps[j])
                if min_gap is None or gap < min_gap:
                    min_gap = gap
                    q = shapely.ops.nearest_points(poly, line_arr[cand[j]])[1]
                    min_gap_at = (q.x, q.y, float(sim.dist[i]))
                if gap >= co.clearance - 1e-9 and gap > 0:
                    continue
                hit_parts.add(part)
                ev = open_events.get(part)
                if ev is None:
                    line = line_arr[cand[j]]
                    inter = poly.intersection(line) if gap <= 0 else None
                    if inter is not None and not inter.is_empty:
                        p = inter.representative_point()
                    else:
                        p = shapely.ops.nearest_points(poly, line)[1]
                    ev = Collision(part, p.x, p.y, float(sim.dist[i]), float(sim.dist[i]), int(sim.leg[i]),
                                   contact=gap <= 0, min_gap=gap)
                    open_events[part] = ev
                    collisions.append(ev)
                else:
                    ev.dist_to = float(sim.dist[i])
                    if gap < ev.min_gap:
                        ev.min_gap = gap
                    if gap <= 0 and not ev.contact:
                        # 여유폭 부족으로 시작했다가 실제로 닿음 → 닿은 위치로 바꿈
                        ev.contact = True
                        inter = poly.intersection(line_arr[cand[j]])
                        if not inter.is_empty:
                            p = inter.representative_point()
                            ev.x, ev.y = p.x, p.y
            for part in list(open_events):
                if part not in hit_parts:
                    del open_events[part]
    n_contact = sum(1 for c in collisions if c.contact)
    n_clear = len(collisions) - n_contact
    if n_contact:
        reasons.append(f"장애물과 충돌 {n_contact}곳")
    if n_clear:
        reasons.append(f"여유폭({co.clearance:.2f} m) 부족 {n_clear}곳 (닿지는 않음)")

    # ---------------- 주행 결과 ----------------
    if sim.jackknife_at is not None:
        reasons.append(f"주행 {sim.jackknife_at:.1f} m 지점에서 트레일러 꺾임각 한계 초과(잭나이프)")
    for leg in sim.legs:
        if not leg.reached:
            gear = "전진" if leg.gear == FORWARD else "후진"
            reasons.append(f"{leg.index + 1}번 구간({gear}) 목표점 도달 실패: {leg.message}")
    if not sim.completed and not any(not l.reached for l in sim.legs):
        reasons.append(sim.failure or "주행이 완료되지 않음")
    max_dev = float(np.max(np.abs(sim.deviation))) if len(sim.deviation) else 0.0
    if max_dev > co.deviation_warn:
        warnings.append(
            f"지정 경로에서 최대 {max_dev:.2f} m 벗어남 - 회전반경이 부족하거나 경유점 간격이 좁을 수 있음"
        )
    sat = np.abs(sim.steer) >= vehicle.max_steer_rad * 0.995
    if len(sim.dist) > 1 and sat.any():
        steps = np.diff(sim.dist, prepend=sim.dist[0])
        sat_len = float(np.sum(steps[sat]))
        if sat_len > 0.5:
            warnings.append(f"최대 조향(핸들 끝까지) 상태로 {sat_len:.1f} m 주행")

    # ---------------- 궤적 ----------------
    tp = tractor_points(vehicle, sim.x, sim.y, sim.theta)
    tracks = {k: tp[k] for k in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr",
                                  "corner_fl", "corner_fr", "corner_rl", "corner_rr")}
    if vehicle.is_articulated:
        trp = trailer_points(vehicle, tp["hitch"], sim.phi)
        for k in ("trailer_wheel_l", "trailer_wheel_r", "trailer_corner_rl",
                  "trailer_corner_rr", "trailer_corner_fl", "trailer_corner_fr"):
            tracks[k] = trp[k]

    env_polys = []
    clr_polys = []
    for i in _sample_indices(sim.dist, co.envelope_step):
        for pts in body_polygons(vehicle, sim.x[i], sim.y[i], sim.theta[i], sim.phi[i]).values():
            env_polys.append(Polygon(pts))
        for pts in body_polygons(vehicle, sim.x[i], sim.y[i], sim.theta[i], sim.phi[i],
                                 margin=co.clearance).values():
            clr_polys.append(Polygon(pts))
    envelope = unary_union(env_polys) if env_polys else Polygon()
    clearance_env = unary_union(clr_polys) if clr_polys else Polygon()

    snapshots = []
    idx = list(_sample_indices(sim.dist, co.snapshot_step))
    # 방향 전환 지점(구간 경계)도 윤곽을 그린다.
    for j in range(1, len(sim.leg)):
        if sim.leg[j] != sim.leg[j - 1]:
            idx.append(j - 1)
    for i in sorted(set(idx)):
        snapshots.append(body_polygons(vehicle, sim.x[i], sim.y[i], sim.theta[i], sim.phi[i]))

    max_art = 0.0
    if vehicle.is_articulated and len(sim.theta):
        art = np.angle(np.exp(1j * (sim.theta - sim.phi)))
        max_art = float(np.degrees(np.max(np.abs(art))))

    feasible = not reasons
    return Analysis(
        vehicle=vehicle, sim=sim, feasible=feasible, reasons=reasons, warnings=warnings,
        collisions=collisions, envelope=envelope, clearance_envelope=clearance_env,
        tracks=tracks, snapshots=snapshots,
        max_steer_deg=float(np.degrees(np.max(np.abs(sim.steer)))) if len(sim.steer) else 0.0,
        max_articulation_deg=max_art,
        min_gap=min_gap, min_gap_at=min_gap_at, gap_probe=probe,
    )
