"""시뮬레이션 결과로 충돌 검사·궤적(스웹트 패스) 계산·회전 가능 여부 판정."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np
from shapely import STRtree
from shapely.geometry import LineString, Polygon
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
            lines.append("충돌 위치")
            for i, c in enumerate(self.collisions, 1):
                part = "트레일러" if c.part == "trailer" else "차체"
                lines.append(
                    f"  {i}. {part}  x={c.x:.2f}, y={c.y:.2f}  "
                    f"(주행 {c.dist_from:.1f}~{c.dist_to:.1f} m 구간)"
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
    collisions: list[Collision] = []
    open_events: dict[str, Collision] = {}
    if tree is not None:
        for i in _sample_indices(sim.dist, co.sample_step):
            polys = body_polygons(vehicle, sim.x[i], sim.y[i], sim.theta[i], sim.phi[i],
                                  margin=co.clearance)
            hit_parts = set()
            for part, pts in polys.items():
                poly = Polygon(pts)
                hits = tree.query(poly, predicate="intersects")
                if len(hits):
                    hit_parts.add(part)
                    if part not in open_events:
                        inter = poly.intersection(lines[int(hits[0])])
                        p = inter.representative_point() if not inter.is_empty else poly.centroid
                        ev = Collision(part, p.x, p.y, float(sim.dist[i]), float(sim.dist[i]),
                                       int(sim.leg[i]))
                        open_events[part] = ev
                        collisions.append(ev)
                    else:
                        open_events[part].dist_to = float(sim.dist[i])
            for part in list(open_events):
                if part not in hit_parts:
                    del open_events[part]
    if collisions:
        reasons.append(
            f"장애물과 충돌 또는 여유폭({co.clearance:.2f} m) 부족 {len(collisions)}곳"
        )

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
    )
