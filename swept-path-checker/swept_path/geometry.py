"""차량 자세(state) → 차체 사각형, 바퀴 위치 계산."""

from __future__ import annotations

import numpy as np

from .vehicles import Vehicle


def _unit(angle: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return np.cos(angle), np.sin(angle)


def tractor_points(v: Vehicle, x, y, theta) -> dict[str, np.ndarray]:
    """트랙터(단일차량) 주요 점. 입력은 스칼라 또는 배열(N,)."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    theta = np.asarray(theta, dtype=float)
    c, s = _unit(theta)
    rear = np.stack([x, y], axis=-1)
    fwd = np.stack([c, s], axis=-1)
    left = np.stack([-s, c], axis=-1)
    front = rear + v.wheelbase * fwd
    half_w = v.width / 2
    half_t = v.track / 2
    nose = front + v.front_overhang * fwd
    tail = rear - v.rear_overhang * fwd
    return {
        "rear_axle": rear,
        "front_axle": front,
        "hitch": rear - v.hitch_offset * fwd,
        "wheel_fl": front + half_t * left,
        "wheel_fr": front - half_t * left,
        "wheel_rl": rear + half_t * left,
        "wheel_rr": rear - half_t * left,
        "corner_fl": nose + half_w * left,
        "corner_fr": nose - half_w * left,
        "corner_rl": tail + half_w * left,
        "corner_rr": tail - half_w * left,
    }


def trailer_points(v: Vehicle, hitch: np.ndarray, phi) -> dict[str, np.ndarray]:
    phi = np.asarray(phi, dtype=float)
    c, s = _unit(phi)
    fwd = np.stack([c, s], axis=-1)
    left = np.stack([-s, c], axis=-1)
    axle = hitch - v.trailer_wheelbase * fwd
    half_w = v.trailer_width / 2
    half_t = v.trailer_track / 2
    nose = hitch + v.trailer_front_overhang * fwd
    tail = axle - v.trailer_rear_overhang * fwd
    return {
        "trailer_axle": axle,
        "trailer_wheel_l": axle + half_t * left,
        "trailer_wheel_r": axle - half_t * left,
        "trailer_corner_fl": nose + half_w * left,
        "trailer_corner_fr": nose - half_w * left,
        "trailer_corner_rl": tail + half_w * left,
        "trailer_corner_rr": tail - half_w * left,
    }


def body_polygons(v: Vehicle, x: float, y: float, theta: float, phi: float | None = None,
                  margin: float = 0.0) -> dict[str, np.ndarray]:
    """차체 외곽 사각형(4x2). margin 만큼 사방으로 키운다."""
    c, s = np.cos(theta), np.sin(theta)
    fwd = np.array([c, s])
    left = np.array([-s, c])
    rear = np.array([x, y])
    nose = rear + (v.wheelbase + v.front_overhang + margin) * fwd
    tail = rear - (v.rear_overhang + margin) * fwd
    hw = v.width / 2 + margin
    polys = {
        "tractor": np.array([nose + hw * left, nose - hw * left, tail - hw * left, tail + hw * left])
    }
    if v.is_articulated and phi is not None:
        hitch = rear - v.hitch_offset * fwd
        tc, ts = np.cos(phi), np.sin(phi)
        tf = np.array([tc, ts])
        tl = np.array([-ts, tc])
        axle = hitch - v.trailer_wheelbase * tf
        tnose = hitch + (v.trailer_front_overhang + margin) * tf
        ttail = axle - (v.trailer_rear_overhang + margin) * tf
        thw = v.trailer_width / 2 + margin
        polys["trailer"] = np.array(
            [tnose + thw * tl, tnose - thw * tl, ttail - thw * tl, ttail + thw * tl]
        )
    return polys


def guide_point(v: Vehicle, x: float, y: float, theta: float, phi: float, gear: int) -> np.ndarray:
    """경로를 따라가는 기준점.

    전진: 트랙터 앞축 중심, 후진: 단일차량은 뒷축 중심 / 세미트레일러는 트레일러 축 중심.
    """
    c, s = np.cos(theta), np.sin(theta)
    if gear > 0:
        return np.array([x + v.wheelbase * c, y + v.wheelbase * s])
    if not v.is_articulated:
        return np.array([x, y])
    hx, hy = x - v.hitch_offset * c, y - v.hitch_offset * s
    return np.array([hx - v.trailer_wheelbase * np.cos(phi), hy - v.trailer_wheelbase * np.sin(phi)])
