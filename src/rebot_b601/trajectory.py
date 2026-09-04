"""Joint-space trajectory generation for streamed setpoints.

Given the current joint positions and a list of waypoints, produce evenly
spaced (in time) setpoints that pass through every waypoint in order while
respecting per-joint velocity and acceleration limits.

The path is piecewise linear in joint space. Each segment is given a nominal
duration equal to the time its slowest joint needs at its velocity limit, so
that at cruise no joint exceeds its limit. A trapezoidal (or triangular, for
short paths) velocity profile is then applied to the path parameter, which
bounds acceleration at the start and end of the motion. Interior waypoints are
passed at cruise speed, as the uFactory module's interpolator does.

All angles are in degrees; everything here is pure Python and unit-tested
without hardware.
"""

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

_EPS = 1e-9


@dataclass
class MoveOptions:
    vmax_deg_s: List[float]  # per joint
    amax_deg_s2: List[float]  # per joint
    hz: float = 50.0
    direct: bool = False  # send the final target only (no interpolation)
    interpolate: bool = True
    wait_at_end: bool = True


def _dedupe(points: List[List[float]]) -> List[List[float]]:
    out: List[List[float]] = []
    for p in points:
        if not out or any(abs(a - b) > 1e-6 for a, b in zip(out[-1], p)):
            out.append(list(p))
    return out


def segment_durations(points: Sequence[Sequence[float]], vmax: Sequence[float]) -> List[float]:
    """Nominal time for each segment at the joints' velocity limits."""
    durations = []
    for a, b in zip(points[:-1], points[1:]):
        durations.append(max(abs(q1 - q0) / max(v, _EPS) for q0, q1, v in zip(a, b, vmax)))
    return durations


def profile(path_length: float, ramp_time: float) -> Tuple[float, float, float]:
    """Trapezoidal profile over a path of length ``path_length`` (in cruise-time
    units, so cruise speed is 1.0) with acceleration ``1/ramp_time``.

    Returns (total_time, ramp_duration, peak_speed).
    """
    if path_length <= _EPS:
        return 0.0, 0.0, 0.0
    if ramp_time <= _EPS:
        return path_length, 0.0, 1.0
    if path_length >= ramp_time:
        # trapezoid: two ramps of ramp_time covering ramp_time of path total
        return path_length + ramp_time, ramp_time, 1.0
    # triangle: never reach cruise speed
    accel = 1.0 / ramp_time
    peak = math.sqrt(path_length * accel)
    ramp = peak / accel
    return 2.0 * ramp, ramp, peak


def position_along(t: float, total: float, ramp: float, peak: float) -> float:
    """Path parameter s(t) for the profile returned by ``profile``."""
    if total <= _EPS:
        return 0.0
    t = min(max(t, 0.0), total)
    if ramp <= _EPS:
        return t
    accel = peak / ramp
    if t < ramp:
        return 0.5 * accel * t * t
    if t <= total - ramp:
        return 0.5 * accel * ramp * ramp + peak * (t - ramp)
    td = total - t
    return (0.5 * accel * ramp * ramp) * 2 + peak * (total - 2 * ramp) - 0.5 * accel * td * td


def plan(
    start: Sequence[float],
    waypoints: Sequence[Sequence[float]],
    opts: MoveOptions,
) -> List[List[float]]:
    """Return the list of setpoints (one per tick at ``opts.hz``), ending exactly
    on the last waypoint. Returns an empty list when there is nothing to do."""
    points = _dedupe([list(start)] + [list(w) for w in waypoints])
    if len(points) < 2:
        return []
    n = len(points[0])
    if any(len(p) != n for p in points):
        raise ValueError("all waypoints must have the same number of joints")

    durations = segment_durations(points, opts.vmax_deg_s)
    cumulative = [0.0]
    for d in durations:
        cumulative.append(cumulative[-1] + d)
    path_length = cumulative[-1]

    ramp_time = min(v / max(a, _EPS) for v, a in zip(opts.vmax_deg_s, opts.amax_deg_s2))
    total, ramp, peak = profile(path_length, ramp_time)
    if total <= _EPS:
        return []

    dt = 1.0 / max(opts.hz, _EPS)
    steps = max(1, int(math.ceil(total / dt)))
    out: List[List[float]] = []
    seg = 0
    for k in range(1, steps + 1):
        t = min(k * dt, total)
        s = position_along(t, total, ramp, peak)
        while seg < len(durations) - 1 and s > cumulative[seg + 1]:
            seg += 1
        seg_len = durations[seg]
        frac = 1.0 if seg_len <= _EPS else min(1.0, max(0.0, (s - cumulative[seg]) / seg_len))
        a, b = points[seg], points[seg + 1]
        out.append([q0 + (q1 - q0) * frac for q0, q1 in zip(a, b)])
    out[-1] = list(points[-1])
    return out


def timed_plan(
    start: Sequence[float],
    waypoints: Sequence[Sequence[float]],
    opts: MoveOptions,
) -> List[Tuple[float, List[float]]]:
    """Like ``plan`` but returns (time_from_start, positions) tuples."""
    dt = 1.0 / max(opts.hz, _EPS)
    return [((k + 1) * dt, q) for k, q in enumerate(plan(start, waypoints, opts))]
