"""Viam arm component for the Seeed Studio reBot Arm B601-DM (6 DoF, Damiao CAN motors)."""

import asyncio
import math
import queue
import threading
import time
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from motorbridge import Mode
from viam.components.arm import Arm, JointPositions, Pose
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, Mesh, PoseInFrame, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.services.motion import MotionClient
from viam.utils import struct_to_dict

from . import damiao, kinematics, spatial
from .bus import DEFAULT_BAUD, LINK_ERRORS, BusError, SharedBus, detect_port, is_motor_timeout
from .damiao import CollisionError, JointHealth, MotorFault, OverTemperatureError
from .ops import SingleOperationManager
from .trajectory import MoveOptions, plan

LOGGER = getLogger(__name__)

ARM_CAN_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
N_JOINTS = 6

# Defaults mirror Seeed's LeRobot reference implementation for this arm.
DEFAULT_SPEED_DEG_S = 60.0
DEFAULT_ACCEL_DEG_S2 = 200.0
MIN_SPEED_DEG_S = 1.0
MAX_SPEED_DEG_S = 180.0
MAX_ACCEL_DEG_S2 = 1000.0
DEFAULT_MOVE_HZ = 50.0
DEFAULT_MONITOR_HZ = 10.0
DEFAULT_MIT_KP = [45.0, 45.0, 45.0, 8.0, 9.0, 8.0]
DEFAULT_MIT_KD = [12.0, 12.0, 12.0, 1.0, 1.0, 1.0]
# Soft limits (deg); slightly inside the URDF limits by default.
DEFAULT_JOINT_LIMITS = [(-150.0, 150.0), (-179.0, 1.0), (-179.0, 1.0), (-107.0, 89.0), (-89.0, 89.0), (-179.0, 179.0)]
DEFAULT_TEMP_WARN_C = 60.0
DEFAULT_TEMP_LIMIT_C = 80.0
DEFAULT_TORQUE_TRIP_POLLS = 3
DEFAULT_MANUAL_KD = 0.5
DEFAULT_MANUAL_HZ = 50.0

_ENSURE_MODE_RETRIES = 9
_SETTLE_SEC = 0.02
_MOVING_VEL_RAD_S = 0.05
_DEFAULT_TOLERANCE_DEG = 2.0
_SETTLE_POLL_SEC = 0.05
_POST_STREAM_SETTLE_S = 2.0  # the last setpoint is the target; the arm only needs to close the tracking gap
_WARN_INTERVAL_S = 30.0


def _as_list(value, n: int, name: str) -> List[float]:
    if isinstance(value, (int, float)):
        return [float(value)] * n
    values = [float(v) for v in value]
    if len(values) != n:
        raise ValueError(f"'{name}' must be a number or a list of {n} numbers")
    return values


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class B601Arm(Arm, EasyResource):
    MODEL = "devrel:rebot-b601:arm"

    def __init__(self, name: str):
        super().__init__(name)
        self.bus: Optional[SharedBus] = None
        self.ops = SingleOperationManager()
        self._torque_enabled = False
        self._last_warn: Dict[str, float] = {}
        self._manual_thread: Optional[threading.Thread] = None
        self._manual_stop = threading.Event()
        self.motion: Optional[MotionClient] = None
        self._last_health: Dict[int, JointHealth] = {}

    # ------------------------------------------------------------------ config

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        # EasyResource's default new() does not call reconfigure, and
        # viam-server only calls Reconfigure on config changes, so the bus
        # must be opened here.
        self = cls(config.name)
        try:
            self.reconfigure(config, dependencies)
        except Exception:
            # A failed build must not pin the serial port: viam-server will
            # retry with a fresh instance, which needs to open it again.
            self._release_bus()
            raise
        return self

    def _release_bus(self):
        bus, self.bus = self.bus, None
        if bus is not None:
            bus.remove_reconnect_callback(self._on_bus_reconnect)
            bus.release()

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        mode = attrs.get("control_mode", "pos_vel")
        if mode not in ("pos_vel", "mit"):
            raise ValueError("control_mode must be 'pos_vel' or 'mit'")
        for key in ("speed_deg_s", "acceleration_deg_s2", "mit_kp", "mit_kd", "torque_limit_nm"):
            if key in attrs:
                _as_list(attrs[key], N_JOINTS, key)
        if "joint_limits_deg" in attrs:
            limits = attrs["joint_limits_deg"]
            if len(limits) != N_JOINTS or any(len(pair) != 2 for pair in limits):
                raise ValueError("joint_limits_deg must be a list of 6 [min, max] pairs")
        if "bad_joints" in attrs:
            for j in attrs["bad_joints"]:
                if int(j) < 0 or int(j) >= N_JOINTS:
                    raise ValueError("bad_joints entries must be joint indices 0-5")
        cg = attrs.get("collision_geometry", "primitives")
        if cg not in kinematics.COLLISION_MODES:
            raise ValueError(f"collision_geometry must be one of {kinematics.COLLISION_MODES}")
        if "move_hz" in attrs and not 1.0 <= float(attrs["move_hz"]) <= 500.0:
            raise ValueError("move_hz must be between 1 and 500")
        deps: List[str] = []
        motion = attrs.get("motion")
        if motion:
            deps.append(str(motion))
        return deps, []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        attrs = struct_to_dict(config.attributes)
        port = attrs.get("port") or detect_port()
        baud = int(attrs.get("baud", DEFAULT_BAUD))

        self.control_mode = attrs.get("control_mode", "pos_vel")
        self.speeds = _as_list(attrs.get("speed_deg_s", DEFAULT_SPEED_DEG_S), N_JOINTS, "speed_deg_s")
        self.accels = _as_list(attrs.get("acceleration_deg_s2", DEFAULT_ACCEL_DEG_S2), N_JOINTS, "acceleration_deg_s2")
        self.move_hz = float(attrs.get("move_hz", DEFAULT_MOVE_HZ))
        self.mit_kp = _as_list(attrs.get("mit_kp", DEFAULT_MIT_KP), N_JOINTS, "mit_kp")
        self.mit_kd = _as_list(attrs.get("mit_kd", DEFAULT_MIT_KD), N_JOINTS, "mit_kd")
        self.tolerance_deg = float(attrs.get("tolerance_deg", _DEFAULT_TOLERANCE_DEG))
        self.enable_on_start = bool(attrs.get("enable_on_start", True))
        # Off by default: disabling torque makes the arm slump under gravity,
        # and close() also runs on every reconfigure.
        self.disable_torque_on_close = bool(attrs.get("disable_torque_on_close", False))
        self.clip_targets = bool(attrs.get("clip_targets", False))
        self.bad_joints = sorted({int(j) for j in attrs.get("bad_joints", [])})
        self.collision_mode = attrs.get("collision_geometry", "primitives")
        self.include_gripper_geometry = bool(attrs.get("include_gripper_geometry", False))
        self.reconnect_enabled = bool(attrs.get("reconnect", True))
        self.torque_limits = (
            _as_list(attrs["torque_limit_nm"], N_JOINTS, "torque_limit_nm") if "torque_limit_nm" in attrs else None
        )
        self.torque_trip_polls = int(attrs.get("torque_trip_polls", DEFAULT_TORQUE_TRIP_POLLS))
        self.temp_warn_c = float(attrs.get("temperature_warn_c", DEFAULT_TEMP_WARN_C))
        self.temp_limit_c = float(attrs.get("temperature_limit_c", DEFAULT_TEMP_LIMIT_C))
        self.can_timeout_ms = int(attrs["can_timeout_ms"]) if "can_timeout_ms" in attrs else None
        self.manual_kp = float(attrs.get("manual_mode_kp", 0.0))
        self.manual_kd = float(attrs.get("manual_mode_kd", DEFAULT_MANUAL_KD))
        self.gravity_scale = float(attrs.get("gravity_scale", 1.0))
        self.payload_kg = float(attrs.get("payload_kg", 0.0))
        gv = attrs.get("gravity_vector", [0.0, 0.0, -spatial.GRAVITY_M_S2])
        self.gravity_vector = tuple(float(v) for v in gv)
        if "joint_limits_deg" in attrs:
            self.joint_limits = [(float(lo), float(hi)) for lo, hi in attrs["joint_limits_deg"]]
        else:
            self.joint_limits = list(DEFAULT_JOINT_LIMITS)

        motion_name = attrs.get("motion")
        self.motion = None
        if motion_name:
            rn = MotionClient.get_resource_name(str(motion_name))
            dep = dependencies.get(rn)
            if dep is None:
                raise ValueError(f"motion service '{motion_name}' not found in dependencies")
            self.motion = dep  # type: ignore[assignment]

        self._exit_manual_mode_sync(restore=False)
        if self.bus is not None and not self.bus.matches(port, baud):
            self._release_bus()
        if self.bus is None:
            self.bus = SharedBus.acquire(port, baud)
            self.bus.on_reconnect(self._on_bus_reconnect)

        if self.enable_on_start:
            self._configure_motors()
        else:
            self._torque_enabled = False
        LOGGER.info(
            "B601 arm '%s' ready on %s (mode=%s, torque %s, collision geometry=%s)",
            self.name,
            port,
            self.control_mode,
            "enabled" if self.enable_on_start else "disabled",
            self.collision_mode,
        )

    # -------------------------------------------------------- bus primitives

    def _bus_call(self, fn, *args, **kwargs):
        """Run a bus operation, reconnecting once if the serial link failed."""
        try:
            return fn(*args, **kwargs)
        except BusError:
            raise
        except LINK_ERRORS as exc:
            if is_motor_timeout(exc):
                # The link is fine; a motor stayed silent. Do not reopen the
                # port for this: check power and CAN wiring instead.
                raise BusError(f"motor did not reply ({exc}); check arm power and CAN wiring") from exc
            if not self.reconnect_enabled or self.bus is None:
                raise BusError(f"serial bridge error: {exc}") from exc
            LOGGER.warning("serial bridge error (%s); reconnecting", exc)
            self.bus.reconnect()
            try:
                return fn(*args, **kwargs)
            except LINK_ERRORS as exc2:
                raise BusError(f"serial bridge error after reconnect: {exc2}") from exc2

    def _on_bus_reconnect(self):
        LOGGER.info("bus reconnected; restoring arm motor state")
        if self._torque_enabled:
            try:
                self._configure_motors()
            except Exception:
                LOGGER.warning("failed to restore arm motors after reconnect", exc_info=True)

    def _configure_motors(self, mode: Optional[Mode] = None):
        target_mode = mode if mode is not None else (Mode.MIT if self.control_mode == "mit" else Mode.POS_VEL)

        def _do():
            with self.bus.lock:
                for can_id in ARM_CAN_IDS:
                    motor = self.bus.motor(can_id)
                    motor.enable()
                    for attempt in range(_ENSURE_MODE_RETRIES + 1):
                        try:
                            motor.ensure_mode(target_mode)
                            break
                        except LINK_ERRORS as exc:
                            # A Damiao motor is busy answering enable() for a moment and
                            # misses the first register read; that timeout is transient
                            # and must be retried (0.2.0 stopped doing so and every
                            # build failed on hardware that a plain scan could see).
                            if not is_motor_timeout(exc) or attempt == _ENSURE_MODE_RETRIES:
                                raise
                            time.sleep(_SETTLE_SEC)
                        except Exception:
                            if attempt == _ENSURE_MODE_RETRIES:
                                raise
                            time.sleep(_SETTLE_SEC)
                    if self.can_timeout_ms is not None:
                        try:
                            motor.set_can_timeout_ms(self.can_timeout_ms)
                        except Exception:
                            LOGGER.warning("could not set CAN timeout on 0x%02x", can_id, exc_info=True)

        self._bus_call(_do)
        self._torque_enabled = True

    def _disable_motors(self):
        def _do():
            with self.bus.lock:
                for can_id in ARM_CAN_IDS:
                    self.bus.motor(can_id).disable()

        self._bus_call(_do)
        self._torque_enabled = False

    def _read_states(self, retries: int = 5) -> Dict[int, Any]:
        return self._bus_call(self.bus.poll_feedback, ARM_CAN_IDS, retries)

    def _read_positions_deg(self) -> List[float]:
        states = self._read_states()
        positions = []
        for can_id in ARM_CAN_IDS:
            state = states[can_id]
            if state is None:
                raise BusError(f"no feedback from motor 0x{can_id:02x}; check power, CAN wiring, and port")
            positions.append(math.degrees(state.pos))
        self._update_health(states)
        return positions

    def _update_health(self, states: Dict[int, Any]):
        for cid, s in states.items():
            if s is not None:
                self._last_health[cid] = JointHealth.from_state(cid, s)

    def _send_targets_deg(self, targets_deg: Sequence[float], vel_deg_s: Optional[Sequence[float]] = None):
        vel = vel_deg_s or self.speeds

        def _do():
            with self.bus.lock:
                for i, can_id in enumerate(ARM_CAN_IDS):
                    motor = self.bus.motor(can_id)
                    pos_rad = math.radians(targets_deg[i])
                    if self.control_mode == "mit":
                        motor.send_mit(pos_rad, 0.0, self.mit_kp[i], self.mit_kd[i], 0.0)
                    else:
                        motor.send_pos_vel(pos_rad, math.radians(vel[i]))

        self._bus_call(_do)

    def _hold_current(self) -> List[float]:
        current = self._read_positions_deg()
        self._send_targets_deg(current)
        return current

    # ------------------------------------------------------- safety checks

    def _warn(self, key: str, msg: str, *args):
        now = time.monotonic()
        if now - self._last_warn.get(key, 0.0) >= _WARN_INTERVAL_S:
            self._last_warn[key] = now
            LOGGER.warning(msg, *args)

    def _check_ready(self, states: Dict[int, Any], auto_clear: bool = True):
        """Decode motor faults before a move. Transient faults are cleared once;
        hard faults raise MotorFault; hot motors raise OverTemperatureError."""
        cleared = False
        for i, cid in enumerate(ARM_CAN_IDS):
            s = states.get(cid)
            if s is None:
                raise BusError(f"no feedback from {JOINT_NAMES[i]} (0x{cid:02x})")
            health = JointHealth.from_state(cid, s)
            self._last_health[cid] = health
            if health.fault:
                if health.transient and auto_clear:
                    LOGGER.warning("%s reports %s; clearing", JOINT_NAMES[i], health.status)
                    self._bus_call(self.bus.motor(cid).clear_error)
                    cleared = True
                    continue
                raise MotorFault(JOINT_NAMES[i], health, hint='fix the cause, then send {"clear_errors": true}')
            if max(health.t_mos_c, health.t_rotor_c) >= self.temp_limit_c:
                raise OverTemperatureError(
                    f"{JOINT_NAMES[i]} is at {max(health.t_mos_c, health.t_rotor_c):.0f} C "
                    f"(limit {self.temp_limit_c:.0f} C); let it cool before moving"
                )
            if max(health.t_mos_c, health.t_rotor_c) >= self.temp_warn_c:
                self._warn(f"temp{cid}", "%s is warm: %.0f C", JOINT_NAMES[i], max(health.t_mos_c, health.t_rotor_c))
        if cleared:
            time.sleep(_SETTLE_SEC)
            self._configure_motors()
            self._check_ready(self._read_states(), auto_clear=False)

    def _check_limits(self, targets: Sequence[float], current: Sequence[float]) -> List[float]:
        """Reject (or clip, if configured) out-of-range targets; pin bad joints."""
        if len(targets) != N_JOINTS:
            raise ValueError(f"expected {N_JOINTS} joint positions, got {len(targets)}")
        out = list(targets)
        for i in self.bad_joints:
            out[i] = current[i]
        for i, target in enumerate(out):
            if i in self.bad_joints:
                continue
            lo, hi = self.joint_limits[i]
            if lo <= target <= hi:
                continue
            if not self.clip_targets:
                raise ValueError(f"{JOINT_NAMES[i]} target {target:.2f} deg is outside the limits [{lo:.1f}, {hi:.1f}]")
            clipped = _clamp(target, lo, hi)
            LOGGER.warning("clipped %s from %.2f to %.2f deg", JOINT_NAMES[i], target, clipped)
            out[i] = clipped
        return out

    def _monitor_tick(self, trip_counts: List[int]):
        """Cheap in-loop check of torque, faults, and temperature."""
        states = self._read_states(retries=1)
        for i, cid in enumerate(ARM_CAN_IDS):
            s = states.get(cid)
            if s is None:
                continue
            health = JointHealth.from_state(cid, s)
            self._last_health[cid] = health
            if health.fault:
                raise MotorFault(JOINT_NAMES[i], health, hint="move aborted")
            if max(health.t_mos_c, health.t_rotor_c) >= self.temp_limit_c:
                raise OverTemperatureError(f"{JOINT_NAMES[i]} over temperature during move; stopping")
            if self.torque_limits is not None:
                if abs(health.torque_nm) > self.torque_limits[i]:
                    trip_counts[i] += 1
                    if trip_counts[i] >= self.torque_trip_polls:
                        raise CollisionError(
                            f"{JOINT_NAMES[i]} torque {health.torque_nm:.2f} Nm exceeded the "
                            f"{self.torque_limits[i]:.2f} Nm limit for {trip_counts[i]} polls"
                        )
                else:
                    trip_counts[i] = 0

    # ------------------------------------------------------- move execution

    def _move_options(self, options=None, extra: Optional[Mapping[str, Any]] = None) -> MoveOptions:
        vmax = list(self.speeds)
        amax = list(self.accels)
        hz = self.move_hz
        direct = False
        interpolate = True
        wait_at_end = True
        if options is not None:
            joints_v = list(getattr(options, "max_vel_degs_per_sec_joints", []) or [])
            joints_a = list(getattr(options, "max_acc_degs_per_sec2_joints", []) or [])
            if len(joints_v) == N_JOINTS:
                vmax = [float(v) for v in joints_v]
            elif getattr(options, "max_vel_degs_per_sec", 0.0):
                vmax = [float(options.max_vel_degs_per_sec)] * N_JOINTS
            if len(joints_a) == N_JOINTS:
                amax = [float(a) for a in joints_a]
            elif getattr(options, "max_acc_degs_per_sec2", 0.0):
                amax = [float(options.max_acc_degs_per_sec2)] * N_JOINTS
        extra = extra or {}
        if "speed_d" in extra:
            vmax = _as_list(extra["speed_d"], N_JOINTS, "speed_d")
        if "speed_r" in extra:
            vmax = [math.degrees(v) for v in _as_list(extra["speed_r"], N_JOINTS, "speed_r")]
        if "acceleration_d" in extra:
            amax = _as_list(extra["acceleration_d"], N_JOINTS, "acceleration_d")
        if "acceleration_r" in extra:
            amax = [math.degrees(a) for a in _as_list(extra["acceleration_r"], N_JOINTS, "acceleration_r")]
        if "move_hz" in extra:
            hz = float(extra["move_hz"])
        if extra.get("direct") is True:
            direct = True
        if extra.get("interpolate") is False:
            interpolate = False
        if extra.get("waitAtEnd") is False or extra.get("wait_at_end") is False:
            wait_at_end = False
        vmax = [_clamp(v, MIN_SPEED_DEG_S, MAX_SPEED_DEG_S) for v in vmax]
        amax = [_clamp(a, 1.0, MAX_ACCEL_DEG_S2) for a in amax]
        return MoveOptions(
            vmax_deg_s=vmax, amax_deg_s2=amax, hz=hz, direct=direct, interpolate=interpolate, wait_at_end=wait_at_end
        )

    def _settle_timeout(self, targets: Sequence[float], current: Sequence[float], opts: MoveOptions) -> float:
        max_delta = max(abs(t - c) for t, c in zip(targets, current))
        return max(2.0, 3.0 * max_delta / max(min(opts.vmax_deg_s), 1.0))

    def _wait_settle(self, targets: Sequence[float], cancel: threading.Event, timeout: float, trip_counts: List[int]):
        deadline = time.monotonic() + timeout
        current = list(targets)
        while time.monotonic() < deadline:
            if cancel.is_set():
                return
            states = self._read_states()
            self._update_health(states)
            current = [math.degrees(states[c].pos) if states[c] is not None else float("nan") for c in ARM_CAN_IDS]
            if all(abs(t - c) <= self.tolerance_deg for t, c in zip(targets, current)):
                return
            time.sleep(_SETTLE_POLL_SEC)
        LOGGER.warning(
            "move timed out after %.1fs; position error %s deg",
            timeout,
            [round(t - c, 2) for t, c in zip(targets, current)],
        )

    def _stream_setpoints(
        self, timed_points: Iterable[Tuple[float, Sequence[float]]], opts: MoveOptions, cancel: threading.Event
    ):
        """Send (time_from_start, positions) setpoints paced on a monotonic
        anchor so per-tick sleep error does not accumulate. Runs safety
        monitoring at DEFAULT_MONITOR_HZ. Returns the last setpoint sent."""
        trip_counts = [0] * N_JOINTS
        monitor_period = 1.0 / DEFAULT_MONITOR_HZ
        anchor = None
        next_monitor = 0.0
        last: Optional[List[float]] = None
        for t, q in timed_points:
            if cancel.is_set():
                break
            if anchor is None:
                anchor = time.monotonic() - t
            due = anchor + t
            while True:
                now = time.monotonic()
                if now >= due or cancel.is_set():
                    break
                time.sleep(min(due - now, 0.005))
            if cancel.is_set():
                break
            self._send_targets_deg(q, opts.vmax_deg_s)
            last = list(q)
            if time.monotonic() >= next_monitor:
                next_monitor = time.monotonic() + monitor_period
                try:
                    self._monitor_tick(trip_counts)
                except (CollisionError, MotorFault, OverTemperatureError):
                    self._hold_current()
                    raise
        return last, trip_counts

    def _run_move(self, waypoints: List[List[float]], opts: MoveOptions):
        with self.ops.new() as cancel:
            self._ensure_not_manual()
            states = self._read_states()
            self._check_ready(states)
            current = [math.degrees(states[c].pos) for c in ARM_CAN_IDS]
            targets = [self._check_limits(w, current) for w in waypoints]
            if not targets:
                return
            final = targets[-1]
            if opts.direct or not opts.interpolate:
                if opts.direct and len(targets) > 1:
                    raise ValueError(f"direct moves take exactly one waypoint, got {len(targets)}")
                trip_counts = [0] * N_JOINTS
                for tgt in targets:
                    if cancel.is_set():
                        return
                    self._send_targets_deg(tgt, opts.vmax_deg_s)
                    if len(targets) > 1 or opts.wait_at_end:
                        self._wait_settle(tgt, cancel, self._settle_timeout(tgt, current, opts), trip_counts)
                    current = tgt
                return
            steps = plan(current, targets, opts)
            dt = 1.0 / opts.hz
            timed = (((k + 1) * dt, q) for k, q in enumerate(steps))
            _, trip_counts = self._stream_setpoints(timed, opts, cancel)
            if cancel.is_set():
                return
            if opts.wait_at_end:
                self._wait_settle(final, cancel, _POST_STREAM_SETTLE_S, trip_counts)

    def _stop_now(self):
        self.ops.cancel_current()
        with self.ops.new():
            self._exit_manual_mode_sync(restore=True)
            self._hold_current()

    def _ensure_not_manual(self):
        if self._manual_thread is not None and self._manual_thread.is_alive():
            raise RuntimeError('arm is in manual mode; send {"manual_mode": "exit"} before moving')

    # ---------------------------------------------------------- manual mode

    def _enter_manual_mode_sync(self):
        if self._manual_thread is not None and self._manual_thread.is_alive():
            return
        self.ops.cancel_current()
        states = self._read_states()
        self._check_ready(states)
        self._configure_motors(Mode.MIT)
        self._manual_stop.clear()
        self._manual_thread = threading.Thread(target=self._manual_loop, name="b601-manual", daemon=True)
        self._manual_thread.start()
        LOGGER.warning("manual mode entered: gravity compensation is experimental; keep a hand on the arm")

    def _exit_manual_mode_sync(self, restore: bool = True):
        thread = self._manual_thread
        if thread is None:
            return
        self._manual_stop.set()
        if thread.is_alive():
            thread.join(timeout=2.0)
        self._manual_thread = None
        if restore and self.bus is not None:
            try:
                self._configure_motors()
                self._hold_current()
            except Exception:
                LOGGER.warning("failed to restore position mode after manual mode", exc_info=True)

    def manual_torques(self, positions_deg: Sequence[float]) -> List[float]:
        """Feed-forward torques (Nm) that cancel gravity at the given pose."""
        rads = [math.radians(d) for d in positions_deg]
        g = spatial.gravity_torques(rads, self.gravity_vector, self.payload_kg)
        out = []
        for i, tau in enumerate(g):
            limit = spatial.JOINT_EFFORT_NM[i] or 1e9
            out.append(_clamp(-tau * self.gravity_scale, -limit, limit))
        return out

    def _manual_loop(self):
        period = 1.0 / DEFAULT_MANUAL_HZ
        while not self._manual_stop.is_set():
            start = time.monotonic()
            try:
                states = self._read_states(retries=1)
                pos = [math.degrees(states[c].pos) if states[c] is not None else None for c in ARM_CAN_IDS]
                if all(p is not None for p in pos):
                    taus = self.manual_torques(pos)

                    def _send(pos=pos, taus=taus):
                        with self.bus.lock:
                            for i, cid in enumerate(ARM_CAN_IDS):
                                self.bus.motor(cid).send_mit(
                                    math.radians(pos[i]), 0.0, self.manual_kp, self.manual_kd, taus[i]
                                )

                    self._bus_call(_send)
            except Exception:
                self._warn(
                    "manual",
                    "manual mode loop error",
                )
                LOGGER.debug("manual loop error", exc_info=True)
            self._manual_stop.wait(max(0.0, period - (time.monotonic() - start)))

    # ------------------------------------------------------------ Viam API

    async def get_joint_positions(self, *, extra=None, timeout=None, **kwargs) -> JointPositions:
        positions = await asyncio.to_thread(self._read_positions_deg)
        return JointPositions(values=positions)

    async def move_to_joint_positions(self, positions: JointPositions, *, extra=None, timeout=None, **kwargs):
        opts = self._move_options(None, extra)
        await asyncio.to_thread(self._run_move, [list(positions.values)], opts)

    async def move_through_joint_positions(
        self, positions: Sequence[JointPositions], options=None, *, extra=None, timeout=None, **kwargs
    ):
        waypoints = [list(p.values) for p in positions]
        opts = self._move_options(options, extra)
        await asyncio.to_thread(self._run_move, waypoints, opts)

    def start_streamed_move(self, extra: Optional[Mapping[str, Any]] = None) -> "StreamedMove":
        return StreamedMove(self, self._move_options(None, extra))

    async def get_end_position(self, *, extra=None, timeout=None, **kwargs) -> Pose:
        positions = await asyncio.to_thread(self._read_positions_deg)
        x, y, z, ox, oy, oz, theta = spatial.end_position(positions)
        return Pose(x=x, y=y, z=z, o_x=ox, o_y=oy, o_z=oz, theta=theta)

    async def move_to_position(self, pose: Pose, *, extra=None, timeout=None, **kwargs):
        if self.motion is None:
            raise NotImplementedError(
                "cartesian moves need a motion service: set the arm's 'motion' attribute to the "
                'name of a motion service (usually "builtin") and add the arm to the frame system'
            )
        destination = PoseInFrame(reference_frame=f"{self.name}_origin", pose=pose)
        await self.motion.move(component_name=self.name, destination=destination, extra=extra, timeout=timeout)

    async def stop(self, *, extra=None, timeout=None, **kwargs):
        await asyncio.to_thread(self._stop_now)

    async def is_moving(self) -> bool:
        if self.ops.running:
            return True
        states = await asyncio.to_thread(self._read_states, 1)
        return any(abs(s.vel) > _MOVING_VEL_RAD_S for s in states.values() if s is not None)

    async def get_kinematics(self, *, extra=None, timeout=None, **kwargs):
        return kinematics.arm_kinematics(self.collision_mode, self.include_gripper_geometry)

    async def get_geometries(self, *, extra=None, timeout=None, **kwargs) -> List[Geometry]:
        positions = await asyncio.to_thread(self._read_positions_deg)
        return kinematics.arm_geometries(positions, self.include_gripper_geometry)

    async def get_3d_models(self, *, extra=None, timeout=None, **kwargs) -> Dict[str, Mesh]:
        return kinematics.arm_3d_models(self.include_gripper_geometry)

    def _health_report(self) -> Dict[str, Any]:
        states = self._read_states()
        self._update_health(states)
        report = {}
        for i, cid in enumerate(ARM_CAN_IDS):
            h = self._last_health.get(cid)
            report[JOINT_NAMES[i]] = h.as_dict() if h else None
        report["manual_mode"] = self._manual_thread is not None and self._manual_thread.is_alive()
        report["torque_enabled"] = self._torque_enabled
        report["bus_reconnects"] = self.bus.reconnects if self.bus else 0
        return report

    async def do_command(self, command: Mapping[str, Any], *, timeout=None, **kwargs) -> Mapping[str, Any]:
        result: Dict[str, Any] = {}
        for name, arg in command.items():
            if name == "set_zero_position":

                def _zero():
                    with self.bus.lock:
                        for can_id in ARM_CAN_IDS:
                            self.bus.motor(can_id).set_zero_position()
                            time.sleep(0.1)

                await asyncio.to_thread(self._bus_call, _zero)
                result[name] = "ok; current pose is now the zero position for all 6 joints"
            elif name == "torque":
                if arg not in ("enable", "disable"):
                    raise ValueError("'torque' must be 'enable' or 'disable'")
                if arg == "enable":
                    await asyncio.to_thread(self._configure_motors)
                else:
                    await asyncio.to_thread(self._exit_manual_mode_sync, False)
                    await asyncio.to_thread(self._disable_motors)
                result[name] = arg + "d"
            elif name in ("clear_errors", "clear_error"):

                def _clear():
                    with self.bus.lock:
                        for can_id in ARM_CAN_IDS:
                            self.bus.motor(can_id).clear_error()

                await asyncio.to_thread(self._bus_call, _clear)
                if self._torque_enabled:
                    await asyncio.to_thread(self._configure_motors)
                result[name] = "ok"
            elif name == "raw_state":
                states = await asyncio.to_thread(self._read_states)
                result[name] = {
                    JOINT_NAMES[i]: (
                        {
                            "pos_deg": math.degrees(s.pos),
                            "vel_rad_s": s.vel,
                            "torque_nm": s.torq,
                            "t_mos_c": getattr(s, "t_mos", None),
                            "t_rotor_c": getattr(s, "t_rotor", None),
                            "status": damiao.status_text(int(getattr(s, "status_code", 1))),
                        }
                        if (s := states[cid]) is not None
                        else None
                    )
                    for i, cid in enumerate(ARM_CAN_IDS)
                }
            elif name in ("get_state", "status", "health"):
                result[name] = await asyncio.to_thread(self._health_report)
            elif name == "load":
                states = await asyncio.to_thread(self._read_states)
                result[name] = [states[c].torq if states[c] is not None else None for c in ARM_CAN_IDS]
            elif name == "set_speed":
                self.speeds = [
                    _clamp(v, MIN_SPEED_DEG_S, MAX_SPEED_DEG_S) for v in _as_list(arg, N_JOINTS, "set_speed")
                ]
                result[name] = self.speeds
            elif name == "set_acceleration":
                self.accels = [_clamp(a, 1.0, MAX_ACCEL_DEG_S2) for a in _as_list(arg, N_JOINTS, "set_acceleration")]
                result[name] = self.accels
            elif name == "get_speed":
                result[name] = self.speeds
            elif name == "get_acceleration":
                result[name] = self.accels
            elif name == "manual_mode" or name in ("enter_manual_mode", "exit_manual_mode"):
                action = arg if name == "manual_mode" else ("enter" if name == "enter_manual_mode" else "exit")
                if action == "enter":
                    await asyncio.to_thread(self._enter_manual_mode_sync)
                    result[name] = "entered manual mode"
                elif action == "exit":
                    await asyncio.to_thread(self._exit_manual_mode_sync, True)
                    result[name] = "exited manual mode"
                else:
                    raise ValueError("'manual_mode' must be 'enter' or 'exit'")
            elif name == "gravity_torques":
                positions = await asyncio.to_thread(self._read_positions_deg)
                result[name] = self.manual_torques(positions)
            else:
                raise ValueError(f"unknown command '{name}'")
        return result

    async def close(self):
        if self.bus is not None:

            def _shutdown():
                self.ops.cancel_current()
                self._exit_manual_mode_sync(restore=False)
                if self.disable_torque_on_close:
                    try:
                        self._disable_motors()
                    except Exception:
                        LOGGER.warning("failed to disable arm motors on close", exc_info=True)
                self._release_bus()

            await asyncio.to_thread(_shutdown)


class StreamedMove:
    """Executes a trajectory that arrives incrementally (MoveThroughJointPositionsStreamed).

    Points are (time_from_start_s, positions_deg). The first point anchors the
    clock; each later point is sent at anchor + time. Points that are already
    past due are sent immediately. ``finish()`` blocks until the worker has
    drained the queue and, if configured, the arm has settled on the last
    point.
    """

    _SENTINEL = object()

    def __init__(self, arm: B601Arm, opts: MoveOptions):
        self.arm = arm
        self.opts = opts
        self._queue: "queue.Queue" = queue.Queue()
        self._error: Optional[BaseException] = None
        self._thread = threading.Thread(target=self._run, name="b601-stream", daemon=True)
        self._started = False

    def feed(self, points: Sequence[Tuple[float, Sequence[float]]]):
        if not self._started:
            self._started = True
            self._thread.start()
        for p in points:
            self._queue.put(p)

    def finish(self):
        if not self._started:
            return
        self._queue.put(self._SENTINEL)
        self._thread.join()
        if self._error is not None:
            raise self._error

    def abort(self):
        self.arm.ops.cancel_current()
        self._queue.put(self._SENTINEL)

    def _points(self, cancel: threading.Event):
        while True:
            item = self._queue.get()
            if item is self._SENTINEL or cancel.is_set():
                return
            yield item

    def _run(self):
        try:
            with self.arm.ops.new() as cancel:
                self.arm._ensure_not_manual()
                states = self.arm._read_states()
                self.arm._check_ready(states)
                current = [math.degrees(states[c].pos) for c in ARM_CAN_IDS]

                def checked():
                    for t, q in self._points(cancel):
                        yield t, self.arm._check_limits(list(q), current)

                last, trip_counts = self.arm._stream_setpoints(checked(), self.opts, cancel)
                if last is not None and self.opts.wait_at_end and not cancel.is_set():
                    self.arm._wait_settle(last, cancel, _POST_STREAM_SETTLE_S, trip_counts)
        except BaseException as exc:  # surfaced to the RPC handler
            self._error = exc
