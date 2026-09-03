"""Viam arm component for the Seeed Studio reBot Arm B601-DM (6 DoF, Damiao CAN motors)."""

import asyncio
import math
import threading
import time
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from viam.components.arm import Arm, JointPositions, KinematicsFileFormat, Pose
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.utils import struct_to_dict

from motorbridge import Mode

from . import spatial
from .bus import DEFAULT_BAUD, SharedBus, detect_port

LOGGER = getLogger(__name__)

ARM_CAN_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Defaults mirror Seeed's LeRobot reference implementation for this arm.
DEFAULT_SPEED_DEG_S = 60.0
DEFAULT_MIT_KP = [45.0, 45.0, 45.0, 8.0, 9.0, 8.0]
DEFAULT_MIT_KD = [12.0, 12.0, 12.0, 1.0, 1.0, 1.0]
# Soft limits (deg); slightly inside the URDF limits by default.
DEFAULT_JOINT_LIMITS = [(-150.0, 150.0), (-179.0, 1.0), (-179.0, 1.0), (-107.0, 89.0), (-89.0, 89.0), (-179.0, 179.0)]

_ENSURE_MODE_RETRIES = 9
_SETTLE_SEC = 0.01
_MOVING_VEL_RAD_S = 0.05
_DEFAULT_TOLERANCE_DEG = 2.0
_MOVE_POLL_SEC = 0.05


def _as_list(value, n: int, name: str) -> List[float]:
    if isinstance(value, (int, float)):
        return [float(value)] * n
    values = [float(v) for v in value]
    if len(values) != n:
        raise ValueError(f"'{name}' must be a number or a list of {n} numbers")
    return values


class B601Arm(Arm, EasyResource):
    MODEL = "devrel:rebot-b601:arm"

    def __init__(self, name: str):
        super().__init__(name)
        self.bus: Optional[SharedBus] = None
        self._target_deg: Optional[List[float]] = None
        self._move_active = threading.Event()
        self._stop_requested = threading.Event()

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        mode = attrs.get("control_mode", "pos_vel")
        if mode not in ("pos_vel", "mit"):
            raise ValueError("control_mode must be 'pos_vel' or 'mit'")
        for key in ("speed_deg_s", "mit_kp", "mit_kd"):
            if key in attrs:
                _as_list(attrs[key], 6, key)
        if "joint_limits_deg" in attrs:
            limits = attrs["joint_limits_deg"]
            if len(limits) != 6 or any(len(pair) != 2 for pair in limits):
                raise ValueError("joint_limits_deg must be a list of 6 [min, max] pairs")
        return [], []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        attrs = struct_to_dict(config.attributes)
        port = attrs.get("port") or detect_port()
        baud = int(attrs.get("baud", DEFAULT_BAUD))

        self.control_mode = attrs.get("control_mode", "pos_vel")
        self.speeds = _as_list(attrs.get("speed_deg_s", DEFAULT_SPEED_DEG_S), 6, "speed_deg_s")
        self.mit_kp = _as_list(attrs.get("mit_kp", DEFAULT_MIT_KP), 6, "mit_kp")
        self.mit_kd = _as_list(attrs.get("mit_kd", DEFAULT_MIT_KD), 6, "mit_kd")
        self.tolerance_deg = float(attrs.get("tolerance_deg", _DEFAULT_TOLERANCE_DEG))
        self.enable_on_start = bool(attrs.get("enable_on_start", True))
        # Off by default: disabling torque makes the arm slump under gravity,
        # and close() also runs on every reconfigure.
        self.disable_torque_on_close = bool(attrs.get("disable_torque_on_close", False))
        if "joint_limits_deg" in attrs:
            self.joint_limits = [(float(lo), float(hi)) for lo, hi in attrs["joint_limits_deg"]]
        else:
            self.joint_limits = list(DEFAULT_JOINT_LIMITS)

        if self.bus is not None and (self.bus.port != port or self.bus.baud != baud):
            self.bus.release()
            self.bus = None
        if self.bus is None:
            self.bus = SharedBus.acquire(port, baud)
        self._target_deg = None

        if self.enable_on_start:
            self._configure_motors()
        LOGGER.info(
            "B601 arm '%s' ready on %s (mode=%s, torque %s)",
            self.name, port, self.control_mode,
            "enabled" if self.enable_on_start else "disabled",
        )

    # --- sync hardware helpers (called via asyncio.to_thread) ---

    def _configure_motors(self):
        target_mode = Mode.MIT if self.control_mode == "mit" else Mode.POS_VEL
        with self.bus.lock:
            for can_id in ARM_CAN_IDS:
                motor = self.bus.motor(can_id)
                motor.enable()
                for attempt in range(_ENSURE_MODE_RETRIES + 1):
                    try:
                        motor.ensure_mode(target_mode)
                        break
                    except Exception:
                        if attempt == _ENSURE_MODE_RETRIES:
                            raise
                        time.sleep(_SETTLE_SEC)

    def _read_positions_deg(self) -> List[float]:
        states = self.bus.poll_feedback(ARM_CAN_IDS)
        positions = []
        for can_id in ARM_CAN_IDS:
            state = states[can_id]
            if state is None:
                raise RuntimeError(
                    f"no feedback from motor 0x{can_id:02x}; check power, CAN wiring, and port"
                )
            positions.append(math.degrees(state.pos))
        return positions

    def _read_velocities_rad_s(self) -> List[float]:
        states = self.bus.poll_feedback(ARM_CAN_IDS)
        return [abs(states[cid].vel) if states[cid] is not None else 0.0 for cid in ARM_CAN_IDS]

    def _send_targets_deg(self, targets_deg: List[float]):
        with self.bus.lock:
            for i, can_id in enumerate(ARM_CAN_IDS):
                motor = self.bus.motor(can_id)
                pos_rad = math.radians(targets_deg[i])
                if self.control_mode == "mit":
                    motor.send_mit(pos_rad, 0.0, self.mit_kp[i], self.mit_kd[i], 0.0)
                else:
                    motor.send_pos_vel(pos_rad, math.radians(self.speeds[i]))

    def _clip(self, targets_deg: List[float]) -> List[float]:
        clipped = []
        for i, target in enumerate(targets_deg):
            lo, hi = self.joint_limits[i]
            value = max(lo, min(hi, target))
            if value != target:
                LOGGER.warning("clipped %s from %.2f to %.2f deg", JOINT_NAMES[i], target, value)
            clipped.append(value)
        return clipped

    def _move_blocking(self, targets_deg: List[float]):
        """Send targets and wait until the arm settles, stop() fires, or timeout."""
        self._stop_requested.clear()
        self._move_active.set()
        try:
            current = self._read_positions_deg()
            self._target_deg = targets_deg
            self._send_targets_deg(targets_deg)

            max_delta = max(abs(t - c) for t, c in zip(targets_deg, current))
            min_speed = min(self.speeds) if self.control_mode == "pos_vel" else 90.0
            timeout = max(2.0, 3.0 * max_delta / max(min_speed, 1.0))

            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if self._stop_requested.is_set():
                    return
                current = self._read_positions_deg()
                if all(abs(t - c) <= self.tolerance_deg for t, c in zip(targets_deg, current)):
                    return
                time.sleep(_MOVE_POLL_SEC)
            LOGGER.warning(
                "move timed out after %.1fs; position error %s deg",
                timeout,
                [round(t - c, 2) for t, c in zip(targets_deg, current)],
            )
        finally:
            self._move_active.clear()

    def _stop_now(self):
        self._stop_requested.set()
        current = self._read_positions_deg()
        self._target_deg = None
        self._send_targets_deg(current)

    # --- Viam arm API ---

    async def get_joint_positions(self, *, extra=None, timeout=None, **kwargs) -> JointPositions:
        positions = await asyncio.to_thread(self._read_positions_deg)
        return JointPositions(values=positions)

    async def move_to_joint_positions(self, positions: JointPositions, *, extra=None, timeout=None, **kwargs):
        targets = list(positions.values)
        if len(targets) != 6:
            raise ValueError(f"expected 6 joint positions, got {len(targets)}")
        targets = self._clip(targets)
        await asyncio.to_thread(self._move_blocking, targets)

    async def get_end_position(self, *, extra=None, timeout=None, **kwargs) -> Pose:
        positions = await asyncio.to_thread(self._read_positions_deg)
        x, y, z, ox, oy, oz, theta = spatial.end_position(positions)
        return Pose(x=x, y=y, z=z, o_x=ox, o_y=oy, o_z=oz, theta=theta)

    async def move_to_position(self, pose: Pose, *, extra=None, timeout=None, **kwargs):
        raise NotImplementedError(
            "cartesian moves are not implemented on the arm directly; add this arm to the "
            "frame system and use the motion service, which plans using the arm's kinematics"
        )

    async def stop(self, *, extra=None, timeout=None, **kwargs):
        await asyncio.to_thread(self._stop_now)

    async def is_moving(self) -> bool:
        if self._move_active.is_set():
            return True
        velocities = await asyncio.to_thread(self._read_velocities_rad_s)
        return any(v > _MOVING_VEL_RAD_S for v in velocities)

    async def get_kinematics(self, *, extra=None, timeout=None, **kwargs) -> Tuple[KinematicsFileFormat.ValueType, bytes]:
        return (KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF, spatial.URDF_PATH.read_bytes())

    async def do_command(self, command: Mapping[str, Any], *, timeout=None, **kwargs) -> Mapping[str, Any]:
        result: Dict[str, Any] = {}
        for name, arg in command.items():
            if name == "set_zero_position":
                def _zero():
                    with self.bus.lock:
                        for can_id in ARM_CAN_IDS:
                            self.bus.motor(can_id).set_zero_position()
                            time.sleep(0.1)
                await asyncio.to_thread(_zero)
                result[name] = "ok; current pose is now the zero position for all 6 joints"
            elif name == "torque":
                if arg not in ("enable", "disable"):
                    raise ValueError("'torque' must be 'enable' or 'disable'")
                def _torque():
                    if arg == "enable":
                        self._configure_motors()
                    else:
                        with self.bus.lock:
                            for can_id in ARM_CAN_IDS:
                                self.bus.motor(can_id).disable()
                await asyncio.to_thread(_torque)
                result[name] = arg + "d"
            elif name == "clear_errors":
                def _clear():
                    with self.bus.lock:
                        for can_id in ARM_CAN_IDS:
                            self.bus.motor(can_id).clear_error()
                await asyncio.to_thread(_clear)
                result[name] = "ok"
            elif name == "raw_state":
                states = await asyncio.to_thread(self.bus.poll_feedback, ARM_CAN_IDS)
                result[name] = {
                    JOINT_NAMES[i]: (
                        {"pos_deg": math.degrees(s.pos), "vel_rad_s": s.vel, "torque_nm": s.torq}
                        if (s := states[cid]) is not None
                        else None
                    )
                    for i, cid in enumerate(ARM_CAN_IDS)
                }
            else:
                raise ValueError(f"unknown command '{name}'")
        return result

    async def close(self):
        if self.bus is not None:
            def _shutdown():
                if self.disable_torque_on_close:
                    try:
                        with self.bus.lock:
                            for can_id in ARM_CAN_IDS:
                                self.bus.motor(can_id).disable()
                    except Exception:
                        LOGGER.warning("failed to disable arm motors on close", exc_info=True)
                self.bus.release()
            await asyncio.to_thread(_shutdown)
            self.bus = None
