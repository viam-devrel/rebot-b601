"""Viam gripper component for the reBot Arm B601-DM parallel gripper (motor 0x07).

The gripper motor drives a leadscrew, so its native "position" is motor
rotation in degrees: 0 deg is fully closed (the calibration pose) and about
-270 deg is fully open. It runs in FORCE_POS mode so grab force is capped by a
torque ratio, letting it stall gently on an object.

Kinematic inputs (``get_current_inputs``/``go_to_inputs``) are the left
finger's travel in metres (0 = closed, 0.05 = fully open), matching the one
prismatic joint in the URDF served by ``get_kinematics``. Motor degrees are
still available through the ``get``/``set`` DoCommands.
"""

import asyncio
import math
import time
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from motorbridge import Mode
from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.utils import struct_to_dict

from . import kinematics
from .bus import DEFAULT_BAUD, LINK_ERRORS, BusError, SharedBus, detect_port, is_motor_timeout
from .damiao import JointHealth, MotorFault
from .ops import SingleOperationManager

LOGGER = getLogger(__name__)

GRIPPER_CAN_ID = 0x07

DEFAULT_OPEN_DEG = -270.0
DEFAULT_CLOSED_DEG = 0.0
DEFAULT_SPEED_DEG_S = 900.0
MIN_SPEED_DEG_S = 10.0
MAX_SPEED_DEG_S = 3000.0
DEFAULT_TORQUE_RATIO = 0.07  # max grip force in [0, 1]
# If the gripper stalls at least this far (deg of motor rotation) short of the
# fully-closed position, we consider it to be holding something.
DEFAULT_HOLDING_THRESHOLD_DEG = 15.0
DEFAULT_STALL_POLLS = 4  # consecutive near-zero-velocity polls that count as settled
DEFAULT_MOVE_TIMEOUT_S = 6.0

_ENSURE_MODE_RETRIES = 9
_SETTLE_SEC = 0.02
_MOVING_VEL_RAD_S = 0.05
_POLL_SEC = 0.05
_ARRIVE_TOL_DEG = 2.0


class B601Gripper(Gripper, EasyResource):
    MODEL = "devrel:rebot-b601:gripper"

    def __init__(self, name: str):
        super().__init__(name)
        self.bus: Optional[SharedBus] = None
        self.ops = SingleOperationManager()
        self._holding = False
        self._last_stall_deg: Optional[float] = None
        self._torque_enabled = False

    # ------------------------------------------------------------------ config

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        # EasyResource's default new() does not call reconfigure; see B601Arm.
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
        ratio = float(attrs.get("torque_ratio", DEFAULT_TORQUE_RATIO))
        if not 0.0 < ratio <= 1.0:
            raise ValueError("torque_ratio must be in (0, 1]")
        cg = attrs.get("collision_geometry", "primitives")
        if cg not in kinematics.COLLISION_MODES:
            raise ValueError(f"collision_geometry must be one of {kinematics.COLLISION_MODES}")
        deps: List[str] = []
        if attrs.get("arm"):
            deps.append(str(attrs["arm"]))
        return deps, []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        attrs = struct_to_dict(config.attributes)
        port = attrs.get("port")
        baud = attrs.get("baud")
        arm_name = attrs.get("arm")
        if arm_name:
            # Inherit the bus settings from the arm when it is a local B601Arm.
            for rn, dep in dependencies.items():
                if rn.name == arm_name and hasattr(dep, "bus") and dep.bus is not None:
                    port = port or dep.bus.port
                    baud = baud or dep.bus.baud
        port = port or detect_port()
        baud = int(baud or DEFAULT_BAUD)

        self.open_deg = float(attrs.get("open_position_deg", DEFAULT_OPEN_DEG))
        self.closed_deg = float(attrs.get("closed_position_deg", DEFAULT_CLOSED_DEG))
        self.speed_deg_s = float(attrs.get("speed_deg_s", DEFAULT_SPEED_DEG_S))
        self.torque_ratio = float(attrs.get("torque_ratio", DEFAULT_TORQUE_RATIO))
        self.holding_threshold_deg = float(attrs.get("holding_threshold_deg", DEFAULT_HOLDING_THRESHOLD_DEG))
        self.stall_polls = int(attrs.get("stall_polls", DEFAULT_STALL_POLLS))
        self.move_timeout_s = float(attrs.get("move_timeout_s", DEFAULT_MOVE_TIMEOUT_S))
        self.collision_mode = attrs.get("collision_geometry", "primitives")
        self.reconnect_enabled = bool(attrs.get("reconnect", True))

        if self.bus is not None and not self.bus.matches(port, baud):
            self._release_bus()
        if self.bus is None:
            self.bus = SharedBus.acquire(port, baud)
            self.bus.on_reconnect(self._on_bus_reconnect)

        self._configure_motor()
        LOGGER.info("B601 gripper '%s' ready on %s", self.name, port)

    # ------------------------------------------------------ hardware helpers

    def _bus_call(self, fn, *args, **kwargs):
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
        if self._torque_enabled:
            try:
                self._configure_motor()
            except Exception:
                LOGGER.warning("failed to restore gripper motor after reconnect", exc_info=True)

    def _configure_motor(self):
        def _do():
            with self.bus.lock:
                motor = self.bus.motor(GRIPPER_CAN_ID)
                motor.enable()
                for attempt in range(_ENSURE_MODE_RETRIES + 1):
                    try:
                        motor.ensure_mode(Mode.FORCE_POS)
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

        self._bus_call(_do)
        self._torque_enabled = True

    def _state(self):
        state = self._bus_call(self.bus.poll_feedback, [GRIPPER_CAN_ID])[GRIPPER_CAN_ID]
        if state is None:
            raise BusError("no feedback from gripper motor 0x07; check power and wiring")
        return state

    def _check_ready(self, state):
        health = JointHealth.from_state(GRIPPER_CAN_ID, state)
        if health.fault:
            if health.transient:
                self._bus_call(self.bus.motor(GRIPPER_CAN_ID).clear_error)
                self._configure_motor()
                return
            raise MotorFault("gripper", health, hint='fix the cause, then send {"clear_errors": true}')

    def _send_target(
        self, target_deg: float, speed_deg_s: Optional[float] = None, torque_ratio: Optional[float] = None
    ):
        speed = speed_deg_s if speed_deg_s is not None else self.speed_deg_s
        ratio = torque_ratio if torque_ratio is not None else self.torque_ratio

        def _do():
            with self.bus.lock:
                self.bus.motor(GRIPPER_CAN_ID).send_force_pos(math.radians(target_deg), math.radians(speed), ratio)

        self._bus_call(_do)

    def _clamp_deg(self, target_deg: float) -> float:
        lo, hi = min(self.open_deg, self.closed_deg), max(self.open_deg, self.closed_deg)
        return max(lo, min(hi, float(target_deg)))

    def _move_until_settled(self, target_deg: float, speed_deg_s=None, torque_ratio=None) -> float:
        """Command target and wait for stall or arrival; returns final position (deg)."""
        with self.ops.new() as cancel:
            state = self._state()
            self._check_ready(state)
            target_deg = self._clamp_deg(target_deg)
            self._send_target(target_deg, speed_deg_s, torque_ratio)
            deadline = time.monotonic() + self.move_timeout_s
            still = 0
            pos = math.degrees(state.pos)
            # Give it a moment to start moving before stall detection kicks in.
            time.sleep(0.2)
            while time.monotonic() < deadline:
                if cancel.is_set():
                    break
                state = self._state()
                pos = math.degrees(state.pos)
                if abs(pos - target_deg) < _ARRIVE_TOL_DEG:
                    break
                still = still + 1 if abs(state.vel) < _MOVING_VEL_RAD_S else 0
                if still >= self.stall_polls:
                    break  # stalled (on an object, or at the mechanical limit)
                time.sleep(_POLL_SEC)
            self._last_stall_deg = pos
            return pos

    # ------------------------------------------------- unit conversions

    def fraction_from_deg(self, pos_deg: float) -> float:
        """0.0 = closed, 1.0 = fully open."""
        span = self.open_deg - self.closed_deg
        if abs(span) < 1e-9:
            return 0.0
        return max(0.0, min(1.0, (pos_deg - self.closed_deg) / span))

    def deg_from_fraction(self, fraction: float) -> float:
        fraction = max(0.0, min(1.0, float(fraction)))
        return self.closed_deg + fraction * (self.open_deg - self.closed_deg)

    def travel_m_from_deg(self, pos_deg: float) -> float:
        return self.fraction_from_deg(pos_deg) * kinematics.FINGER_TRAVEL_M

    def deg_from_travel_m(self, travel_m: float) -> float:
        return self.deg_from_fraction(float(travel_m) / kinematics.FINGER_TRAVEL_M)

    # ---------------------------------------------------------- Viam API

    async def open(self, *, extra=None, timeout=None, **kwargs):
        await asyncio.to_thread(self._move_until_settled, self.open_deg)
        self._holding = False

    async def grab(self, *, extra=None, timeout=None, **kwargs) -> bool:
        pos = await asyncio.to_thread(self._move_until_settled, self.closed_deg)
        # Stalling well short of fully closed means the jaws met an object.
        self._holding = abs(pos - self.closed_deg) > self.holding_threshold_deg
        return self._holding

    async def is_holding_something(self, *, extra=None, timeout=None, **kwargs) -> Gripper.HoldingStatus:
        meta = {"stall_position_deg": self._last_stall_deg} if self._last_stall_deg is not None else {}
        return Gripper.HoldingStatus(is_holding_something=self._holding, meta=meta)

    async def stop(self, *, extra=None, timeout=None, **kwargs):
        def _stop():
            self.ops.cancel_current()
            with self.ops.new():
                pos = math.degrees(self._state().pos)
                self._send_target(pos)

        await asyncio.to_thread(_stop)

    async def is_moving(self) -> bool:
        if self.ops.running:
            return True
        state = await asyncio.to_thread(self._state)
        return abs(state.vel) > _MOVING_VEL_RAD_S

    async def get_kinematics(self, *, extra=None, timeout=None, **kwargs):
        return kinematics.gripper_kinematics(self.collision_mode)

    async def get_geometries(self, *, extra=None, timeout=None, **kwargs) -> List[Geometry]:
        state = await asyncio.to_thread(self._state)
        return kinematics.gripper_geometries(self.travel_m_from_deg(math.degrees(state.pos)))

    async def get_current_inputs(self, *, extra=None, timeout=None, **kwargs):
        """Single input: left-finger travel in metres (0 = closed, 0.05 = fully open)."""
        state = await asyncio.to_thread(self._state)
        return [self.travel_m_from_deg(math.degrees(state.pos))]

    async def go_to_inputs(self, values, *, extra=None, timeout=None, **kwargs):
        if len(values) != 1:
            raise ValueError(f"expected 1 input (finger travel in metres), got {len(values)}")
        await asyncio.to_thread(self._move_until_settled, self.deg_from_travel_m(values[0]))

    async def do_command(self, command: Mapping[str, Any], *, timeout=None, **kwargs) -> Mapping[str, Any]:
        result: Dict[str, Any] = {}
        for name, arg in command.items():
            if name == "set_zero_position":
                await asyncio.to_thread(self._bus_call, self.bus.motor(GRIPPER_CAN_ID).set_zero_position)
                result[name] = "ok; current (closed) position is now zero"
            elif name == "torque":
                if arg not in ("enable", "disable"):
                    raise ValueError("'torque' must be 'enable' or 'disable'")

                def _torque(arg=arg):
                    if arg == "enable":
                        self._configure_motor()
                    else:
                        with self.bus.lock:
                            self.bus.motor(GRIPPER_CAN_ID).disable()
                        self._torque_enabled = False

                await asyncio.to_thread(self._bus_call, _torque)
                result[name] = arg + "d"
            elif name in ("clear_errors", "clear_error"):
                await asyncio.to_thread(self._bus_call, self.bus.motor(GRIPPER_CAN_ID).clear_error)
                await asyncio.to_thread(self._configure_motor)
                result[name] = "ok"
            elif name in ("raw_state", "get_state", "status", "health"):
                state = await asyncio.to_thread(self._state)
                h = JointHealth.from_state(GRIPPER_CAN_ID, state).as_dict()
                h["open_fraction"] = self.fraction_from_deg(h["pos_deg"])
                h["holding"] = self._holding
                result[name] = h
            elif name == "get":
                state = await asyncio.to_thread(self._state)
                pos = math.degrees(state.pos)
                result[name] = {"pos_deg": pos, "open_fraction": self.fraction_from_deg(pos)}
            elif name == "set":
                # number: motor degrees; {"fraction": f} or {"deg": d}
                if isinstance(arg, Mapping):
                    if "fraction" in arg:
                        target = self.deg_from_fraction(float(arg["fraction"]))
                    elif "deg" in arg:
                        target = float(arg["deg"])
                    else:
                        raise ValueError('\'set\' takes a number (deg) or {"fraction": f} / {"deg": d}')
                else:
                    target = float(arg)
                pos = await asyncio.to_thread(self._move_until_settled, target)
                result[name] = {"pos_deg": pos, "open_fraction": self.fraction_from_deg(pos)}
            elif name in ("set_speed", "set_gripper_speed"):
                self.speed_deg_s = max(MIN_SPEED_DEG_S, min(MAX_SPEED_DEG_S, float(arg)))
                result[name] = self.speed_deg_s
            elif name in ("get_speed", "get_gripper_speed"):
                result[name] = self.speed_deg_s
            elif name in ("set_force", "set_torque", "set_gripper_torque"):
                ratio = float(arg)
                if not 0.0 < ratio <= 1.0:
                    raise ValueError("force/torque ratio must be in (0, 1]")
                self.torque_ratio = ratio
                result[name] = self.torque_ratio
            elif name in ("get_force", "get_torque", "get_gripper_torque"):
                result[name] = self.torque_ratio
            elif name in ("grab_with_force", "grab_with_torque"):
                params = arg if isinstance(arg, Mapping) else {}
                if "fraction" in params:
                    target = self.deg_from_fraction(float(params["fraction"]))
                else:
                    target = float(params.get("position", params.get("deg", self.closed_deg)))
                speed = float(params["speed"]) if "speed" in params else None
                ratio = float(params.get("force", params.get("torque", self.torque_ratio)))
                if not 0.0 < ratio <= 1.0:
                    raise ValueError("force/torque ratio must be in (0, 1]")
                pos = await asyncio.to_thread(self._move_until_settled, target, speed, ratio)
                self._holding = abs(pos - target) > self.holding_threshold_deg
                result[name] = {"pos_deg": pos, "holding": self._holding}
            else:
                raise ValueError(f"unknown command '{name}'")
        return result

    async def close(self):
        if self.bus is not None:

            def _shutdown():
                self.ops.cancel_current()
                self._release_bus()

            await asyncio.to_thread(_shutdown)
