"""Viam gripper component for the reBot Arm B601-DM parallel gripper (motor 0x07).

The gripper motor drives a leadscrew, so "position" is motor rotation in
degrees: 0 deg is fully closed (the calibration pose) and about -270 deg is
fully open. It runs in FORCE_POS mode so grab force is capped by a torque
ratio, letting it stall gently on an object.
"""

import asyncio
import math
import threading
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.utils import struct_to_dict

from motorbridge import Mode

from .bus import DEFAULT_BAUD, SharedBus, detect_port

LOGGER = getLogger(__name__)

GRIPPER_CAN_ID = 0x07

DEFAULT_OPEN_DEG = -270.0
DEFAULT_CLOSED_DEG = 0.0
DEFAULT_SPEED_DEG_S = 900.0
DEFAULT_TORQUE_RATIO = 0.07  # max grip force in [0, 1]
# If the gripper stalls at least this far (deg of motor rotation) short of the
# fully-closed position, we consider it to be holding something.
DEFAULT_HOLDING_THRESHOLD_DEG = 15.0

_ENSURE_MODE_RETRIES = 9
_SETTLE_SEC = 0.01
_MOVING_VEL_RAD_S = 0.05
_POLL_SEC = 0.05
_STALL_POLLS = 4  # consecutive near-zero-velocity polls that count as settled
_MOVE_TIMEOUT_S = 6.0


class B601Gripper(Gripper, EasyResource):
    MODEL = "devrel:rebot-b601:gripper"

    def __init__(self, name: str):
        super().__init__(name)
        self.bus: Optional[SharedBus] = None
        self._move_active = threading.Event()
        self._stop_requested = threading.Event()
        self._holding = False

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        ratio = float(attrs.get("torque_ratio", DEFAULT_TORQUE_RATIO))
        if not 0.0 < ratio <= 1.0:
            raise ValueError("torque_ratio must be in (0, 1]")
        return [], []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        attrs = struct_to_dict(config.attributes)
        port = attrs.get("port") or detect_port()
        baud = int(attrs.get("baud", DEFAULT_BAUD))

        self.open_deg = float(attrs.get("open_position_deg", DEFAULT_OPEN_DEG))
        self.closed_deg = float(attrs.get("closed_position_deg", DEFAULT_CLOSED_DEG))
        self.speed_deg_s = float(attrs.get("speed_deg_s", DEFAULT_SPEED_DEG_S))
        self.torque_ratio = float(attrs.get("torque_ratio", DEFAULT_TORQUE_RATIO))
        self.holding_threshold_deg = float(
            attrs.get("holding_threshold_deg", DEFAULT_HOLDING_THRESHOLD_DEG)
        )

        if self.bus is not None and (self.bus.port != port or self.bus.baud != baud):
            self.bus.release()
            self.bus = None
        if self.bus is None:
            self.bus = SharedBus.acquire(port, baud)

        self._configure_motor()
        LOGGER.info("B601 gripper '%s' ready on %s", self.name, port)

    # --- sync hardware helpers ---

    def _configure_motor(self):
        with self.bus.lock:
            motor = self.bus.motor(GRIPPER_CAN_ID)
            motor.enable()
            for attempt in range(_ENSURE_MODE_RETRIES + 1):
                try:
                    motor.ensure_mode(Mode.FORCE_POS)
                    break
                except Exception:
                    if attempt == _ENSURE_MODE_RETRIES:
                        raise
                    time.sleep(_SETTLE_SEC)

    def _state(self):
        state = self.bus.poll_feedback([GRIPPER_CAN_ID])[GRIPPER_CAN_ID]
        if state is None:
            raise RuntimeError("no feedback from gripper motor 0x07; check power and wiring")
        return state

    def _send_target(self, target_deg: float):
        with self.bus.lock:
            self.bus.motor(GRIPPER_CAN_ID).send_force_pos(
                math.radians(target_deg), math.radians(self.speed_deg_s), self.torque_ratio
            )

    def _move_until_settled(self, target_deg: float) -> float:
        """Command target and wait for stall or arrival; returns final position (deg)."""
        self._stop_requested.clear()
        self._move_active.set()
        try:
            self._send_target(target_deg)
            deadline = time.monotonic() + _MOVE_TIMEOUT_S
            still = 0
            pos = math.degrees(self._state().pos)
            # Give it a moment to start moving before stall detection kicks in.
            time.sleep(0.2)
            while time.monotonic() < deadline:
                if self._stop_requested.is_set():
                    break
                state = self._state()
                pos = math.degrees(state.pos)
                if abs(pos - target_deg) < 2.0:
                    break
                still = still + 1 if abs(state.vel) < _MOVING_VEL_RAD_S else 0
                if still >= _STALL_POLLS:
                    break  # stalled (on an object, or at the mechanical limit)
                time.sleep(_POLL_SEC)
            return pos
        finally:
            self._move_active.clear()

    # --- Viam gripper API ---

    async def open(self, *, extra=None, timeout=None, **kwargs):
        await asyncio.to_thread(self._move_until_settled, self.open_deg)
        self._holding = False

    async def grab(self, *, extra=None, timeout=None, **kwargs) -> bool:
        pos = await asyncio.to_thread(self._move_until_settled, self.closed_deg)
        # Stalling well short of fully closed means the jaws met an object.
        self._holding = abs(pos - self.closed_deg) > self.holding_threshold_deg
        return self._holding

    async def is_holding_something(self, *, extra=None, timeout=None, **kwargs) -> Gripper.HoldingStatus:
        return Gripper.HoldingStatus(is_holding_something=self._holding)

    async def stop(self, *, extra=None, timeout=None, **kwargs):
        def _stop():
            self._stop_requested.set()
            pos = math.degrees(self._state().pos)
            self._send_target(pos)
        await asyncio.to_thread(_stop)

    async def is_moving(self) -> bool:
        if self._move_active.is_set():
            return True
        state = await asyncio.to_thread(self._state)
        return abs(state.vel) > _MOVING_VEL_RAD_S

    async def get_kinematics(self, *, extra=None, timeout=None, **kwargs):
        raise NotImplementedError("the B601 gripper does not provide a kinematics file")

    async def get_current_inputs(self, *, extra=None, timeout=None, **kwargs):
        """Single input: motor position in degrees (0 = closed, open_position_deg = open)."""
        state = await asyncio.to_thread(self._state)
        return [math.degrees(state.pos)]

    async def go_to_inputs(self, values, *, extra=None, timeout=None, **kwargs):
        if len(values) != 1:
            raise ValueError(f"expected 1 input (motor degrees), got {len(values)}")
        lo, hi = min(self.open_deg, self.closed_deg), max(self.open_deg, self.closed_deg)
        target = max(lo, min(hi, float(values[0])))
        await asyncio.to_thread(self._move_until_settled, target)

    async def do_command(self, command: Mapping[str, Any], *, timeout=None, **kwargs) -> Mapping[str, Any]:
        result: Dict[str, Any] = {}
        for name, arg in command.items():
            if name == "set_zero_position":
                await asyncio.to_thread(self.bus.motor(GRIPPER_CAN_ID).set_zero_position)
                result[name] = "ok; current (closed) position is now zero"
            elif name == "torque":
                if arg not in ("enable", "disable"):
                    raise ValueError("'torque' must be 'enable' or 'disable'")
                def _torque():
                    if arg == "enable":
                        self._configure_motor()
                    else:
                        with self.bus.lock:
                            self.bus.motor(GRIPPER_CAN_ID).disable()
                await asyncio.to_thread(_torque)
                result[name] = arg + "d"
            elif name == "raw_state":
                state = await asyncio.to_thread(self._state)
                result[name] = {
                    "pos_deg": math.degrees(state.pos),
                    "vel_rad_s": state.vel,
                    "torque_nm": state.torq,
                }
            else:
                raise ValueError(f"unknown command '{name}'")
        return result

    async def close(self):
        if self.bus is not None:
            await asyncio.to_thread(self.bus.release)
            self.bus = None
