"""Viam gripper component for the reBot Arm B601 parallel gripper (motor 0x07).

The gripper motor drives a leadscrew, so its native "position" is motor
rotation in degrees: 0 deg is fully closed (the calibration pose) and the fully
open angle is whatever ``open_position_deg`` says: about -270 deg on the
B601-DM and +340 deg on the B601-RS, both measured from a zero set with the
jaws closed.

The B601-DM runs in FORCE_POS mode, so grab force is capped by a torque ratio
and the jaws stall gently on an object; one ``send_force_pos`` carries position,
speed and force together, so a single send is the whole move.

RobStride has no force-limited position mode, so the B601-RS runs in profile
position (POS_VEL, RobStride's native mode 2, PP) instead, and there nothing the
module can put in a frame or a parameter sets the speed: the bench tried the
generic frame's velocity field, the ``limit_spd`` parameter and the PP frame's
``vel_max``, and 10 to 200 deg/s looked identical through all three. So RS moves
are not one far-away target for the firmware to plan; the module interpolates
them with ``trajectory.plan`` and streams one setpoint per tick, exactly as
``arm.py`` does, and ``speed_deg_s`` is the spacing of those setpoints. Grip
force is capped by ``limit_cur``, an absolute current limit set by the
``grip_current_a`` attribute. That is a different knob from DM's
``torque_ratio``, not the same one in other units, so RS has its own attribute
and ignores ``torque_ratio``. Force commands and holding detection are still not
available there.

Kinematic inputs (``get_current_inputs``/``go_to_inputs``) are the left
finger's travel in metres (0 = closed, ``model.gripper.travel_m`` = fully
open), matching the one prismatic joint in the URDF served by
``get_kinematics``. Motor degrees are still available through the
``get``/``set`` DoCommands.
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

from . import kinematics, spatial
from .bus import (
    DEFAULT_BAUD,
    LINK_ERRORS,
    VARIANT_VENDOR,
    VARIANTS,
    BusError,
    SharedBus,
    detect_port,
    is_motor_timeout,
)
from .damiao import JointHealth, MotorFault
from .ops import SingleOperationManager
from .trajectory import MoveOptions, plan

LOGGER = getLogger(__name__)

GRIPPER_CAN_ID = 0x07

DEFAULT_OPEN_DEG = -270.0
# RS: measured on the bench by jogging the jaw from closed to its hard stop. It is only correct
# if the motor was zeroed with the jaws closed -- the procedure in the README -- so the value in
# use is logged at configure: a wrong one misreports finger travel silently instead of failing.
RS_OPEN_DEG = 340.0
DEFAULT_CLOSED_DEG = 0.0
MAX_OPEN_TRAVEL_DEG = 1440.0  # four motor turns; the measured jaw travel is 270 (DM) / 340 (RS)
DEFAULT_SPEED_DEG_S = 900.0
RS_SPEED_DEG_S = math.degrees(5.0)  # the vendor's vlim for the gripper; DM's is DEFAULT_SPEED_DEG_S
MIN_SPEED_DEG_S = 10.0
MAX_SPEED_DEG_S = 3000.0  # DM's fast leadscrew, whose default is 900
# RS needs its own ceiling: 3000 deg/s is 52 rad/s, an order of magnitude past anything an rs-00
# does, so DM's bound caps nothing there and a config near the top of it (the bench ran 3000) is
# nonsense that nothing would have questioned. Seeed's reference vlim for this gripper is 5 rad/s,
# so allow twice that: headroom above the vendor figure without pretending a speed the motor
# cannot reach is configurable.
RS_MAX_SPEED_DEG_S = math.degrees(10.0)
# The acceleration, for both the planned ramp and the PP frame's acc_set. Derived from the speed
# rather than configured: acc = speed / RS_ACC_RAMP_S, so the jaw reaches whatever rate was asked
# for in RS_ACC_RAMP_S seconds. 0.1 s is under a tenth of a full-travel move (340 deg at the
# vendor's 5 rad/s takes ~1.2 s), so it is still a ramp rather than a step. It becomes an
# attribute only if the bench asks for one.
RS_ACC_RAMP_S = 0.1
# The module paces RS moves setpoint by setpoint (see _stream_until_settled), so the PP frame's
# vel_max is no longer the speed knob -- it is the ceiling for the hop to the next setpoint, and
# a hop of one tick's travel has to fit in one tick. Commanding it at exactly the paced speed
# would leave no headroom for the frame's own acceleration ramp and the jaw would trail its
# setpoints forever, so the frame asks for twice the paced speed.
RS_TRACK_HEADROOM = 2.0
DEFAULT_TORQUE_RATIO = 0.07  # DM: max grip force in [0, 1], a fraction of the motor's rated torque
# RS: the absolute grip current cap in amps, written to limit_cur. 1.0 A is bench-validated on a
# B601-RS against the motor's 16 A factory limit (1, 4, 5, 10 and 16 A were all tried): at 1.0 A
# the jaw closes on a cardboard box and holds it without crushing it. The "was X A" line logged
# at configure still reports the motor's own limit, so it can be retuned from evidence.
DEFAULT_GRIP_CURRENT_A = 1.0
# If the gripper stalls at least this far (deg of motor rotation) short of the
# fully-closed position, we consider it to be holding something.
DEFAULT_HOLDING_THRESHOLD_DEG = 15.0
DEFAULT_STALL_POLLS = 4  # consecutive polls with no motion (DM: velocity; RS: position delta) that count as settled
DEFAULT_MOVE_TIMEOUT_S = 6.0

_ENSURE_MODE_RETRIES = 9
_SETTLE_SEC = 0.02
_MOVING_VEL_RAD_S = 0.05
_POLL_SEC = 0.05
_ARRIVE_TOL_DEG = 2.0
# Every DoCommand that sets or uses a grip-force ratio. RobStride has no force-limited
# position mode, so none of them mean anything there.
_RS_UNSUPPORTED = frozenset(
    {
        "set_force",
        "set_torque",
        "set_gripper_torque",
        "get_force",
        "get_torque",
        "get_gripper_torque",
        "grab_with_force",
        "grab_with_torque",
    }
)
_RS_SETTLE_DELTA_DEG = 0.5  # a moving motor covers ~14 deg per poll at the 5 rad/s vlim, so this is a wide margin
# RobStride parameter id. limit_cur is the neighbour of limit_spd (0x7017, confirmed by Seeed's
# reference stack, reBotArm_control_py/actuator/rebotarm.py:311) in the standard RobStride
# parameter table, one before mechPos (0x7019, which this module already uses), and motorbridge's
# native library lists the same name in the same place. It is still inferred, so the write is
# read back and logged: see _write_grip_current.
RS_RID_LIMIT_CUR = 0x7018
_FAULT_HINT = 'fix the cause, then send {"clear_errors": true} to this gripper'


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
        variant = attrs.get("variant", "dm")
        if variant not in VARIANTS:
            raise ValueError("variant must be 'dm' (Damiao, USB serial bridge) or 'rs' (RobStride, CAN)")
        if variant == "rs":
            if not attrs.get("port"):
                raise ValueError(
                    "variant 'rs' needs 'port': the CAN channel, e.g. \"can0\" (Linux SocketCAN) "
                    'or "PCAN_USBBUS1" (macOS PCAN). The arm dependency cannot supply it: under '
                    "viam-server that dependency is a gRPC client, not the local arm object"
                )
            if float(attrs.get("grip_current_a", DEFAULT_GRIP_CURRENT_A)) <= 0.0:
                raise ValueError("grip_current_a must be greater than 0 (amps)")
        # The open angle only means anything as a span from the closed one, and a zero or absurd
        # span misreports finger travel silently rather than failing, so reject it here.
        span = abs(
            float(attrs.get("open_position_deg", RS_OPEN_DEG if variant == "rs" else DEFAULT_OPEN_DEG))
            - float(attrs.get("closed_position_deg", DEFAULT_CLOSED_DEG))
        )
        if not 0.0 < span <= MAX_OPEN_TRAVEL_DEG:
            raise ValueError(
                "open_position_deg must be a usable jaw angle: the motor angle at the fully open "
                "jaw, in degrees from closed_position_deg, differing from it by more than 0 and "
                f"at most {MAX_OPEN_TRAVEL_DEG:.0f} deg"
            )
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
        self.variant = attrs.get("variant", "dm")
        rs = self.variant == "rs"
        vendor = VARIANT_VENDOR[self.variant]
        self.model = spatial.MODELS[self.variant]
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
        bus_vendor = SharedBus.vendor_of(port)
        if bus_vendor is not None and bus_vendor != vendor:
            raise ValueError(
                f"gripper variant '{self.variant}' expects {vendor} motors but {port} is already "
                f"open for {bus_vendor} motors; set the gripper's 'variant' to match the arm"
            )

        self.open_deg = float(attrs.get("open_position_deg", RS_OPEN_DEG if rs else DEFAULT_OPEN_DEG))
        self.closed_deg = float(attrs.get("closed_position_deg", DEFAULT_CLOSED_DEG))
        self.speed_deg_s = self._clamp_speed(
            float(attrs.get("speed_deg_s", RS_SPEED_DEG_S if rs else DEFAULT_SPEED_DEG_S))
        )
        if rs:
            LOGGER.info(
                "gripper open_position_deg is %.1f deg (%s), closed %.1f: correct only if motor "
                "0x07 was zeroed with the jaws closed",
                self.open_deg,
                "configured" if "open_position_deg" in attrs else "bench-measured default",
                self.closed_deg,
            )
        self.torque_ratio = float(attrs.get("torque_ratio", DEFAULT_TORQUE_RATIO))
        self.grip_current_a = float(attrs.get("grip_current_a", DEFAULT_GRIP_CURRENT_A))
        if rs and "torque_ratio" in attrs:
            LOGGER.warning(
                "torque_ratio is ignored on the B601-RS: it is a fraction of a Damiao motor's "
                "rated torque in FORCE_POS, and RobStride has no equivalent. Cap RS grip force "
                "with 'grip_current_a' instead, an absolute current limit in amps"
            )
        self.holding_threshold_deg = float(attrs.get("holding_threshold_deg", DEFAULT_HOLDING_THRESHOLD_DEG))
        self.stall_polls = int(attrs.get("stall_polls", DEFAULT_STALL_POLLS))
        self.move_timeout_s = float(attrs.get("move_timeout_s", DEFAULT_MOVE_TIMEOUT_S))
        self.collision_mode = attrs.get("collision_geometry", "primitives")
        self.reconnect_enabled = bool(attrs.get("reconnect", True))

        if self.bus is not None and not self.bus.matches(port, baud, vendor):
            self._release_bus()
        if self.bus is None:
            self.bus = SharedBus.acquire(port, baud, vendor)
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

    def _read_param(self, motor, rid: int) -> float:
        """Diagnostic only. Nothing is computed from this, so a failed read is not fatal: it
        returns nan, which the read-back checks below treat as a mismatch."""
        try:
            return float(motor.robstride_get_param_f32(rid))
        except Exception:
            return float("nan")

    def _write_grip_current(self, motor):
        """RobStride's squeeze ceiling is the limit_cur parameter, which no command frame carries.

        limit_spd (0x7017) used to be written here too, as Seeed's stack does. It is not any more:
        the bench found speed_deg_s from 10 to 200 deg/s making no visible difference through it,
        the module now paces the move itself, and a speed cap written at exactly the paced speed
        would only rob the jaw of the headroom it needs to reach the next setpoint in time.

        limit_cur is written as an absolute current, never derived from what the motor already
        holds: deriving it would mean re-reading a value we wrote ourselves, and every restart
        would ratchet the cap down."""
        was = self._read_param(motor, RS_RID_LIMIT_CUR)
        try:
            motor.robstride_write_param_f32(RS_RID_LIMIT_CUR, self.grip_current_a)
        except Exception:
            # The RID is inferred, so a firmware that does not have it must not stop the
            # gripper building; it just runs uncapped, which is what 0.6.0 already did.
            LOGGER.warning(
                "gripper limit_cur (RID 0x%04X) could not be set; grip force is NOT capped",
                RS_RID_LIMIT_CUR,
                exc_info=True,
            )
            return
        got = self._read_param(motor, RS_RID_LIMIT_CUR)
        LOGGER.info(
            "gripper limit_cur (RID 0x%04X) was %.3f A (informational only), set to %.3f A from "
            "grip_current_a, read back %.3f A",
            RS_RID_LIMIT_CUR,
            was,
            self.grip_current_a,
            got,
        )
        if not abs(got - self.grip_current_a) <= max(0.05, 0.02 * self.grip_current_a):
            LOGGER.warning(
                "gripper limit_cur read back %.3f A, not the %.3f A written: RID 0x%04X may not "
                "be limit_cur on this firmware, so grip force is NOT capped",
                got,
                self.grip_current_a,
                RS_RID_LIMIT_CUR,
            )

    def _configure_motor(self):
        rs = self.variant == "rs"

        def _do():
            with self.bus.lock:
                motor = self.bus.motor(GRIPPER_CAN_ID)
                # A RobStride in profile position resumes its last internal setpoint, which is
                # stale after a restart, so read where the jaws are before the motor goes live
                # and there is no window with torque on and no setpoint. Same idea as the arm's
                # _hold_current().
                hold = self.bus.poll_feedback([GRIPPER_CAN_ID], positions_only=True)[GRIPPER_CAN_ID] if rs else None
                motor.enable()
                for attempt in range(_ENSURE_MODE_RETRIES + 1):
                    try:
                        motor.ensure_mode(Mode.POS_VEL if rs else Mode.FORCE_POS)
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
                if rs:
                    # RobStride motors stream status frames (and so fill get_state)
                    # only once asked to; without this, every read is a param round-trip.
                    motor.robstride_set_active_report(True)
                    self._write_grip_current(motor)
                    if hold is not None:
                        self._rs_send(motor, hold.pos, math.radians(self.speed_deg_s))
                    else:
                        LOGGER.warning(
                            "gripper position unreadable at configure; the motor keeps whatever "
                            "profile-position target it already had and may lurch"
                        )

        self._bus_call(_do)
        self._torque_enabled = True

    def _state(self):
        states = self._bus_call(self.bus.poll_feedback, [GRIPPER_CAN_ID], positions_only=not self._torque_enabled)
        state = states[GRIPPER_CAN_ID]
        if state is None:
            raise BusError("no feedback from gripper motor 0x07; check power and wiring")
        return state

    def _live_state(self):
        """Feedback for a jaw that may be moving, or None on RS if the read times out.

        Bench 2026-09-19: with torque on, ``_state()`` takes the streamed-frame path and the
        RobStride status stream runs about half a second behind the motor, so every poll of a
        move in flight returns the same stale angle -- which reads as "stopped". The mechPos
        parameter read is a request/response round trip and is therefore current; it is the
        same path the arm uses with torque off. It costs one round trip per poll (bounded by
        ``bus._MECHPOS_TIMEOUT_MS``), and a timed-out read comes back as None rather than an
        exception: a missing sample, never evidence that the jaw is still.

        DM's streamed frame is current, and it carries the velocity the settle test needs.
        """
        if self.variant != "rs":
            return self._state()
        return self._bus_call(self.bus.poll_feedback, [GRIPPER_CAN_ID], positions_only=True)[GRIPPER_CAN_ID]

    def _check_ready(self, state):
        health = JointHealth.from_state(GRIPPER_CAN_ID, state, self.bus.vendor)
        if health.fault:
            if health.transient:
                self._bus_call(self.bus.motor(GRIPPER_CAN_ID).clear_error)
                self._configure_motor()
                return
            raise MotorFault("gripper", health, hint=_FAULT_HINT)

    def _send_target(
        self, target_deg: float, speed_deg_s: Optional[float] = None, torque_ratio: Optional[float] = None
    ):
        speed = speed_deg_s if speed_deg_s is not None else self.speed_deg_s
        ratio = torque_ratio if torque_ratio is not None else self.torque_ratio

        def _do():
            motor = self.bus.motor(GRIPPER_CAN_ID)
            with self.bus.lock:
                if self.variant == "rs":
                    # RobStride has no FORCE_POS; profile position keeps speed meaningful and
                    # leaves the firmware current limit as the only squeeze ceiling.
                    self._rs_send(motor, math.radians(target_deg), math.radians(speed))
                else:
                    motor.send_force_pos(math.radians(target_deg), math.radians(speed), ratio)

        self._bus_call(_do)

    def _rs_send(self, motor, pos_rad: float, speed_rad_s: float):
        """The dedicated profile-position frame, not the generic send_pos_vel one: Mode.POS_VEL on
        a RobStride already *is* native mode 2 (PP), so this matches the send to the mode we
        selected rather than changing modes.

        Its vel_max is not what makes the jaw fast or slow -- the spacing of the setpoints is (see
        _stream_until_settled). It is kept as a per-hop ceiling with RS_TRACK_HEADROOM of slack, so
        that whether or not this firmware honours it, the motor can always reach the next setpoint
        within a tick.
        """
        vel_max = speed_rad_s * RS_TRACK_HEADROOM
        motor.robstride_send_pos_vel_pp(pos_rad, vel_max, vel_max / RS_ACC_RAMP_S)

    def _clamp_speed(self, speed_deg_s: float) -> float:
        """The ceiling is per variant: see RS_MAX_SPEED_DEG_S. Applied to the configured value as
        well as to set_speed, so a nonsense config attribute is visible rather than silent."""
        hi = RS_MAX_SPEED_DEG_S if self.variant == "rs" else MAX_SPEED_DEG_S
        speed = max(MIN_SPEED_DEG_S, min(hi, float(speed_deg_s)))
        if speed != float(speed_deg_s):
            LOGGER.warning(
                "gripper speed_deg_s %.1f is outside the %s range %.1f-%.1f deg/s; clamped to %.1f",
                float(speed_deg_s),
                self.variant,
                MIN_SPEED_DEG_S,
                hi,
                speed,
            )
        return speed

    def _clamp_deg(self, target_deg: float) -> float:
        lo, hi = min(self.open_deg, self.closed_deg), max(self.open_deg, self.closed_deg)
        return max(lo, min(hi, float(target_deg)))

    def _move_until_settled(self, target_deg: float, speed_deg_s=None, torque_ratio=None) -> float:
        """Command target and wait for stall or arrival; returns final position (deg).

        DM sends the whole move in one frame: send_force_pos carries position, speed and force
        ratio together and the firmware honours all three. RS cannot, so it streams (below).
        """
        with self.ops.new() as cancel:
            state = self._state()
            self._check_ready(state)
            target_deg = self._clamp_deg(target_deg)
            if self.variant == "rs":
                live = self._live_state()
                start = math.degrees((live if live is not None else state).pos)
                speed = self._clamp_speed(speed_deg_s) if speed_deg_s is not None else self.speed_deg_s
                return self._stream_until_settled(target_deg, start, speed, cancel)
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

    def _plan_setpoints(self, start_deg: float, target_deg: float, speed_deg_s: float) -> List[float]:
        """The move, interpolated here instead of by the motor: one setpoint per tick at
        1/_POLL_SEC Hz, spaced so the jaw covers speed_deg_s.

        The same trajectory.plan the arm uses. One jaw is one degree of freedom, so every
        per-joint list is length 1 and every setpoint comes back as a 1-element list.
        """
        opts = MoveOptions(
            vmax_deg_s=[speed_deg_s],
            amax_deg_s2=[speed_deg_s / RS_ACC_RAMP_S],
            hz=1.0 / _POLL_SEC,
        )
        return [q[0] for q in plan([start_deg], [[target_deg]], opts)]

    def _stream_until_settled(self, target_deg: float, start_deg: float, speed_deg_s: float, cancel) -> float:
        """RS: pace the move here, one setpoint per tick, the way arm.py does.

        Handed a far-away target a RobStride plans the move itself, and nothing the module can set
        changes how fast it plans it: the bench tried the generic frame's velocity field, the
        limit_spd parameter and the PP frame's vel_max, and speed_deg_s from 10 to 200 deg/s looked
        the same through all three. The arm never had that problem because it never asks the motor
        to plan -- speed lives in the spacing of the setpoints, which works whatever the firmware's
        own profile generator does. This is that loop, for one joint.

        Position is still read with the mechPos round trip (_live_state): the streamed frame runs
        about half a second behind the motor, which during a move reads as "stopped".
        """
        # A run of the target after the plan: the jaw is up to a tick behind the last setpoint,
        # and the stall test needs a few ticks against a fixed point to conclude.
        planned = self._plan_setpoints(start_deg, target_deg, speed_deg_s)
        points = planned + [target_deg] * (self.stall_polls + 1)
        # move_timeout_s is slack past the planned duration rather than the whole budget: at
        # 10 deg/s a full-travel move legitimately takes half a minute.
        deadline = time.monotonic() + len(points) * _POLL_SEC + self.move_timeout_s
        # Stalled means the setpoints have run away from the jaw *and* the jaw is not moving.
        # Streaming is what makes the first half available; "the position stopped changing" on its
        # own cannot tell a stalled jaw from a slow one. The tolerance is one tick of travel, so
        # the worst the motor is ever asked for past a stalled jaw is stall_polls ticks of it.
        lag_tol = max(_ARRIVE_TOL_DEG, speed_deg_s * _POLL_SEC)
        pos = start_deg
        moved = False
        still = 0
        anchor = time.monotonic()
        for k, cmd in enumerate(points):
            if cancel.is_set() or time.monotonic() > deadline:
                break
            due = anchor + k * _POLL_SEC
            while True:
                now = time.monotonic()
                if now >= due or cancel.is_set():
                    break
                time.sleep(min(due - now, 0.005))
            if cancel.is_set():
                break
            self._send_target(cmd, speed_deg_s)
            state = self._live_state()
            if state is None:
                # mechPos timed out: no sample this tick. Counting it as "stopped" would make a
                # quiet bus indistinguishable from a stall, so the streak restarts instead. The
                # read already blocked for bus._MECHPOS_TIMEOUT_MS and the deadline still bounds
                # the loop.
                still = 0
                continue
            prev, pos = pos, math.degrees(state.pos)
            moved = moved or abs(pos - prev) >= _RS_SETTLE_DELTA_DEG
            # Arrival only ends the trailing run: the plan is streamed out in full, so the last
            # thing the motor is asked for is the target itself and not the setpoint before it.
            if k + 1 >= len(planned) and abs(pos - target_deg) < _ARRIVE_TOL_DEG:
                break
            # Against the *previous* setpoint: the jaw cannot have reached the one just sent.
            behind = k > 0 and abs(points[k - 1] - pos) > lag_tol
            still = still + 1 if behind and abs(pos - prev) < _RS_SETTLE_DELTA_DEG else 0
            if still >= self.stall_polls:
                break  # stalled (on an object, or at the mechanical limit)
        self._last_stall_deg = pos
        if not cancel.is_set() and abs(pos - target_deg) >= _ARRIVE_TOL_DEG:
            # The stream stopped short, so the motor is still driving at a setpoint the jaw cannot
            # reach and would grind into the object until it faults. Hold where it actually
            # stopped. Not done on DM: FORCE_POS caps the current by design, and that standing
            # push is how a DM grab keeps its grip on the object.
            if moved:
                self._send_target(pos, speed_deg_s)
            else:
                # Never seeing the jaw move means the readings are the suspect part, not the jaw.
                # Re-commanding one would turn a bad read into a physical reversal -- the 0.6.0
                # bench failure where open() drove straight back to closed.
                LOGGER.warning(
                    "gripper never moved off %.1f deg while driving to %.1f: feedback is stale "
                    "or the motor is unresponsive, so the setpoints stop where they are rather "
                    "than being re-issued from a reading we do not trust",
                    pos,
                    target_deg,
                )
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
        return self.fraction_from_deg(pos_deg) * self.model.gripper.travel_m

    def deg_from_travel_m(self, travel_m: float) -> float:
        return self.deg_from_fraction(float(travel_m) / self.model.gripper.travel_m)

    # ---------------------------------------------------------- Viam API

    async def open(self, *, extra=None, timeout=None, **kwargs):
        await asyncio.to_thread(self._move_until_settled, self.open_deg)
        self._holding = False

    async def grab(self, *, extra=None, timeout=None, **kwargs) -> bool:
        pos = await asyncio.to_thread(self._move_until_settled, self.closed_deg)
        # Stalling well short of fully closed means the jaws met an object. Holding
        # detection needs a force signal RS does not give and a threshold tuned on
        # hardware; until then RS answers "cannot tell", encoded as False.
        self._holding = False if self.variant == "rs" else abs(pos - self.closed_deg) > self.holding_threshold_deg
        return self._holding

    async def is_holding_something(self, *, extra=None, timeout=None, **kwargs) -> Gripper.HoldingStatus:
        meta = {"stall_position_deg": self._last_stall_deg} if self._last_stall_deg is not None else {}
        return Gripper.HoldingStatus(is_holding_something=self._holding, meta=meta)

    async def stop(self, *, extra=None, timeout=None, **kwargs):
        def _stop():
            self.ops.cancel_current()
            with self.ops.new():
                # The jaw is moving, so the streamed frame lags it (see _live_state); holding
                # at a stale angle would walk it backwards. Fall back only if mechPos times out.
                state = self._live_state()
                pos = math.degrees(state.pos if state is not None else self._state().pos)
                self._send_target(pos)

        await asyncio.to_thread(_stop)

    async def is_moving(self) -> bool:
        if self.ops.running:
            return True
        if self.variant == "rs":
            # One sample cannot give a delta and RS velocity is unusable, so an in-flight
            # operation is all we can honestly report. Same call the arm makes.
            return False
        state = await asyncio.to_thread(self._state)
        return abs(state.vel) > _MOVING_VEL_RAD_S

    async def get_kinematics(self, *, extra=None, timeout=None, **kwargs):
        return kinematics.gripper_kinematics(self.model, self.collision_mode)

    async def get_geometries(self, *, extra=None, timeout=None, **kwargs) -> List[Geometry]:
        state = await asyncio.to_thread(self._state)
        return kinematics.gripper_geometries(self.model, self.travel_m_from_deg(math.degrees(state.pos)))

    async def get_current_inputs(self, *, extra=None, timeout=None, **kwargs):
        """Single input: left-finger travel in metres (0 = closed, model.gripper.travel_m = fully open)."""
        state = await asyncio.to_thread(self._state)
        return [self.travel_m_from_deg(math.degrees(state.pos))]

    async def go_to_inputs(self, values, *, extra=None, timeout=None, **kwargs):
        if len(values) != 1:
            raise ValueError(f"expected 1 input (finger travel in metres), got {len(values)}")
        await asyncio.to_thread(self._move_until_settled, self.deg_from_travel_m(values[0]))

    async def do_command(self, command: Mapping[str, Any], *, timeout=None, **kwargs) -> Mapping[str, Any]:
        result: Dict[str, Any] = {}
        for name, arg in command.items():
            if self.variant == "rs" and name in _RS_UNSUPPORTED:
                raise ValueError(
                    f"'{name}' is not supported on the B601-RS yet: RobStride has no force-limited "
                    "position mode. Cap grip force with the 'grip_current_a' attribute instead, an "
                    "absolute current limit applied at configure that cannot be changed live"
                )
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
                h = JointHealth.from_state(GRIPPER_CAN_ID, state, self.bus.vendor).as_dict()
                h["open_fraction"] = self.fraction_from_deg(h["pos_deg"])
                h["holding"] = self._holding
                if h["fault"]:
                    h["hint"] = _FAULT_HINT
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
                # RS needs no write: the next move's setpoint spacing is the speed.
                self.speed_deg_s = self._clamp_speed(float(arg))
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
