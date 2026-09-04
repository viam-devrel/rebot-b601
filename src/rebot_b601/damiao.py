"""Damiao motor status decoding and register ids.

The high nibble of the first byte in every Damiao feedback frame is the motor
status ("ERR" in the vendor manual). motorbridge surfaces it as
``MotorState.status_code``. Codes 0x8 and above are faults that the motor
latches until ``clear_error`` (or a power cycle for hardware faults).
"""

from dataclasses import dataclass
from typing import Optional

STATUS_DISABLED = 0x0
STATUS_ENABLED = 0x1

STATUS_TEXT = {
    0x0: "disabled",
    0x1: "enabled",
    0x8: "over-voltage",
    0x9: "under-voltage",
    0xA: "over-current",
    0xB: "MOS over-temperature",
    0xC: "rotor over-temperature",
    0xD: "communication loss",
    0xE: "overload",
}

# Faults that clear themselves once the cause is gone; a single clear_error
# after the bus is healthy again is enough.
TRANSIENT_FAULTS = {0xD}
# Faults that need the operator to remove the cause (cool down, unjam).
HARD_FAULTS = {0x8, 0x9, 0xA, 0xB, 0xC, 0xE}

# Damiao RW register ids (see motorbridge.damiao_registers).
RID_UV_VALUE = 0
RID_OT_VALUE = 2
RID_OC_VALUE = 3
RID_ACC = 4
RID_DEC = 5
RID_MAX_SPD = 6
RID_TIMEOUT = 9
RID_CTRL_MODE = 10


def status_text(code: int) -> str:
    return STATUS_TEXT.get(code, f"unknown status 0x{code:x}")


def is_fault(code: int) -> bool:
    return code >= 0x8


def is_transient(code: int) -> bool:
    return code in TRANSIENT_FAULTS


@dataclass
class JointHealth:
    """Decoded health of one motor, built from a MotorState."""

    can_id: int
    status_code: int
    status: str
    fault: bool
    transient: bool
    pos_deg: float
    vel_rad_s: float
    torque_nm: float
    t_mos_c: float
    t_rotor_c: float

    @classmethod
    def from_state(cls, can_id: int, state) -> "JointHealth":
        import math

        code = int(getattr(state, "status_code", STATUS_ENABLED))
        return cls(
            can_id=can_id,
            status_code=code,
            status=status_text(code),
            fault=is_fault(code),
            transient=is_transient(code),
            pos_deg=math.degrees(state.pos),
            vel_rad_s=state.vel,
            torque_nm=state.torq,
            t_mos_c=float(getattr(state, "t_mos", 0.0)),
            t_rotor_c=float(getattr(state, "t_rotor", 0.0)),
        )

    def as_dict(self) -> dict:
        return {
            "can_id": self.can_id,
            "status_code": self.status_code,
            "status": self.status,
            "fault": self.fault,
            "pos_deg": self.pos_deg,
            "vel_rad_s": self.vel_rad_s,
            "torque_nm": self.torque_nm,
            "t_mos_c": self.t_mos_c,
            "t_rotor_c": self.t_rotor_c,
        }


class MotorFault(RuntimeError):
    """Raised when a motor reports a fault that blocks motion."""

    def __init__(self, joint_name: str, health: JointHealth, hint: Optional[str] = None):
        self.joint_name = joint_name
        self.health = health
        msg = f"{joint_name} (0x{health.can_id:02x}) reports {health.status}"
        if hint:
            msg += f"; {hint}"
        super().__init__(msg)


class CollisionError(RuntimeError):
    """Raised when measured torque exceeds the configured limit during a move."""


class OverTemperatureError(RuntimeError):
    """Raised when a motor is above the configured hard temperature limit."""
