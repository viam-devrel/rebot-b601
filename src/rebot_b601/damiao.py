"""Damiao motor status decoding and register ids.

The high nibble of the first byte in every Damiao feedback frame is the motor
status ("ERR" in the vendor manual). motorbridge surfaces it as
``MotorState.status_code``. Codes 0x8 and above are faults that the motor
latches until ``clear_error`` (or a power cycle for hardware faults). RobStride
motors reuse the same field for a 6-bit fault bitfield instead, decoded here by
``ROBSTRIDE_FAULT_BITS``.
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

# RobStride status frames carry a 6-bit fault field (bits 16-21 of the feedback id);
# motorbridge exposes it as MotorState.status_code. Zero is healthy. Bench check
# 2026-09-18: rs-06 motors on a low supply read 0x1.
ROBSTRIDE_FAULT_BITS = {
    0x01: "undervoltage",
    0x02: "over-current",
    0x04: "over-temperature",
    0x08: "magnetic encoder fault",
    0x10: "HALL encoder fault",
    0x20: "not calibrated",
}

# Damiao RW register ids (see motorbridge.damiao_registers).
RID_UV_VALUE = 0
RID_OT_VALUE = 2
RID_OC_VALUE = 3
RID_ACC = 4
RID_DEC = 5
RID_MAX_SPD = 6
RID_TIMEOUT = 9
RID_CTRL_MODE = 10


def status_text(code: int, vendor: str = "damiao") -> str:
    if vendor == "robstride":
        if code == 0:
            return "ok"
        names = [name for bit, name in ROBSTRIDE_FAULT_BITS.items() if code & bit]
        return ", ".join(names) if names else f"robstride fault bits 0x{code:x}"
    return STATUS_TEXT.get(code, f"unknown status 0x{code:x}")


def is_fault(code: int, vendor: str = "damiao") -> bool:
    if vendor == "robstride":
        return code != 0
    return code >= 0x8


def is_transient(code: int, vendor: str = "damiao") -> bool:
    return vendor == "damiao" and code in TRANSIENT_FAULTS


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
    # True when the state came from a RobStride mechPos parameter read (bus.PositionOnlyState):
    # only pos_deg is measured; the other fields are unknown and must not drive safety checks.
    position_only: bool = False

    @classmethod
    def from_state(cls, can_id: int, state, vendor: str = "damiao") -> "JointHealth":
        import math

        position_only = bool(getattr(state, "position_only", False))
        healthy = 0 if vendor == "robstride" else STATUS_ENABLED
        code = int(getattr(state, "status_code", healthy))
        return cls(
            can_id=can_id,
            status_code=code,
            status="position only (no status frame)" if position_only else status_text(code, vendor),
            fault=False if position_only else is_fault(code, vendor),
            transient=False if position_only else is_transient(code, vendor),
            pos_deg=math.degrees(state.pos),
            vel_rad_s=state.vel,
            torque_nm=state.torq,
            t_mos_c=float(getattr(state, "t_mos", 0.0)),
            t_rotor_c=float(getattr(state, "t_rotor", 0.0)),
            position_only=position_only,
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
            "position_only": self.position_only,
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
