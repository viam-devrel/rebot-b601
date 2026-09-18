import threading
import time

from src.rebot_b601 import damiao
from src.rebot_b601.ops import SingleOperationManager


def test_status_decoding():
    assert damiao.status_text(0x1) == "enabled"
    assert damiao.status_text(0xA) == "over-current"
    assert damiao.is_fault(0xD) and damiao.is_transient(0xD)
    assert damiao.is_fault(0xB) and not damiao.is_transient(0xB)
    assert not damiao.is_fault(0x1)
    assert "unknown" in damiao.status_text(0x5)


def test_single_operation_cancels_previous():
    ops = SingleOperationManager()
    started = threading.Event()
    cancelled = threading.Event()

    def worker():
        with ops.new() as cancel:
            started.set()
            cancel.wait(5.0)
            cancelled.set()

    t = threading.Thread(target=worker)
    t.start()
    started.wait(1.0)
    assert ops.running
    t0 = time.monotonic()
    with ops.new():
        assert cancelled.is_set()
        assert time.monotonic() - t0 < 1.0
        assert ops.running
    t.join(1.0)
    assert not ops.running


def test_robstride_status_is_a_fault_bitfield():
    assert damiao.status_text(0x0, "robstride") == "ok"
    assert damiao.status_text(0x1, "robstride") == "undervoltage"
    assert damiao.status_text(0x21, "robstride") == "undervoltage, not calibrated"
    assert "0x40" in damiao.status_text(0x40, "robstride")
    assert damiao.status_text(0x41, "robstride") == "undervoltage, unknown bits 0x40"
    assert not damiao.is_fault(0x0, "robstride")
    assert damiao.is_fault(0x1, "robstride") and not damiao.is_transient(0x1, "robstride")
    assert not damiao.is_transient(0xD, "robstride")  # 0xD is a Damiao transient code, never a RobStride one
    # Damiao decode is untouched
    assert damiao.status_text(0x1) == "enabled" and not damiao.is_fault(0x1)


def test_joint_health_uses_the_vendor_decode():
    class State:
        status_code = 0x1
        pos = vel = torq = 0.0
        t_mos = 30.0
        t_rotor = 0.0

    rs = damiao.JointHealth.from_state(1, State(), "robstride")
    assert rs.fault and not rs.transient and rs.status == "undervoltage"
    assert not rs.position_only
    dm = damiao.JointHealth.from_state(1, State())
    assert not dm.fault and dm.status == "enabled"


def test_position_only_state_is_not_a_health_reading():
    class PosOnly:
        position_only = True
        status_code = 0
        pos = 0.25
        vel = torq = t_mos = t_rotor = 0.0

    h = damiao.JointHealth.from_state(3, PosOnly(), "robstride")
    assert h.position_only and not h.fault and h.status == "position only (no status frame)"
    assert h.as_dict()["position_only"] is True
