"""B601-RS (RobStride over CAN) behaviour. DM behaviour is covered by the other test files."""

import math

import pytest
from viam.proto.component.arm import JointPositions

from src.rebot_b601 import bus as bus_mod
from src.rebot_b601.arm import ARM_CAN_IDS, B601Arm
from src.rebot_b601.bus import BusError, SharedBus, canonical_device
from src.rebot_b601.damiao import JointHealth, MotorFault
from tests.conftest import FAST, make_config

RS = dict(FAST, variant="rs", port="can0")


def test_robstride_bus_registers_rs_models_on_host_id_fd(factory):
    bus = SharedBus.acquire("can0", vendor="robstride")
    for cid in range(1, 8):
        m = bus.motor(cid)
        assert m.vendor == "robstride"
        assert m.feedback_id == 0xFD
        assert m.model == ("rs-06" if cid <= 3 else "rs-00")
    bus.release()


def test_damiao_bus_is_unchanged(factory):
    bus = SharedBus.acquire("/dev/fake0")
    m = bus.motor(1)
    assert m.vendor == "damiao" and m.feedback_id == 0x11 and m.model == "4340P"
    bus.release()


def test_can_channel_opens_plain_controller_and_serial_opens_dm_bridge(monkeypatch):
    calls = []

    class StubController:
        def __init__(self, channel):
            calls.append(("can", channel))

        @classmethod
        def from_dm_serial(cls, serial_port, baud):
            calls.append(("serial", serial_port, baud))
            return cls.__new__(cls)

    monkeypatch.setattr(bus_mod, "Controller", StubController)
    bus_mod._open_controller("can0", 921600)
    bus_mod._open_controller("PCAN_USBBUS1", 921600)
    bus_mod._open_controller("/dev/ttyACM0", 921600)
    assert calls == [("can", "can0"), ("can", "PCAN_USBBUS1"), ("serial", "/dev/ttyACM0", 921600)]


def test_can_channel_is_its_own_cache_key():
    assert canonical_device("can0") == "can0"
    assert canonical_device("PCAN_USBBUS1") == "PCAN_USBBUS1"


def test_vendor_mismatch_on_shared_channel_is_an_error(factory):
    bus = SharedBus.acquire("can0", vendor="robstride")
    with pytest.raises(BusError, match="robstride"):
        SharedBus.acquire("can0", vendor="damiao")
    assert not bus.matches("can0", bus.baud, "damiao")
    assert bus.matches("can0", bus.baud, "robstride")
    bus.release()


def test_unknown_vendor_is_rejected(factory):
    # "can1" is not cached, so the constructor's guard fires, not the mismatch branch.
    with pytest.raises(BusError, match="unknown motor vendor"):
        SharedBus.acquire("can1", vendor="robstide")


def test_can_channel_acquires_share_one_controller(factory):
    a = SharedBus.acquire("can0", vendor="robstride")
    b = SharedBus.acquire("can0", vendor="robstride")
    assert a is b
    assert len(factory.controllers) == 1
    a.release()
    b.release()


def test_poll_feedback_falls_back_to_mechpos_when_nothing_streams(factory):
    bus = SharedBus.acquire("can0", vendor="robstride")
    for cid in range(1, 8):
        m = bus.motor(cid)
        m.stream_state = False
        m.pos = math.radians(10.0 * cid)
    states = bus.poll_feedback([1, 2, 3], retries=2, settle_s=0.0)
    assert [round(math.degrees(states[c].pos), 3) for c in (1, 2, 3)] == [10.0, 20.0, 30.0]
    assert states[1].status_code == 0 and states[1].vel == 0.0 and states[1].torq == 0.0
    assert isinstance(states[1], bus_mod.PositionOnlyState) and states[1].position_only
    assert JointHealth.from_state(1, states[1], "robstride").position_only
    bus.release()


def test_poll_feedback_leaves_none_when_param_read_fails_too(factory):
    bus = SharedBus.acquire("can0", vendor="robstride")
    m = bus.motor(1)
    m.stream_state = False

    def boom(param_id, timeout_ms=1000):
        raise bus_mod._CallError("param read timed out")

    m.robstride_get_param_f32 = boom
    assert bus.poll_feedback([1], retries=2, settle_s=0.0)[1] is None
    bus.release()


def test_damiao_bus_never_reads_robstride_params(factory):
    bus = SharedBus.acquire("/dev/fake0")
    m = bus.motor(1)
    m.stream_state = False
    assert bus.poll_feedback([1], retries=2, settle_s=0.0)[1] is None
    bus.release()


def test_in_loop_sample_never_pays_for_param_reads(factory):
    bus = SharedBus.acquire("can0", vendor="robstride")
    m = bus.motor(1)
    m.stream_state = False
    assert bus.poll_feedback([1], retries=1, settle_s=0.0)[1] is None
    assert m.param_reads == 0
    bus.release()


def test_streaming_robstride_state_is_used_without_param_reads(factory):
    bus = SharedBus.acquire("can0", vendor="robstride")
    m = bus.motor(1)
    m.robstride_set_active_report(True)
    m.pos = 0.5
    s = bus.poll_feedback([1], retries=2, settle_s=0.0)[1]
    assert s is not None and not getattr(s, "position_only", False) and s.pos == 0.5
    assert m.param_reads == 0
    bus.release()


def test_robstride_state_needs_active_report_in_the_fake(factory):
    bus = SharedBus.acquire("can0", vendor="robstride")
    m = bus.motor(1)
    assert isinstance(bus.poll_feedback([1], retries=2, settle_s=0.0)[1], bus_mod.PositionOnlyState)
    m.robstride_set_active_report(True)
    assert not getattr(bus.poll_feedback([1], retries=2, settle_s=0.0)[1], "position_only", False)
    bus.release()


def test_rs_variant_builds_a_robstride_arm_with_active_report(factory):
    arm = B601Arm.new(make_config("arm", **RS), {})
    ctrl = factory.latest
    assert ctrl.port == "can0" and arm.bus.vendor == "robstride"
    for cid in ARM_CAN_IDS:
        m = ctrl.motors[cid]
        assert m.vendor == "robstride" and m.enabled and m.active_report
        assert m.can_timeout_ms is None
    # streaming works, so the health report is real and no parameter reads were needed
    report = arm._health_report()
    assert all(report[j]["position_only"] is False for j in ("joint1", "joint6"))
    assert all(ctrl.motors[c].param_reads == 0 for c in ARM_CAN_IDS)
    assert arm.mit_kp == [50.0, 150.0, 150.0, 50.0, 50.0, 50.0]
    assert arm.mit_kd == [3.0, 10.0, 10.0, 5.0, 4.0, 4.0]
    assert arm.joint_limits == [
        (-160.0, 160.0),
        (-1.0, 179.0),
        (-1.0, 179.0),
        (-89.0, 89.0),
        (-89.0, 89.0),
        (-179.0, 179.0),
    ]
    assert arm.moving_vel_rad_s == 0.15


def test_dm_defaults_are_unchanged(factory):
    arm = B601Arm.new(make_config("arm", port="/dev/fake0"), {})
    assert arm.variant == "dm" and arm.bus.vendor == "damiao"
    assert arm.mit_kp == [45.0, 45.0, 45.0, 8.0, 9.0, 8.0]
    assert arm.joint_limits[1] == (-179.0, 1.0)
    assert arm.moving_vel_rad_s == 0.05
    assert not factory.latest.motors[1].active_report


def test_validate_config_rs_rules():
    with pytest.raises(ValueError, match="variant"):
        B601Arm.validate_config(make_config("a", variant="xx"))
    with pytest.raises(ValueError, match="port"):
        B601Arm.validate_config(make_config("a", variant="rs"))
    with pytest.raises(ValueError, match="can_timeout_ms"):
        B601Arm.validate_config(make_config("a", variant="rs", port="can0", can_timeout_ms=500))
    assert B601Arm.validate_config(make_config("a", variant="rs", port="can0")) == ([], [])


async def test_rs_positions_come_from_mechpos_when_nothing_streams(factory):
    arm = B601Arm.new(make_config("arm", **RS), {})
    for cid, m in factory.latest.motors.items():
        m.stream_state = False
        m.pos = math.radians(10.0 * cid)
    got = (await arm.get_joint_positions()).values
    assert [round(v, 3) for v in got] == [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    assert arm._health_report()["joint1"]["position_only"] is True


async def test_position_only_states_do_not_drive_safety_checks(factory):
    """Without status frames the arm may still move, but the monitor must treat torque,
    temperature and fault as unknown rather than as zero."""
    arm = B601Arm.new(make_config("arm", **dict(RS, torque_limit_nm=1.0, temperature_limit_c=50.0)), {})
    ctrl = factory.latest
    for m in ctrl.motors.values():
        m.stream_state = False
        m.vel_cap = 100.0
    trip_counts = [2] * 6
    arm._monitor_tick(trip_counts)  # retries=1: no states at all, nothing touched
    assert trip_counts == [2] * 6
    arm._check_ready(arm._read_states())  # position-only states: no MotorFault, no OverTemperatureError
    await arm.move_to_joint_positions(JointPositions(values=[5, 5, 5, 0, 0, 0]))
    assert arm._health_report()["joint2"]["position_only"] is True


async def test_rs_fault_bits_block_a_move_with_a_readable_name(factory):
    arm = B601Arm.new(make_config("arm", **RS), {})
    factory.latest.motors[2].status_code = 0x1  # undervoltage
    with pytest.raises(MotorFault, match="joint2 .* undervoltage"):
        await arm.move_to_joint_positions(JointPositions(values=[0, 0, 0, 0, 0, 0]))
    report = arm._health_report()
    assert report["joint2"]["status"] == "undervoltage" and report["joint2"]["fault"]


async def test_rs_move_and_read_round_trip(factory):
    arm = B601Arm.new(make_config("arm", **RS), {})
    for m in factory.latest.motors.values():
        m.vel_cap = 100.0
    target = [10.0, 20.0, 30.0, 5.0, 5.0, 40.0]  # inside the RS limits (joints 2/3 are positive on RS)
    await arm.move_to_joint_positions(JointPositions(values=target))
    got = (await arm.get_joint_positions()).values
    assert all(math.isclose(a, b, abs_tol=1.0) for a, b in zip(got, target))
    assert not await arm.is_moving()


async def test_rs_rejects_dm_shaped_targets(factory):
    arm = B601Arm.new(make_config("arm", **RS), {})
    with pytest.raises(ValueError, match="outside the limits"):
        await arm.move_to_joint_positions(JointPositions(values=[0, -90, 0, 0, 0, 0]))


def test_switching_variant_on_the_same_port_reopens_the_bus(factory):
    arm = B601Arm.new(make_config("arm", **RS), {})
    first = arm.bus
    arm.reconfigure(make_config("arm", **dict(FAST, port="can0")), {})
    assert arm.bus is not first and arm.bus.vendor == "damiao"
    assert first.controller is None  # released


def test_gripper_refuses_to_attach_to_an_rs_arm(factory):
    from src.rebot_b601.gripper import B601Gripper

    arm = B601Arm.new(make_config("arm", **RS), {})
    deps = {arm.get_resource_name("arm"): arm}
    with pytest.raises(ValueError, match="not supported on the B601-RS"):
        B601Gripper.new(make_config("gripper", arm="arm"), deps)
    assert arm.bus.controller is not None  # the arm's bus is untouched by the failed gripper build


def test_rs_attributes_still_override_the_defaults(factory):
    arm = B601Arm.new(make_config("arm", **dict(RS, mit_kp=[1, 2, 3, 4, 5, 6], joint_limits_deg=[[-10, 10]] * 6)), {})
    assert arm.mit_kp == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert arm.joint_limits == [(-10.0, 10.0)] * 6


def test_reconnect_reenables_active_report_on_rs(factory):
    arm = B601Arm.new(make_config("arm", **RS), {})
    for cid in ARM_CAN_IDS:
        factory.latest.motors[cid].active_report = False
    factory.persist(factory.latest)
    arm.bus.reconnect()
    assert all(factory.latest.motors[cid].active_report for cid in ARM_CAN_IDS)
    assert arm._torque_enabled


def test_monitor_tick_ignores_position_only_states_directly(factory, monkeypatch):
    arm = B601Arm.new(make_config("arm", **dict(RS, torque_limit_nm=1.0, temperature_limit_c=50.0)), {})
    monkeypatch.setattr(
        arm,
        "_read_states",
        lambda retries=1: {
            cid: bus_mod.PositionOnlyState(
                can_id=cid, arbitration_id=0xFD, status_code=0, pos=0.0, vel=0.0, torq=5.0, t_mos=99.0, t_rotor=0.0
            )
            for cid in ARM_CAN_IDS
        },
    )
    trip_counts = [2] * 6
    arm._monitor_tick(trip_counts)
    assert trip_counts == [2] * 6


async def test_rs_manual_mode_has_no_gravity_feedforward(factory):
    arm = B601Arm.new(make_config("arm", **dict(RS, gravity_scale=1.0)), {})
    assert arm.gravity_scale == 0.0
    assert arm.manual_torques([0.0] * 6) == [0.0] * 6
    with pytest.raises(NotImplementedError):
        await arm.do_command({"gravity_torques": True})
    dm = B601Arm.new(make_config("dm", **dict(FAST, gravity_scale=0.5)), {})
    assert dm.gravity_scale == 0.5


async def test_raw_state_on_rs_reports_health_dicts(factory):
    arm = B601Arm.new(make_config("arm", **RS), {})
    r = await arm.do_command({"raw_state": True})
    for name in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"):
        assert r["raw_state"][name]["position_only"] is False
        assert r["raw_state"][name]["status"] == "ok"
