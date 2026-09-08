import asyncio
import math

import pytest
from viam.proto.component.arm import JointPositions, MoveOptions

from src.rebot_b601.arm import ARM_CAN_IDS, B601Arm
from src.rebot_b601.bus import BusError
from src.rebot_b601.damiao import CollisionError, MotorFault, OverTemperatureError
from tests.conftest import FAST, make_config


def positions(ctrl):
    return [math.degrees(ctrl.motors[c].pos) for c in ARM_CAN_IDS]


def test_new_opens_bus_and_enables_torque(factory):
    arm = B601Arm.new(make_config("arm", port="/dev/fake0"), {})
    ctrl = factory.latest
    assert ctrl.port == "/dev/fake0" and ctrl.baud == 921600
    assert all(ctrl.motors[c].enabled for c in ARM_CAN_IDS)
    assert arm._torque_enabled


def test_validate_config_rejects_bad_values():
    with pytest.raises(ValueError):
        B601Arm.validate_config(make_config("a", control_mode="nope"))
    with pytest.raises(ValueError):
        B601Arm.validate_config(make_config("a", speed_deg_s=[1, 2]))
    with pytest.raises(ValueError):
        B601Arm.validate_config(make_config("a", bad_joints=[7]))
    with pytest.raises(ValueError):
        B601Arm.validate_config(make_config("a", collision_geometry="voxels"))
    deps, _ = B601Arm.validate_config(make_config("a", motion="builtin"))
    assert deps == ["builtin"]


async def test_move_to_joint_positions_streams_and_settles(fast_arm):
    arm, ctrl = fast_arm
    await arm.move_to_joint_positions(JointPositions(values=[10, -20, -30, 5, 5, 40]))
    got = (await arm.get_joint_positions()).values
    assert all(math.isclose(a, b, abs_tol=1.0) for a, b in zip(got, [10, -20, -30, 5, 5, 40]))
    # many interpolated setpoints were sent, not a single jump
    assert len(ctrl.motors[1].commands) > 5
    assert not await arm.is_moving()


async def test_move_through_joint_positions_visits_waypoints_in_order(fast_arm):
    arm, ctrl = fast_arm
    wps = [JointPositions(values=[20, 0, 0, 0, 0, 0]), JointPositions(values=[20, -30, 0, 0, 0, 0])]
    await arm.move_through_joint_positions(wps, None)
    cmds = [c[1] for c in ctrl.motors[2].commands]  # joint2 position setpoints (rad)
    first_move_idx = next(i for i, p in enumerate(cmds) if p < math.radians(-0.5))
    # joint1 had already reached 20 deg when joint2 started moving
    j1_at = ctrl.motors[1].commands[first_move_idx][1]
    assert math.isclose(math.degrees(j1_at), 20.0, abs_tol=1.0)
    assert math.isclose(positions(ctrl)[1], -30.0, abs_tol=1.0)


async def test_move_options_override_speed(fast_arm):
    arm, ctrl = fast_arm
    opts = MoveOptions(max_vel_degs_per_sec=30.0)
    mo = arm._move_options(opts, None)
    assert mo.vmax_deg_s == [30.0] * 6
    mo = arm._move_options(None, {"speed_d": 90, "acceleration_d": 500, "direct": True, "waitAtEnd": False})
    assert mo.vmax_deg_s == [90.0] * 6 and mo.amax_deg_s2 == [500.0] * 6 and mo.direct and not mo.wait_at_end
    mo = arm._move_options(MoveOptions(max_vel_degs_per_sec_joints=[10, 20, 30, 40, 50, 60]), None)
    assert mo.vmax_deg_s == [10, 20, 30, 40, 50, 60]
    # clamped to the hardware envelope
    assert arm._move_options(None, {"speed_d": 10000}).vmax_deg_s[0] == 180.0


async def test_out_of_limit_target_is_rejected_by_default(fast_arm):
    arm, ctrl = fast_arm
    with pytest.raises(ValueError, match="outside the limits"):
        await arm.move_to_joint_positions(JointPositions(values=[0, 90, 0, 0, 0, 0]))
    assert positions(ctrl)[1] == 0.0


async def test_clip_targets_option(factory):
    arm = B601Arm.new(make_config("arm", clip_targets=True, **FAST), {})
    ctrl = factory.latest
    for m in ctrl.motors.values():
        m.vel_cap = 100.0
    await arm.move_to_joint_positions(JointPositions(values=[0, 90, 0, 0, 0, 0]))
    assert math.isclose(positions(ctrl)[1], 1.0, abs_tol=1.0)


async def test_bad_joints_are_held(factory):
    arm = B601Arm.new(make_config("arm", bad_joints=[5], **FAST), {})
    ctrl = factory.latest
    for m in ctrl.motors.values():
        m.vel_cap = 100.0
    ctrl.motors[6].pos = math.radians(33.0)
    await arm.move_to_joint_positions(JointPositions(values=[10, 0, 0, 0, 0, 999]))
    assert math.isclose(positions(ctrl)[5], 33.0, abs_tol=0.5)
    assert math.isclose(positions(ctrl)[0], 10.0, abs_tol=1.0)


async def test_direct_move_sends_single_setpoint(fast_arm):
    arm, ctrl = fast_arm
    await arm.move_to_joint_positions(JointPositions(values=[5, 0, 0, 0, 0, 0]), extra={"direct": True})
    pos_cmds = [c for c in ctrl.motors[1].commands if c[0] == "pos_vel"]
    assert len(pos_cmds) == 1
    with pytest.raises(ValueError):
        await arm.move_through_joint_positions([JointPositions(values=[0] * 6)] * 2, None, extra={"direct": True})


async def test_stop_cancels_running_move(fast_arm):
    arm, ctrl = fast_arm
    for m in ctrl.motors.values():
        m.vel_cap = 100.0
    task = asyncio.create_task(
        arm.move_to_joint_positions(JointPositions(values=[100, 0, 0, 0, 0, 0]), extra={"speed_d": 20})
    )
    await asyncio.sleep(0.3)
    assert await arm.is_moving()
    await arm.stop()
    await asyncio.wait_for(task, 2.0)
    assert positions(ctrl)[0] < 30.0
    assert not arm.ops.running


async def test_end_position_and_kinematics(fast_arm):
    arm, ctrl = fast_arm
    pose = await arm.get_end_position()
    assert math.isclose(pose.x, 260.3, abs_tol=0.5) and math.isclose(pose.z, 191.7, abs_tol=0.5)
    kin = await arm.get_kinematics()
    assert len(kin) == 2 and b"<box" in kin[1]
    geos = await arm.get_geometries()
    assert len(geos) == 7
    models = await arm.get_3d_models()
    assert "link6" in models


async def test_hard_fault_blocks_move_with_decoded_message(fast_arm):
    arm, ctrl = fast_arm
    ctrl.motors[3].status_code = 0xA  # over-current on joint3
    with pytest.raises(MotorFault, match="joint3 .* over-current"):
        await arm.move_to_joint_positions(JointPositions(values=[1, 0, 0, 0, 0, 0]))
    res = await arm.do_command({"clear_errors": True})
    assert res["clear_errors"] == "ok" and ctrl.motors[3].status_code == 0x1


async def test_transient_fault_is_cleared_automatically(fast_arm):
    arm, ctrl = fast_arm
    ctrl.motors[2].status_code = 0xD  # communication loss
    await arm.move_to_joint_positions(JointPositions(values=[1, 0, 0, 0, 0, 0]))
    assert ctrl.motors[2].errors_cleared == 1 and ctrl.motors[2].status_code == 0x1


async def test_over_temperature_refuses_move(fast_arm):
    arm, ctrl = fast_arm
    ctrl.motors[4].t_mos = 95.0
    with pytest.raises(OverTemperatureError):
        await arm.move_to_joint_positions(JointPositions(values=[1, 0, 0, 0, 0, 0]))


async def test_torque_limit_trips_collision(factory):
    arm = B601Arm.new(make_config("arm", torque_limit_nm=2.0, torque_trip_polls=1, **FAST), {})
    ctrl = factory.latest
    for m in ctrl.motors.values():
        m.vel_cap = 100.0
    ctrl.motors[2].torq = 5.0
    with pytest.raises(CollisionError, match="joint2"):
        await arm.move_to_joint_positions(JointPositions(values=[40, 0, 0, 0, 0, 0]), extra={"speed_d": 20})
    # the arm was told to hold where it was, not to keep going to 40
    assert positions(ctrl)[0] < 39.0
    assert not arm.ops.running


async def test_link_failure_reconnects_and_restores_torque(fast_arm, factory):
    arm, ctrl = fast_arm
    factory.persist(ctrl)
    ctrl.link_down = True
    got = (await arm.get_joint_positions()).values
    assert len(got) == 6
    assert len(factory.controllers) == 2 and arm.bus.reconnects == 1
    assert all(factory.latest.motors[c].enabled for c in ARM_CAN_IDS)


async def test_link_failure_without_reconnect_raises_bus_error(factory):
    arm = B601Arm.new(make_config("arm", reconnect=False, **FAST), {})
    factory.latest.link_down = True
    with pytest.raises(BusError):
        await arm.get_joint_positions()


async def test_do_commands(fast_arm):
    arm, ctrl = fast_arm
    res = await arm.do_command({"set_speed": 45, "set_acceleration": 300})
    assert res["set_speed"] == [45.0] * 6 and res["set_acceleration"] == [300.0] * 6
    res = await arm.do_command({"status": True})
    assert res["status"]["joint1"]["status"] == "enabled" and res["status"]["torque_enabled"]
    res = await arm.do_command({"load": True})
    assert len(res["load"]) == 6
    res = await arm.do_command({"torque": "disable"})
    assert res["torque"] == "disabled" and not ctrl.motors[1].enabled
    res = await arm.do_command({"torque": "enable"})
    assert ctrl.motors[1].enabled
    res = await arm.do_command({"set_zero_position": True})
    assert all(ctrl.motors[c].zeroed == 1 for c in ARM_CAN_IDS)
    res = await arm.do_command({"raw_state": True})
    assert res["raw_state"]["joint1"]["status"] == "enabled"
    with pytest.raises(ValueError):
        await arm.do_command({"bogus": 1})


async def test_manual_mode_sends_gravity_feedforward(fast_arm):
    arm, ctrl = fast_arm
    res = await arm.do_command({"manual_mode": "enter"})
    assert res["manual_mode"] == "entered manual mode"
    await asyncio.sleep(0.15)
    mit = [c for c in ctrl.motors[2].commands if c[0] == "mit"]
    assert mit, "manual loop should stream MIT frames"
    _, _, _, kp, kd, tau = mit[-1]
    assert kp == 0.0 and kd == 0.5
    expected = arm.manual_torques([0.0] * 6)[1]
    assert math.isclose(tau, expected, rel_tol=1e-6) and abs(tau) > 1.0
    with pytest.raises(RuntimeError, match="manual mode"):
        await arm.move_to_joint_positions(JointPositions(values=[0] * 6))
    res = await arm.do_command({"manual_mode": "exit"})
    assert res["manual_mode"] == "exited manual mode"
    assert ctrl.motors[2].mode is not None
    await arm.move_to_joint_positions(JointPositions(values=[1, 0, 0, 0, 0, 0]))


async def test_move_to_position_requires_motion_service(fast_arm):
    arm, _ = fast_arm
    from viam.proto.common import Pose

    with pytest.raises(NotImplementedError, match="motion"):
        await arm.move_to_position(Pose(x=1, y=2, z=3, o_z=1))


async def test_move_to_position_delegates_to_motion(factory):
    from viam.proto.common import Pose
    from viam.services.motion import MotionClient

    class FakeMotion:
        def __init__(self):
            self.calls = []

        async def move(self, component_name, destination, **kw):
            self.calls.append((component_name, destination))
            return True

    fm = FakeMotion()
    deps = {MotionClient.get_resource_name("builtin"): fm}
    arm = B601Arm.new(make_config("arm", motion="builtin", **FAST), deps)
    await arm.move_to_position(Pose(x=100, y=0, z=200, o_z=1, theta=0))
    assert fm.calls and fm.calls[0][0] == "arm" and fm.calls[0][1].reference_frame == "arm_origin"


async def test_streamed_move_paces_points(fast_arm):
    arm, ctrl = fast_arm
    session = arm.start_streamed_move(None)
    session.feed([(0.0, [0, 0, 0, 0, 0, 0]), (0.1, [5, 0, 0, 0, 0, 0]), (0.2, [10, 0, 0, 0, 0, 0])])
    session.feed([(0.3, [15, 0, 0, 0, 0, 0])])
    await asyncio.to_thread(session.finish)
    assert math.isclose(positions(ctrl)[0], 15.0, abs_tol=1.0)
    sent = [round(math.degrees(c[1])) for c in ctrl.motors[1].commands if c[0] == "pos_vel"]
    assert sent[:4] == [0, 5, 10, 15]


async def test_streamed_move_rejects_out_of_limit(fast_arm):
    arm, ctrl = fast_arm
    session = arm.start_streamed_move(None)
    session.feed([(0.0, [0, 0, 0, 0, 0, 0]), (0.05, [0, 90, 0, 0, 0, 0])])
    with pytest.raises(ValueError):
        await asyncio.to_thread(session.finish)


async def test_close_releases_bus(fast_arm):
    arm, ctrl = fast_arm
    await arm.close()
    assert ctrl.closed and arm.bus is None
