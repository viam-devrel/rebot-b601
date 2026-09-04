import math

import pytest

from src.rebot_b601.arm import B601Arm
from src.rebot_b601.bus import SharedBus
from src.rebot_b601.damiao import MotorFault
from src.rebot_b601.gripper import GRIPPER_CAN_ID, B601Gripper
from tests.conftest import make_config


@pytest.fixture
def gripper(factory):
    g = B601Gripper.new(make_config("gripper", port="/dev/fake0", move_timeout_s=2.0), {})
    motor = factory.latest.motors[GRIPPER_CAN_ID]
    motor.vel_cap = 1000.0
    return g, motor


async def test_open_and_grab_without_object(gripper):
    g, motor = gripper
    await g.open()
    assert math.isclose(math.degrees(motor.pos), -270.0, abs_tol=2.0)
    held = await g.grab()
    assert held is False
    assert math.isclose(math.degrees(motor.pos), 0.0, abs_tol=2.0)
    status = await g.is_holding_something()
    assert status.is_holding_something is False


async def test_grab_detects_object_by_stall(gripper):
    g, motor = gripper
    await g.open()
    motor.stall_at = math.radians(-120.0)  # jaws meet an object here
    held = await g.grab()
    assert held is True
    status = await g.is_holding_something()
    assert status.is_holding_something and math.isclose(status.meta["stall_position_deg"], -120.0, abs_tol=2.0)


async def test_inputs_are_finger_travel_in_metres(gripper):
    g, motor = gripper
    await g.go_to_inputs([0.025])
    assert math.isclose(math.degrees(motor.pos), -135.0, abs_tol=2.0)
    inputs = await g.get_current_inputs()
    assert len(inputs) == 1 and math.isclose(inputs[0], 0.025, abs_tol=0.001)
    with pytest.raises(ValueError):
        await g.go_to_inputs([1, 2])
    kin = await g.get_kinematics()
    assert b"prismatic" in kin[1]
    geos = await g.get_geometries()
    assert len(geos) == 3


async def test_do_commands(gripper):
    g, motor = gripper
    res = await g.do_command({"set": {"fraction": 0.5}})
    assert math.isclose(res["set"]["open_fraction"], 0.5, abs_tol=0.02)
    res = await g.do_command({"get": True})
    assert math.isclose(res["get"]["pos_deg"], -135.0, abs_tol=2.0)
    res = await g.do_command({"set_speed": 500, "set_force": 0.2})
    assert res["set_speed"] == 500.0 and res["set_force"] == 0.2
    res = await g.do_command({"grab_with_force": {"position": 0, "speed": 800, "force": 0.5}})
    assert res["grab_with_force"]["holding"] is False
    last = [c for c in motor.commands if c[0] == "force_pos"][-1]
    assert math.isclose(last[2], math.radians(800)) and last[3] == 0.5
    with pytest.raises(ValueError):
        await g.do_command({"set_force": 2.0})
    res = await g.do_command({"status": True})
    assert res["status"]["status"] == "enabled" and "open_fraction" in res["status"]


async def test_fault_blocks_grab(gripper):
    g, motor = gripper
    motor.status_code = 0xE
    with pytest.raises(MotorFault, match="overload"):
        await g.grab()
    await g.do_command({"clear_errors": True})
    assert motor.status_code == 0x1


async def test_gripper_shares_bus_with_arm_and_inherits_port(factory):
    arm = B601Arm.new(make_config("arm", port="/dev/fake7"), {})
    deps = {arm.get_resource_name("arm"): arm}
    g = B601Gripper.new(make_config("gripper", arm="arm"), deps)
    assert g.bus is arm.bus and g.bus.port == "/dev/fake7"
    assert len(factory.controllers) == 1
    reqs, _ = B601Gripper.validate_config(make_config("gripper", arm="arm"))
    assert reqs == ["arm"]
    await g.close()
    assert not factory.latest.closed  # arm still holds the bus
    await arm.close()
    assert factory.latest.closed
    assert SharedBus._instances == {}
