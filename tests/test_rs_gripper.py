import asyncio
import math
import time

import pytest

from src.rebot_b601 import spatial
from src.rebot_b601.gripper import GRIPPER_CAN_ID, B601Gripper
from tests.conftest import make_config

RS = dict(variant="rs", port="can0", open_position_deg=-120.0)


@pytest.fixture
def rs_gripper(factory):
    g = B601Gripper.new(make_config("gripper", **RS, move_timeout_s=2.0), {})
    motor = factory.latest.motors[GRIPPER_CAN_ID]
    motor.vel_cap = 1000.0
    return g, motor


def test_rs_needs_a_port(factory):
    with pytest.raises(ValueError, match="port"):
        B601Gripper.validate_config(make_config("gripper", variant="rs", open_position_deg=-120.0))


def test_rs_needs_an_open_position(factory):
    with pytest.raises(ValueError, match="open_position_deg"):
        B601Gripper.validate_config(make_config("gripper", variant="rs", port="can0"))


def test_dm_still_validates_without_either(factory):
    deps, _ = B601Gripper.validate_config(make_config("gripper", port="/dev/fake0"))
    assert deps == []


def test_rs_gripper_holds_the_rs_model_and_a_robstride_bus(rs_gripper):
    g, motor = rs_gripper
    assert g.variant == "rs"
    assert g.model is spatial.MODELS["rs"]
    assert g.bus.vendor == "robstride"
    assert motor.active_report is True


def test_rs_travel_uses_the_rs_spec(rs_gripper):
    g, _ = rs_gripper
    assert g.travel_m_from_deg(g.open_deg) == pytest.approx(spatial.MODELS["rs"].gripper.travel_m)
    assert g.travel_m_from_deg(g.closed_deg) == pytest.approx(0.0)


def test_torque_ratio_on_rs_warns_and_is_ignored(factory, caplog):
    import logging

    with caplog.at_level(logging.WARNING, logger="src.rebot_b601.gripper"):
        B601Gripper.new(make_config("gripper", **RS, torque_ratio=0.5), {})
    assert any("torque_ratio" in r.getMessage() for r in caplog.records)


async def test_rs_moves_send_profile_position_not_force_pos(rs_gripper):
    g, motor = rs_gripper
    await g.open()
    assert {c[0] for c in motor.commands} == {"pos_vel"}
    pos, vlim = motor.commands[-1][1], motor.commands[-1][2]
    assert pos == pytest.approx(math.radians(g.open_deg))
    assert vlim == pytest.approx(math.radians(g.speed_deg_s))


async def test_rs_settles_on_position_delta_despite_velocity_noise(rs_gripper):
    """The velocity predicate never fires with this noise, so the loop would run to the
    timeout and return the same position. Timing is what discriminates the two branches."""
    g, motor = rs_gripper
    motor.vel_noise = -0.150  # the bench artefact: a resting motor never reads near zero
    motor.stall_at = math.radians(-30.0)  # an object stops the jaws well short of open
    t0 = time.monotonic()
    pos = await asyncio.to_thread(g._move_until_settled, g.open_deg)
    elapsed = time.monotonic() - t0
    assert pos == pytest.approx(-30.0, abs=1.0)
    assert elapsed < g.move_timeout_s / 2, "settled on the timeout, not on the position delta"


async def test_dm_settling_is_unaffected(gripper):
    """A regression guard, not a discriminator: it proves adding the vendor branch did not
    disturb DM, which the RS predicate would also satisfy here."""
    g, motor = gripper
    motor.stall_at = math.radians(-30.0)
    t0 = time.monotonic()
    pos = await asyncio.to_thread(g._move_until_settled, g.open_deg)
    assert pos == pytest.approx(-30.0, abs=1.0)
    assert time.monotonic() - t0 < g.move_timeout_s / 2


async def test_rs_reads_position_by_parameter_when_torque_is_off(rs_gripper):
    g, motor = rs_gripper
    await g.do_command({"torque": "disable"})
    motor.pos = math.radians(-42.0)  # moved by hand; the cached frame is stale
    before = motor.param_reads
    state = await asyncio.to_thread(g._state)
    assert motor.param_reads > before, "must read mechPos, not the frozen frame"
    assert math.degrees(state.pos) == pytest.approx(-42.0, abs=0.1)


async def test_rs_is_moving_reports_only_an_operation_in_flight(rs_gripper):
    g, motor = rs_gripper
    motor.vel_noise = -0.150
    assert await g.is_moving() is False


async def test_rs_grab_closes_and_reports_no_holding(rs_gripper):
    g, motor = rs_gripper
    motor.pos = math.radians(g.open_deg)  # start open, or the close ends on the first poll
    motor.stall_at = math.radians(-20.0)  # an object stops the jaws short of closed
    assert await g.grab() is False
    status = await g.is_holding_something()
    assert status.is_holding_something is False
    assert "stall_position_deg" in status.meta


@pytest.mark.parametrize("cmd", [{"set_force": 0.5}, {"get_force": True}, {"grab_with_force": {}}])
async def test_force_commands_refuse_on_rs(rs_gripper, cmd):
    g, _ = rs_gripper
    with pytest.raises(ValueError, match="B601-RS"):
        await g.do_command(cmd)


async def test_force_commands_still_work_on_dm(gripper):
    g, _ = gripper
    assert (await g.do_command({"set_force": 0.5}))["set_force"] == 0.5
