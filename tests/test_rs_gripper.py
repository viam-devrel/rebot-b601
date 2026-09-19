import asyncio
import logging
import math
import time

import pytest
from motorbridge import CallError

from src.rebot_b601 import spatial
from src.rebot_b601.gripper import (
    DEFAULT_GRIP_CURRENT_A,
    GRIPPER_CAN_ID,
    MAX_SPEED_DEG_S,
    RS_ACC_RAMP_S,
    RS_MAX_SPEED_DEG_S,
    RS_OPEN_DEG,
    RS_RID_LIMIT_CUR,
    RS_TRACK_HEADROOM,
    B601Gripper,
)
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


def test_rs_open_position_defaults_to_the_bench_measurement(factory):
    """340 deg was measured by jogging the jaw from closed to its hard stop, so RS no longer
    fails to configure without the attribute."""
    B601Gripper.validate_config(make_config("gripper", variant="rs", port="can0"))
    g = B601Gripper.new(make_config("gripper", variant="rs", port="can0"), {})
    assert g.open_deg == RS_OPEN_DEG == 340.0


def test_rs_open_position_is_still_overridable(rs_gripper):
    g, _ = rs_gripper
    assert g.open_deg == -120.0


@pytest.mark.parametrize("open_deg", [0.0, 100000.0])
def test_an_unusable_open_position_is_still_rejected(factory, open_deg):
    """A default removes the hard requirement, not the check: a zero or absurd span silently
    misreports finger travel instead of failing."""
    with pytest.raises(ValueError, match="open_position_deg"):
        B601Gripper.validate_config(make_config("gripper", variant="rs", port="can0", open_position_deg=open_deg))


def test_rs_logs_the_open_position_in_use(factory, caplog):
    """It is only right if the motor was zeroed with the jaws closed, and nothing else would
    show that assumption on a running machine."""
    with caplog.at_level(logging.INFO, logger="src.rebot_b601.gripper"):
        B601Gripper.new(make_config("gripper", variant="rs", port="can0"), {})
    line = next(r.getMessage() for r in caplog.records if "open_position_deg" in r.getMessage())
    assert "340.0" in line
    assert "zeroed with the jaws closed" in line


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


def test_rs_holds_its_current_position_at_configure(rs_gripper):
    """Profile position resumes a stale internal setpoint after a restart, so configure has to
    command where the jaws already are or they lurch."""
    g, motor = rs_gripper
    motor.pos, motor.target = math.radians(-55.0), None  # moved by hand while the module was down
    motor.commands.clear()
    g._configure_motor()
    kind, pos, _, _ = motor.commands[-1]
    assert kind == "pos_vel_pp"
    assert pos == pytest.approx(math.radians(-55.0))


def test_dm_does_not_command_a_hold_at_configure(gripper):
    g, motor = gripper
    motor.commands.clear()
    g._configure_motor()
    assert motor.commands == []


def test_rs_clamps_the_configured_speed_and_warns(factory, caplog):
    """The bench config carried speed_deg_s: 3000 -- 52 rad/s, far past the motor -- and nothing
    said so, because only set_speed enforced the range."""
    with caplog.at_level(logging.WARNING, logger="src.rebot_b601.gripper"):
        g = B601Gripper.new(make_config("gripper", **RS, speed_deg_s=3000.0), {})
    assert g.speed_deg_s == pytest.approx(RS_MAX_SPEED_DEG_S)
    assert any("clamped" in r.getMessage() for r in caplog.records)


def test_dm_keeps_the_faster_leadscrew_ceiling(factory):
    assert RS_MAX_SPEED_DEG_S < MAX_SPEED_DEG_S
    g = B601Gripper.new(make_config("gripper", port="/dev/fake0", speed_deg_s=3000.0), {})
    assert g.speed_deg_s == pytest.approx(MAX_SPEED_DEG_S)


def test_rs_writes_grip_current_a_to_limit_cur(factory):
    B601Gripper.new(make_config("gripper", **RS, grip_current_a=0.6), {})
    motor = factory.latest.motors[GRIPPER_CAN_ID]
    assert motor.params[RS_RID_LIMIT_CUR] == pytest.approx(0.6)


def test_rs_grip_current_defaults_without_the_attribute(rs_gripper):
    _, motor = rs_gripper
    assert motor.params[RS_RID_LIMIT_CUR] == pytest.approx(DEFAULT_GRIP_CURRENT_A)


def test_rs_grip_current_does_not_ratchet_across_restarts(rs_gripper):
    """limit_cur lives in motor RAM and survives a resource restart, so a cap derived from what
    the motor already holds would shrink every time the module rebuilt until the jaw could not
    close at all. Each new resource is a fresh object against the same motor: the absolute value
    must land unchanged every time."""
    _, motor = rs_gripper
    for _ in range(3):
        B601Gripper.new(make_config("gripper", **RS), {})
        assert motor.params[RS_RID_LIMIT_CUR] == pytest.approx(DEFAULT_GRIP_CURRENT_A)


def test_rs_rejects_a_non_positive_grip_current(factory):
    with pytest.raises(ValueError, match="grip_current_a"):
        B601Gripper.validate_config(make_config("gripper", **RS, grip_current_a=0.0))


def test_torque_ratio_on_rs_warns_and_points_at_grip_current(factory, caplog):
    with caplog.at_level(logging.WARNING, logger="src.rebot_b601.gripper"):
        B601Gripper.new(make_config("gripper", **RS, torque_ratio=0.5), {})
    warning = next(r.getMessage() for r in caplog.records if "torque_ratio" in r.getMessage())
    assert "grip_current_a" in warning


async def test_rs_moves_send_profile_position_not_force_pos(rs_gripper):
    """The dedicated PP frame, not the generic pos_vel one: Mode.POS_VEL on a RobStride is native
    mode 2 (PP). Its vel_max no longer sets the speed -- the setpoint spacing does -- so it goes
    out with RS_TRACK_HEADROOM of slack, enough to reach the next setpoint within a tick whether
    or not the firmware honours it."""
    g, motor = rs_gripper
    await g.open()
    assert {c[0] for c in motor.commands} == {"pos_vel_pp"}
    _, pos, vel_max, acc = motor.commands[-1]
    assert pos == pytest.approx(math.radians(g.open_deg))
    assert vel_max == pytest.approx(math.radians(g.speed_deg_s) * RS_TRACK_HEADROOM)
    assert acc == pytest.approx(vel_max / RS_ACC_RAMP_S)


def test_rs_speed_is_the_spacing_of_the_setpoints(rs_gripper):
    """The fix for speed_deg_s. Every knob RobStride offers was tried -- the generic frame's
    velocity field, limit_spd, the PP frame's vel_max -- and 10 to 200 deg/s looked identical
    on the bench, because the *motor* was planning the move. Plan it here, as the arm does, and
    a tenth of the speed is ten times the setpoints."""
    g, _ = rs_gripper
    slow = g._plan_setpoints(0.0, -120.0, 10.0)
    fast = g._plan_setpoints(0.0, -120.0, 100.0)
    assert len(slow) > 8 * len(fast)
    assert slow == sorted(slow, reverse=True), "setpoints must walk from start to target"
    assert slow[0] > -5.0 and slow[-1] == pytest.approx(-120.0)


async def test_rs_streams_the_move_instead_of_one_far_target(rs_gripper):
    """End to end: a move is many setpoints, and a slower speed_deg_s is more of them."""
    g, motor = rs_gripper
    motor.commands.clear()
    await g.do_command({"set": {"deg": -20.0}})
    fast = [math.degrees(c[1]) for c in motor.commands]
    assert len(fast) > 1, "one setpoint: the motor is still the one planning the move"
    assert fast == sorted(fast, reverse=True)
    await g.do_command({"set_speed": 30.0})
    motor.commands.clear()
    await g.do_command({"set": {"deg": 0.0}})
    assert len(motor.commands) > 3 * len(fast)


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


async def test_rs_recommands_the_position_the_jaws_reached_after_a_stall(rs_gripper):
    """Left driving at an unreachable target, profile position grinds into the object until the
    motor faults. The last frame must ask for where the jaws actually stopped."""
    g, motor = rs_gripper
    motor.pos = math.radians(g.open_deg)
    motor.stall_at = math.radians(-20.0)  # an object stops the jaws short of closed
    await g.grab()
    kind, pos, _, _ = motor.commands[-1]
    assert kind == "pos_vel_pp"
    assert math.degrees(pos) == pytest.approx(-20.0, abs=1.0)
    assert math.degrees(pos) != pytest.approx(g.closed_deg, abs=1.0)


async def test_rs_stall_stops_the_stream_and_holds_where_the_jaw_stopped(rs_gripper):
    """Streaming makes the stall obvious -- the setpoints run away from the jaw -- and the last
    thing sent must be the angle the jaw actually reached, not a setpoint past the object."""
    g, motor = rs_gripper
    motor.pos = motor.target = math.radians(g.open_deg)  # at rest, fully open
    await g.do_command({"set_speed": 30.0})  # slow enough that a tick is 1.5 deg
    motor.stall_at = math.radians(-100.0)  # an object, 20 deg into a 120 deg close
    motor.commands.clear()
    pos = await asyncio.to_thread(g._move_until_settled, g.closed_deg)
    assert pos == pytest.approx(-100.0, abs=1.0)
    sent = [math.degrees(c[1]) for c in motor.commands]
    assert sent[-1] == pytest.approx(-100.0, abs=1.0), "did not hold at the measured position"
    assert max(sent[:-1]) < -50.0, "kept streaming toward the target after the jaw stopped"


async def test_rs_does_not_recommand_when_the_jaws_arrive(rs_gripper):
    g, motor = rs_gripper
    await g.open()
    assert motor.commands[-1][1] == pytest.approx(math.radians(g.open_deg))


async def test_rs_does_not_false_stall_when_the_streamed_frame_is_stale(rs_gripper):
    """Bench 2026-09-19: the RS status stream runs about half a second behind the motor, so every
    50 ms poll of a move read the same angle, the loop called it a stall a third of a second in,
    and then re-commanded that stale angle -- open() moved a centimetre and drove back to closed.
    Position samples during a move must come from the mechPos round trip, which is current."""
    g, motor = rs_gripper
    motor.stream_lag = True  # get_state() freezes; mechPos stays live
    pos = await asyncio.to_thread(g._move_until_settled, g.open_deg)
    assert pos == pytest.approx(g.open_deg, abs=2.0)
    assert math.degrees(motor.commands[-1][1]) == pytest.approx(g.open_deg, abs=2.0)


async def test_rs_does_not_recommand_a_position_it_never_saw_move(rs_gripper, caplog):
    """If the jaw never appears to move, the reading is what is suspect, not the jaw; commanding
    from it turns a bad read into a physical reversal. Say so instead."""
    g, motor = rs_gripper
    motor.stall_at = 0.0  # jammed: the jaw cannot leave where it is
    motor.commands.clear()
    with caplog.at_level(logging.WARNING, logger="src.rebot_b601.gripper"):
        await asyncio.to_thread(g._move_until_settled, g.open_deg)
    sent = [math.degrees(c[1]) for c in motor.commands]
    assert sent == sorted(sent, reverse=True), "a setpoint went backwards: the stale reading was re-commanded"
    assert sent[-1] > g.open_deg, "kept streaming to the target through a stall"
    assert any("stale" in r.getMessage() for r in caplog.records)


async def test_rs_move_survives_a_mechpos_timeout(rs_gripper):
    """A timed-out parameter read is a missing sample, not a still jaw: it must neither wedge the
    loop nor be counted toward the stall streak."""
    g, motor = rs_gripper
    motor.stream_lag = True
    reads = {"n": 0}
    real = motor.robstride_get_param_f32

    def flaky(param_id, timeout_ms=1000):
        if param_id == 0x7019:
            reads["n"] += 1
            if reads["n"] % 2:  # every other mechPos read goes unanswered
                raise CallError("param 0x7019 read timed out")
        return real(param_id, timeout_ms)

    motor.robstride_get_param_f32 = flaky
    pos = await asyncio.to_thread(g._move_until_settled, g.open_deg)
    assert pos == pytest.approx(g.open_deg, abs=2.0)
