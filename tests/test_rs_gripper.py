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
