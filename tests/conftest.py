import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from viam.proto.app.robot import ComponentConfig  # noqa: E402
from viam.utils import dict_to_struct  # noqa: E402

from src.rebot_b601 import arm_service  # noqa: E402
from src.rebot_b601.bus import SharedBus  # noqa: E402
from tests.fake_bus import FakeFactory  # noqa: E402

arm_service.install()


@pytest.fixture
def factory(monkeypatch):
    f = FakeFactory()
    monkeypatch.setattr(SharedBus, "controller_factory", staticmethod(f))
    SharedBus.reset_instances()
    yield f
    SharedBus.reset_instances()


def make_config(name: str, **attrs) -> ComponentConfig:
    return ComponentConfig(name=name, attributes=dict_to_struct(attrs))


FAST = dict(port="/dev/fake0", speed_deg_s=180, acceleration_deg_s2=1000, move_hz=100, tolerance_deg=1.0)


@pytest.fixture
def fast_arm(factory):
    from src.rebot_b601.arm import B601Arm

    arm = B601Arm.new(make_config("arm", **FAST), {})
    ctrl = factory.latest
    for m in ctrl.motors.values():
        m.vel_cap = 100.0  # rad/s: the fake keeps up with any streamed setpoint
    yield arm, ctrl
