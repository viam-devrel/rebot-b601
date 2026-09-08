"""SharedBus lifecycle: alias-safe caching, leak protection, and lock diagnostics."""

import gc
import os

import pytest
from motorbridge import CallError

from src.rebot_b601 import bus as bus_mod
from src.rebot_b601.arm import B601Arm
from src.rebot_b601.bus import BusError, SharedBus
from src.rebot_b601.gripper import B601Gripper
from tests.conftest import make_config

LOCK_MSG = "open serial port X failed: Unable to acquire exclusive lock on serial port"


@pytest.fixture
def device(tmp_path):
    """A real file standing in for /dev/ttyACM0, plus a by-id style symlink to it."""
    real = tmp_path / "ttyACM0"
    real.write_bytes(b"")
    link = tmp_path / "usb-HDSC_CDC_Device_0000-if00"
    link.symlink_to(real)
    return str(real), str(link)


def test_aliases_of_one_device_share_one_bus(factory, device):
    real, link = device
    a = SharedBus.acquire(link)
    b = SharedBus.acquire(real)
    assert a is b
    assert len(factory.controllers) == 1
    assert a.port == link  # configured spelling is kept for logs
    assert a.matches(real, bus_mod.DEFAULT_BAUD) and a.matches(link, bus_mod.DEFAULT_BAUD)
    assert not a.matches(real, 115200)
    a.release()
    assert not factory.latest.closed  # still held once
    b.release()
    assert factory.latest.closed
    assert SharedBus._instances == {}


def test_baud_mismatch_on_shared_device_is_an_error(factory):
    SharedBus.acquire("/dev/fake0", 921600)
    with pytest.raises(BusError, match="already open at 921600"):
        SharedBus.acquire("/dev/fake0", 115200)


def test_dropped_bus_still_closes_its_controller(factory):
    bus = SharedBus.acquire("/dev/fake0")
    ctrl = factory.latest
    SharedBus.reset_instances()  # simulate losing track of the bus without release()
    del bus
    gc.collect()
    assert ctrl.closed


def test_reconnect_does_not_leak_the_old_controller(factory):
    bus = SharedBus.acquire("/dev/fake0")
    first = factory.latest
    bus.reconnect(attempts=1)
    assert first.closed
    assert factory.latest is not first and not factory.latest.closed
    bus.release()
    assert factory.latest.closed


def test_failed_arm_build_releases_the_port(factory, monkeypatch):
    def boom(self):
        raise RuntimeError("motors did not answer")

    monkeypatch.setattr(B601Arm, "_configure_motors", boom)
    with pytest.raises(RuntimeError, match="did not answer"):
        B601Arm.new(make_config("arm", port="/dev/fake0"), {})
    assert factory.latest.closed
    assert SharedBus._instances == {}


def test_failed_gripper_build_releases_the_port(factory, monkeypatch):
    def boom(self):
        raise RuntimeError("gripper did not answer")

    monkeypatch.setattr(B601Gripper, "_configure_motor", boom)
    with pytest.raises(RuntimeError, match="did not answer"):
        B601Gripper.new(make_config("gripper", port="/dev/fake0"), {})
    assert factory.latest.closed
    assert SharedBus._instances == {}


@pytest.mark.skipif(not os.path.isdir("/proc/self/fd"), reason="needs Linux /proc")
def test_lock_held_by_this_process_is_reclaimed(factory, device, monkeypatch):
    real, link = device
    leaked_fd = os.open(real, os.O_RDONLY)  # a controller that was dropped without close()

    calls = []
    inner = factory.__call__

    def flaky(port, baud):
        calls.append(port)
        if len(calls) == 1:
            raise CallError(LOCK_MSG)
        return inner(port, baud)

    monkeypatch.setattr(SharedBus, "controller_factory", staticmethod(flaky))
    bus = SharedBus.acquire(link)
    assert bus.controller is factory.latest
    assert len(calls) == 2
    with pytest.raises(OSError):
        os.fstat(leaked_fd)  # the leaked descriptor was closed for us
    bus.release()


def test_lock_held_elsewhere_names_the_holder(factory, monkeypatch):
    def always_locked(port, baud):
        raise CallError(LOCK_MSG)

    monkeypatch.setattr(SharedBus, "controller_factory", staticmethod(always_locked))
    monkeypatch.setattr(bus_mod, "_own_holders", lambda device: [])
    monkeypatch.setattr(bus_mod, "_other_holders", lambda device: [(4242, "python lerobot/teleop.py")])
    with pytest.raises(BusError) as info:
        SharedBus.acquire("/dev/fake0")
    msg = str(info.value)
    assert "exclusive lock" in msg and "pid 4242" in msg and "lerobot" in msg
    assert SharedBus._instances == {}


def test_non_lock_open_failure_is_reported_plainly(factory):
    factory.fail_next_open = 1
    with pytest.raises(BusError, match="cannot open /dev/fake0") as info:
        SharedBus.acquire("/dev/fake0")
    assert "held by" not in str(info.value)
