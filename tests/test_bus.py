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


def test_release_frees_motor_handles_before_the_controller(factory):
    bus = SharedBus.acquire("/dev/fake0")
    handles = [bus.motor(cid) for cid in (0x01, 0x07)]
    bus.release()
    assert all(h.closed for h in handles) and factory.latest.closed


def test_dropped_bus_frees_motor_handles_too(factory):
    bus = SharedBus.acquire("/dev/fake0")
    handle = bus.motor(0x01)
    SharedBus.reset_instances()
    del bus
    gc.collect()
    assert handle.closed


def test_reconnect_does_not_leak_the_old_controller(factory):
    bus = SharedBus.acquire("/dev/fake0")
    first = factory.latest
    old_handle = bus.motor(0x01)
    bus.reconnect(attempts=1)
    assert first.closed and old_handle.closed
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


# ---------------------------------------------------------------- discovery

HDSC = bus_mod.UsbSerialPort(
    device="/dev/ttyACM1",
    vid="2e88",
    pid="4603",
    serial="00000000050C",
    product="HDSC CDC Device",
    by_id="/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00",
)
TEENSY = bus_mod.UsbSerialPort(
    device="/dev/ttyACM0", vid="16c0", pid="0483", serial="1124", product="USB Serial", by_id=None
)


def test_detect_port_picks_the_b601_by_usb_identity_not_enumeration_order(monkeypatch):
    monkeypatch.setattr(bus_mod, "usb_serial_ports", lambda: [TEENSY, HDSC])
    assert bus_mod.detect_port() == HDSC.by_id


def test_detect_port_refuses_to_guess_when_no_b601_is_attached(monkeypatch):
    monkeypatch.setattr(bus_mod, "usb_serial_ports", lambda: [TEENSY])
    monkeypatch.setattr(bus_mod.glob, "glob", lambda pattern: [])
    with pytest.raises(BusError) as info:
        bus_mod.detect_port()
    msg = str(info.value)
    assert "no B601 USB-CAN board found" in msg and "/dev/ttyACM0 (16c0:0483" in msg


def test_acquire_reopens_a_bus_left_closed_by_a_failed_reconnect(factory):
    bus = SharedBus.acquire("/dev/fake0")
    bus._close_controller()  # what reconnect() leaves behind when every attempt fails
    again = SharedBus.acquire("/dev/fake0")
    assert again is bus and bus.controller is factory.latest and len(factory.controllers) == 2
    bus.release()
    bus.release()
    assert SharedBus._instances == {}


def test_motor_timeout_is_not_treated_as_a_dead_link(factory, monkeypatch):
    from tests.fake_bus import FakeMotor

    monkeypatch.setattr(FakeMotor, "default_mode_timeouts", 1000)  # never answers
    with pytest.raises(BusError, match="motor did not reply"):
        B601Arm.new(make_config("arm", port="/dev/fake0"), {})
    assert len(factory.controllers) == 1  # no close-and-reopen of the port
    assert SharedBus._instances == {}  # and the failed build released it


async def test_transient_register_timeout_after_enable_is_retried(factory, monkeypatch):
    """A Damiao motor misses the first register read right after enable(); the build must survive it."""
    from tests.fake_bus import FakeMotor

    monkeypatch.setattr(FakeMotor, "default_mode_timeouts", 3)
    arm = B601Arm.new(make_config("arm", port="/dev/fake0"), {})
    assert all(m.mode is not None and m.mode_timeouts == 0 for m in factory.latest.motors.values())
    assert arm.bus.reconnects == 0 and len(factory.controllers) == 1
    await arm.close()

    from src.rebot_b601.gripper import B601Gripper

    monkeypatch.setattr(FakeMotor, "default_mode_timeouts", 3)
    g = B601Gripper.new(make_config("gripper", port="/dev/fake1"), {})
    assert factory.latest.motors[0x07].mode is not None
    await g.close()


async def test_discovery_emits_arm_and_gripper_configs_per_board(monkeypatch):
    from src.rebot_b601.discovery import B601Discovery

    second = bus_mod.UsbSerialPort("/dev/ttyACM2", "2e88", "4603", "0000000007AB", "HDSC CDC Device", None)
    monkeypatch.setattr(bus_mod, "usb_serial_ports", lambda: [TEENSY, HDSC, second])
    disc = B601Discovery("disc")
    configs = await disc.discover_resources()
    assert [c.name for c in configs] == ["rebot-arm", "rebot-gripper", "rebot-arm-2", "rebot-gripper-2"]
    arm, gripper, arm2, _ = configs
    assert arm.model == "devrel:rebot-b601:arm" and arm.attributes["port"] == HDSC.by_id
    assert arm.frame.parent == "world"
    assert gripper.attributes["arm"] == "rebot-arm" and list(gripper.depends_on) == ["rebot-arm"]
    assert gripper.frame.parent == "rebot-arm"
    assert arm2.attributes["port"] == "/dev/ttyACM2"  # no by-id link: fall back to the device node
    listing = await disc.do_command({"serial_ports": True})
    assert [p["b601"] for p in listing["serial_ports"]] == [False, True, True]


async def test_discovery_with_no_boards_is_empty_not_an_error(monkeypatch):
    from src.rebot_b601.discovery import B601Discovery

    monkeypatch.setattr(bus_mod, "usb_serial_ports", lambda: [TEENSY])
    assert await B601Discovery("disc").discover_resources() == []
