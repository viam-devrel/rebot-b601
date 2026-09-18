"""B601-RS (RobStride over CAN) behaviour. DM behaviour is covered by the other test files."""

import pytest

from src.rebot_b601 import bus as bus_mod
from src.rebot_b601.bus import BusError, SharedBus, canonical_device


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
