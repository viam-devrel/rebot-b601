"""Shared access to the B601's USB-CAN serial bridge.

The arm (motors 0x01-0x06) and the gripper (motor 0x07) live on the same CAN
bus behind one serial device, but are separate Viam components. This module
hands both of them the same motorbridge Controller, guarded by a single lock,
so their traffic never interleaves mid-transaction.

``SharedBus.reconnect`` tears the controller down and reopens it after a serial
fault; components re-register their motors lazily through ``motor()`` and
re-apply their mode/enable state through ``on_reconnect`` callbacks.
"""

import glob
import threading
import time
from typing import Callable, Dict, Iterable, List, Optional

from motorbridge import Controller

# Damiao motor model per CAN id, from Seeed's B601-DM reference implementation.
MOTOR_MODELS = {
    0x01: "4340P",
    0x02: "4340P",
    0x03: "4340P",
    0x04: "4310",
    0x05: "4310",
    0x06: "4310",
    0x07: "4310",
}
FEEDBACK_ID_OFFSET = 0x10  # motor 0x01 replies on 0x11, etc.

DEFAULT_BAUD = 921600

# Errors that indicate the serial link itself failed (as opposed to a motor
# reporting a fault). motorbridge raises CallError for ABI failures; the OS
# raises OSError/serial exceptions when the device disappears.
try:
    from motorbridge import CallError as _CallError
    from motorbridge import MotorBridgeError as _MBError
except Exception:  # pragma: no cover - very old motorbridge
    _CallError = _MBError = RuntimeError

LINK_ERRORS = (_CallError, _MBError, OSError)


class BusError(RuntimeError):
    """The serial bridge is unavailable (unplugged, powered off, or busy)."""


def detect_port() -> str:
    """Find the B601's USB-CAN bridge (enumerates as an 'HDSC CDC Device')."""
    matches = sorted(glob.glob("/dev/serial/by-id/usb-HDSC_CDC_Device_*"))
    if matches:
        return matches[0]
    return "/dev/ttyACM0"


def _open_controller(port: str, baud: int):
    return Controller.from_dm_serial(serial_port=port, baud=baud)


class SharedBus:
    """One motorbridge Controller per serial port, shared across components."""

    _instances: Dict[str, "SharedBus"] = {}
    _instances_lock = threading.Lock()
    # Test hook: replace to construct a fake controller instead of opening serial.
    controller_factory: Callable[[str, int], object] = staticmethod(_open_controller)

    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.lock = threading.RLock()
        self.controller = None
        self._motors: dict = {}
        self._refcount = 0
        self._reconnect_callbacks: List[Callable[[], None]] = []
        self.reconnects = 0
        self._open()

    def _open(self):
        try:
            self.controller = type(self).controller_factory(self.port, self.baud)
        except LINK_ERRORS as exc:
            raise BusError(f"cannot open {self.port} @ {self.baud}: {exc}") from exc
        self._motors = {}

    @classmethod
    def acquire(cls, port: str, baud: int = DEFAULT_BAUD) -> "SharedBus":
        with cls._instances_lock:
            bus = cls._instances.get(port)
            if bus is None:
                bus = cls(port, baud)
                cls._instances[port] = bus
            bus._refcount += 1
            return bus

    @classmethod
    def reset_instances(cls):
        """Drop every cached bus (tests only)."""
        with cls._instances_lock:
            cls._instances.clear()

    def release(self):
        with self._instances_lock:
            self._refcount -= 1
            if self._refcount <= 0:
                with self.lock:
                    try:
                        if self.controller is not None:
                            self.controller.close()
                    except Exception:
                        pass
                    self.controller = None
                self._instances.pop(self.port, None)

    def on_reconnect(self, callback: Callable[[], None]):
        """Register a callback that restores motor state after a reconnect."""
        with self.lock:
            self._reconnect_callbacks.append(callback)

    def remove_reconnect_callback(self, callback: Callable[[], None]):
        with self.lock:
            if callback in self._reconnect_callbacks:
                self._reconnect_callbacks.remove(callback)

    def reconnect(self, attempts: int = 5, backoff_s: float = 0.5):
        """Close and reopen the serial controller, then replay reconnect callbacks.

        Raises BusError when every attempt fails.
        """
        with self.lock:
            try:
                if self.controller is not None:
                    self.controller.close()
            except Exception:
                pass
            self.controller = None
            last: Optional[Exception] = None
            for attempt in range(attempts):
                try:
                    self._open()
                    break
                except BusError as exc:
                    last = exc
                    time.sleep(backoff_s * (attempt + 1))
            if self.controller is None:
                raise BusError(f"reconnect to {self.port} failed after {attempts} attempts: {last}")
            self.reconnects += 1
            for cb in list(self._reconnect_callbacks):
                cb()

    def motor(self, can_id: int):
        """Get (or lazily register) the motor handle for a CAN id."""
        with self.lock:
            if self.controller is None:
                raise BusError(f"{self.port} is not open")
            m = self._motors.get(can_id)
            if m is None:
                m = self.controller.add_damiao_motor(can_id, can_id + FEEDBACK_ID_OFFSET, MOTOR_MODELS[can_id])
                self._motors[can_id] = m
            return m

    def poll_feedback(self, can_ids: Iterable[int], retries: int = 5, settle_s: float = 0.02):
        """Request and collect fresh feedback for the given motors.

        A single poll does not always drain every motor's reply off the bus, so
        request/poll is retried until every motor has reported (or retries run
        out). Returns {can_id: MotorState | None}. ``retries=1`` gives a cheap
        best-effort sample for in-loop monitoring.
        """
        can_ids = list(can_ids)
        with self.lock:
            motors = {cid: self.motor(cid) for cid in can_ids}
            states = {cid: None for cid in can_ids}
            for attempt in range(max(1, retries)):
                for cid, m in motors.items():
                    if states[cid] is None:
                        m.request_feedback()
                try:
                    self.controller.poll_feedback_once()
                except LINK_ERRORS:
                    raise
                except Exception:
                    pass  # retry below; callers decide how to react to gaps
                for cid, m in motors.items():
                    if states[cid] is None:
                        states[cid] = m.get_state()
                if all(s is not None for s in states.values()):
                    break
                if attempt + 1 < retries:
                    time.sleep(settle_s)
            return states
