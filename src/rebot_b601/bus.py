"""Shared access to the B601's USB-CAN serial bridge.

The arm (motors 0x01-0x06) and the gripper (motor 0x07) live on the same CAN
bus behind one serial device, but are separate Viam components. This module
hands both of them the same motorbridge Controller, guarded by a single lock,
so their traffic never interleaves mid-transaction.
"""

import glob
import threading
import time

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


def detect_port() -> str:
    """Find the B601's USB-CAN bridge (enumerates as an 'HDSC CDC Device')."""
    matches = sorted(glob.glob("/dev/serial/by-id/usb-HDSC_CDC_Device_*"))
    if matches:
        return matches[0]
    return "/dev/ttyACM0"


class SharedBus:
    """One motorbridge Controller per serial port, shared across components."""

    _instances: dict = {}
    _instances_lock = threading.Lock()

    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.lock = threading.RLock()
        self.controller = Controller.from_dm_serial(serial_port=port, baud=baud)
        self._motors: dict = {}
        self._refcount = 0

    @classmethod
    def acquire(cls, port: str, baud: int = DEFAULT_BAUD) -> "SharedBus":
        with cls._instances_lock:
            bus = cls._instances.get(port)
            if bus is None:
                bus = cls(port, baud)
                cls._instances[port] = bus
            bus._refcount += 1
            return bus

    def release(self):
        with self._instances_lock:
            self._refcount -= 1
            if self._refcount <= 0:
                with self.lock:
                    try:
                        self.controller.close()
                    except Exception:
                        pass
                self._instances.pop(self.port, None)

    def motor(self, can_id: int):
        """Get (or lazily register) the motor handle for a CAN id."""
        with self.lock:
            m = self._motors.get(can_id)
            if m is None:
                m = self.controller.add_damiao_motor(
                    can_id, can_id + FEEDBACK_ID_OFFSET, MOTOR_MODELS[can_id]
                )
                self._motors[can_id] = m
            return m

    def poll_feedback(self, can_ids, retries: int = 5):
        """Request and collect fresh feedback for the given motors.

        A single poll does not always drain every motor's reply off the bus, so
        request/poll is retried until every motor has reported (or retries run
        out). Returns {can_id: MotorState | None}.
        """
        with self.lock:
            motors = {cid: self.motor(cid) for cid in can_ids}
            states = {cid: None for cid in can_ids}
            for attempt in range(retries):
                for cid, m in motors.items():
                    if states[cid] is None:
                        m.request_feedback()
                try:
                    self.controller.poll_feedback_once()
                except Exception:
                    pass  # retry below; callers decide how to react to gaps
                for cid, m in motors.items():
                    if states[cid] is None:
                        states[cid] = m.get_state()
                if all(s is not None for s in states.values()):
                    break
                time.sleep(0.02)
            return states
