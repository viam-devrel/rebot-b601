"""Shared access to the B601's USB-CAN serial bridge.

The arm (motors 0x01-0x06) and the gripper (motor 0x07) live on the same CAN
bus behind one serial device, but are separate Viam components. This module
hands both of them the same motorbridge Controller, guarded by a single lock,
so their traffic never interleaves mid-transaction.

``SharedBus.reconnect`` tears the controller down and reopens it after a serial
fault; components re-register their motors lazily through ``motor()`` and
re-apply their mode/enable state through ``on_reconnect`` callbacks.

Serial-lock hygiene (a field incident: a module process refused its own port
for days):

* buses are cached by the *resolved* device path, so ``/dev/ttyACM0`` and its
  ``/dev/serial/by-id/...`` symlink never open the same device twice;
* every controller has a finalizer, so a bus that is dropped without
  ``release()`` still closes its descriptor (motorbridge itself has none);
* when the OS refuses the exclusive lock, the error names the holder. If the
  holder is this very process (a leaked descriptor), it is closed and the
  open is retried once.
"""

import glob
import os
import threading
import time
import weakref
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from motorbridge import Controller
from viam.logging import getLogger

LOGGER = getLogger(__name__)

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


def canonical_device(port: str) -> str:
    """Resolve symlinks so every alias of a serial device maps to one bus."""
    return os.path.realpath(port)


def _open_controller(port: str, baud: int):
    return Controller.from_dm_serial(serial_port=port, baud=baud)


def _quiet_close(controller) -> None:
    try:
        if controller is not None:
            controller.close()
    except Exception:
        pass


_LOCK_HINTS = ("lock", "busy")


def _release_leaked_fd(fd: int) -> None:
    """Close a descriptor we no longer track, undoing its serial exclusivity first.

    Closing releases the flock, but the TIOCEXCL flag a serial library sets on
    the tty can outlive the descriptor, so clear it explicitly (TIOCNXCL) before
    closing. Both calls are best-effort; the fd may not even be a tty.
    """
    try:
        import fcntl
        import termios

        fcntl.ioctl(fd, termios.TIOCNXCL)
    except Exception:
        pass
    try:
        os.close(fd)
    except OSError:
        pass


def _looks_like_lock_error(exc: BaseException) -> bool:
    text = str(exc).lower()
    return any(h in text for h in _LOCK_HINTS)


def _fds_on_device(pid: str, device: str) -> List[int]:
    """Descriptors of process ``pid`` that point at ``device`` (Linux /proc only)."""
    found: List[int] = []
    try:
        entries = os.listdir(f"/proc/{pid}/fd")
    except OSError:
        return found
    for fd in entries:
        try:
            if os.readlink(f"/proc/{pid}/fd/{fd}") == device:
                found.append(int(fd))
        except (OSError, ValueError):
            continue
    return found


def _own_holders(device: str) -> List[int]:
    return _fds_on_device("self", device)


def _other_holders(device: str) -> List[Tuple[int, str]]:
    """(pid, cmdline) of other processes holding ``device``. Needs root to see all."""
    me = os.getpid()
    holders: List[Tuple[int, str]] = []
    try:
        pids = [d for d in os.listdir("/proc") if d.isdigit() and int(d) != me]
    except OSError:
        return holders
    for pid in pids:
        if not _fds_on_device(pid, device):
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            cmd = "?"
        holders.append((int(pid), cmd))
    return holders


def describe_holders(device: str) -> str:
    """Human-readable list of who has ``device`` open, for error messages."""
    parts = []
    own = _own_holders(device)
    if own:
        parts.append(f"held by this module process itself (fd {', '.join(map(str, own))})")
    others = _other_holders(device)
    for pid, cmd in others:
        parts.append(f"held by pid {pid} ({cmd or '?'})")
    if not parts:
        return "no holder visible from /proc (another user's process, or not Linux)"
    return "; ".join(parts)


class SharedBus:
    """One motorbridge Controller per serial port, shared across components."""

    _instances: Dict[str, "SharedBus"] = {}
    _instances_lock = threading.Lock()
    # Test hook: replace to construct a fake controller instead of opening serial.
    controller_factory: Callable[[str, int], object] = staticmethod(_open_controller)

    def __init__(self, port: str, baud: int):
        self.port = port  # as configured, for logs
        self.device = canonical_device(port)  # cache key
        self.baud = baud
        self.lock = threading.RLock()
        self.controller = None
        self._finalizer: Optional[weakref.finalize] = None
        self._motors: dict = {}
        self._refcount = 0
        self._reconnect_callbacks: List[Callable[[], None]] = []
        self.reconnects = 0
        self._open()

    # ----------------------------------------------------------- open/close

    def _open_once(self):
        try:
            return type(self).controller_factory(self.port, self.baud)
        except LINK_ERRORS as exc:
            raise BusError(f"cannot open {self.port} @ {self.baud}: {exc}") from exc

    def _open(self):
        try:
            controller = self._open_once()
        except BusError as exc:
            if not _looks_like_lock_error(exc):
                raise
            leaked = _own_holders(self.device)
            if not leaked:
                raise BusError(f"{exc}; {describe_holders(self.device)}") from exc
            # No live bus owns this device (or we would not be opening it), so
            # any descriptor of ours that points at it belongs to a controller
            # that was dropped without close(). Reclaim it rather than failing
            # forever.
            LOGGER.warning(
                "%s is already open in this process (fd %s) with no live bus; "
                "closing the leaked descriptor and retrying",
                self.device,
                ", ".join(map(str, leaked)),
            )
            for fd in leaked:
                _release_leaked_fd(fd)
            try:
                controller = self._open_once()
            except BusError as exc2:
                raise BusError(f"{exc2}; {describe_holders(self.device)}") from exc2
        self._set_controller(controller)

    def _set_controller(self, controller):
        self._detach_finalizer()
        self.controller = controller
        self._motors = {}
        if controller is not None:
            # Fires if this bus is garbage-collected without release(); the
            # callback must not reference self or it would never be collected.
            self._finalizer = weakref.finalize(self, _quiet_close, controller)

    def _detach_finalizer(self):
        if self._finalizer is not None:
            self._finalizer.detach()
            self._finalizer = None

    def _close_controller(self):
        self._detach_finalizer()
        controller, self.controller = self.controller, None
        self._motors = {}
        _quiet_close(controller)

    # --------------------------------------------------------------- cache

    @classmethod
    def acquire(cls, port: str, baud: int = DEFAULT_BAUD) -> "SharedBus":
        device = canonical_device(port)
        with cls._instances_lock:
            bus = cls._instances.get(device)
            if bus is None:
                bus = cls(port, baud)
                cls._instances[device] = bus
            elif bus.baud != baud:
                raise BusError(f"{port} is already open at {bus.baud} baud; cannot reopen it at {baud}")
            bus._refcount += 1
            return bus

    def matches(self, port: str, baud: int) -> bool:
        """True when ``port``/``baud`` name this same bus (aliases resolved)."""
        return canonical_device(port) == self.device and int(baud) == self.baud

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
                    self._close_controller()
                if self._instances.get(self.device) is self:
                    self._instances.pop(self.device, None)

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
            self._close_controller()
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
