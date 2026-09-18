"""Shared access to the B601's CAN bus (a USB serial bridge, or a CAN channel).

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
  ``/dev/serial/by-id/...`` symlink never open the same device twice (a CAN
  channel name such as ``can0`` or ``PCAN_USBBUS1`` is not a path, so it is
  cached verbatim);
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
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from motorbridge import Controller
from viam.logging import getLogger

LOGGER = getLogger(__name__)

VENDORS = ("damiao", "robstride")

# Motor model per CAN id and vendor, from Seeed's B601-DM and B601-RS reference configs.
MOTOR_MODELS = {
    "damiao": {0x01: "4340P", 0x02: "4340P", 0x03: "4340P", 0x04: "4310", 0x05: "4310", 0x06: "4310", 0x07: "4310"},
    "robstride": {
        0x01: "rs-06",
        0x02: "rs-06",
        0x03: "rs-06",
        0x04: "rs-00",
        0x05: "rs-00",
        0x06: "rs-00",
        0x07: "rs-00",
    },
}
FEEDBACK_ID_OFFSET = 0x10  # Damiao: motor 0x01 replies on 0x11, etc.
ROBSTRIDE_HOST_ID = 0xFD  # RobStride: every motor addresses the host as 0xFD

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


# The B601's USB-CAN bridge enumerates as an "HDSC CDC Device".
B601_USB_VID = "2e88"
B601_USB_PID = "4603"


@dataclass(frozen=True)
class UsbSerialPort:
    """A USB serial device as the kernel identifies it, without opening it."""

    device: str  # /dev/ttyACM0
    vid: str  # lowercase hex, e.g. "2e88"
    pid: str
    serial: str
    product: str
    by_id: Optional[str]  # stable /dev/serial/by-id/... symlink, if udev made one

    @property
    def path(self) -> str:
        return self.by_id or self.device


def _sysfs_attr(usb_dir: str, name: str) -> Optional[str]:
    try:
        with open(os.path.join(usb_dir, name)) as f:
            return f.read().strip()
    except OSError:
        return None


def _by_id_link(device: str) -> Optional[str]:
    for link in sorted(glob.glob("/dev/serial/by-id/*")):
        if os.path.realpath(link) == device:
            return link
    return None


def usb_serial_ports() -> List[UsbSerialPort]:
    """Enumerate USB serial ttys with their USB identity (Linux sysfs; empty elsewhere).

    Identification never opens a port, so it cannot disturb a device that another
    driver is talking to.
    """
    ports: List[UsbSerialPort] = []
    for tty in sorted(glob.glob("/sys/class/tty/ttyACM*") + glob.glob("/sys/class/tty/ttyUSB*")):
        # ttyACM: <usb device>/<interface>/ttyACMn ; ttyUSB: <usb device>/<interface>/ttyUSBn/ttyUSBn
        usb_dir = os.path.realpath(os.path.join(tty, "device", ".."))
        vid = _sysfs_attr(usb_dir, "idVendor")
        if vid is None:
            usb_dir = os.path.realpath(os.path.join(usb_dir, ".."))
            vid = _sysfs_attr(usb_dir, "idVendor")
        if vid is None:
            continue
        device = "/dev/" + os.path.basename(tty)
        ports.append(
            UsbSerialPort(
                device=device,
                vid=vid.lower(),
                pid=(_sysfs_attr(usb_dir, "idProduct") or "").lower(),
                serial=_sysfs_attr(usb_dir, "serial") or "",
                product=_sysfs_attr(usb_dir, "product") or "",
                by_id=_by_id_link(device),
            )
        )
    return ports


def find_b601_ports() -> List[UsbSerialPort]:
    """Every attached B601 USB-CAN bridge, identified by USB vendor/product id."""
    return [p for p in usb_serial_ports() if (p.vid, p.pid) == (B601_USB_VID, B601_USB_PID)]


def detect_port() -> str:
    """Find the B601's USB-CAN bridge without opening anything.

    Only a device the kernel identifies as the HDSC bridge is ever returned.
    There is deliberately no "/dev/ttyACM0" fallback: guessing a port means
    opening someone else's device (a haptic controller, a GPS, ...) and
    stealing its bytes. Raises BusError when no bridge is present.
    """
    boards = find_b601_ports()
    if boards:
        return boards[0].path
    # No sysfs (macOS, containers): fall back to udev's descriptive symlink name.
    matches = sorted(glob.glob("/dev/serial/by-id/usb-HDSC_CDC_Device_*"))
    if matches:
        return matches[0]
    present = ", ".join(f"{p.device} ({p.vid}:{p.pid} {p.product or '?'})" for p in usb_serial_ports()) or "none"
    raise BusError(
        f"no B601 USB-CAN board found (HDSC CDC Device, USB {B601_USB_VID}:{B601_USB_PID}); "
        f"USB serial devices present: {present}. Check the cable, or set 'port' explicitly."
    )


def is_serial_port(port: str) -> bool:
    """True for a serial device (the Damiao USB bridge): any absolute path.

    CAN channel names (``can0``, ``PCAN_USBBUS1``) are not paths, so anything
    that does not start with "/" is a CAN channel.
    """
    return port.startswith("/")


def canonical_device(port: str) -> str:
    """Resolve symlinks so every alias of a serial device maps to one bus.

    CAN channel names are not paths and are used as-is.
    """
    if not is_serial_port(port):
        return port
    return os.path.realpath(port)


def _open_controller(port: str, baud: int):
    if is_serial_port(port):
        return Controller.from_dm_serial(serial_port=port, baud=baud)
    # A CAN channel: SocketCAN on Linux (can0), PCAN via libPCBUSB on macOS (can0 or
    # PCAN_USBBUS1). The vendor never decides the transport; the port string does.
    return Controller(port)


def _quiet_close(controller, motors: Optional[dict] = None) -> None:
    """Free motor handles, then the controller.

    Order matters: each motorbridge Motor handle holds its own reference to the
    serial bus, so ``Controller.close()`` alone leaves the port open until every
    handle is freed, and Motor has no destructor. This is how a module process
    ended up locked out of its own port for days.
    """
    if motors:
        for m in list(motors.values()):
            try:
                close = getattr(m, "close", None)
                if close is not None:
                    close()
            except Exception:
                pass
        motors.clear()
    try:
        if controller is not None:
            controller.close()
    except Exception:
        pass


_LOCK_HINTS = ("lock", "busy")

# motorbridge reports a motor that did not answer as a CallError, the same type
# it uses for a dead serial link. Reopening the port is the wrong response to a
# silent motor (it is what lost the port to another driver in the field), so
# these are told apart by message.
_MOTOR_TIMEOUT_HINTS = ("not received within", "no feedback", "no reply", "did not respond", "no response")


def is_motor_timeout(exc: BaseException) -> bool:
    """True when a link-class error is really a motor failing to answer."""
    text = str(exc).lower()
    return any(h in text for h in _MOTOR_TIMEOUT_HINTS)


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
    """One motorbridge Controller per port, shared across components.

    A port is either a serial device (cached by its resolved path) or a CAN
    channel name such as ``can0``/``PCAN_USBBUS1`` (cached verbatim).
    """

    _instances: Dict[str, "SharedBus"] = {}
    _instances_lock = threading.Lock()
    # Test hook: replace to construct a fake controller instead of opening serial.
    controller_factory: Callable[[str, int], object] = staticmethod(_open_controller)

    def __init__(self, port: str, baud: int, vendor: str = "damiao"):
        if vendor not in VENDORS:
            raise BusError(f"unknown motor vendor '{vendor}'; expected one of {VENDORS}")
        self.port = port  # as configured, for logs
        self.device = canonical_device(port)  # cache key
        self.baud = baud
        self.vendor = vendor
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
        # One dict per controller: motor() fills it in place and the finalizer
        # below holds the same object, so handles created later are still freed.
        self._motors = {}
        if controller is not None:
            # Fires if this bus is garbage-collected without release(); the
            # callback must not reference self or it would never be collected.
            self._finalizer = weakref.finalize(self, _quiet_close, controller, self._motors)

    def _detach_finalizer(self):
        if self._finalizer is not None:
            self._finalizer.detach()
            self._finalizer = None

    def _close_controller(self):
        self._detach_finalizer()
        controller, self.controller = self.controller, None
        _quiet_close(controller, self._motors)

    # --------------------------------------------------------------- cache

    @classmethod
    def acquire(cls, port: str, baud: int = DEFAULT_BAUD, vendor: str = "damiao") -> "SharedBus":
        device = canonical_device(port)
        with cls._instances_lock:
            bus = cls._instances.get(device)
            if bus is None:
                bus = cls(port, baud, vendor)
                cls._instances[device] = bus
            elif bus.baud != baud:
                raise BusError(f"{port} is already open at {bus.baud} baud; cannot reopen it at {baud}")
            elif bus.vendor != vendor:
                raise BusError(f"{port} is already open for {bus.vendor} motors; cannot reopen it for {vendor}")
            elif bus.controller is None:
                # A previous reconnect gave up and left the bus closed while
                # someone still held a reference. Reopen rather than hand back
                # a dead bus, which would fail every call with "is not open".
                with bus.lock:
                    bus._open()
            bus._refcount += 1
            return bus

    def matches(self, port: str, baud: int, vendor: str = "damiao") -> bool:
        """True when ``port``/``baud``/``vendor`` name this same bus (aliases resolved)."""
        return canonical_device(port) == self.device and int(baud) == self.baud and vendor == self.vendor

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
                model = MOTOR_MODELS[self.vendor][can_id]
                if self.vendor == "robstride":
                    m = self.controller.add_robstride_motor(can_id, ROBSTRIDE_HOST_ID, model)
                else:
                    m = self.controller.add_damiao_motor(can_id, can_id + FEEDBACK_ID_OFFSET, model)
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
