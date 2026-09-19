"""In-memory stand-in for motorbridge's Controller/Motor used by the unit tests.

Each FakeMotor tracks toward its last commanded setpoint at the commanded
velocity cap whenever feedback is polled, so blocking moves behave like the
real arm (they take time and settle) without hardware. Knobs on the motors let
tests inject faults, stalls, temperatures, and serial-link failures.
"""

import math
import threading
import time
from typing import Dict, List, Optional

from motorbridge import CallError
from motorbridge.models import MotorState


class FakeMotor:
    default_mode_timeouts = 0  # tests set this before motors are created (they are created lazily)

    def __init__(self, controller: "FakeController", can_id: int, feedback_id: int, model: str, vendor: str = "damiao"):
        self.controller = controller
        self.can_id = can_id
        self.feedback_id = feedback_id
        self.model = model
        self.vendor = vendor
        self.pos = 0.0  # rad
        self.vel = 0.0  # rad/s
        self.vel_noise = 0.0  # RobStride: a resting motor reported -0.150 rad/s on the bench
        self.torq = 0.0
        self.t_mos = 30.0
        self.t_rotor = 30.0
        self.status_code = 0x0
        self.enabled = False
        self.mode = None
        self.target: Optional[float] = None
        self.vel_cap = 1.0  # rad/s
        self.stall_at: Optional[float] = None  # rad: stop here (object in the jaws)
        self.teleport = False  # jump to target on next poll
        self.zeroed = 0
        self.errors_cleared = 0
        self.can_timeout_ms: Optional[int] = None
        self.commands: List[tuple] = []
        self._last_update = time.monotonic()
        self._requested = False
        self.mode_failures = 0  # ensure_mode raises this many times first
        # ensure_mode times out (CallError, like a real motor busy after enable) this many times first
        self.mode_timeouts = FakeMotor.default_mode_timeouts
        self.closed = False  # motorbridge.Motor.close() was called (frees the handle's bus reference)
        self.active_report = False  # RobStride: status frames stream only when this is on
        self.stream_state = True  # False: get_state() never fills, only param reads work
        # RobStride, bench 2026-09-19: the status stream runs ~0.5 s behind the motor, so a frame
        # read during a move describes where the jaw was, not where it is. True freezes get_state()
        # at the frame captured when it was set, while mechPos reads stay live: the extreme of
        # that lag, and enough to tell a stale read from a slow motor.
        self.stream_lag = False
        self.param_reads = 0
        # RobStride RW parameters the module writes: limit_spd (rad/s) and limit_cur (A).
        # The limit_cur value stands in for the motor's factory current limit.
        self.params: Dict[int, float] = {0x7017: 5.0, 0x7018: 4.0}
        self.param_writes: List[tuple] = []
        self._frozen = None  # RobStride: the last frame, served after disable() like the real cache

    # --- motorbridge.Motor API ---
    def close(self):
        self.closed = True

    def enable(self):
        self.controller._check_link()
        self.enabled = True
        if self.vendor == "damiao" and self.status_code == 0x0:
            self.status_code = 0x1  # Damiao reports "enabled"; RobStride's field is fault bits, 0 = healthy

    def disable(self):
        self.controller._check_link()
        self.enabled = False
        self.status_code = 0x0
        self.target = None
        if self.vendor == "robstride":
            # Bench 2026-09-18: a stopped RobStride motor stops streaming status frames and
            # motorbridge keeps serving the last one, so get_state() freezes here.
            self._frozen = self._state()

    def ensure_mode(self, mode):
        self.controller._check_link()
        if self.mode_timeouts > 0:
            self.mode_timeouts -= 1
            raise CallError("ensure_mode failed: register 10 not received within 100ms")
        if self.mode_failures > 0:
            self.mode_failures -= 1
            raise RuntimeError("mode not settled")
        self.mode = mode

    def send_pos_vel(self, pos, vel):
        self.controller._check_link()
        self.commands.append(("pos_vel", pos, vel))
        self.target, self.vel_cap = pos, abs(vel)

    def robstride_send_pos_vel_pp(self, pos, vel_max, acc_set):
        self.controller._check_link()
        self.commands.append(("pos_vel_pp", pos, vel_max, acc_set))
        self.target, self.vel_cap = pos, abs(vel_max)

    def send_mit(self, pos, vel, kp, kd, tau):
        self.controller._check_link()
        self.commands.append(("mit", pos, vel, kp, kd, tau))
        if kp > 0:
            self.target, self.vel_cap = pos, 10.0

    def send_force_pos(self, pos, vel, ratio):
        self.controller._check_link()
        self.commands.append(("force_pos", pos, vel, ratio))
        self.target, self.vel_cap = pos, abs(vel)

    def request_feedback(self):
        self.controller._check_link()
        self._requested = True

    def get_state(self):
        if not self.stream_state:
            return None
        if self.vendor == "robstride":
            # Real RobStride motors ignore request_feedback(); state arrives only as
            # streamed status frames, which need active report on and the motor running.
            if self.stream_lag:
                if self._frozen is None:
                    self._frozen = self._state()
                return self._frozen
            if not self.enabled and self._frozen is not None:
                return self._frozen
            return self._state() if self.active_report else None
        if not self._requested:
            return None
        self._requested = False
        return self._state()

    def _state(self):
        return MotorState(
            can_id=self.can_id,
            arbitration_id=self.feedback_id,
            status_code=self.status_code,
            pos=self.pos,
            vel=self.vel + self.vel_noise,
            torq=self.torq,
            t_mos=self.t_mos,
            t_rotor=self.t_rotor,
        )

    def set_zero_position(self):
        self.controller._check_link()
        self.zeroed += 1
        self.pos = 0.0
        self.target = None

    def clear_error(self):
        self.controller._check_link()
        self.errors_cleared += 1
        if self.vendor == "robstride":
            self.status_code = 0x0
        elif self.status_code >= 0x8:
            self.status_code = 0x1 if self.enabled else 0x0

    def set_can_timeout_ms(self, ms):
        self.can_timeout_ms = ms

    def robstride_set_active_report(self, enabled: bool):
        self.controller._check_link()
        self.active_report = bool(enabled)

    def robstride_get_param_f32(self, param_id: int, timeout_ms: int = 1000) -> float:
        self.controller._check_link()
        self.param_reads += 1
        if param_id == 0x7019:  # mechPos, rad
            self.step()
            return self.pos
        if param_id in self.params:
            return self.params[param_id]
        raise CallError(f"param 0x{param_id:04x} read timed out")

    def robstride_write_param_f32(self, param_id: int, value: float) -> None:
        self.controller._check_link()
        self.param_writes.append((param_id, float(value)))
        self.params[param_id] = float(value)

    # --- simulation ---
    def step(self):
        now = time.monotonic()
        dt = max(0.0, now - self._last_update)
        self._last_update = now
        if self.target is None or not self.enabled:
            self.vel = 0.0
            return
        target = self.target
        if self.stall_at is not None:
            # moving toward closed (increasing) past the object is impossible
            if (target > self.pos and self.stall_at >= self.pos) or (target < self.pos and self.stall_at <= self.pos):
                if (target - self.pos) * (target - self.stall_at) > 0:
                    target = self.stall_at
        delta = target - self.pos
        if self.teleport:
            self.pos, self.vel = target, 0.0
            return
        step = self.vel_cap * dt
        if abs(delta) <= step or dt == 0.0:
            self.pos = target
            self.vel = 0.0
        else:
            self.pos += math.copysign(step, delta)
            self.vel = math.copysign(self.vel_cap, delta)


class FakeController:
    instances: List["FakeController"] = []

    def __init__(self, port: str, baud: int, fail_open: bool = False):
        if fail_open:
            raise CallError("cannot open port")
        self.port, self.baud = port, baud
        self.motors: Dict[int, FakeMotor] = {}
        self.closed = False
        self.link_down = False
        self.polls = 0
        self.lock = threading.Lock()
        FakeController.instances.append(self)

    def _check_link(self):
        if self.link_down or self.closed:
            raise CallError("serial link down")

    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str) -> FakeMotor:
        self._check_link()
        m = self.motors.get(motor_id)
        if m is None:
            m = FakeMotor(self, motor_id, feedback_id, model)
            self.motors[motor_id] = m
        return m

    def add_robstride_motor(self, motor_id: int, feedback_id: int, model: str) -> FakeMotor:
        self._check_link()
        m = self.motors.get(motor_id)
        if m is None:
            m = FakeMotor(self, motor_id, feedback_id, model, vendor="robstride")
            self.motors[motor_id] = m
        return m

    def poll_feedback_once(self):
        self._check_link()
        self.polls += 1
        for m in self.motors.values():
            m.step()

    def close(self):
        self.closed = True


class FakeFactory:
    """Callable used as SharedBus.controller_factory; remembers controllers so
    tests can reach into them and can be told to fail the next open."""

    def __init__(self):
        self.controllers: List[FakeController] = []
        self.fail_next_open = 0
        self.shared_state: Dict[int, FakeMotor] = {}

    def __call__(self, port: str, baud: int) -> FakeController:
        if self.fail_next_open > 0:
            self.fail_next_open -= 1
            raise CallError("cannot open port")
        c = FakeController(port, baud)
        # keep motor state across reconnects, as real hardware would
        for cid, m in self.shared_state.items():
            m.controller = c
            m._requested = False
            c.motors[cid] = m
        self.controllers.append(c)
        return c

    @property
    def latest(self) -> FakeController:
        return self.controllers[-1]

    def persist(self, controller: FakeController):
        self.shared_state = dict(controller.motors)
